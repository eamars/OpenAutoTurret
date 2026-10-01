from pathlib import Path
import json
from dataclasses import replace
import os
import tempfile
import unittest

import numpy as np
from scipy.integrate import quad
from scipy.special import ndtr

from Firmware.commissioning.model_family import FamilyNative
from Firmware.tools.adr0022_nuisance_verification import (
    generate,true_model,fit_group,forward_gate,run_from_data,mechanics_gauge_probe,training_prefix,
    FIT_INITIAL,FIXTURE,analytic_electrical_trace,prescribed_input,independent_noisy_copy,gaussian_bin_deviance,
    expected_encoder_information,bin_final_source_admission,BIN_FINAL_SEEDS,BIN_FINAL_TRAIN)


class NuisanceVerificationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.native = FamilyNative(Path(os.environ["OTA_AXIS_CORE_LIBRARY"]))

    def test_native_generated_inverse_cannot_claim_independent_forward_validation(self):
        model = true_model()
        data,_ = generate(self.native,model,"development-pulses",1601,False,"native")
        self.assertEqual(forward_gate(self.native,model,data)["status"],"NOT_RUN_NATIVE_SELF_GENERATED")
        with tempfile.TemporaryDirectory() as directory:
            result = fit_group(self.native,model,"actuator_tau","native-fixture",data,
                               {"native-fixture":data},Path(directory),120)
        self.assertTrue(result["optimizer"]["success"])
        self.assertEqual(result["gates"]["synthetic_parameter_recovery"],"PASS")
        self.assertEqual(result["gates"]["selection_trajectory"],"NOT_RUN")
        self.assertFalse(result["synthetic_scope_verified"])

    def test_independent_current_pair_recovers_with_native_masks_and_unchanged_forward_gate(self):
        model = true_model()
        data,_ = generate(self.native,model,"development-pulses",1601,True,"independent")
        selection,_ = generate(self.native,model,"development-alternating",1601,True,"independent")
        run = run_from_data("independent-fixture",data)
        self.assertEqual(int(run.q_new.sum()),4001)
        self.assertEqual(int(run.v_new.sum()),201)
        self.assertEqual(int(run.current_new.sum()),4001)
        self.assertTrue(np.all(np.diff(run.tx_t)>0))
        self.assertLess(run.tx_t[0],run.t[0]-model.transport_delay-model.current_delay)
        with tempfile.TemporaryDirectory() as directory:
            result = fit_group(self.native,model,"current_pair","independent-fixture",data,
                {"independent-fixture":data,"selection":selection},Path(directory),120)
            prediction_path = Path(directory)/"current_pair-fit-independent-fixture-predict-selection.npz"
            with np.load(prediction_path,allow_pickle=False) as archive:
                self.assertEqual(archive["prediction"].shape,(4001,6))
                np.testing.assert_array_equal(archive["v_new"],selection["v_new"])
        self.assertEqual(result["forward"]["limits"]["current"],1e-9)
        self.assertTrue(result["synthetic_scope_verified"])
        self.assertEqual(result["gates"]["physical_stage3a"],"NOT_RUN")
        self.assertFalse(result["gates"]["deployment_authorized"])

    def test_reported_current_ambiguities_are_distinguished_by_motion(self):
        model = true_model()
        data,_ = generate(self.native,model,"development-pulses",1601,False,"native")
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            source = output/"retained.npz"
            np.savez_compressed(source,**data)
            mechanics_gauge_probe(self.native,model,source,output)
            evidence = json.loads((output/"mechanics-gauge-results.json").read_text())
        self.assertEqual(len(evidence["results"]),2)
        for row in evidence["results"]:
            self.assertLess(row["observation_rms_difference"]["current"],1e-9)
            self.assertTrue(row["motion_discriminates"])
            self.assertFalse(row["reported_current_discriminates"])

    def test_training_prefix_is_causal_and_keeps_native_masks_and_initial_state(self):
        model = true_model()
        data,_ = generate(self.native,model,"development-pulses",1601,True,"native")
        prefix = training_prefix(data,1.1)
        self.assertEqual(len(prefix["t"]),1101)
        np.testing.assert_array_equal(prefix["v_new"],data["v_new"][:1101])
        np.testing.assert_array_equal(prefix["initial"],data["initial"])
        self.assertEqual(prefix["tx_t"][0],data["tx_t"][0])
        self.assertLessEqual(prefix["tx_t"][-1],prefix["t"][-1])
        pred = self.native.rollout(model,prefix["t"],prefix["tx_t"],prefix["tx_A"],prefix["initial"])
        self.assertLess(np.max(np.abs(pred[:,0]-data["truth"][:1101,0])),1e-5)
        self.assertLess(np.max(np.abs(pred[:,3]-data["truth"][:1101,3])),1e-4)
        self.assertLess(np.max(np.abs(pred[:,4]-data["truth"][:1101,4])),1e-9)
        self.assertEqual(len(data["t"]),4001)

    def test_analytic_electrical_initializer_matches_fractional_grid_and_equal_poles(self):
        model = replace(true_model(),q_min=-100.,q_max=100.)
        data,_ = generate(self.native,model,"information-full-chirp-30s",1601,False,"native")
        data = training_prefix(data,5.)
        for changes in (FIT_INITIAL,{}, {"actuator_tau":.012,"current_tau":.012},
                        {"transport_delay":.008,"current_delay":.004}):
            candidate = replace(model,**changes)
            actual,current,_ = analytic_electrical_trace(candidate,data)
            pred = self.native.rollout(candidate,data["t"],data["tx_t"],data["tx_A"],data["initial"])
            self.assertLess(np.max(np.abs(actual-pred[:,2])),1e-9)
            self.assertLess(np.max(np.abs(current-pred[:,4])),1e-9)

    def test_reporting_delays_cannot_change_mechanics_or_other_reported_channel(self):
        model = replace(true_model(),q_min=-100.,q_max=100.,**FIT_INITIAL)
        t = np.arange(4001)*.001
        tx_t,tx_A = prescribed_input("information-mechanics-octave-362s")
        causal = tx_t<=t[-1]
        tx_t,tx_A = tx_t[causal],tx_A[causal]
        baseline = self.native.rollout(model,t,tx_t,tx_A,np.zeros(5))
        for field,other,own in (("gyro_delay",4,3),("current_delay",3,4)):
            for step in (1.1e-8,1e-5):
                changed = self.native.rollout(replace(model,**{field:getattr(model,field)+step}),
                    t,tx_t,tx_A,np.zeros(5))
                np.testing.assert_array_equal(changed[:,:3],baseline[:,:3])
                np.testing.assert_array_equal(changed[:,other],baseline[:,other])
                self.assertGreater(np.max(np.abs(changed[:,own]-baseline[:,own])),0.)

    def test_linear_ablation_rejects_noisy_subgroup_and_restart_claims(self):
        model = true_model()
        with tempfile.TemporaryDirectory() as directory:
            for group,data,initial in (
                ("mechanics_and_nuisance",{"noisy":True},None),
                ("gyro_pair",{"noisy":False},None),
                ("mechanics_and_nuisance",{"noisy":False},model)):
                with self.assertRaisesRegex(ValueError,"original nontruth initializer"):
                    fit_group(self.native,model,group,"scope-check",data,{},
                        Path(directory),120,initial_model=initial,loss="linear")

    def test_fresh_draw_copy_preserves_original_noise_and_freshness_protocol(self):
        model = true_model()
        base,_ = generate(self.native,model,"development-pulses",1601,False,"independent")
        original,_ = generate(self.native,model,"development-pulses",1601,True,"independent")
        copied = independent_noisy_copy(base,1601)
        for key in ("q","v","current","q_new","v_new","current_new","initial","truth"):
            np.testing.assert_array_equal(copied[key],original[key])
        np.testing.assert_array_equal(copied["noise_sources"],original["noise_sources"])

    def test_encoder_bin_deviance_preserves_center_slope_and_unfloored_tail(self):
        sigma,quantum = FIXTURE["encoder_noise"],FIXTURE["encoder_quantum"]
        r,slope = gaussian_bin_deviance(0.,0.,quantum,sigma)
        self.assertEqual(float(r),0.)
        self.assertGreater(float(slope),0.)
        small = sigma*1e-10
        rp,_ = gaussian_bin_deviance(small,0.,quantum,sigma)
        rm,_ = gaussian_bin_deviance(-small,0.,quantum,sigma)
        self.assertAlmostEqual(float((rp-rm)/(2*small)/slope),1.,places=12)
        b = quantum/(2*sigma)
        p0 = quad(lambda x:np.exp(-x*x/2)/np.sqrt(2*np.pi),-b,b)[0]
        for d in (.01,.01001,.5,b,8.):
            probability = quad(lambda x:np.exp(-x*x/2)/np.sqrt(2*np.pi),-b-d,b-d,
                epsabs=1e-25,epsrel=1e-12)[0]
            expected = np.sqrt(2*(np.log(p0)-np.log(probability)))
            value,derivative = gaussian_bin_deviance(sigma*d,0.,quantum,sigma)
            self.assertAlmostEqual(float(value)/expected,1.,places=8)
            h = sigma*1e-5
            upper,_ = gaussian_bin_deviance(sigma*d+h,0.,quantum,sigma)
            lower,_ = gaussian_bin_deviance(sigma*d-h,0.,quantum,sigma)
            self.assertLess(abs(float((upper-lower)/(2*h)/derivative)-1),1e-5)
        tail,tail_slope = gaussian_bin_deviance(sigma*np.array([-1e6,1e6]),0.,quantum,sigma)
        self.assertTrue(np.isfinite(tail).all())
        self.assertTrue(np.isfinite(tail_slope).all())
        self.assertEqual(float(tail[0]),-float(tail[1]))
        self.assertTrue(np.all(tail_slope>0))

    def test_expected_encoder_information_matches_direct_cdf_score(self):
        sigma,quantum = FIXTURE["encoder_noise"],FIXTURE["encoder_quantum"]
        phases = quantum*np.array([-.49,0.,.15,.49])
        information,mass = expected_encoder_information(phases)
        expected = np.zeros_like(phases)
        for offset in range(-3,4):
            lo,hi = ((offset-.5)*quantum-phases)/sigma,((offset+.5)*quantum-phases)/sigma
            probability = np.where(lo>=0,ndtr(-lo)-ndtr(-hi),ndtr(hi)-ndtr(lo))
            prime = (np.exp(-lo**2/2)-np.exp(-hi**2/2))/(sigma*np.sqrt(2*np.pi))
            expected += prime**2/probability
        np.testing.assert_allclose(information,expected,rtol=1e-12)
        np.testing.assert_allclose(mass,1.,atol=1e-14)
        self.assertTrue(np.all(information<1/sigma**2))

    def test_final_admission_rejects_reused_seed_or_input_metadata(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for seed,input_name in ((BIN_FINAL_SEEDS[0],"old-input"),(3,BIN_FINAL_TRAIN)):
                path = root/"old-dataset.npz"
                np.savez_compressed(path,seed=seed,input_name=input_name)
                with self.assertRaisesRegex(ValueError,"already retained"):
                    bin_final_source_admission(root)
                path.unlink()
            np.savez_compressed(root/"old-dataset.npz",seed=3,input_name="old-input")
            result = bin_final_source_admission(root)
            self.assertEqual(result["status"],"PASS")
            self.assertEqual(result["datasets_with_scalar_seed"],1)


if __name__=="__main__":
    unittest.main()
