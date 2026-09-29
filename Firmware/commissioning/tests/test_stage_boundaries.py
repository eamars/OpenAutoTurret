from __future__ import annotations
import importlib.util
import json
from pathlib import Path
import unittest
import numpy as np

from Firmware.commissioning.contracts import Reason,Rejected,digest
from Firmware.commissioning.identification import observe_gyro,identify
from Firmware.commissioning.measurement import calibrate_causal_filter,calibrated_bandwidth
from Firmware.commissioning.native import Native
from Firmware.commissioning.synthetic import fixture,runs
from Firmware.commissioning.adaptation import choose_information,select_supplemental
from Firmware.commissioning.probe_control import fixture_control
from Firmware.commissioning.provenance import method_identity,identification_component

ADR=Path(__file__).resolve().parents[2]/"docs/ADR-002.2"
spec=importlib.util.spec_from_file_location("adr0022_evidence_contract",ADR/"reference/contracts.py")
evidence=importlib.util.module_from_spec(spec);spec.loader.exec_module(evidence)


class BoundaryTests(unittest.TestCase):
    def test_stage1_pass_does_not_require_physical_prerequisites(self):
        req=json.loads((ADR/"contracts/requirements.json").read_text())["requirements"]
        rows=[{"id":r["id"],"status":"PASS","execution":"software","evidence_ids":["test-evidence"]}
              for r in req if r["stage"] in ("1","all") and r["evidence_kind"]=="software"]
        self.assertEqual(evidence.stage1_completion_reasons(req,rows),[])
        self.assertTrue(evidence.completion_reasons(req,rows,False))
        for rid in ("S1-05","S1-06"):
            self.assertEqual(next(r for r in req if r["id"]==rid)["stage"],"2")
    def test_stage1_missing_software_is_not_ready(self):
        req=json.loads((ADR/"contracts/requirements.json").read_text())["requirements"]
        self.assertIn("UNMET_S1-03",evidence.stage1_completion_reasons(req,[]))
    def test_reuse_tracks_numerical_dependencies_instead_of_relabelling_old_fits(self):
        import copy
        method=method_identity(Native());original=identification_component(method)
        other=copy.deepcopy(method);other['source_files']['synthesis.py']='a'*64
        self.assertEqual(identification_component(other),original)
        other['source_files']['identification.py']='b'*64
        self.assertNotEqual(identification_component(other),original)
        other=copy.deepcopy(method);other['native_sha256']='c'*64
        self.assertNotEqual(identification_component(other),original)
    def test_measurement_filter_fitted_and_used_causally(self):
        rng=np.random.default_rng(22);t=np.arange(0,5,.005)
        reference=.1*np.sin(4*t)+.03*np.sin(19*t)
        measured=observe_gyro(reference,t,.022)+rng.normal(0,1e-5,len(t))
        fit=calibrate_causal_filter(t,reference,measured,noise_sigma=1e-5,tau_bound_s=.1)
        self.assertAlmostEqual(fit["filter_tau_s"],.022,delta=1e-5)
        prefix=observe_gyro(reference[:500],t[:500],.022)
        np.testing.assert_array_equal(prefix,observe_gyro(reference,t,.022)[:500])
    def test_sampling_rate_alone_does_not_establish_bandwidth(self):
        t=np.arange(0,10,.005)
        with self.assertRaises(Rejected):calibrated_bandwidth(t,np.zeros(len(t)),np.zeros(len(t)),.001)
    def test_broadband_correlated_measurement_defines_a_band(self):
        rng=np.random.default_rng(91);t=np.arange(0,12,.005);reference=rng.normal(0,.1,len(t))
        measured=reference+rng.normal(0,.001,len(t))
        result=calibrated_bandwidth(t,reference,measured,.001)
        self.assertLessEqual(result["valid_band_hz"][1],40.00001)
        self.assertGreater(result["valid_band_hz"][1],10.)
    def test_full_fit_rejects_independent_unmodelled_resonance(self):
        spec,truth,identity=fixture();train=runs(spec,truth,identity)
        holdout=runs(spec,truth,identity,seed=23,repetitions=2,mismatch=True)
        with self.assertRaises(Rejected) as exc:
            identify(Native(),spec,train,holdout,delay_bound_s=.025,bootstrap=False)
        self.assertEqual(exc.exception.reason,Reason.MODEL_INADEQUATE)
    def test_selector_uses_injected_bounds_and_deterministic_ties(self):
        snapshot,_,envelope,_=fixture_control();n=snapshot.spec.size
        args=(Native(),snapshot.spec,snapshot.theta,snapshot.uncertainty,envelope,np.eye(n)*1e-3)
        kwargs=dict(initial_position=0.,posture=0.,direction=1,noise_sigma=.001,sample_hz=100.)
        selected=choose_information(*args,**kwargs)
        self.assertGreater(len(selected),0)
        again=choose_information(*args,**kwargs)
        self.assertEqual([r["case_id"] for r in selected],[r["case_id"] for r in again])
        self.assertLessEqual(len(selected),3)
        for r in selected:self.assertLessEqual(np.max(np.abs(r["successful_tx"])),envelope.current_a)
    def test_supplement_selector_chooses_grid_and_direction_without_manual_selection(self):
        snapshot,_,envelope,_=fixture_control();n=snapshot.spec.size
        selected=select_supplemental(Native(),snapshot,envelope,np.eye(n)*1e-3,noise_sigma=.001,sample_hz=100.)
        self.assertEqual(len(selected),3)
        self.assertEqual(len({r['selection_id'] for r in selected}),3)
        for row in selected:
            self.assertIn(row['direction'],(-1,1));self.assertIn(row['posture_rad'],snapshot.spec.z_nodes)
            self.assertGreater(row['score'],0.)


if __name__=="__main__":unittest.main()
