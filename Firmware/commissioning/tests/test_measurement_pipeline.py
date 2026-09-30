from __future__ import annotations
import copy
from dataclasses import replace
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from Firmware.commissioning.contracts import Identity,Reason,Rejected,digest
from Firmware.commissioning.normalization import normalize_raw,current_si
from Firmware.commissioning.measurement import estimate_observer,lever_arm_calibration,accelerometer_model,convert_encoder


def raw_fixture():
    t=np.arange(0,3,.01);q=.1+.1*t;v=np.full(len(t),.1)
    def stream():
        return {"sample_time_s":t.tolist(),"receive_time_s":(t+.001).tolist(),"sequence":list(range(1,len(t)+1)),
                "generation":[1]*len(t),"valid":[True]*len(t)}
    calibration={"clocks":{name:{"scale":1.,"offset_s":0.,"source_range_s":[0.,3.],"residual_p99_s":.0001}
                            for name in ("encoder","gyro","current","tx")},
        "max_gap_s":{name:.02 for name in ("encoder","gyro","current","tx")},
        "encoder":{"counts_per_motor_turn":65536,"motor_turns_per_output_turn":1.,"sign":1,
                   "physical_zero_rad":0.,"session_offset_rad":0.},
        "imu":{"body_to_sensor":np.eye(3).tolist(),"gyro_bias_rad_s":[.01,.02,.03],"gyro_noise_rad_s":[1e-5]*3},
        "current":{"scale_A_per_count":.001,"bias_A":0.,"source":"iqf","filter_tau_s":.02},
        "current_feedback_cap_A":2.,"tx_quantum_A":.0001,"sigma_q_rad":.0001,"sigma_v_rad_s":.0001,
        "valid_band_hz":[.5,15.],"gyro_filter_tau_s":0.}
    calibration["identity_hash"]=digest(calibration)
    identity=Identity(digest("raw-fixture-hardware"),calibration["identity_hash"],digest("raw-fixture-O"),"SYNTHETIC")
    raw={"axis":"yaw","identity":identity.__dict__,"run_id":"raw-independent","mode":"current",
         "readback_verified":True,"capture_ready":True,"posture_time_s":t.tolist(),"other_axis_rad":[0.]*len(t)}
    raw["encoder"]={**stream(),"raw_count":np.round(q*65536/(2*np.pi)).astype(int).tolist()}
    raw["gyro"]={**stream(),"pitch_rad":[0.]*len(t),"raw_rad_s":np.c_[np.full(len(t),.01),np.full(len(t),.02),v+.03].tolist()}
    raw["current"]={**stream(),"raw":[100.]*len(t)}
    raw["tx"]={**stream(),"successful_A":[.1]*len(t),"limited_A":[.1]*len(t),"requested_A":[.12]*len(t),
               "success":[True]*len(t),"owner_epoch":[1]*len(t),"direction":[1]*len(t)}
    return raw,calibration,identity


class MeasurementPipelineTests(unittest.TestCase):
    def test_bounded_pitch_encoding_crosses_existing_normalization_boundary(self):
        raw,cal,identity=raw_fixture();raw['axis']='pitch'
        t=np.asarray(raw['encoder']['sample_time_s']);q=.1+.1*t
        raw['encoder']['raw_count']=np.rint((q+12.5)*65535/25).astype(int).tolist()
        raw['gyro']['raw_rad_s']=np.tile([.01,.12,.03],(len(q),1)).tolist()
        cal['encoder']={'encoding':'bounded_count','raw_min':0,'raw_max':65535,
            'shaft_min_rad':-12.5,'shaft_max_rad':12.5,'motor_turns_per_output_turn':1.,'sign':1,
            'physical_zero_rad':0.,'session_offset_rad':0.}
        cal['identity_hash']=digest({k:v for k,v in cal.items() if k!='identity_hash'})
        identity=replace(identity,measurement=cal['identity_hash']);raw['identity']=identity.__dict__
        run=normalize_raw(raw,cal,identity)
        np.testing.assert_allclose(run.q,q,atol=25/65535/2,rtol=0)
        np.testing.assert_allclose(run.v,.1,atol=1e-12)
        self.assertEqual(run.identity.provenance,'SYNTHETIC')
        # A finite end-to-end jump is not a modulo wrap, and both endpoints count.
        np.testing.assert_allclose(convert_encoder([0,65535,0],cal['encoder']),[-12.5,12.5,-12.5])
        for bad in ([65536],[-1],[1.5],[True],['12']):
            with self.assertRaises(Rejected):convert_encoder(bad,cal['encoder'])

    def test_measured_encoder_cannot_guess_encoding_or_mix_mapping_fields(self):
        _,cal,_=raw_fixture()
        with self.assertRaises(Rejected):convert_encoder([1,2,3],cal['encoder'],measured=True)
        with self.assertRaises(Rejected):convert_encoder([1,2,3],{**cal['encoder'],'encoding':'bounded_count'})
        with self.assertRaises(Rejected):convert_encoder([1,2,3],{**cal['encoder'],'encoding':'unknown'})

    def test_raw_units_coordinates_clock_and_tx_normalize(self):
        raw,cal,identity=raw_fixture();run=normalize_raw(raw,cal,identity)
        np.testing.assert_allclose(run.v,.1,atol=1e-12);np.testing.assert_allclose(run.tx,.1)
        self.assertGreater(run.q[-1],run.q[0]);self.assertTrue(run.q_new.all())
    def test_failed_tx_cannot_be_interpolated_into_valid_input(self):
        raw,cal,identity=raw_fixture();raw["tx"]["success"][10]=False
        with self.assertRaises(Rejected) as cm:normalize_raw(raw,cal,identity)
        self.assertEqual(cm.exception.reason,Reason.DATA_INVALID)
    def test_truthy_false_string_is_not_success(self):
        raw,cal,identity=raw_fixture();raw["tx"]["success"][10]="false"
        with self.assertRaises(Rejected):normalize_raw(raw,cal,identity)
    def test_changed_calibration_with_old_hash_rejected(self):
        raw,cal,identity=raw_fixture();cal["encoder"]["sign"]=-1
        with self.assertRaises(Rejected):normalize_raw(raw,cal,identity)
    def test_asynchronous_sources_preserve_new_sample_masks(self):
        raw,cal,identity=raw_fixture()
        for key,value in raw["gyro"].items():raw["gyro"][key]=value[::2]
        cal["valid_band_hz"]=[.5,8.]
        cal["identity_hash"]=digest({k:v for k,v in cal.items() if k!="identity_hash"})
        identity=replace(identity,measurement=cal["identity_hash"]);raw["identity"]=identity.__dict__
        run=normalize_raw(raw,cal,identity)
        self.assertLess(run.v_new.sum(),len(run.t));self.assertGreater(run.v_new.sum(),100)
    def test_unqualified_current_feedback_triggers_protection_reason(self):
        raw,cal,identity=raw_fixture();raw["current"]["raw"][20]=9999
        with self.assertRaises(Rejected) as cm:normalize_raw(raw,cal,identity)
        self.assertEqual(cm.exception.reason,Reason.HARD_ABORT)
    def test_observer_noise_is_computed_from_data(self):
        rng=np.random.default_rng(32);t=np.arange(0,2,.01)
        q=.1*t*t+rng.normal(0,2e-5,len(t));v=.2*t+rng.normal(0,8e-5,len(t))
        oq=rng.normal(0,2e-5,500);ov=rng.normal(0,8e-5,500)
        observer=estimate_observer(t,q,v,oq,ov,measurement_hash=digest("measurement"),provenance="SYNTHETIC")
        self.assertAlmostEqual(observer.encoder_variance,np.var(oq,ddof=1))
        self.assertGreater(observer.process_variance,1e-10)
    def test_lever_arm_and_bias_recovered_when_excited(self):
        rng=np.random.default_rng(62);n=100;omega=rng.normal(0,.2,(n,3));alpha=rng.normal(0,.2,(n,3))
        arm=np.array([.03,-.02,.01]);bias=np.array([.02,-.01,.04]);gravity=np.array([0,0,-9.81])
        measured=np.cross(alpha,arm)+np.cross(omega,np.cross(omega,arm))-gravity+bias
        result=lever_arm_calibration(np.tile(np.eye(3),(n,1,1)),np.zeros((n,3)),omega,alpha,gravity,measured)
        np.testing.assert_allclose(result["lever_arm_m"],arm,atol=1e-10)
        np.testing.assert_allclose(result["accel_bias_m_s2"],bias,atol=1e-10)
    def test_accelerometer_frames_and_changing_body_gravity(self):
        rng=np.random.default_rng(301);n=120
        mount=Rotation.from_euler('xyz',[.3,-.2,.7]).as_matrix()
        body_to_world=Rotation.from_euler('xyz',rng.normal(0,.3,(n,3))).as_matrix()
        gravity=np.einsum('nji,j->ni',body_to_world,[0.,0.,-9.81])
        arm=np.array([.04,-.03,.02]);bias=np.array([.02,.01,-.01])
        omega=rng.normal(0,.4,(n,3));alpha=rng.normal(0,.3,(n,3));a0=rng.normal(0,.2,(n,3))
        body=a0+np.cross(alpha,arm)+np.cross(omega,np.cross(omega,arm))-gravity
        measured=body@mount.T+bias
        np.testing.assert_allclose(accelerometer_model(mount,a0,omega,alpha,arm,gravity,bias),measured)
        fit=lever_arm_calibration(np.tile(mount,(n,1,1)),a0,omega,alpha,gravity,measured)
        np.testing.assert_allclose(fit['lever_arm_m'],arm,atol=1e-10)
        np.testing.assert_allclose(fit['accel_bias_m_s2'],bias,atol=1e-10)


if __name__=="__main__":unittest.main()
