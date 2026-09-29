from __future__ import annotations
import copy
from dataclasses import replace
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from Firmware.commissioning.contracts import *
from Firmware.commissioning.model import basis, coefficients
from Firmware.commissioning.measurement import *
from Firmware.commissioning.identification import check_split, integral_design, bounded_initializer, validation_report
from Firmware.commissioning.synthetic import fixture, runs
from Firmware.commissioning.probe_control import fixture_control
from Firmware.commissioning.native import Native, Controller, parameters, CObservation, CReference
from Firmware.commissioning.synthesis import runtime_values, candidate_grid, linear_check
from Firmware.commissioning.metrics import shaped_velocity, shaped_step, motion_metrics
from Firmware.commissioning.protocol import Ownership, SimulatedEndpoint
from Firmware.commissioning.parameter_catalog import core_registry, catalog, pending_document, validate_parameter_document
from Firmware.commissioning.adaptation import FailurePolicy, ChangeMonitor, reuse, first_stimulus, template
from Firmware.commissioning.breakaway import estimate_interval


class RejectionTests(unittest.TestCase):
    def reject(self,reason,function,*args,**kwargs):
        with self.assertRaises(Rejected) as caught:function(*args,**kwargs)
        self.assertEqual(caught.exception.reason,reason)


class ParameterTests(RejectionTests):
    def setUp(self):self.snapshot,self.observer,self.envelope,self.simulation=fixture_control()
    def test_complete_binding_roundtrip(self):
        s=self.snapshot;doc=json.loads(json.dumps(s.document()))
        self.assertEqual(PlantSnapshot.bind(doc,s.spec,s.identity).identity_hash,s.identity_hash)
    def test_simulated_parameters_cannot_become_measured(self):
        s=self.snapshot
        self.reject(Reason.DATA_INVALID,PlantSnapshot.bind,s.document(),s.spec,s.identity,physical=True)
    def test_units_cannot_be_copied_by_filename(self):
        s=self.snapshot;doc=s.document();doc["units"]["a"]="kg*m^2"
        self.reject(Reason.DATA_INVALID,PlantSnapshot.bind,doc,s.spec,s.identity)
    def test_stale_operating_point_rejected(self):
        s=self.snapshot
        self.reject(Reason.OPERATING_POINT_CHANGED,PlantSnapshot.bind,s.document(),s.spec,
                    replace(s.identity,operating_point=digest("new-payload")))
    def test_spatial_table_no_extrapolation(self):
        self.reject(Reason.OPERATING_POINT_CHANGED,basis,(-1,0,1),1.01)
    def test_periodic_table_continuity(self):
        nodes=np.linspace(-np.pi,np.pi,8,endpoint=False)
        self.assertTrue(np.allclose(basis(nodes,np.pi-1e-9,periodic=True),basis(nodes,-np.pi+1e-9,periodic=True),atol=1e-8))
    def test_pending_contract_has_no_invented_values(self):
        pending=pending_document()
        self.assertTrue(all(v["value"] is None for v in pending["parameters"].values()))
        self.reject(Reason.DATA_INVALID,validate_parameter_document,pending)
    def test_every_catalog_entry_covers_required_semantics(self):
        for value in catalog().values():
            self.assertTrue(all(value[k] for k in ("meaning","unit","classification","estimator","identifiability",
                "constraints","uncertainty","applicability_and_reuse","unknown_invalid_stale")))
    def test_joint_uncertainty_cannot_be_fewer_than_128(self):
        doc=self.snapshot.document();doc["uncertainty"].pop()
        self.reject(Reason.DATA_INVALID,PlantSnapshot.bind,doc,self.snapshot.spec,self.snapshot.identity)
    def test_unknown_delay_is_not_zero(self):
        self.reject(Reason.DATA_INVALID,candidate_grid,self.snapshot,self.observer,self.envelope,latency_p99_s=np.nan)
    def test_booleans_and_numeric_strings_do_not_become_measured_parameters(self):
        for invalid in (True,'0.12'):
            doc=self.snapshot.document();doc['theta'][0]=invalid
            self.reject(Reason.DATA_INVALID,PlantSnapshot.bind,doc,self.snapshot.spec,self.snapshot.identity)


class MeasurementTests(RejectionTests):
    def test_clock_drift_offset_recovered(self):
        t=np.arange(100.)
        fit=clock_calibration(t,t*1.00012+4.2,maximum_residual_s=.0001)
        self.assertAlmostEqual(fit["scale"],1.00012,places=10)
        self.assertAlmostEqual(fit["offset_s"],4.2,places=10)
    def test_clock_jump_rejected(self):
        t=np.arange(100.);host=t.copy();host[50:]+=.02
        self.reject(Reason.MEASUREMENT_LIMITED,clock_calibration,t,host,maximum_residual_s=.0001)
    def test_sequence_and_timestamp_must_both_be_new(self):
        for t,seq in (([0,.01,.01],[1,2,3]),([0,.01,.02],[1,1,2])):
            self.reject(Reason.DATA_INVALID,verify_stream,t,seq,[1,1,1],[True]*3,max_gap_s=.03)
    def test_generation_change_splits_session(self):
        self.reject(Reason.DATA_INVALID,verify_stream,[0,.01,.02],[1,2,3],[1,1,2],[True]*3,max_gap_s=.03)
    def test_gap_is_measurement_limited(self):
        self.reject(Reason.MEASUREMENT_LIMITED,verify_stream,[0,.01,.2],[1,2,3],[1,1,1],[True]*3,max_gap_s=.03)
    def test_output_gear_ratio_and_wrap(self):
        q=EncoderMapping(8192,2.,-1,.1,.2).convert([8190,8191,0,1])
        self.assertTrue(np.allclose(np.diff(q),-np.pi/8192))
    def test_mount_bias_and_axis_rate_recovered(self):
        rng=np.random.default_rng(19);n=200
        pitch=np.linspace(-.3,.3,n);rate=rng.normal(0,.1,(n,2))
        R=Rotation.from_euler("xyz",[.4,-.3,1.2]);bias=np.array([.002,-.003,.005])
        expected=np.einsum("nij,nj->ni",kinematic_axes(pitch),rate)
        gyro=R.apply(expected)+bias
        stationary=bias+rng.normal(0,1e-6,(500,3))
        fit=imu_calibration(pitch,rate,gyro,stationary)
        self.assertTrue(np.allclose(fit["body_to_sensor"],R.as_matrix(),atol=2e-6))
        self.assertTrue(np.allclose(axis_rates(pitch,gyro,fit),rate,atol=2e-6))
        wrong={**fit,"body_to_sensor":np.eye(3).tolist()}
        self.reject(Reason.DATA_INVALID,axis_rates,pitch,gyro,wrong)
    def test_one_axis_cannot_calibrate_all_mount_directions(self):
        self.reject(Reason.INSUFFICIENT_EXCITATION,imu_calibration,np.zeros(200),
                    np.tile([.1,0],(200,1)),np.tile([0,0,.1],(200,1)),np.zeros((200,3)))
    def test_unknown_lever_arm_has_no_angular_acceleration_claim(self):
        self.reject(Reason.MEASUREMENT_LIMITED,accelerometer_model,np.eye(3),[0,0,0],[0,0,0],
                    [0,0,0],None,[0,0,-9.81],[0,0,0])
    def test_accelerometer_specific_force_includes_gravity(self):
        v=accelerometer_model(np.eye(3),[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,-9.81],[0,0,0])
        np.testing.assert_allclose(v,[0,0,9.81])


class IdentificationTests(RejectionTests):
    @classmethod
    def setUpClass(cls):
        cls.spec,cls.theta,cls.identity=fixture()
        cls.train=runs(cls.spec,cls.theta,cls.identity)
        cls.holdout=runs(cls.spec,cls.theta,cls.identity,seed=23,repetitions=2)
    def test_whole_run_leakage_rejected(self):
        self.reject(Reason.DATA_INVALID,check_split,self.train,self.train[:2])
    def test_renaming_training_data_does_not_create_holdout(self):
        renamed=copy.deepcopy(self.train[:2])
        for run in renamed:run.run_id="renamed-"+run.run_id
        self.reject(Reason.DATA_INVALID,check_split,self.train,renamed)
    def test_insufficient_information_rejected(self):
        self.reject(Reason.INSUFFICIENT_EXCITATION,bounded_initializer,np.ones((200,36)),np.ones(200))
    def test_posture_direction_parameter_columns_identifiable(self):
        X,y,_=integral_design(self.spec,self.train)
        fitted,condition=bounded_initializer(X,y)
        self.assertLess(condition,1e6);self.assertLess(np.max(np.abs(fitted[:6]-self.theta[:6])),.02)
    def test_timing_anomaly_rejected(self):
        run=copy.deepcopy(self.train[0]);run.t[20]=run.t[19]
        self.reject(Reason.DATA_INVALID,run.validate)
    def test_missing_samples_not_200hz(self):
        run=copy.deepcopy(self.train[0]);run.v_new[:]=False;run.v_new[::10]=True
        self.reject(Reason.MEASUREMENT_LIMITED,run.validate)
    def test_noise_above_target_cannot_pass(self):
        run=copy.deepcopy(self.train[0]);run.sigma_q=.01
        self.reject(Reason.MEASUREMENT_LIMITED,run.validate)
    def test_unsupported_resonance_is_model_inadequate(self):
        bad=runs(self.spec,self.theta,self.identity,repetitions=1,mismatch=True)
        report=validation_report(Native(),self.spec,self.theta,bad)
        self.assertTrue(all(not r["passed"] for r in report))


if __name__=="__main__":unittest.main()
