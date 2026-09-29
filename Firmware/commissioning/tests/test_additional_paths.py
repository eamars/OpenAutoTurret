from __future__ import annotations
from dataclasses import asdict,replace
import copy
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import numpy as np
from Firmware.commissioning.contracts import ModelSpec,Identity,digest,Rejected
from Firmware.commissioning.synthetic import fixture,runs
from Firmware.commissioning.identification import identify,observe_gyro
from Firmware.commissioning.native import Native
from Firmware.commissioning.calibrate import load_dataset
from Firmware.commissioning.measurement import kinematic_axes
from Firmware.commissioning.protocol import Ownership
from Firmware.commissioning.tests.test_measurement_pipeline import raw_fixture


class AdditionalPaths(unittest.TestCase):
    def test_pitch_gravity_huber_scaling_recovers_from_outlier_initialization(self):
        spec,truth,identity=fixture('pitch','PAYLOAD_UP')
        result=identify(Native(),spec,runs(spec,truth,identity,seed=2202),
                        runs(spec,truth,identity,seed=2203,repetitions=2),delay_bound_s=.025,bootstrap=False)
        self.assertLess(np.max(np.abs(result['theta'][:6]/truth[:6]-1)),.01)
        self.assertLess(abs(result['theta'][-1]-truth[-1]),.001)
    def test_eight_node_periodic_model_identifies_across_wrap(self):
        _,base,_=fixture()
        spec=ModelSpec('yaw',tuple(np.arange(8)*np.pi/4),(-.3,0.,.3),periodic=True)
        h=np.array([[[.12*d+.025*z+.03*np.sin(q) for q in spec.q_nodes]
                     for z in spec.z_nodes] for d in (-1,1)])
        truth=np.r_[base[:6],h.ravel(),.008]
        identity=Identity(digest('periodic-H'),digest('periodic-C'),digest(truth.tolist()),'SYNTHETIC')
        result=identify(Native(),spec,runs(spec,truth,identity,seed=81),
                        runs(spec,truth,identity,seed=82,repetitions=2),delay_bound_s=.025,bootstrap=False)
        self.assertLess(np.max(np.abs(result['theta'][:6]/truth[:6]-1)),.02)
        self.assertLess(np.max(np.abs(result['theta'][6:-1]-truth[6:-1])),.003)
    def test_filtered_gyro_is_fitted_with_its_causal_observation_model(self):
        spec,truth,identity=fixture()
        train=runs(spec,truth,identity,seed=131);hold=runs(spec,truth,identity,seed=132,repetitions=2)
        for row in train+hold:
            row.v=observe_gyro(row.v,row.t,.022);row.gyro_filter_tau_s=.022
        result=identify(Native(),spec,train,hold,delay_bound_s=.025,bootstrap=False)
        self.assertLess(np.max(np.abs(result['theta'][:6]/truth[:6]-1)),.02)
        self.assertLess(abs(result['theta'][-1]-truth[-1]),.001)
    def test_raw_bundle_flows_through_normalization_into_the_same_identifier(self):
        spec,truth,_=fixture();_,cal,_=raw_fixture()
        cal['encoder'].update(counts_per_motor_turn=1048576,physical_zero_rad=-1.)
        cal['sigma_q_rad']=2e-5;cal['sigma_v_rad_s']=8e-5
        cal['imu']['gyro_bias_rad_s']=[0.,0.,0.]
        for mapping in cal['clocks'].values():mapping['source_range_s']=[0.,8.]
        cal['identity_hash']=digest({k:v for k,v in cal.items() if k!='identity_hash'})
        identity=Identity(digest('raw-H'),cal['identity_hash'],digest('raw-O'),'SYNTHETIC')
        def raw(row):
            n=len(row.t)
            def stream():return {'sample_time_s':row.t.tolist(),'receive_time_s':(row.t+.001).tolist(),
                                'sequence':list(range(1,n+1)),'generation':[1]*n,'valid':[True]*n}
            rate=np.c_[row.v,np.zeros(n)]
            gyro=np.einsum('nij,nj->ni',kinematic_axes(row.z),rate)
            return {'axis':'yaw','identity':asdict(identity),'run_id':row.run_id,'mode':'current',
                    'readback_verified':True,'capture_ready':True,'posture_time_s':row.t.tolist(),
                    'other_axis_rad':row.z.tolist(),
                    'encoder':{**stream(),'raw_count':np.round((row.q+1)*1048576/(2*np.pi)).astype(int).tolist()},
                    'gyro':{**stream(),'pitch_rad':row.z.tolist(),'raw_rad_s':gyro.tolist()},
                    'current':{**stream(),'raw':(row.tx/.001).tolist()},
                    'tx':{**stream(),'requested_A':row.tx.tolist(),'limited_A':row.tx.tolist(),
                          'successful_A':row.tx.tolist(),'success':[True]*n,'owner_epoch':[1]*n,
                          'direction':row.direction.tolist()}}
        document={'model_spec':asdict(spec),'identity':asdict(identity),'calibration':cal,
                  'raw_train':[raw(row) for row in runs(spec,truth,identity,seed=301)],
                  'raw_holdout':[raw(row) for row in runs(spec,truth,identity,seed=302,repetitions=2)]}
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'bundle.json';path.write_text(json.dumps(document))
            _,loaded,actual,train,hold=load_dataset(path)
            result=identify(Native(),loaded,train,hold,delay_bound_s=.025,bootstrap=False)
        self.assertEqual(actual,identity)
        self.assertLess(np.max(np.abs(result['theta'][:6]/truth[:6]-1)),.02)
    def test_process_crash_releases_lock_and_new_epoch_invalidates_old_commands(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'owner'
            script=('import os,sys;from pathlib import Path;'
                    'from Firmware.commissioning.protocol import Ownership;'
                    'owner=Ownership(Path(sys.argv[1]),10);owner.acquire();os._exit(17)')
            result=subprocess.run([sys.executable,'-c',script,str(path)],timeout=20)
            self.assertEqual(result.returncode,17)
            owner=Ownership(path,10)
            try:
                self.assertEqual(owner.acquire(),2)
                with self.assertRaises(Rejected):owner.check(1)
            finally:owner.close()


if __name__=='__main__':unittest.main()
