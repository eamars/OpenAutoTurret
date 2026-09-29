from __future__ import annotations
import copy
from dataclasses import replace
import io
from pathlib import Path
import os
import subprocess
import tempfile
import unittest
import numpy as np

from Firmware.commissioning.contracts import Reason,Rejected,digest
from Firmware.commissioning.native import *
from Firmware.commissioning.probe_control import fixture_control
from Firmware.commissioning.synthesis import runtime_values
from Firmware.commissioning.metrics import shaped_velocity,shaped_step,motion_metrics
from Firmware.commissioning.parameter_catalog import core_registry,bind_core_profile
from Firmware.commissioning.protocol import Ownership,SimulatedEndpoint
from Firmware.commissioning.replay import write_replay
from Firmware.commissioning.adaptation import FailurePolicy,ChangeMonitor,reuse,template,first_stimulus
from Firmware.commissioning.breakaway import estimate_interval


class CoreTests(unittest.TestCase):
    def setUp(self):
        self.native=Native();self.s,self.o,self.e,self.sim=fixture_control()
        self.values=runtime_values(self.s.theta,12.,self.e,self.o,.005)
        self.p=parameters(self.s.spec,self.s.theta,self.o,self.values,self.s.start_intervals[...,1],self.s.start_censored)
    def observation(self,t,seq,**values):
        return CObservation(**{"now":t,"encoder_time":t,"gyro_time":t,"position":0.,"gyro_rate":0.,
            "encoder_seq":seq,"gyro_seq":seq,"generation":1,"encoder_valid":1,"gyro_valid":1,**values})
    def test_bidirectional_velocity_and_step_quality(self):
        for case in [shaped_velocity(np.deg2rad(v),self.e) for v in (3,5,10,-3,-5,-10)]+[
            shaped_step(np.deg2rad(q),self.e) for q in (.5,1,5,-.5,-1,-5)]:
            t,refs,kw=case
            trace=self.native.closed_rollout(self.p,self.p,self.sim,refs,(0.,0.))
            self.assertTrue(motion_metrics(t,refs,trace,**kw)["passed"])
    def test_failed_tx_does_not_update_applied_history(self):
        with Controller(self.native,self.p) as c:
            c.reset(0,0,0,.12);out=c.step(self.observation(.005,1),CReference(0,0,0,0))
            self.assertFalse(c.ack(out,successful=False));self.assertNotEqual(c.step(self.observation(.01,2),CReference()).status,0)
    def test_missing_ack_blocks_another_output(self):
        with Controller(self.native,self.p) as c:
            c.reset(0,0,0,.12);out=c.step(self.observation(.005,1),CReference())
            self.assertEqual(out.status,0)
            self.assertNotEqual(c.step(self.observation(.01,2),CReference()).status,0)
    def test_dt_fault_and_generation_change_require_reinitialization(self):
        for observation in (self.observation(.2,1),self.observation(.005,1,generation=2)):
            with Controller(self.native,self.p) as c:
                c.reset(0,0,0,.12);self.assertNotEqual(c.step(observation,CReference()).status,0)
    def test_new_sequence_without_new_timestamp_rejected(self):
        with Controller(self.native,self.p) as c:
            c.reset(0,0,0,.12);out=c.step(self.observation(.005,1),CReference());c.ack(out)
            out=c.step(self.observation(.01,2,encoder_time=.005),CReference())
            self.assertNotEqual(out.status,0)
    def test_final_current_and_slew_arbitration(self):
        with Controller(self.native,self.p) as c:
            c.reset(0,0,0,.12);previous=.12
            for k in range(1,60):
                out=c.step(self.observation(k*.005,k),CReference(.2,1.,20.,0.))
                self.assertLessEqual(abs(out.limited),self.e.current_a)
                self.assertLessEqual(abs(out.limited-previous),self.e.slew_a_s*.005+1e-12)
                c.ack(out);previous=out.limited
    def test_bumpless_profile_change_retains_support_current(self):
        with Controller(self.native,self.p) as c:
            c.reset(0,0,0,.12);out=c.step(self.observation(.005,1),CReference());c.ack(out)
            new=parameters(self.s.spec,self.s.theta,self.o,{**self.values,"kp":self.values["kp"]*1.5},
                           self.s.start_intervals[...,1],self.s.start_censored)
            self.assertTrue(c.switch(new,CReference()))
            changed=c.step(self.observation(.01,2),CReference())
            self.assertAlmostEqual(changed.limited,out.limited,places=12)
            self.assertNotEqual(changed.limited,0.)
    def test_unqualified_encoder_fallback_is_rejected(self):
        with Controller(self.native,self.p) as c:
            c.reset(0,0,0,.12)
            for k in range(1,9):
                out=c.step(self.observation(k*.005,k,gyro_valid=0),CReference())
                if out.status:break
                c.ack(out)
            self.assertNotEqual(out.status,0)
    def test_verified_encoder_fallback_remains_explicit(self):
        p=parameters(self.s.spec,self.s.theta,replace(self.o,encoder_only_verified=True),self.values,
                     self.s.start_intervals[...,1],self.s.start_censored)
        with Controller(self.native,p) as c:
            c.reset(0,0,0,.12)
            for k in range(1,9):
                out=c.step(self.observation(k*.005,k,gyro_valid=0),CReference());self.assertEqual(out.status,0);c.ack(out)
            self.assertEqual(out.encoder_only,1)
    def test_real_executables_and_library_replay_match(self):
        directory=Path(os.environ["OTA_STAGE1_BUILD"])
        binaries=[directory/"axis_control_core/commissiond",directory/"control/controld"]
        t,refs,_=shaped_velocity(np.deg2rad(5),self.e)
        expected=self.native.closed_rollout(self.p,self.p,self.sim,refs,(0.,0.))
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/"replay.txt";write_replay(path,self.p,self.p,self.sim,refs)
            outputs=[]
            for binary in binaries:
                run=subprocess.run([str(binary),"--axis-core-replay",str(path)],capture_output=True,text=True,timeout=15,check=True)
                self.assertIn("execution=SYNTHETIC",run.stdout.splitlines()[0])
                actual=np.loadtxt(io.StringIO(run.stdout),delimiter=",")
                np.testing.assert_array_equal(actual,expected);outputs.append(run.stdout)
            self.assertEqual(outputs[0],outputs[1])


class ProtocolTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.clock=[0.]
        self.owner=Ownership(Path(self.tmp.name)/"owner",1.,lambda:self.clock[0]);self.epoch=self.owner.acquire()
        s,o,e,_=fixture_control();native=Native();values=runtime_values(s.theta,12,e,o,.005)
        fields,self.current=core_registry(s,o,values)
        self.endpoint=SimulatedEndpoint(self.owner,fields,self.current,digest("test-spec"),
                                       validator=lambda current:bind_core_profile(native,s,o,current))
    def tearDown(self):self.owner.close();self.tmp.cleanup()
    def ready(self):
        p=self.endpoint;h=p.prepare_profile(self.current,epoch=self.epoch);p.apply_profile(epoch=self.epoch)
        r=p.verify_profile(h,epoch=self.epoch);p.prepare_capture(5);return h,r
    def test_transaction_and_capture(self):
        h,r=self.ready();p=self.endpoint;p.run_case("a",h,r["revision"],p.test_spec_hash,epoch=self.epoch)
        p.sample({"requested_A":.1,"limited_A":.1,"successful_A":.1},epoch=self.epoch)
        self.assertTrue(p.finish_case(epoch=self.epoch)["valid"])
    def test_refused_write_never_runs(self):
        p=self.endpoint;h=p.prepare_profile(self.current,epoch=self.epoch);p.failed_writes.add("controller.kp")
        with self.assertRaises(Rejected):p.apply_profile(epoch=self.epoch)
        with self.assertRaises(Rejected):p.run_case("a",h,0,p.test_spec_hash,epoch=self.epoch)
        self.assertEqual(p.run_count,0)
    def test_readback_mismatch_never_runs(self):
        p=self.endpoint;h=p.prepare_profile(self.current,epoch=self.epoch);p.apply_profile(epoch=self.epoch)
        p.readback_overrides["controller.kp"]=123.
        with self.assertRaises(Rejected):p.verify_profile(h,epoch=self.epoch)
        self.assertEqual(p.run_count,0)
    def test_invalid_core_value_rejected_before_apply(self):
        with self.assertRaises(Rejected):self.endpoint.prepare_profile({**self.current,"controller.kp":-1},epoch=self.epoch)
    def test_external_cap_is_read_only(self):
        with self.assertRaises(Rejected):self.endpoint.prepare_profile({**self.current,"controller.current_cap":9},epoch=self.epoch)
    def test_second_owner_cannot_acquire_os_lock(self):
        second=Ownership(self.owner.path,1.)
        with self.assertRaises(Rejected):second.acquire()
    def test_epoch_changes_and_queued_commands_expire(self):
        old=self.epoch;self.owner.close();new=self.owner.acquire();self.assertGreater(new,old)
        with self.assertRaises(Rejected):self.owner.check(old)
    def test_lease_timeout_blocks_output(self):
        self.clock[0]=2.
        with self.assertRaises(Rejected):self.owner.check(self.epoch)
    def test_capture_overflow_invalidates_case(self):
        h,r=self.ready();p=self.endpoint;p.run_case("a",h,r["revision"],p.test_spec_hash,epoch=self.epoch)
        for _ in range(5):p.sample({},epoch=self.epoch)
        with self.assertRaises(Rejected):p.sample({},epoch=self.epoch)
        self.assertFalse(p.finish_case(epoch=self.epoch)["valid"])


class AdaptationTests(unittest.TestCase):
    def test_fixed_failure_budgets(self):
        p=FailurePolicy()
        self.assertEqual(p.handle(Reason.DATA_INVALID,"a"),"RETRY_SAME_CASE_ONCE")
        self.assertEqual(p.handle(Reason.DATA_INVALID,"a"),"STOP_REPAIR_ACQUISITION")
        for _ in range(2):self.assertEqual(p.handle(Reason.INSUFFICIENT_EXCITATION),"SELECT_AT_MOST_THREE_INFORMATION_CASES")
        self.assertEqual(p.handle(Reason.INSUFFICIENT_EXCITATION),"STOP_INFORMATION_BUDGET_EXHAUSTED")
        self.assertEqual(p.handle(Reason.HARD_ABORT),"STOP_CAMPAIGN_NO_AUTORETRY")
        with self.assertRaises(Rejected):p.handle(Reason.DATA_INVALID)
    def test_all_other_failures_have_a_fixed_branch(self):
        for reason in Reason:self.assertIsInstance(FailurePolicy().handle(reason,"a"),str)
    def test_three_valid_residual_windows_trigger_without_retuning(self):
        monitor=ChangeMonitor();events=[]
        for k in range(3):events.append(monitor.window(start=k*2,end=k*2+2,
            normalized_residual=np.full(100,2.1),actual_samples=100,effective_hz=50))
        self.assertEqual(events,["MONITOR_ONLY","MONITOR_ONLY","CHANGE_DETECTED"])
    def test_invalid_window_cannot_build_change_evidence(self):
        monitor=ChangeMonitor()
        for k in range(4):self.assertEqual(monitor.window(start=k*2,end=k*2+2,normalized_residual=[9]*100,
            actual_samples=100,effective_hz=50,protection_intervened=True),"INVALID_WINDOW")
    def test_all_32_templates_are_bounded_and_deterministic(self):
        for k in range(32):
            t,u=template(k,4,.005);np.testing.assert_array_equal(template(k,4,.005)[1],u)
            self.assertLessEqual(np.max(np.abs(u)),1.);self.assertAlmostEqual(u[0],0.);self.assertAlmostEqual(u[-1],0.)
    def test_censored_start_has_unknown_upper_endpoint(self):
        t=np.arange(0,1,.005)
        result=estimate_interval(t,t*.1,np.zeros(len(t)),np.zeros(len(t)),direction=1,
                                 sigma_velocity=.001,encoder_quantum=.0001)
        self.assertTrue(result["censored"]);self.assertIsNone(result["sustained_motion_A"])
    def test_three_counts_do_not_prove_sustained_motion(self):
        t=np.arange(0,1,.005);q=np.zeros(len(t));q[40:43]=.0003;v=np.zeros(len(t));v[40:43]=.1
        self.assertTrue(estimate_interval(t,t*.1,q,v,direction=1,sigma_velocity=.001,
                                          encoder_quantum=.0001)["censored"])


if __name__=="__main__":unittest.main()
