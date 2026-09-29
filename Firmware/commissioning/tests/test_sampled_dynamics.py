from __future__ import annotations
import unittest
import numpy as np
from Firmware.commissioning.native import Native,Simulation,parameters
from Firmware.commissioning.probe_control import fixture_control
from Firmware.commissioning.synthesis import runtime_values,linear_check,linear_system
from Firmware.commissioning.sampled_analysis import lifted_loop,sampled_margins
from Firmware.commissioning.metrics import shaped_velocity


class SampledDynamicsTests(unittest.TestCase):
    def setUp(self):
        self.s,self.o,self.e,self.sim=fixture_control();self.native=Native()
        self.values=runtime_values(self.s.theta,10.,self.e,self.o,.005)
        self.params=parameters(self.s.spec,self.s.theta,self.o,self.values,
                               self.s.start_intervals[...,1],self.s.start_censored)
    def test_uniform_lift_reproduces_discrete_core_matrix(self):
        original,_=linear_system(.1,.06,.04,self.values,self.o,.005,.008)
        parts=lifted_loop(.1,.06,.04,.04,self.values,self.o,self.sim,.008)
        a=np.sort(np.abs(np.linalg.eigvals(original)))
        b=np.sort(np.abs(np.linalg.eigvals(parts[4])))
        np.testing.assert_allclose(a[-5:],b[-5:],rtol=1e-8,atol=1e-9)
    def test_filtered_asynchronous_measurement_has_explicit_poles_and_margins(self):
        sim=Simulation(.005,1e-5,2e-5,8e-5,.005,.01,2,3,92)
        values=runtime_values(self.s.theta,6.,self.e,self.o,.005)
        result=linear_check(self.s,self.o,values,.005,self.s.theta,0.,0.,1,sim)
        self.assertEqual(result["sampling_superperiod"],6)
        self.assertLess(result["spectral_radius"],1.)
        self.assertTrue(np.isfinite(result["phase_margin_deg"]))
        params=parameters(self.s.spec,self.s.theta,self.o,values,self.s.start_intervals[...,1],self.s.start_censored)
        t,refs,_=shaped_velocity(np.deg2rad(5),self.e)
        trace=self.native.closed_rollout(params,params,sim,refs,(0.,0.))
        self.assertTrue(np.isfinite(trace).all());self.assertFalse(np.any(trace[:,10]))
    def test_large_delay_invalidates_the_same_high_bandwidth(self):
        good=linear_check(self.s,self.o,self.values,.005,self.s.theta,0.,0.,1)
        theta=self.s.theta.copy();theta[-1]=.2
        bad=linear_check(self.s,self.o,self.values,.005,theta,0.,0.,1)
        self.assertLess(good["spectral_radius"],1.);self.assertFalse(bad["passed"])
    def test_reversal_brakes_before_changing_direction_and_finishes(self):
        dt=.005;t=np.arange(dt,10+dt/2,dt);speed=np.deg2rad(5)
        v=np.zeros(len(t));a=np.zeros(len(t));ramp=.2
        for begin,start,end in ((2.,0.,speed),(5.,speed,-speed),(8.,-speed,0.)):
            active=(t>=begin)&(t<begin+ramp);phase=(t[active]-begin)/ramp
            v[t>=begin+ramp]=end
            v[active]=start+(end-start)*(1-np.cos(np.pi*phase))/2
            a[active]=(end-start)*np.pi*np.sin(np.pi*phase)/(2*ramp)
        q=np.cumsum(v)*dt;refs=np.c_[q,v,a,np.zeros(len(t))]
        trace=self.native.closed_rollout(self.params,self.params,self.sim,refs,(0.,0.))
        self.assertIn(4,trace[:,9]);self.assertIn(2,trace[:,9])
        self.assertLess(abs(trace[-1,0]-q[-1]),np.deg2rad(.15))
        self.assertLess(abs(trace[-1,1]),np.deg2rad(.1))
        self.assertLessEqual(np.max(np.abs(trace[:,5])),self.e.current_a)
    def test_posture_motion_and_static_gravity_are_distinct_from_zero_current(self):
        s,o,e,sim=fixture_control("pitch")
        values=runtime_values(s.theta,10.,e,o,.005)
        params=parameters(s.spec,s.theta,o,values,s.start_intervals[...,1],s.start_censored)
        t,refs,_=shaped_velocity(np.deg2rad(3),e)
        refs[:,3]=.2*np.sin(t*.4)
        trace=self.native.closed_rollout(params,params,sim,refs,(0.,0.))
        self.assertFalse(np.any(trace[:,10]))
        self.assertGreater(abs(trace[-1,5]),.05)
        self.assertLess(abs(trace[-1,0]-refs[-1,0]),np.deg2rad(.15))


if __name__=="__main__":unittest.main()
