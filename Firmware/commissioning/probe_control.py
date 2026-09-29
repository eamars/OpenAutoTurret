"""Small executable C++ control-boundary probe, explicitly synthetic."""
import json
import os
import numpy as np
from .synthetic import fixture
from .measurement import ObserverSpec
from .adaptation import Envelope
from .contracts import PlantSnapshot
from .native import Native, Simulation, parameters
from .synthesis import runtime_values, linear_check
from .metrics import shaped_velocity, shaped_step, motion_metrics


def fixture_control(axis="yaw"):
    spec,theta,identity=fixture(axis)
    observer=ObserverSpec(4e-10,6.4e-9,.1,.03,.03,4e-10,6.4e-9,False,
                          identity.measurement,"SYNTHETIC")
    envelope=Envelope(2.,30.,.6,2.,30.,-1.,1.,12.,True,"SYNTHETIC")
    starts=theta[6:-1].reshape(2,3,5).copy()
    starts[0]-=.015;starts[1]+=.015
    intervals=np.stack([starts-np.array([-1,1])[:,None,None]*.005,starts],axis=-1)
    snapshot=PlantSnapshot(spec,identity,theta,np.tile(theta,(128,1)),("a"*64,), ("b"*64,),
                           (.5,15.),intervals,np.zeros((2,3,5),bool),{"synthetic_oracle":True})
    return snapshot,observer,envelope,Simulation(.005,0.,0.,0.,0.,0.,1,1,22)


def main():
    native=Native();snap,observer,envelope,simulation=fixture_control()
    values=runtime_values(snap.theta,float(os.environ.get("PROBE_WN","12")),envelope,observer,.005)
    params=parameters(snap.spec,snap.theta,observer,values,snap.start_intervals[...,1],snap.start_censored)
    reports=[]
    for case in [shaped_velocity(np.deg2rad(v),envelope) for v in (3,5,10,-3,-5,-10)]+[
            shaped_step(np.deg2rad(q),envelope) for q in (.5,1,5,-.5,-1,-5)]:
        t,refs,kwargs=case
        out=native.closed_rollout(params,params,simulation,refs,(0.,0.))
        if kwargs.get("step_rad") is None and refs[:,1].min()<0:
            np.savetxt("run/adr0022-local/negative-control.csv",np.c_[t,refs,out],delimiter=",")
        reports.append(motion_metrics(t,refs,out,**kwargs))
    print(json.dumps({"linear":linear_check(snap,observer,values,.005,snap.theta,0.,0.,1),
                      "cases":reports},indent=2))


if __name__=="__main__":main()
