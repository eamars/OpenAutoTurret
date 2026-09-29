"""Directed total-current startup intervals; censoring never becomes a measured threshold."""
from __future__ import annotations
import numpy as np
from .contracts import Reason, require


def estimate_interval(time, successful_tx, q, velocity, *, direction, sigma_velocity,
                      encoder_quantum, sustained_s=.06):
    t,u,q,v=(np.asarray(x,float) for x in (time,successful_tx,q,velocity))
    require(direction in (-1,1) and t.ndim==1 and len(t)>=10 and
            t.shape==u.shape==q.shape==v.shape and
            all(np.isfinite(x).all() for x in (t,u,q,v)) and np.all(np.diff(t)>0),
            Reason.DATA_INVALID,"invalid directed breakaway raw trace")
    require(sigma_velocity>0 and encoder_quantum>=0 and sustained_s>0,
            Reason.MEASUREMENT_LIMITED,"startup needs noise/quantization and a positive sustained window")
    require(np.all(direction*np.diff(u)>=-1e-9),Reason.DATA_INVALID,
            "startup ramp must be monotonic in the tested direction")
    for first in range(len(t)-1):
        last=int(np.searchsorted(t,t[first]+sustained_s))
        if last>=len(t):break
        if (np.all(direction*v[first:last+1]>3*sigma_velocity) and
            direction*(q[last]-q[first])>max(encoder_quantum,3*sigma_velocity*sustained_s)):
            before=max(0,first-1)
            return {"not_moving_A":float(u[before]),"sustained_motion_A":float(u[last]),
                    "censored":False,"first_motion_s":float(t[first]),
                    "sustained_confirmed_s":float(t[last]),
                    "definition":"directed_total_current_not_increment"}
    return {"not_moving_A":float(u[-1]),"sustained_motion_A":None,"censored":True,
            "first_motion_s":None,"sustained_confirmed_s":None,
            "definition":"directed_total_current_not_increment"}


def assemble_intervals(spec,records):
    intervals=np.full((2,3,len(spec.q_nodes),2),np.nan)
    censored=np.ones(intervals.shape[:-1],bool)
    seen=set()
    for record in records:
        key=(record["direction"],record["posture_index"],record["position_index"])
        require(key not in seen and key[0] in (-1,1) and 0<=key[1]<3 and
                0<=key[2]<len(spec.q_nodes),Reason.DATA_INVALID,"duplicate/invalid breakaway cell")
        seen.add(key);di=0 if key[0]<0 else 1
        result=estimate_interval(**record["trace"],direction=key[0])
        intervals[di,key[1],key[2],0]=result["not_moving_A"]
        if not result["censored"]:intervals[di,key[1],key[2],1]=result["sustained_motion_A"]
        censored[di,key[1],key[2]]=result["censored"]
    require(len(seen)==6*len(spec.q_nodes),Reason.INSUFFICIENT_EXCITATION,
            "missing breakaway position/direction/posture coverage")
    return intervals,censored
