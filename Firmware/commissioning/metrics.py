"""Frozen offline quality evaluation; no adjustment from observed failures."""
from __future__ import annotations
import numpy as np
from scipy.signal import butter, sosfilt
from .contracts import Reason, require


def motion_metrics(t, references, trace, *, zero_reference_time, command_time, step_rad=None,
                   gyro_bandwidth_hz=20., position_jitter_limit_rad=None):
    t=np.asarray(t);references=np.asarray(references);trace=np.asarray(trace)
    require(trace.shape==(len(t),12) and references.shape==(len(t),4) and
            np.isfinite(trace).all() and np.all(np.diff(t)>0), Reason.DATA_INVALID,
            "invalid full trace for evaluation")
    position_jitter_limit = (np.deg2rad(.15) if position_jitter_limit_rad is None
                             else position_jitter_limit_rad)
    require(isinstance(position_jitter_limit, (int, float)) and not isinstance(position_jitter_limit, bool)
            and np.isfinite(position_jitter_limit) and position_jitter_limit > 0,
            Reason.DATA_INVALID, "finite positive declared position-jitter quality limit required")
    q,v=trace[:,0],trace[:,1];rq,rv=references[:,0],references[:,1]
    dt=float(np.median(np.diff(t)))
    steady=(np.abs(references[:,2])<1e-9)&(np.abs(rv)>np.deg2rad(.1))&(t>=command_time)
    steady &= t < zero_reference_time-.05
    require(np.count_nonzero(steady)*dt>=2 or step_rad is not None,
            Reason.DATA_INVALID, "steady reference must reach a plateau for at least two seconds")
    speed_limit=max(np.deg2rad(.5),float(np.max(np.abs(rv)))*.1)
    result={}
    passed=True
    if steady.any() and step_rad is None:
        ratio=float(np.mean(v[steady])/np.mean(rv[steady]))
        # 200ms local linear position estimate, separate from gyro-band vibration.
        count=max(3,round(.2/dt));half=count//2
        centered=np.arange(-half,half+1)*dt
        local=np.convolve(q,centered[::-1]/np.dot(centered,centered),mode="same")
        require(not steady[:half].any() and not steady[-half:].any(),Reason.DATA_INVALID,
                "fixed local velocity windows need capture before and after the plateau")
        speed_error=float(np.sqrt(np.mean((local[steady]-rv[steady])**2)))
        jitter=float(np.std(local[steady]-rv[steady]))
        residual=q[steady]-rq[steady]
        design=np.c_[t[steady],np.ones(steady.sum())]
        residual-=design@np.linalg.lstsq(design,residual,rcond=None)[0]
        p95=float(np.quantile(residual,.95)-np.quantile(residual,.05))
        span=float(np.ptp(residual))
        active=float(np.mean(np.sign(rv[steady])*v[steady]>.05*np.abs(rv[steady])))
        high=min(20.,1/dt/5,gyro_bandwidth_hz)
        require(np.isfinite(high) and high>.5,Reason.MEASUREMENT_LIMITED,"no calibrated gyro quality band")
        band=butter(3,[.5,high],btype="bandpass",fs=1/dt,output="sos")
        vibration=sosfilt(band,v-rv)
        gyro_rms=float(np.sqrt(np.mean(vibration[steady]**2)))
        passed &= (.9<=ratio<=1.1 and active>=.95 and speed_error<=speed_limit and jitter<=speed_limit
                   and p95<=position_jitter_limit and span<=np.deg2rad(.3) and gyro_rms<=speed_limit)
        result.update(tracking_ratio=ratio,active_fraction=active,speed_rms_rad_s=speed_error,
                      speed_jitter_rad_s=jitter,position_jitter_rad=p95,position_range_rad=span,
                      gyro_band_rms_rad_s=gyro_rms,gyro_band_hz=[.5,high])
    stop=(t>=zero_reference_time)&(t<=zero_reference_time+2+dt/2)
    require(np.count_nonzero(stop)*dt>=2-dt,Reason.DATA_INVALID,"complete fixed two-second stop window required")
    stop_q=q[np.flatnonzero(stop)[0]]
    drift=float(np.max(np.abs(q[stop]-stop_q)))
    passed &= drift<=np.deg2rad(.15)
    result["stop_drift_rad"]=drift
    if np.max(np.abs(rv))>=np.deg2rad(5)-1e-8:
        # Continuous motion evidence, not three encoder counts.
        wanted=np.sign(rv);good=(wanted*v>np.deg2rad(.1))&(t>=command_time)
        sustained=max(2,round(.06/dt));first=None
        starts=np.flatnonzero(np.convolve(good.astype(int),np.ones(sustained,dtype=int),"valid")==sustained)
        if len(starts):first=t[starts[0]+sustained-1]
        latency=float(first-command_time) if first is not None else float(t[-1])
        result["sustained_start_s"]=latency;passed &= latency<=.2
    if step_rad is not None:
        after=t>=command_time
        target=float(rq[-1]);error=np.abs(q-target)
        final=float(np.max(error[t>=zero_reference_time+1]))
        sign=1 if step_rad>=0 else -1
        overshoot=float(max(0.,np.max(sign*(q[after]-target))))
        deadline=command_time+1 if abs(step_rad)<=np.deg2rad(1)+1e-9 else zero_reference_time+1
        settled=bool(np.all(error[t>=deadline]<=np.deg2rad(.15)))
        passed &= final<=np.deg2rad(.15) and overshoot<=max(np.deg2rad(.15),abs(step_rad)*.1) and settled
        result.update(step_error_rad=final,overshoot_rad=overshoot,settled_before_deadline=settled)
    result["output_limited_fraction"]=float(np.mean(np.abs(trace[:,4]-trace[:,5])>1e-9))
    result["passed"]=bool(passed and not np.any(trace[:,10]))
    return result


def shaped_velocity(speed, envelope, dt=.005, *, position=0., posture=0.):
    # A raised-cosine velocity transition has bounded acceleration and jerk.
    ramp=max(abs(speed)*np.pi/(2*envelope.acceleration_rad_s2),
             np.sqrt(abs(speed)*np.pi**2/(2*envelope.jerk_rad_s3)),dt*2)
    ramp=np.ceil(ramp/dt)*dt
    start=2.;end=start+2*ramp+2.5
    t=np.arange(dt,end+2+dt/2,dt)
    v=np.zeros(len(t));acc=np.zeros(len(t))
    up=(t>=start)&(t<start+ramp);flat=(t>=start+ramp)&(t<end-ramp);down=(t>=end-ramp)&(t<end)
    for mask,offset,sgn in ((up,start,1),(down,end-ramp,-1)):
        phase=(t[mask]-offset)/ramp
        v[mask]=speed*(1-sgn*np.cos(np.pi*phase))/2
        acc[mask]=sgn*speed*np.pi*np.sin(np.pi*phase)/(2*ramp)
    v[flat]=speed
    q=position+np.cumsum(v)*dt
    refs=np.c_[q,v,acc,np.full(len(t),posture)]
    if envelope.angle_min_rad is not None:
        require(q.min()>=envelope.angle_min_rad and q.max()<=envelope.angle_max_rad,
                Reason.ENVELOPE_LIMITED,"shaped velocity trajectory leaves angle envelope")
    return t,refs,{"zero_reference_time":float(end),"command_time":start}


def shaped_step(distance,envelope,dt=.005,*,position=0.,posture=0.):
    # Quintic smoothstep derivatives have fixed maxima; deadlines freeze before simulation.
    duration=max(1.875*abs(distance)/envelope.velocity_rad_s,
                 np.sqrt(5.774*abs(distance)/envelope.acceleration_rad_s2),
                 (60*abs(distance)/envelope.jerk_rad_s3)**(1/3),2*dt)
    duration=np.ceil(duration/dt)*dt;start=2.;stop=start+duration
    t=np.arange(dt,stop+3+dt/2,dt);s=np.clip((t-start)/duration,0,1)
    q=position+distance*(10*s**3-15*s**4+6*s**5)
    v=distance*(30*s**2-60*s**3+30*s**4)/duration
    a=distance*(60*s-180*s**2+120*s**3)/duration**2
    if envelope.angle_min_rad is not None:
        require(q.min()>=envelope.angle_min_rad and q.max()<=envelope.angle_max_rad,
                Reason.ENVELOPE_LIMITED,"position trajectory leaves angle envelope")
    return t,np.c_[q,v,a,np.full(len(t),posture)],{
        "zero_reference_time":float(stop),"command_time":start,"step_rad":distance}
