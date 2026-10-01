"""Frozen failure budgets, identity reuse, change detection and signal selection."""
from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from .contracts import Reason, Rejected, require
from .model import coefficients, features


@dataclass(frozen=True)
class Envelope:
    current_a: float
    slew_a_s: float
    velocity_rad_s: float | None
    acceleration_rad_s2: float | None
    jerk_rad_s3: float | None
    angle_min_rad: float | None
    angle_max_rad: float | None
    duration_s: float
    stop_verified: bool
    provenance: str

    def __post_init__(self):
        required = (self.current_a, self.slew_a_s, self.duration_s)
        require(np.isfinite(required).all() and min(required)>0, Reason.DATA_INVALID,
                "positive finite current, slew and duration bounds required")
        for bound in (self.velocity_rad_s, self.acceleration_rad_s2, self.jerk_rad_s3):
            require(bound is None or (np.isfinite(bound) and bound>0), Reason.DATA_INVALID,
                    "declared kinematic bounds must be positive and finite")
        require((self.angle_min_rad is None)==(self.angle_max_rad is None), Reason.DATA_INVALID,
                "angle bounds must be declared together")
        if self.angle_min_rad is not None:
            require(np.isfinite((self.angle_min_rad,self.angle_max_rad)).all() and
                    self.angle_min_rad<self.angle_max_rad, Reason.DATA_INVALID,
                    "declared angle bounds must be finite and ordered")
        require(type(self.stop_verified) is bool and self.provenance in ("SYNTHETIC", "MEASURED"),
                Reason.DATA_INVALID, "envelope provenance missing")


class FailurePolicy:
    def __init__(self): self.retries={};self.information_rounds=0;self.aborted=False
    def handle(self, reason: Reason, case_id=""):
        require(not self.aborted, Reason.HARD_ABORT, "campaign already aborted")
        if reason == Reason.HARD_ABORT:
            self.aborted=True;return "STOP_CAMPAIGN_NO_AUTORETRY"
        if reason == Reason.DATA_INVALID:
            count=self.retries.get(case_id,0)
            if count: return "STOP_REPAIR_ACQUISITION"
            self.retries[case_id]=1;return "RETRY_SAME_CASE_ONCE"
        if reason == Reason.INSUFFICIENT_EXCITATION:
            if self.information_rounds>=2:return "STOP_INFORMATION_BUDGET_EXHAUSTED"
            self.information_rounds+=1;return "SELECT_AT_MOST_THREE_INFORMATION_CASES"
        return {Reason.MEASUREMENT_LIMITED:"STOP_REPAIR_MEASUREMENT",
                Reason.OPERATING_POINT_CHANGED:"SEGMENT_AND_UPDATE_PARAMETERS",
                Reason.MODEL_INADEQUATE:"STOP_PROMOTION_MODEL_INADEQUATE",
                Reason.ENVELOPE_LIMITED:"STOP_QUALITY_KEEP_APPROVED_BOUNDARIES",
                Reason.INTEGRATION_MISMATCH:"STOP_PROMOTION_REPAIR_INTEGRATION"}[reason]


class ChangeMonitor:
    def __init__(self): self.count=0;self.last_end=None
    def window(self, *, start, end, normalized_residual, actual_samples, effective_hz,
               valid=True, reference_changed=False, protection_intervened=False, declared=False):
        if declared:
            self.count=0;return "OPERATING_POINT_CHANGED"
        require(end>start and (self.last_end is None or start>=self.last_end), Reason.DATA_INVALID,
                "monitor windows must be nonoverlapping and monotonic")
        self.last_end=end
        usable=(valid and not reference_changed and not protection_intervened and end-start>=2 and
                effective_hz>=2.5 and actual_samples>=2*effective_hz)
        if not usable:
            self.count=0;return "INVALID_WINDOW"
        values=np.asarray(normalized_residual,float)
        require(np.isfinite(values).all() and len(values)>=actual_samples, Reason.DATA_INVALID,
                "monitor requires actual finite samples")
        self.count=self.count+1 if np.sqrt(np.mean(values**2))>2 else 0
        return "CHANGE_DETECTED" if self.count>=3 else "MONITOR_ONLY"


def reuse(previous_identity, current_identity, *, covered, prediction_passed):
    if previous_identity.hardware!=current_identity.hardware or previous_identity.measurement!=current_identity.measurement:
        return "INVALIDATE_AFFECTED_ASSETS"
    if covered and prediction_passed:return "REUSE_EXACT"
    return "UPDATE_PARAMETERS"


def template(case_id, duration, dt):
    require(type(case_id) is int and 0<=case_id<32, Reason.DATA_INVALID, "fixed template ID 0..31 required")
    t=np.arange(0,duration+dt/2,dt);phase=t/duration
    family, scale=divmod(case_id,8)
    cycles=1+scale%4
    window=np.sin(np.pi*phase)**2
    if family==0:u=window*(2*phase-1)
    elif family==1:u=window*(np.sin(2*np.pi*cycles*phase)+.4*np.sin(2*np.pi*(cycles+1)*phase))/1.4
    elif family==2:u=window*np.sin(2*np.pi*(phase+cycles*phase**2/2))
    else:u=window*np.tanh(8*np.sin(2*np.pi*cycles*phase))
    return t,u*(.25+.75*(scale//4))


def choose_information(native, spec, theta, uncertainty, envelope, prior_information,
                       *, initial_position, posture, direction, noise_sigma, sample_hz,
                       _all_templates=False, maximum_first_hold_s=None):
    require(sample_hz>0 and noise_sigma>0, Reason.MEASUREMENT_LIMITED, "sampling/noise not calibrated")
    if maximum_first_hold_s is not None:
        require(np.isfinite(maximum_first_hold_s) and maximum_first_hold_s>0,
                Reason.DATA_INVALID, "maximum-first hold duration must be positive and finite")
        maximum_first_current=direction*float(first_stimulus(None,envelope,
            previous_factor=0,maximum_first=True)[0])
    p=spec.size;prior=np.asarray(prior_information,float)
    require(prior.shape==(p,p) and np.isfinite(prior).all() and np.allclose(prior,prior.T),
            Reason.DATA_INVALID, "Fisher prior must be finite and symmetric")
    eigenvalues=np.linalg.eigvalsh(prior)
    require(eigenvalues.min()>=-1e-10*max(1.,float(np.max(np.abs(eigenvalues)))),
            Reason.DATA_INVALID, "Fisher prior must be positive semidefinite")
    scores=[]
    for case_id in range(32):
        t,shape=template(case_id,min(envelope.duration_s,4.),1/sample_hz)
        _,_,load=coefficients(spec,theta,initial_position,posture,direction)
        amplitude=max(0.,envelope.current_a-abs(float(load)))
        u=float(load)+direction*amplitude*shape
        template_duration=float(t[-1]);prefix=None;final_zero_ramp_s=0.
        if maximum_first_hold_s is not None:
            times=[0.];currents=[0.]
            def append_linear(target,duration):
                if duration<=0:return
                offsets=np.linspace(0.,duration,max(1,int(np.ceil(duration*sample_hz)))+1)[1:]
                start=times[-1];current=currents[-1]
                times.extend(start+offsets)
                currents.extend(current+(target-current)*offsets/duration)
            ramp_s=abs(maximum_first_current)/envelope.slew_a_s
            transition_s=abs(float(u[0])-maximum_first_current)/envelope.slew_a_s
            append_linear(maximum_first_current,ramp_s)
            append_linear(maximum_first_current,maximum_first_hold_s)
            append_linear(float(u[0]),transition_s)
            prefix_duration=times[-1]
            prefix={"method":"first_stimulus","maximum_first":True,"factor_index":0,
                    "current_A":maximum_first_current,"slew_A_s":envelope.slew_a_s,
                    "ramp_s":ramp_s,"hold_s":float(maximum_first_hold_s),
                    "transition_to_template_s":transition_s,"duration_s":prefix_duration,
                    "startup_or_motion_guaranteed":False}
            times.extend(prefix_duration+t[1:]);currents.extend(u[1:])
            final_zero_ramp_s=abs(currents[-1])/envelope.slew_a_s
            append_linear(0.,final_zero_ramp_s)
            t=np.asarray(times,float);u=np.asarray(currents,float)
            if t[-1]>envelope.duration_s:continue
        if np.max(np.abs(u))>envelope.current_a:continue
        intervals=np.diff(t);increments=np.abs(np.diff(u))
        if maximum_first_hold_s is None:
            if np.max(increments/intervals)>envelope.slew_a_s:continue
        else:
            slew_increment=envelope.slew_a_s*intervals
            # The exact ramp acquires rounding when its local times are added
            # to the prefix. Propagate one representable step of each input
            # and product; this is numerical resolution, not extra slew authority.
            roundoff=(np.spacing(np.abs(u[:-1]))+np.spacing(np.abs(u[1:]))+
                      envelope.slew_a_s*(np.spacing(t[:-1])+np.spacing(t[1:]))+
                      np.spacing(slew_increment))
            if np.any(increments>slew_increment+roundoff):continue
        z=np.full(len(t),posture);dirs=np.full(len(t),direction)
        try:out=native.rollout(spec,theta,t,u,z,dirs,(initial_position,0.))
        except Rejected:continue
        worst_a=float(np.max(np.asarray(uncertainty)[:,:3]))
        worst_load=float(np.max(np.abs(np.asarray(uncertainty)[:,6:-1])))
        brake=envelope.current_a-worst_load
        if brake<=0:continue
        delay=float(np.max(np.asarray(uncertainty)[:,-1]))+2*envelope.current_a/envelope.slew_a_s
        stop=np.abs(out[:,1])*delay+out[:,1]**2*worst_a/(2*brake)
        if envelope.angle_min_rad is not None and (np.any(out[:,0]-stop<envelope.angle_min_rad) or
            np.any(out[:,0]+stop>envelope.angle_max_rad)):continue
        if envelope.velocity_rad_s is not None and np.max(np.abs(out[:,1]))>envelope.velocity_rad_s:continue
        s,h=features(spec,out[:,0],z,dirs)
        a,b,hh=coefficients(spec,theta,out[:,0],z,dirs)
        acc=(u-b*out[:,1]-hh)/a
        if envelope.acceleration_rad_s2 is not None and np.max(np.abs(acc))>envelope.acceleration_rad_s2:continue
        if envelope.jerk_rad_s3 is not None and np.max(np.abs(np.gradient(acc,t)))>envelope.jerk_rad_s3:continue
        X=np.c_[s*acc[:,None],s*out[:,1,None],h,-np.gradient(u,t)]/noise_sigma
        if maximum_first_hold_s is None:information=X.T@X/sample_hz
        else:
            intervals=np.diff(t)
            weights=np.r_[intervals[0]/2,(intervals[:-1]+intervals[1:])/2,intervals[-1]/2]
            information=X.T@(weights[:,None]*X)
        sign,score=np.linalg.slogdet(prior+information+np.eye(p)*1e-12)
        if sign>0:scores.append({"case_id":case_id,"score":float(score/t[-1]),
                                "time":t,"successful_tx":u,"information":information,
                                "stop_verified":envelope.stop_verified,
                                "predicted_stop_rad":stop,
                                "absent_external_bounds":[key for key in
                                    ("velocity_rad_s","acceleration_rad_s2","jerk_rad_s3","angle_min_rad","angle_max_rad")
                                    if getattr(envelope,key) is None],
                                **({"maximum_first_prefix":prefix,"template_duration_s":template_duration,
                                    "final_zero_ramp_s":final_zero_ramp_s} if prefix is not None else {})})
    require(bool(scores), Reason.ENVELOPE_LIMITED, "no complete stimulus plus stop fits the injected envelope")
    if _all_templates:return scores
    # Tie breaking is deterministic; no text interpretation or agent-selected amplitudes.
    selected=[];accumulated=prior+np.eye(p)*1e-12
    while scores and len(selected)<3:
        baseline=np.linalg.slogdet(accumulated)[1]
        for row in scores:
            row['score']=float((np.linalg.slogdet(accumulated+row['information'])[1]-baseline)/row['time'][-1])
        winner=min(scores,key=lambda row:(-row['score'],row['case_id']))
        selected.append(winner);scores.remove(winner);accumulated+=winner['information']
    return selected


def select_supplemental(native,snapshot,envelope,prior_information,*,noise_sigma,sample_hz):
    """Choose position/posture/direction as well as waveform, without an agent choice."""
    candidates=[]
    for iz,posture in enumerate(snapshot.spec.z_nodes):
        for iq,position in enumerate(snapshot.spec.q_nodes):
            for direction in (-1,1):
                try:
                    options=choose_information(native,snapshot.spec,snapshot.theta,snapshot.uncertainty,
                        envelope,prior_information,initial_position=position,posture=posture,
                        direction=direction,noise_sigma=noise_sigma,sample_hz=sample_hz,_all_templates=True)
                except Rejected as exc:
                    if exc.reason==Reason.ENVELOPE_LIMITED:continue
                    raise
                for row in options:
                    candidates.append({**row,'position_rad':position,'posture_rad':posture,'direction':direction,
                                       'selection_id':(iz,iq,direction,row['case_id'])})
    require(bool(candidates),Reason.ENVELOPE_LIMITED,'no informative grid cell fits the injected stimulus and stop envelope')
    selected=[];information=np.asarray(prior_information,float).copy()+np.eye(snapshot.spec.size)*1e-12
    while candidates and len(selected)<3:
        baseline=np.linalg.slogdet(information)[1]
        for row in candidates:
            row['score']=float((np.linalg.slogdet(information+row['information'])[1]-baseline)/row['time'][-1])
        winner=min(candidates,key=lambda row:(-row['score'],row['selection_id']))
        selected.append(winner);candidates.remove(winner);information+=winner['information']
    return selected


def first_stimulus(approved_minimum, envelope, *, previous_factor=0, maximum_first=False):
    # Fixed factors expand the legacy seed or descend from approved authority.
    factors=(1.,1.5,2.,3.)
    require(type(previous_factor) is int and 0<=previous_factor<len(factors),
            Reason.INSUFFICIENT_EXCITATION, "initial stimulus information budget exhausted")
    if maximum_first:
        result=np.full(np.asarray(approved_minimum if approved_minimum is not None else [0.],float).shape,
                       envelope.current_a/factors[previous_factor])
    else:
        require(approved_minimum is not None, Reason.ENVELOPE_LIMITED,
                "unknown plant requires an independently approved minimum stimulus")
        result=np.asarray(approved_minimum,float)*factors[previous_factor]
    require(np.isfinite(result).all() and np.max(np.abs(result))<=envelope.current_a,
            Reason.ENVELOPE_LIMITED, "approved current envelope exhausted")
    return result
