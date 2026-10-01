"""Offline bounded nuisance identification probes with separate gate predicates.

Native-generated data diagnose the inverse fit only. Independent oracle data
must also pass forward numerics before any declared synthetic scope is verified.
All inputs/parameters are synthetic; this tool never accesses the station.
"""
from __future__ import annotations

import argparse
from dataclasses import replace
import json
import math
from pathlib import Path
import sys
import time

import numpy as np
from scipy.optimize import least_squares
from scipy.interpolate import CubicSpline
from scipy.signal import lfilter
from scipy.special import erf, erfcx, log_ndtr
from scipy.integrate import quad

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from Firmware.commissioning.model_family import (
    FamilyNative, FamilyRun, fit_family, _bounded_central_jacobian, _errors)
from Firmware.commissioning.contracts import Reason, Rejected
from Firmware.tools.adr0022_closed_loop_estimator_probe import (
    FIXTURE, RECOVERY_GATES, NOISE_SOURCES, estimator_model, sensor_scales,
    measurement_errors, trajectory_gate, save)


TRUTH = {"actuator_tau":.0073,"transport_delay":.0087,"gyro_tau":.015,
         "gyro_delay":.0043,"current_tau":.012,"current_delay":.0029}
INITIAL = {"actuator_tau":.013,"transport_delay":.015,"gyro_tau":.024,
           "gyro_delay":.011,"current_tau":.021,"current_delay":.008}
BOUNDS = {"actuator_tau":(.001,.030),"transport_delay":(0.,.025),
          "gyro_tau":(.001,.040),"gyro_delay":(0.,.020),
          "current_tau":(.001,.035),"current_delay":(0.,.020)}
GROUPS = {**{key:(key,) for key in TRUTH},
          "actuator_pair":("actuator_tau","transport_delay"),
          "gyro_pair":("gyro_tau","gyro_delay"),
          "current_pair":("current_tau","current_delay"),
          "joint_all":tuple(TRUTH)}
MECHANICS_TRUTH = {key:FIXTURE[key] for key in
    ("a","viscous","coulomb_negative","coulomb_positive")}
MECHANICS_INITIAL = {"a":.13,"viscous":.085,"coulomb_negative":.105,"coulomb_positive":.13}
MECHANICS_BOUNDS = {"a":(.06,.15),"viscous":(.015,.12),
    "coulomb_negative":(.07,.15),"coulomb_positive":(.07,.15)}
FIT_TRUTH = {**MECHANICS_TRUTH,**TRUTH}
FIT_INITIAL = {**MECHANICS_INITIAL,**INITIAL}
FIT_BOUNDS = {**MECHANICS_BOUNDS,**BOUNDS}
GROUPS["mechanics_and_nuisance"] = tuple(FIT_TRUTH)
GROUPS["reported_sensors"] = ("gyro_tau","gyro_delay","current_tau","current_delay")
DEVELOPMENT_SEEDS = (1601,)
FINAL_SEEDS = (2203,2503,2801)
FRESH_RECOVERY_SEEDS = (3203,3511,3803)
FRESH_JOINT_SEEDS = (4001,4507,4801)
FRESH_MECHANICS_SEEDS = (7103,7517,7907)
FRESH_MECHANICS_VALIDATION = {
    "joint-verification-staggered-6s":90011,
    "joint-verification-ramped-dwell-8s":90031}
BIN_FINAL_TRAIN = "joint-final-mixed-720s"
BIN_FINAL_SEEDS = (10103,10301,10613)
BIN_FINAL_VALIDATION = {"joint-final-shaped-reversals-7s":110017,
    "joint-final-two-tone-12s":110031}


def true_model():
    return replace(estimator_model(),actuator="first_order",**TRUTH,max_step=.000125)


def prescribed_input(name):
    if name==BIN_FINAL_TRAIN:
        times = np.r_[-.1,np.arange(0.,720.,.001)]
        phase_t = np.mod(times,4.8)
        pulses = np.select([(phase_t>=.13)&(phase_t<1.23),(phase_t>=1.67)&(phase_t<3.02),
            (phase_t>=3.53)&(phase_t<4.27)],[.31,-.295,.325],default=0.)
        x = np.maximum(times-48.7,0.)
        third = (720.-48.7-1.3)/3
        phase = 4.1*np.minimum(x,third)+6.7*np.clip(x-third,0.,third)+9.3*np.maximum(x-2*third,0.)
        currents = np.where((times>=0.)&(times<48.),pulses,
            np.where((times>48.7)&(times<718.7),.33*np.sin(2*np.pi*phase+.37),0.))
    elif name=="joint-final-shaped-reversals-7s":
        times = np.r_[-.1,np.arange(0.,7.,.001)]
        currents = np.interp(times,[0.,.1773,.4317,.9031,1.2379,1.6041,2.0813,
            2.3371,2.9297,3.2673,3.9079,4.2211,4.7393,5.1097,5.7171,6.0773,6.5891,7.],
            [0.,0.,.325,.325,0.,-.315,-.315,0.,.255,.315,0.,-.285,-.285,
             .305,.12,0.,-.32,0.],left=0.)
    elif name=="joint-final-two-tone-12s":
        times = np.r_[-.1,np.arange(0.,12.,.001)]
        x = np.maximum(times-.2873,0.)
        envelope = np.minimum(np.clip(x/.413,0.,1.),np.clip((11.417-times)/.537,0.,1.))
        currents = envelope*(.18*np.sin(2*np.pi*2.3*x+.19)+.145*np.sin(2*np.pi*7.1*x+.83))
    elif name=="joint-verification-staggered-6s":
        times = [-.1,.1237,.5119,.8761,1.1023,1.4907,1.8813,2.2171,
            2.6629,3.0983,3.5551,4.0127,4.4699,4.9001,5.3119,5.7937]
        currents = [0.,.29,.18,0.,-.31,-.13,0.,.24,.32,0.,-.28,-.17,
            0.,.31,-.32,0.]
    elif name=="joint-verification-ramped-dwell-8s":
        times = np.r_[-.1,np.arange(0.,8.,.001)]
        knots = [0.,.2031,.4079,1.2113,1.4791,2.1977,2.5119,2.7137,
            3.1991,3.4037,4.1879,4.3931,5.0777,5.2993,6.1019,
            6.3971,7.0213,7.6011,8.]
        currents = np.interp(times,knots,[0.,0.,.31,.31,0.,-.30,-.30,0.,
            .28,-.24,0.,.30,.30,0.,-.32,-.32,.27,0.,0.],left=0.)
    elif name.startswith("information-mechanics-octave-"):
        duration = float(name.rsplit("-",1)[1].removesuffix("s"))
        times = np.r_[-.1,np.arange(0.,duration,.001)]
        phase_t = np.mod(times,4.)
        pulses = np.select([(phase_t>=.1)&(phase_t<1.),(phase_t>=1.35)&(phase_t<2.6),
            (phase_t>=2.95)&(phase_t<3.55)],[.27,-.30,.32],default=0.)
        x = np.maximum(times-32.5,0.)
        third = (duration-32.-1.5)/3
        phase = 3.*np.minimum(x,third)+5.5*np.clip(x-third,0.,third)+8.*np.maximum(x-2*third,0.)
        currents = np.where((times>=0.)&(times<32.),pulses,
            np.where((times>32.5)&(times<duration-1.),.32*np.sin(2*np.pi*phase),0.))
    elif name=="development-pulses":
        times = [-.1,.1013,.2679,.4211,.7027,.9519,1.1733,1.5105,1.7417,
                 2.0821,2.3109,2.6315,2.9139,3.3017,3.6813]
        currents = [0.,.24,.32,0.,-.26,-.14,0.,.28,.11,-.30,0.,.13,.31,-.29,0.]
    elif name=="development-alternating":
        times = [-.1,.1577,.3951,.6883,.8779,1.2111,1.5073,1.8059,2.0971,
                 2.4317,2.6551,2.9153,3.1807,3.5119,3.7781]
        currents = [0.,-.31,0.,.27,.12,0.,-.24,.31,0.,-.30,0.,.23,-.25,.32,0.]
    elif name=="final-asymmetric-bursts":
        times = [-.1,.1271,.2149,.4763,.7511,.9937,1.2601,1.5393,1.7849,
                 2.1343,2.4127,2.7031,3.0157,3.2221,3.5989,3.8593]
        currents = [0.,.33,.18,-.28,0.,.25,-.32,.09,0.,-.23,.29,0.,-.31,.24,-.27,0.]
    elif name=="final-dwell-reversal":
        times = [-.1,.1891,.5497,.9113,1.2307,1.3879,1.6651,1.9373,
                 2.3419,2.6117,2.9329,3.2683,3.5471,3.8237]
        currents = [0.,-.27,0.,.32,.14,0.,-.31,.25,0.,.29,-.28,0.,.24,-.30]
    elif name.startswith(("information-eightHz-","information-octave-")):
        duration = float(name.rsplit("-",1)[1].removesuffix("s"))
        times = np.r_[-.1,np.arange(0.,duration,.001)]
        x = np.maximum(times-.5,0.)
        if name.startswith("information-eightHz-"):
            phase = 8.*x
        else:
            third = (duration-1.5)/3
            phase = 3.*np.minimum(x,third)+5.5*np.clip(x-third,0.,third)+8.*np.maximum(x-2*third,0.)
        currents = np.where((times>.5)&(times<duration-1.),.32*np.sin(2*np.pi*phase),0.)
    elif name in ("information-multisine-24s","information-chirp-30s","information-full-chirp-30s"):
        duration = 24. if name=="information-multisine-24s" else 30.
        times = np.r_[-.1,np.arange(0.,duration,.001)]
        x = np.maximum(times-.5,0.)
        polarity = np.where((np.floor(x/3.)%2)==0,1.,-1.)
        if name=="information-multisine-24s":
            oscillation = .03*np.sin(2*np.pi*1.1*x)+.04*np.sin(2*np.pi*3.1*x)+.04*np.sin(2*np.pi*7.7*x)
        elif name=="information-chirp-30s":
            oscillation = .10*np.sin(2*np.pi*(x+3.5*x*x/duration))
        else:
            oscillation = .32*np.sin(2*np.pi*(x+3.5*x*x/duration))
        currents = np.where(times>.5,(0. if name=="information-full-chirp-30s" else .215*polarity)+oscillation,0.)
    else:
        raise ValueError(name)
    return np.asarray(times),np.asarray(currents)


def duration_for_input(name):
    if name==BIN_FINAL_TRAIN:
        return 720.
    if name in BIN_FINAL_VALIDATION:
        return 7. if name=="joint-final-shaped-reversals-7s" else 12.
    if name in FRESH_MECHANICS_VALIDATION:
        return 6. if name=="joint-verification-staggered-6s" else 8.
    if name.startswith(("information-eightHz-","information-octave-","information-mechanics-octave-")):
        return float(name.rsplit("-",1)[1].removesuffix("s"))
    return 24. if name=="information-multisine-24s" else 30. if name in (
        "information-chirp-30s","information-full-chirp-30s") else 4.


def generate(native,model,input_name,seed,noisy,generator):
    t = np.arange(int(round(duration_for_input(input_name)/.001))+1,dtype=float)*.001
    tx_t,tx_A = prescribed_input(input_name)
    if generator=="native":
        truth = native.rollout(model,t,tx_t,tx_A,np.zeros(5))
        origin = "NATIVE_SELF_GENERATED_INVERSE_DIAGNOSTIC_ONLY"
        events = []
    else:
        from Firmware.commissioning.synthetic_family_oracle import independent_rollout
        oracle = independent_rollout(model,t,tx_t,tx_A,np.zeros(5))
        truth,events = oracle.trace,oracle.events
        origin = "INDEPENDENT_ORACLE"
    rng = np.random.default_rng(seed)
    q,current = truth[:,0].copy(),truth[:,4].copy()
    v_new = np.arange(len(t))%20==0
    v = truth[:,3].copy()
    if noisy:
        q += rng.normal(0,FIXTURE["encoder_noise"],len(t))
        q = np.round(q/FIXTURE["encoder_quantum"])*FIXTURE["encoder_quantum"]
        current += rng.normal(0,FIXTURE["current_noise"],len(t))
        v[v_new] += rng.normal(0,FIXTURE["gyro_noise"],int(v_new.sum()))
    # Carried readings exist, but only native masks enter the residual.
    v = v[(np.arange(len(t))//20)*20]
    return {"t":t,"q":q,"v":v,"current":current,"q_new":np.ones(len(t),bool),
        "v_new":v_new,"current_new":np.ones(len(t),bool),"tx_t":tx_t,"tx_A":tx_A,
        "initial":np.zeros(5),"truth":truth,"seed":seed,"noisy":noisy,
        "noise_sources":np.asarray(NOISE_SOURCES if noisy else (),dtype="U32"),
        "input_name":input_name,"generator":origin},events


def run_from_data(label,data):
    return FamilyRun(run_id=label,source_id=data["generator"]+"/"+label,
        t=data["t"],q=data["q"],v=data["v"],current=data["current"],
        q_new=data["q_new"],v_new=data["v_new"],current_new=data["current_new"],
        tx_t=data["tx_t"],tx_A=data["tx_A"],initial=data["initial"],
        provenance="SYNTHETIC",configuration_id="declared-first-order-coulomb-nuisance-fixture",
        calibration_revision="synthetic-current-scale-clock-and-initial-state-datum",
        encoder_quantum=FIXTURE["encoder_quantum"] if data["noisy"] else 0.,
        **sensor_scales(data)).validate()


def load_dataset(path):
    with np.load(path,allow_pickle=False) as archive:
        data = {key:archive[key] for key in archive.files}
    for key in ("generator","input_name"):
        data[key] = str(data[key])
    data["noisy"],data["seed"] = bool(data["noisy"]),int(data["seed"])
    return data


def forward_gate(native,model,data):
    pred = native.rollout(model,data["t"],data["tx_t"],data["tx_A"],data["initial"])
    errors = {name:float(np.sqrt(np.mean((pred[data[mask],index]-data["truth"][data[mask],index])**2)))
        for name,index,mask in (("q",0,"q_new"),("gyro",3,"v_new"),("current",4,"current_new"))}
    if data["generator"]!="INDEPENDENT_ORACLE":
        return {"status":"NOT_RUN_NATIVE_SELF_GENERATED", "errors":errors}
    passed = errors["q"]<=RECOVERY_GATES["numerical_q_rms_rad"] and \
        errors["gyro"]<=RECOVERY_GATES["numerical_gyro_rms_rad_s"] and errors["current"]<=1e-9
    return {"status":"PASS" if passed else "FAIL","errors":errors,
        "limits":{"q":RECOVERY_GATES["numerical_q_rms_rad"],
                  "gyro":RECOVERY_GATES["numerical_gyro_rms_rad_s"],"current":1e-9}}


def _log1mexp(negative):
    values = np.asarray(negative,dtype=float)
    if np.any(values>=0):
        raise FloatingPointError("strictly negative log probability ratio required")
    result = np.empty_like(values)
    low = values < -np.log(2.)
    result[low] = np.log1p(-np.exp(values[low]))
    result[~low] = np.log(-np.expm1(values[~low]))
    return result


def gaussian_bin_deviance(mean,observed,quantum,sigma):
    """Signed exact-bin Gaussian deviance and its mean derivative, preset noise.

    The center series preserves its nonzero slope without probability floors.
    Far tails factor erfcx so two large log-CDF values are never subtracted.
    """
    if not quantum>0 or not sigma>0:
        raise ValueError("positive known quantizer quantum and pre-rounding sigma required")
    d = (np.asarray(mean,dtype=float)-np.asarray(observed,dtype=float))/sigma
    if not np.isfinite(d).all():
        raise ValueError("finite Gaussian bin means/observations required")
    b = quantum/(2*sigma)
    p0 = float(erf(b/np.sqrt(2.)))
    phi = np.exp(-b*b/2)/np.sqrt(2*np.pi)
    A = b*phi/p0
    C = b*(3-b*b)*phi/(12*p0)
    E = b*(-b**4+10*b*b-15)*phi/(360*p0)
    if not np.isfinite([p0,A,C,E]).all() or not A>0:
        raise FloatingPointError("central Gaussian bin information not numerically resolved")
    h = abs(d)
    residual,slope = np.empty_like(d),np.empty_like(d)
    center = h<=.01
    if np.any(center):
        x = d[center]
        x2 = x*x
        u = -A*x2+C*x2*x2+E*x2*x2*x2
        factor = np.ones_like(u)
        np.divide(np.log1p(u),u,out=factor,where=u!=0)
        ratio = np.sqrt(2*(A-C*x2-E*x2*x2)*factor)
        residual[center] = x*ratio
        slope[center] = (2*A-4*C*x2-6*E*x2*x2)/((1+u)*ratio*sigma)
    outer = ~center
    if np.any(outer):
        x = h[outer]
        logmass,logscore = np.empty_like(x),np.empty_like(x)
        tail = x>=b
        if np.any(~tail):
            y = x[~tail]
            upper = log_ndtr(b-y)
            logmass[~tail] = upper+_log1mexp(log_ndtr(-b-y)-upper)
            logscore[~tail] = -.5*(b-y)**2-.5*np.log(2*np.pi)+\
                _log1mexp(-2*b*y)-logmass[~tail]
        if np.any(tail):
            y = x[tail]
            t,w = y-b,2*b
            logS = np.log(erfcx(t/np.sqrt(2.)))
            delta = -t*w-w*w/2+np.log(erfcx((t+w)/np.sqrt(2.)))-logS
            remainder = _log1mexp(delta)
            logmass[tail] = -np.log(2.)-t*t/2+logS+remainder
            logscore[tail] = .5*np.log(2/np.pi)-logS-remainder+_log1mexp(-2*b*y)
        deviance = np.log(p0)-logmass
        if np.any(deviance<=0) or not np.isfinite(deviance).all():
            raise FloatingPointError("positive finite off-center bin deviance required; no floor applied")
        magnitude = np.sqrt(2*deviance)
        residual[outer] = np.sign(d[outer])*magnitude
        slope[outer] = np.exp(logscore)/magnitude/sigma
    if not np.isfinite(residual).all() or not np.isfinite(slope).all():
        raise FloatingPointError("Gaussian bin deviance precision failure")
    return residual,slope


def pristine_linear_fit(native,model,run,fields,budget,progress,*,loss_role="NOISELESS PRISTINE DEVELOPMENT ABLATION ONLY",encoder_objective="interval"):
    """Tool-only loss ablation; keep the existing physical residual and steps."""
    bounds = {field:FIT_BOUNDS[field] for field in fields}
    x0 = np.asarray([getattr(model,field) for field in fields])
    lower,upper = np.asarray([bounds[field] for field in fields]).T
    scales = np.maximum(np.abs(x0),(upper-lower)*.05)
    steps = scales*1e-6
    calls = 0
    accepted = []
    invalid_evaluations = []
    count = int(run.q_new.sum()+run.v_new.sum()+run.current_new.sum())
    began = time.monotonic()
    def unpack(x):
        return replace(model,**dict(zip(fields,x)))
    def objective(x):
        nonlocal calls
        calls += 1
        if progress and calls%25==0:
            progress("output_error",{"residual_evaluations":calls,
                "elapsed_s":time.monotonic()-began})
        try:
            prediction = native.rollout(unpack(x),run.t,run.tx_t,run.tx_A,run.initial)
            eq,ev,ei = _errors(run,prediction)
            if encoder_objective=="gaussian_bins":
                encoder,_ = gaussian_bin_deviance(prediction[run.q_new,0],run.q[run.q_new],
                    run.encoder_quantum,FIXTURE["encoder_noise"])
            else:
                encoder = eq/run.sigma_q
            return np.r_[encoder,ev/run.sigma_v,ei/run.sigma_current]
        except Rejected as exc:
            if exc.reason!=Reason.MODEL_INADEQUATE:
                raise
            invalid_evaluations.append({"call":calls,"coordinates":x.tolist(),
                "run_id":run.run_id,"reason":str(exc)})
            return np.full(count,1e6+np.linalg.norm(x-x0))
    def jacobian(x):
        accepted.append({"coordinates":x.tolist(),"characteristic_step_norm":
            None if not accepted else float(np.linalg.norm(
                (x-np.asarray(accepted[-1]["coordinates"]))/scales))})
        return _bounded_central_jacobian(objective,x,lower,upper,steps)
    result = least_squares(objective,x0,bounds=(lower,upper),loss="linear",
        method="trf",max_nfev=budget,x_scale=scales,jac=jacobian)
    last_invalid_count = len(invalid_evaluations)
    objective(result.x)
    final_invalid = invalid_evaluations[last_invalid_count:]
    bound_hits = [field for field,value,lo,hi in zip(fields,result.x,lower,upper)
        if min(value-lo,hi-value)<=1e-5*(hi-lo)]
    return {"model":unpack(result.x),"optimizer":{
        "success":bool(result.success and np.isfinite(result.x).all() and not final_invalid),
        "evaluations":int(result.nfev),"jacobian_evaluations":int(result.njev),
        "message":str(result.message),"termination_status":int(result.status),
        "residual_evaluations_including_jacobian_and_diagnostics":calls,
        "cost":float(result.cost),"optimality":float(result.optimality),
        "coordinate_scales":scales.tolist(),"absolute_derivative_steps":steps.tolist(),
        "derivative_method":"bounded central differences of declared raw residual objective",
        "accepted_coordinate_steps":accepted,"coordinates":fields,
        "coordinate_values":result.x.tolist(),"bounds":bounds,
        "initial_latent_states":[run.initial.tolist()],"initial_state_count_per_run":1,
        "loss":"linear","loss_role":loss_role,
        "encoder_objective":encoder_objective,
        "encoder_pre_rounding_noise_sigma_rad":FIXTURE["encoder_noise"] if encoder_objective=="gaussian_bins" else None,
        "encoder_quantum_rad":run.encoder_quantum,
        "initialization":{"original_coordinates":x0.tolist(),
            "policy":"original nontruth supplied initializer; no restart or truth seed"},
        "normalized_residual_rms":float(np.sqrt(np.mean(result.fun**2))),
        "parameter_bound_hits":bound_hits,"elapsed_s":time.monotonic()-began,
        "invalid_evaluations":invalid_evaluations,"final_infeasible_runs":final_invalid,
        "closed_loop_bias_qualification":"UNQUALIFIED",
        "parameter_status":"ESTIMATED_DIAGNOSTIC_NOT_PHYSICALLY_IDENTIFIED"}}


def fit_group(native,model,name,train_label,train,targets,output,budget,*,initial_model=None,loss="huber"):
    fields = GROUPS[name]
    initial = initial_model if initial_model is not None else replace(model,
        **{key:FIT_INITIAL[key] for key in fields})
    began = time.monotonic()
    def progress(stage,values):
        if name=="mechanics_and_nuisance" and stage=="output_error" and values["residual_evaluations"]%100==0:
            print(json.dumps({"case":train_label,"progress":values}),flush=True)
    if loss=="linear":
        if train["noisy"] or name!="mechanics_and_nuisance" or initial_model is not None:
            raise ValueError("linear ablation requires pristine joint10 and original nontruth initializer")
        fit = pristine_linear_fit(native,initial,run_from_data(train_label,train),fields,budget,progress)
    elif loss in ("linear_independent_noise","linear_gaussian_bins"):
        if name!="mechanics_and_nuisance" or initial_model is not None or not train["noisy"] or \
            train["generator"]!="INDEPENDENT_ORACLE" or set(map(str,train["noise_sources"]))!=set(NOISE_SOURCES):
            raise ValueError("noisy linear verification requires declared independent full-noise joint10 and original start")
        exact_bins = loss=="linear_gaussian_bins"
        fit = pristine_linear_fit(native,initial,run_from_data(train_label,train),fields,budget,progress,
            encoder_objective="gaussian_bins" if exact_bins else "interval",
            loss_role=("KNOWN PRESET GAUSSIAN ENCODER BIN LIKELIHOOD AND GAUSSIAN GYRO/CURRENT; FIXTURE CONDITIONAL"
                if exact_bins else "FIXTURE CONDITIONAL INDEPENDENT-NOISE EMPIRICAL INTERVAL-RESIDUAL SURROGATE; NOT EXACT LIKELIHOOD"))
    elif loss=="huber":
        fit = fit_family(native,initial,[run_from_data(train_label,train)],
            bounds={key:FIT_BOUNDS[key] for key in fields},max_nfev=budget,progress=progress)
    else:
        raise ValueError("unsupported diagnostic loss")
    fitted = fit.pop("model")
    relative = {key:float(getattr(fitted,key)/FIT_TRUTH[key]-1) for key in fields}
    predictions = []
    for label,data in targets.items():
        pred = native.rollout(fitted,data["t"],data["tx_t"],data["tx_A"],data["initial"])
        np.savez_compressed(output/(name+"-fit-"+train_label+"-predict-"+label+".npz"),
            t=data["t"],prediction=pred,q_new=data["q_new"],v_new=data["v_new"],current_new=data["current_new"])
        errors = measurement_errors(data,pred)
        predictions.append({"case":label,"role":"TRAINING" if label==train_label else "SELECTION",
            "errors":errors,"trajectory_gate":trajectory_gate(data,errors)})
    recovery_limit = RECOVERY_GATES["noisy_relative" if train["noisy"] else "pristine_relative"]
    numerics = forward_gate(native,model,train)
    training_predictions = [row for row in predictions if row["role"]=="TRAINING"]
    selection_predictions = [row for row in predictions if row["role"]=="SELECTION"]
    integrity = all(np.isfinite(train[key]).all() for key in ("t","q","v","current","tx_t","tx_A","truth"))
    gates = {"data_integrity":"PASS" if integrity else "FAIL", "forward_numerics":numerics["status"],
        "optimizer_termination_reason":fit["optimizer"]["message"],
        "optimizer_converged":fit["optimizer"]["success"],
        "synthetic_parameter_recovery":"PASS" if not fit["optimizer"]["parameter_bound_hits"] and
            all(abs(value)<=recovery_limit for value in relative.values()) else "FAIL",
        "training_trajectory":("PASS" if all(row["trajectory_gate"]["passed"] for row in training_predictions)
            else "FAIL") if training_predictions else "NOT_RUN",
        "selection_trajectory":("PASS" if all(row["trajectory_gate"]["passed"] for row in selection_predictions)
            else "FAIL") if selection_predictions else "NOT_RUN",
        "historical_regression":"NOT_RUN", "prospective_prediction":"NOT_RUN",
        "physical_stage3a":"NOT_RUN", "physical_stage3b":"NOT_RUN", "deployment_authorized":False}
    inverse_passed = gates["optimizer_converged"] and all(gates[key]=="PASS" for key in
        ("synthetic_parameter_recovery","training_trajectory","selection_trajectory"))
    result = {"group":name,"training_case":train_label,"model":fitted.document(),
        "free_fields":fields,"relative_parameter_errors":relative,"recovery_limit":recovery_limit,
        "optimizer":fit["optimizer"],"predictions":predictions,"gates":gates,"forward":numerics,
        "inverse_probe_passed":inverse_passed,"synthetic_scope_verified":inverse_passed and numerics["status"]=="PASS",
        "elapsed_s":time.monotonic()-began,"promotion_blocked":True}
    save(output/(name+"-fit-"+train_label+".json"),result)
    return result


def gyro_information_diagnosis(native,model,source,output):
    """Training-only Gaussian/Huber and likelihood profiles for the gyro pair."""
    with np.load(source,allow_pickle=False) as archive:
        data = {key:archive[key] for key in archive.files}
    data["noisy"] = bool(data["noisy"])
    mask = data["v_new"]
    sigma = sensor_scales(data)["sigma_v"]
    def objective(x):
        pred = native.rollout(replace(model,gyro_tau=x[0],gyro_delay=x[1]),
            data["t"],data["tx_t"],data["tx_A"],data["initial"])
        return (pred[mask,3]-data["v"][mask])/sigma
    fits = []
    for loss in ("huber","linear"):
        fit = least_squares(objective,[INITIAL["gyro_tau"],INITIAL["gyro_delay"]],
            bounds=([BOUNDS["gyro_tau"][0],BOUNDS["gyro_delay"][0]],
                    [BOUNDS["gyro_tau"][1],BOUNDS["gyro_delay"][1]]),
            loss=loss,jac="3-point",diff_step=1e-6,x_scale=[.02,.01],max_nfev=120)
        jac = np.column_stack([(objective(fit.x+np.eye(2)[k]*1e-7)-
            objective(fit.x-np.eye(2)[k]*1e-7))/(2e-7) for k in range(2)])
        information = jac.T @ jac
        covariance = np.linalg.inv(information)
        fits.append({"loss":loss,"parameters":fit.x.tolist(),"relative_errors":
            (fit.x/np.array([TRUTH["gyro_tau"],TRUTH["gyro_delay"]])-1).tolist(),
            "total_gyro_lag_s":float(fit.x.sum()),"cost":float(fit.cost),
            "success":bool(fit.success),"nfev":int(fit.nfev),"optimality":float(fit.optimality),
            "gaussian_local_standard_errors_s":np.sqrt(np.diag(covariance)).tolist(),
            "local_correlation":float(covariance[0,1]/np.sqrt(covariance[0,0]*covariance[1,1])),
            "uncertainty_role":"DECLARED_SYNTHETIC_GAUSSIAN_LOCAL_DIAGNOSTIC; NOT_PHYSICAL_CONFIDENCE"})
    profiles = []
    for tau in np.linspace(.011,.019,17):
        fit = least_squares(lambda x:objective([tau,x[0]]),[.004],
            bounds=([0.],[.020]),loss="linear",jac="3-point",diff_step=1e-6,x_scale=.01,max_nfev=60)
        profiles.append({"gyro_tau":float(tau),"best_gyro_delay":float(fit.x[0]),
                         "gaussian_cost":float(fit.cost),"converged":bool(fit.success)})
    truth_residual = objective([TRUTH["gyro_tau"],TRUTH["gyro_delay"]])
    save(output/"gyro-information.json",{"source":str(source),"fitting_data_only":True,
        "fits":fits,"profile":profiles,"true_parameter_gaussian_cost":float(.5*truth_residual @ truth_residual),
        "unchanged_recovery_gate":.05,"native_gyro_rate_Hz":50,"native_gyro_samples":int(mask.sum()),
        "noise_sigma_rad_s":sigma,"promotion_blocked":True})
    print(json.dumps(fits),flush=True)


def information_design(native,model,output):
    # Prior point is the declared nontruth initializer, not a fit selected using
    # final seeds. Two candidate waveforms are declared before the forecasts.
    prior = replace(model,gyro_tau=INITIAL["gyro_tau"],gyro_delay=INITIAL["gyro_delay"],q_min=-100.,q_max=100.)
    designs = []
    for name in ("information-multisine-24s","information-chirp-30s","information-full-chirp-30s",
                 "information-eightHz-30s","information-octave-30s"):
        began = time.monotonic()
        tx_t,tx_A = prescribed_input(name)
        t = np.arange(int(round(duration_for_input(name)/.001))+1)*.001
        mask = np.arange(len(t))%20==0
        columns = []
        for field in ("gyro_tau","gyro_delay"):
            plus = native.rollout(replace(prior,**{field:getattr(prior,field)+1e-6}),t,tx_t,tx_A,np.zeros(5))
            minus = native.rollout(replace(prior,**{field:getattr(prior,field)-1e-6}),t,tx_t,tx_A,np.zeros(5))
            columns.append((plus[mask,3]-minus[mask,3])/(2e-6*FIXTURE["gyro_noise"]))
        jac = np.column_stack(columns)
        covariance = np.linalg.inv(jac.T @ jac)
        result = {"input":name,"duration_s":duration_for_input(name),"gyro_samples":int(mask.sum()),
            "prior_gyro_tau":prior.gyro_tau,"prior_gyro_delay":prior.gyro_delay,
            "predicted_gaussian_standard_errors_s":np.sqrt(np.diag(covariance)).tolist(),
            "predicted_correlation":float(covariance[0,1]/np.sqrt(covariance[0,0]*covariance[1,1])),
            "input_max_abs_A":float(np.max(np.abs(tx_A))),"elapsed_s":time.monotonic()-began,
            "role":"PRIOR_DESIGN_INFORMATION; NOT_FINAL_FIT_OR_PHYSICAL_UNCERTAINTY"}
        designs.append(result)
        print(json.dumps(result),flush=True)
    save(output/"prior-design-information.json",{"prior":prior.document(),"designs":designs,
        "noise_sigma_rad_s":FIXTURE["gyro_noise"],"unchanged_accuracy_targets_s":[.00075,.000215],
        "final_noise_seeds_not_used":FINAL_SEEDS,"physical_actions":False})


def information_training(native,model,output,budget):
    # Selection follows only the preceding prior information calculation. The
    # original unused final input histories/seeds remain unchanged.
    model = replace(model,q_min=-100.,q_max=100.)
    save(output/"information-training-contract.json",{
        "selected_training_input":"information-full-chirp-30s","development_seed":1601,
        "selection_basis":"lowest predicted gyro-pair covariance at declared nontruth prior; no final noise data",
        "prior_gyro_tau_s":INITIAL["gyro_tau"],"prior_gyro_delay_s":INITIAL["gyro_delay"],
        "final_inputs":["final-asymmetric-bursts","final-dwell-reversal"],"final_seeds":FINAL_SEEDS,
        "quality_gates":RECOVERY_GATES,"gyro_sampling_Hz":50,"gyro_noise_sigma_rad_s":.005,
        "synthetic_command_cap_A":.32,"native_library":str(native.lib._name),
        "q_guard":"Numerical +/-100rad for the longer synthetic fixture; no physical travel authority",
        "initial_free_coordinates":{key:INITIAL[key] for key in GROUPS["gyro_pair"]},
        "bounds":{key:BOUNDS[key] for key in GROUPS["gyro_pair"]},"max_nfev":budget,
        "prediction_rule":"Fit richer training first; frozen parameters predict independent final whole runs"})
    results = []
    for noisy in (False,True):
        label = "information-full-chirp-30s-"+("noisy" if noisy else "pristine")+"-1601"
        data,events = generate(native,model,"information-full-chirp-30s",1601,noisy,"independent")
        np.savez_compressed(output/(label+".npz"),**data)
        save(output/(label+"-events.json"),events)
        # Freeze the fitted parameters before generating final measurement noise.
        temporary = fit_group(native,model,"gyro_pair",label,data,{label:data},output,budget)
        fitted = replace(model,**{key:temporary["model"][key] for key in GROUPS["gyro_pair"]})
        save(output/(label+"-frozen-fit.json"),temporary)
        predictions = []
        for seed in (FINAL_SEEDS[:1] if not noisy else FINAL_SEEDS):
            for input_name in ("final-asymmetric-bursts","final-dwell-reversal"):
                target_label = input_name+"-"+("noisy" if noisy else "pristine")+"-"+str(seed)
                target,target_events = generate(native,model,input_name,seed,noisy,"independent")
                np.savez_compressed(output/(target_label+".npz"),**target)
                save(output/(target_label+"-events.json"),target_events)
                pred = native.rollout(fitted,target["t"],target["tx_t"],target["tx_A"],target["initial"])
                np.savez_compressed(output/(label+"-predict-"+target_label+".npz"),
                    t=target["t"],prediction=pred,q_new=target["q_new"],v_new=target["v_new"],current_new=target["current_new"])
                errors = measurement_errors(target,pred)
                predictions.append({"case":target_label,"role":"FROZEN_PROSPECTIVE_SYNTHETIC_PREDICTION",
                    "errors":errors,"trajectory_gate":trajectory_gate(target,errors),
                    "forward":forward_gate(native,model,target)})
        temporary["prospective_predictions"] = predictions
        temporary["gates"]["prospective_prediction"] = "PASS" if all(row["trajectory_gate"]["passed"] for row in predictions) else "FAIL"
        temporary["inverse_probe_passed"] = temporary["gates"]["optimizer_converged"] and all(
            temporary["gates"][key]=="PASS" for key in ("synthetic_parameter_recovery","training_trajectory","prospective_prediction"))
        temporary["synthetic_scope_verified"] = temporary["inverse_probe_passed"] and \
            temporary["gates"]["forward_numerics"]=="PASS" and all(
            row["forward"]["status"]=="PASS" for row in predictions)
        results.append(temporary)
        save(output/"information-training-results.json",results)
        print(json.dumps({"case":label,"relative":temporary["relative_parameter_errors"],
            "gates":temporary["gates"],"prospective_passes":sum(row["trajectory_gate"]["passed"] for row in predictions),
            "prospective_cases":len(predictions)}),flush=True)


def fresh_recovery_verification(native,model,output,budget):
    model = replace(model,q_min=-100.,q_max=100.)
    selected = "information-octave-180s"
    prior = replace(model,gyro_tau=INITIAL["gyro_tau"],gyro_delay=INITIAL["gyro_delay"])
    t = np.arange(180001)*.001
    tx_t,tx_A = prescribed_input(selected)
    mask = np.arange(len(t))%20==0
    columns = []
    for field in ("gyro_tau","gyro_delay"):
        plus = native.rollout(replace(prior,**{field:getattr(prior,field)+1e-6}),t,tx_t,tx_A,np.zeros(5))
        minus = native.rollout(replace(prior,**{field:getattr(prior,field)-1e-6}),t,tx_t,tx_A,np.zeros(5))
        columns.append((plus[mask,3]-minus[mask,3])/(2e-6*FIXTURE["gyro_noise"]))
    covariance = np.linalg.inv(np.column_stack(columns).T @ np.column_stack(columns))
    forecast = np.sqrt(np.diag(covariance))
    save(output/"fresh-recovery-contract.json",{
        "input":selected,"fresh_train_seeds":FRESH_RECOVERY_SEEDS,"duration_s":180.,
        "selection_basis":"prior information forecast;30s candidate delaySE164.8us requires~159s for a3-sigma accuracy target; rounded to180s before generation",
        "prior_gyro_tau_s":prior.gyro_tau,"prior_gyro_delay_s":prior.gyro_delay,
        "predicted_gaussian_standard_errors_s":forecast.tolist(),
        "accuracy_targets_s":[.00075,.000215],"standard_error_targets_s":[.00025,.000215/3],
        "method_frozen":"same whole-run Huber/TRF, bounded central physical derivatives, nontruth initializer",
        "free_group":"gyro_pair","quality_gates":RECOVERY_GATES,"gyro_rate_Hz":50,
        "gyro_noise_sigma_rad_s":.005,"synthetic_current_cap_A":.32,
        "preexisting_final_predictions_are_historical":"Duration/information method continuation; no final-fit winner chosen",
        "physical_actions":False,"promotion_blocked":True})
    if not np.all(forecast<=np.array([.00025,.000215/3])):
        save(output/"information-insufficient.json",{"status":"NEEDS_MORE_INFORMATION",
            "forecast_s":forecast.tolist(),"targets_s":[.00025,.000215/3]})
        return
    # Same deterministic independent latent fixture; independent noise supplies
    # three fresh TRAIN realizations. No truth start or model change per seed.
    base,_ = generate(native,model,selected,FRESH_RECOVERY_SEEDS[0],False,"independent")
    targets = {}
    for source in ("final-asymmetric-bursts","final-dwell-reversal"):
        target,_ = generate(native,model,source,1601,False,"independent")
        targets[source] = target
    results = []
    for seed in FRESH_RECOVERY_SEEDS:
        noisy = {key:value.copy() if isinstance(value,np.ndarray) else value for key,value in base.items()}
        rng = np.random.default_rng(seed)
        noisy["q"] = np.round((base["q"]+rng.normal(0,FIXTURE["encoder_noise"],len(t)))/FIXTURE["encoder_quantum"])*FIXTURE["encoder_quantum"]
        noisy["current"] = base["current"]+rng.normal(0,FIXTURE["current_noise"],len(t))
        values = base["truth"][mask,3]+rng.normal(0,FIXTURE["gyro_noise"],int(mask.sum()))
        noisy["v"] = values[(np.arange(len(t))//20)]
        noisy.update(seed=seed,noisy=True,noise_sources=np.asarray(NOISE_SOURCES,dtype="U32"))
        label = selected+"-noisy-"+str(seed)
        np.savez_compressed(output/(label+".npz"),**noisy)
        result = fit_group(native,model,"gyro_pair",label,noisy,{label:noisy,**targets},output,budget)
        results.append(result)
        save(output/"fresh-recovery-results.json",results)
        print(json.dumps({"case":label,"relative":result["relative_parameter_errors"],
            "gates":result["gates"],"nfev":result["optimizer"]["evaluations"],"elapsed_s":result["elapsed_s"]}),flush=True)
    save(output/"fresh-recovery-summary.json",{"cases":len(results),
        "recovery_passes":sum(row["gates"]["synthetic_parameter_recovery"]=="PASS" for row in results),
        "scope_passes":sum(row["synthetic_scope_verified"] for row in results),
        "information_forecast_s":forecast.tolist(),"physical_actions":False})


def retained_joint_probe(native,model,source,output,budget):
    model = replace(model,q_min=-100.,q_max=100.)
    data = load_dataset(source)
    save(output/"retained-joint-contract.json",{"source":str(source),"role":"DEVELOPMENT",
        "group":"joint_all","free_fields":GROUPS["joint_all"],"initial":INITIAL,"bounds":BOUNDS,
        "budget":budget,"quality_gates":RECOVERY_GATES,"physical_actions":False,
        "no_new_noise_or_final_partition_selection":True})
    targets = {source.stem:data}
    for name in ("development-pulses","development-alternating"):
        target,_ = generate(native,model,name,1601,True,"independent")
        targets[name+"-noisy-1601"] = target
    result = fit_group(native,model,"joint_all",source.stem,data,targets,output,budget)
    save(output/"retained-joint-result.json",result)
    print(json.dumps({"relative":result["relative_parameter_errors"],"nfev":result["optimizer"]["evaluations"],
        "gates":result["gates"],"elapsed_s":result["elapsed_s"]}),flush=True)


def fresh_joint_verification(native,model,output,budget,duration=210.):
    model = replace(model,q_min=-100.,q_max=100.)
    prior = replace(model,**INITIAL)
    name = "information-octave-"+str(int(duration))+"s"
    t = np.arange(int(round(duration/.001))+1)*.001
    tx_t,tx_A = prescribed_input(name)
    gyro_mask = np.arange(len(t))%20==0
    scales = sensor_scales({"noisy":True})
    columns = []
    for field in GROUPS["joint_all"]:
        plus = native.rollout(replace(prior,**{field:getattr(prior,field)+1e-6}),t,tx_t,tx_A,np.zeros(5))
        minus = native.rollout(replace(prior,**{field:getattr(prior,field)-1e-6}),t,tx_t,tx_A,np.zeros(5))
        difference = (plus-minus)/(2e-6)
        columns.append(np.r_[difference[:,0]/scales["sigma_q"],
            difference[gyro_mask,3]/scales["sigma_v"],difference[:,4]/scales["sigma_current"]])
    jac = np.column_stack(columns)
    covariance = np.linalg.inv(jac.T @ jac)
    forecast = np.sqrt(np.diag(covariance))
    targets = np.asarray([TRUTH[key]*.05/3 for key in GROUPS["joint_all"]])
    save(output/"fresh-joint-contract.json",{"input":name,"fresh_train_seeds":FRESH_JOINT_SEEDS,
        "selection_basis":"180s prior gyro-delaySE71.8599us misses3-sigma target71.6667us; round duration upward210s and recompute before generation",
        "prior_parameters":{key:INITIAL[key] for key in GROUPS["joint_all"]},
        "forecast_standard_errors_s":dict(zip(GROUPS["joint_all"],map(float,forecast))),
        "standard_error_targets_5percent_over3_s":dict(zip(GROUPS["joint_all"],map(float,targets))),
        "quality_gates":RECOVERY_GATES,"max_nfev":budget,"free_fields":GROUPS["joint_all"],
        "forecast_role":"LOCAL_PRIOR_GAUSSIAN_INFORMATION; NOT_PHYSICAL_UNCERTAINTY_OR_FINAL_FIT",
        "sampling":"Native50Hzgyro/1kHzencoder-current; supplied initial state once per complete run",
        "physical_actions":False,"promotion_blocked":True})
    if not np.all(forecast<=targets):
        save(output/"joint-information-insufficient.json",{"status":"NEEDS_MORE_INFORMATION",
            "unsupported_fields":[key for key,have,need in zip(GROUPS["joint_all"],forecast,targets) if have>need]})
        return
    base,_ = generate(native,model,name,FRESH_JOINT_SEEDS[0],False,"independent")
    prediction_cases = {}
    for source in ("development-pulses","development-alternating"):
        target,_ = generate(native,model,source,1601,False,"independent")
        prediction_cases[source] = target
    results = []
    for seed in FRESH_JOINT_SEEDS:
        data = {key:value.copy() if isinstance(value,np.ndarray) else value for key,value in base.items()}
        rng = np.random.default_rng(seed)
        data["q"] = np.round((base["q"]+rng.normal(0,FIXTURE["encoder_noise"],len(t)))/FIXTURE["encoder_quantum"])*FIXTURE["encoder_quantum"]
        data["current"] = base["current"]+rng.normal(0,FIXTURE["current_noise"],len(t))
        gyro_values = base["truth"][gyro_mask,3]+rng.normal(0,FIXTURE["gyro_noise"],int(gyro_mask.sum()))
        data["v"] = gyro_values[np.arange(len(t))//20]
        data.update(seed=seed,noisy=True,noise_sources=np.asarray(NOISE_SOURCES,dtype="U32"))
        label = name+"-joint-noisy-"+str(seed)
        np.savez_compressed(output/(label+".npz"),**data)
        result = fit_group(native,model,"joint_all",label,data,{label:data,**prediction_cases},output,budget)
        results.append(result)
        save(output/"fresh-joint-results.json",results)
        print(json.dumps({"case":label,"relative":result["relative_parameter_errors"],"gates":result["gates"],
            "nfev":result["optimizer"]["evaluations"],"elapsed_s":result["elapsed_s"]}),flush=True)
    save(output/"fresh-joint-summary.json",{"cases":len(results),
        "recovery_passes":sum(row["gates"]["synthetic_parameter_recovery"]=="PASS" for row in results),
        "scope_passes":sum(row["synthetic_scope_verified"] for row in results),"physical_actions":False})


def joint_prior_derivatives(native,model,output):
    prior = replace(model,q_min=-100.,q_max=100.,**INITIAL)
    t = np.arange(180001)*.001
    tx_t,tx_A = prescribed_input("information-octave-180s")
    gyro_mask = np.arange(len(t))%20==0
    scales = sensor_scales({"noisy":True})
    results = []
    for field in ("actuator_tau","transport_delay"):
        previous = None
        for h in (1e-6,1e-7,1e-8):
            plus = native.rollout(replace(prior,**{field:getattr(prior,field)+h}),t,tx_t,tx_A,np.zeros(5))
            minus = native.rollout(replace(prior,**{field:getattr(prior,field)-h}),t,tx_t,tx_A,np.zeros(5))
            difference = (plus-minus)/(2*h)
            column = np.r_[difference[:,0]/scales["sigma_q"],difference[gyro_mask,3]/scales["sigma_v"],
                difference[:,4]/scales["sigma_current"]]
            result = {"coordinate":field,"absolute_step":h,"normalized_column_norm":float(np.linalg.norm(column)),
                "relative_change_previous":None if previous is None else float(np.linalg.norm(column-previous)/np.linalg.norm(previous)),
                "q_max_perturbation_difference_rad":float(np.max(np.abs(plus[:,0]-minus[:,0]))),
                "positive_stick_transitions":int(np.sum(np.diff(plus[:,5])!=0)),
                "negative_stick_transitions":int(np.sum(np.diff(minus[:,5])!=0))}
            results.append(result)
            previous = column
            print(json.dumps(result),flush=True)
    save(output/"joint-prior-derivatives.json",{"prior":prior.document(),"results":results,
        "role":"PRIOR_NUMERICAL_DIAGNOSTIC; NOT_GLOBAL_IDENTIFIABILITY"})


def prior_joint_information(native,prior,data,fields):
    """Noise-scaled local center-observation information; no recovery claim."""
    scales = sensor_scales({"noisy":True})
    columns = []
    changes = {}
    for field in fields:
        previous = None
        characteristic = max(abs(getattr(prior,field)),.05*np.ptp(FIT_BOUNDS[field]))
        for factor in (1e-6,1e-7):
            h = characteristic*factor
            plus = native.rollout(replace(prior,**{field:getattr(prior,field)+h}),
                data["t"],data["tx_t"],data["tx_A"],data["initial"])
            minus = native.rollout(replace(prior,**{field:getattr(prior,field)-h}),
                data["t"],data["tx_t"],data["tx_A"],data["initial"])
            difference = (plus-minus)/(2*h)
            column = np.r_[difference[data["q_new"],0]/scales["sigma_q"],
                difference[data["v_new"],3]/scales["sigma_v"],
                difference[data["current_new"],4]/scales["sigma_current"]]
            if previous is not None:
                changes[field] = float(np.linalg.norm(column-previous)/np.linalg.norm(previous))
            previous = column
        columns.append(column)
    jac = np.column_stack(columns)
    norms = np.linalg.norm(jac,axis=0)
    normalized = jac/norms
    singular = np.linalg.svd(normalized,compute_uv=False)
    covariance = np.linalg.inv(normalized.T@normalized)/norms[:,None]/norms[None,:]
    errors = np.sqrt(np.diag(covariance))
    correlation = covariance/errors[:,None]/errors[None,:]
    return {"prior":{key:getattr(prior,key) for key in fields},
        "free_fields":fields,"normalized_singular_values":singular.tolist(),
        "normalized_rank":int(np.sum(singular>singular[0]*1e-8)),
        "column_norms":dict(zip(fields,map(float,norms))),
        "derivative_relative_change_decade":changes,
        "derivative_consistency":"PASS" if max(changes.values())<=.01 else "UNSTABLE_DERIVATIVES",
        "derivative_consistency_target":.01,
        "local_gaussian_standard_errors":dict(zip(fields,map(float,errors))),
        "correlation":correlation.tolist(),
        "role":"LOCAL_PRIOR_GAUSSIAN_INFORMATION; NOT_GLOBAL_IDENTIFIABILITY_OR_PHYSICAL_UNCERTAINTY",
        "encoder_information_approximation":"measurement center derivatives; final fit retains quantization interval residual"}


def training_prefix(data,duration):
    """Causal TRAIN prefix, retaining input prehistory and the one initial state."""
    count = int(np.searchsorted(data["t"],data["t"][0]+duration,side="right"))
    if count<2:
        raise ValueError("training prefix needs at least two observation samples")
    observation_keys = ("t","q","v","current","q_new","v_new","current_new","truth")
    tx_count = int(np.searchsorted(data["tx_t"],data["t"][count-1],side="right"))
    prefix = {}
    for key,value in data.items():
        if key in observation_keys:
            prefix[key] = value[:count].copy()
        elif key in ("tx_t","tx_A"):
            prefix[key] = value[:tx_count].copy()
        else:
            prefix[key] = value.copy() if isinstance(value,np.ndarray) else value
    return prefix


def analytic_electrical_trace(model,data):
    """Exact uniform-ZOH electrical states for the declared zero-state fixture."""
    t,tx_t,tx_A = data["t"],data["tx_t"],data["tx_A"]
    dt = float(t[1]-t[0])
    if not (np.max(np.abs(np.diff(t)-dt))<1e-9 and
            np.max(np.abs(np.diff(tx_t[1:])-dt))<1e-9 and
            np.array_equal(data["initial"],np.zeros(5)) and
            model.actuator_gain==model.current_gain==1. and
            model.actuator_bias==model.current_bias==0. and tx_A[0]==0.):
        raise ValueError("electrical initializer requires declared uniform-ZOH zero-state/unit-scale fixture")
    def held(delay):
        indices = np.searchsorted(tx_t,t-delay+1e-12,side="right")-1
        if np.any(indices<0):
            raise ValueError("electrical initializer lacks causal input prehistory")
        r = float((t[0]-tx_t[1]-delay)%dt)
        if min(r,dt-r)<1e-12:
            r = 0.
        return tx_A[indices],r
    desired,r = held(model.transport_delay)
    ea = np.exp(-dt/model.actuator_tau)
    er = np.exp(-r/model.actuator_tau)
    actual = lfilter([1-er,er-ea],[1.,-ea],desired)
    desired,r = held(model.transport_delay+model.current_delay)
    ec = np.exp(-dt/model.current_tau)
    def cascade_step(time):
        ta,tc = model.actuator_tau,model.current_tau
        if abs(ta-tc)<=max(ta,tc)*1e-6:
            tau = (ta+tc)/2
            return 1-np.exp(-time/tau)*(1+time/tau)
        return 1-(ta*np.exp(-time/ta)-tc*np.exp(-time/tc))/(ta-tc)
    s0,s1,s2 = (cascade_step(r+k*dt) for k in range(3))
    pole_sum,pole_product = ea+ec,ea*ec
    numerator = [s0,s1-(pole_sum+1)*s0,
        s2-(pole_sum+1)*s1+(pole_product+pole_sum)*s0]
    reported = lfilter(numerator,[1.,-pole_sum,pole_product],desired)
    areas = np.r_[0.,np.cumsum(tx_A[:-1]*np.diff(tx_t))]
    def input_area(query):
        indices = np.searchsorted(tx_t,query+1e-12,side="right")-1
        return areas[indices]+tx_A[indices]*(query-tx_t[indices])
    actuator_integral = input_area(t-model.transport_delay)-input_area(t[0]-model.transport_delay)-model.actuator_tau*actual
    return actual,reported,actuator_integral


def moving_joint_initializer(native,model,data,output,budget):
    """TRAIN-only smooth inverse basin entry; final qualification uses hybrid OE."""
    fields = GROUPS["mechanics_and_nuisance"]
    prior = replace(model,**FIT_INITIAL)
    actual,reported,_ = analytic_electrical_trace(prior,data)
    check = native.rollout(prior,data["t"],data["tx_t"],data["tx_A"],data["initial"])
    helper_errors = {"actual_current_max_abs_A":float(np.max(np.abs(actual-check[:,2]))),
        "reported_current_max_abs_A":float(np.max(np.abs(reported-check[:,4])))}
    save(output/"electrical-initializer-forward.json",{"errors":helper_errors,
        "limit_A":1e-9,"role":"ANALYTIC_HELPER_VS_NATIVE; INDEPENDENT_ORACLE_FORWARD_PREREQUISITE_RETAINED_SEPARATELY"})
    if max(helper_errors.values())>1e-9:
        raise ValueError("analytic electrical initializer disagrees with native before inverse fitting")
    times = data["t"]
    gyro_t = times[data["v_new"]]
    gyro = CubicSpline(gyro_t,data["v"][data["v_new"]],extrapolate=False)
    derivative,anti = gyro.derivative(),gyro.antiderivative()
    sigma = sensor_scales(data)
    kinematic,moving = [],[]
    starts = gyro_t[(gyro_t>=.04)&(gyro_t<=times[-1]-.15)]
    for duration in (.02,.04,.08,.12):
        ends = starts+duration
        indices0 = np.rint((starts-times[0])/.001).astype(int)
        indices1 = np.rint((ends-times[0])/.001).astype(int)
        delta_q = data["q"][indices1]-data["q"][indices0]
        weight = np.sqrt(2*sigma["sigma_q"]**2+2*(prior.gyro_tau*sigma["sigma_v"])**2+
            (duration*sigma["sigma_v"])**2)
        kinematic.append((starts,ends,delta_q,weight))
    for duration in (.02,.04,.06):
        ends = starts+duration
        grid = starts[:,None]+np.linspace(0.,duration,5)[None,:]+prior.gyro_delay
        velocity = gyro(grid)+prior.gyro_tau*derivative(grid)
        valid = (np.min(np.abs(velocity),axis=1)>=.02)&(
            np.all(velocity>0,axis=1)|np.all(velocity<0,axis=1))
        i0 = np.rint((starts[valid]-times[0])/.001).astype(int)
        i1 = np.rint((ends[valid]-times[0])/.001).astype(int)
        sign = np.sign(velocity[valid,0])
        delta_q = data["q"][i1]-data["q"][i0]
        weight = np.sqrt(2*prior.a**2*sigma["sigma_v"]**2*(1+(prior.gyro_tau/.02)**2)+
            2*prior.viscous**2*sigma["sigma_q"]**2)
        moving.append((starts[valid],ends[valid],i0,i1,sign,delta_q,weight))
    save(output/"moving-joint-initializer-contract.json",{
        "free_fields":fields,"nontruth_initial":FIT_INITIAL,"max_nfev":budget,
        "kinematic_windows":sum(len(row[0]) for row in kinematic),
        "fixed_moving_windows":sum(len(row[0]) for row in moving),
        "moving_window_durations_s":[.02,.04,.06],"minimum_prior_reconstructed_speed_rad_s":.02,
        "moving_window_mask":"fixed from TRAIN gyro at nontruth prior; five same-sign speed checks per window",
        "equations":["dq=integral(h(t+dg))+tau_g*delta_h",
            "a*delta_v+B*dq+sign*Fc*dt+load*dt=integral(actual_current)",
            "reported_current=exact two-pole cascade of prescribed successful TX"],
        "interpolation_role":"native50Hz gyro cubic spline for initialization only; final objective uses original native masks",
        "correlated_window_residuals":"basin-entry diagnostic, no confidence or physical support claim",
        "no_truth_or_selection_initialization":True,"physical_actions":False})
    def objective(x):
        candidate = replace(model,**dict(zip(fields,x)))
        _,current,integral = analytic_electrical_trace(candidate,data)
        rows = [(current[data["current_new"]]-data["current"][data["current_new"]])/sigma["sigma_current"]]
        dg,tg = candidate.gyro_delay,candidate.gyro_tau
        for start,end,dq,weight in kinematic:
            rows.append((anti(end+dg)-anti(start+dg)+tg*(gyro(end+dg)-gyro(start+dg))-dq)/weight)
        for start,end,i0,i1,sign,dq,weight in moving:
            delta_v = gyro(end+dg)-gyro(start+dg)+tg*(derivative(end+dg)-derivative(start+dg))
            friction = np.where(sign>0,candidate.coulomb_positive,-candidate.coulomb_negative)
            rows.append((candidate.a*delta_v+candidate.viscous*dq+
                (friction+candidate.load_offset)*(end-start)-(integral[i1]-integral[i0]))/weight)
        return np.concatenate(rows)
    x0 = np.asarray([FIT_INITIAL[key] for key in fields])
    began = time.monotonic()
    calls = 0
    def counted(x):
        nonlocal calls
        calls += 1
        return objective(x)
    fit = least_squares(counted,x0,bounds=([FIT_BOUNDS[key][0] for key in fields],
        [FIT_BOUNDS[key][1] for key in fields]),loss="huber",method="trf",
        jac="3-point",diff_step=1e-5,x_scale=x0,max_nfev=budget)
    fitted = replace(model,**dict(zip(fields,fit.x)))
    result = {"parameters":dict(zip(fields,map(float,fit.x))),"success":bool(fit.success),
        "message":str(fit.message),"evaluations":int(fit.nfev),"residual_evaluations":calls,
        "cost":float(fit.cost),"optimality":float(fit.optimality),
        "relative_parameter_errors":{key:getattr(fitted,key)/FIT_TRUTH[key]-1 for key in fields},
        "elapsed_s":time.monotonic()-began,"role":"TRAIN_ONLY_BASIN_INITIALIZER; NO_RECOVERY_PROMOTION"}
    save(output/"moving-joint-initializer-result.json",result)
    print(json.dumps({"moving_initializer":result}),flush=True)
    return fitted,result


def retained_mechanics_probe(native,model,source,output,budget,prefix_initialization=False,moving_initialization=False):
    model = replace(model,q_min=-100.,q_max=100.)
    data = load_dataset(source)
    fields = GROUPS["mechanics_and_nuisance"]
    prior = replace(model,**FIT_INITIAL)
    save(output/"mechanics-joint-contract.json",{
        "source":str(source),"source_role":"CONSUMED_CYCLE02_DATA_NOW_DEVELOPMENT",
        "free_fields":fields,"nontruth_initial":FIT_INITIAL,"bounds":FIT_BOUNDS,
        "gauges":{"actuator_gain":model.actuator_gain,"current_gain":model.current_gain,
            "actuator_bias":model.actuator_bias,"current_bias":model.current_bias,
            "gyro_bias":model.gyro_bias,"load_offset":model.load_offset,
            "static_negative":model.static_negative,"static_positive":model.static_positive,
            "clock_offsets":"known zero, delays expressed separately","initial_state":"known supplied once"},
        "structural_confounds_to_check":["free actuator gain versus A-equivalent mechanics scale",
            "actuator/current-filter cascade pole permutation","mechanical pole versus actuator pole",
            "filter lag versus pure delay","unknown constant load versus directional friction"],
        "quality_gates":RECOVERY_GATES,"max_nfev":budget,"numerical_current_gate_A":1e-9,
        "method":"whole-run Huber/TRF, bounded central derivatives; no multistart or truth seed",
        "initialization":({"policy":"TRAIN-only uninterrupted12s prefix then full-runOE",
            "prefix_max_nfev":budget//3,"full_run_max_nfev":budget-budget//3,
            "reason":"baseline full-run120fit remains far from comparable truth cost with large optimality; test earlier basin entry before cumulative long-run objective",
            "same_free_coordinates":True,"initial_state_reset_after_prefix":False}
            if prefix_initialization else {"policy":"TRAIN-only moving-equation initializer followed by native full-runOE",
                "initializer_max_nfev":budget//3,"full_run_max_nfev":budget-budget//3}
            if moving_initialization else {"policy":"supplied nontruth point"}),
        "input":"retained prescribed successful-TX octave, native encoder/current1kHz and gyro50Hz",
        "q_guard":"numerical +/-100rad; no physical travel authority",
        "no_new_noise_or_final_partition_used":True,"physical_actions":False,"promotion_blocked":True})
    forward = forward_gate(native,model,data)
    save(output/"mechanics-joint-forward.json",forward)
    if forward["status"]!="PASS":
        print(json.dumps({"forward":forward,"inverse":"NOT_RUN"}),flush=True)
        return
    information = prior_joint_information(native,prior,data,fields)
    save(output/"mechanics-joint-prior-information.json",information)
    print(json.dumps({"prior_normalized_rank":information["normalized_rank"],
        "local_standard_errors":information["local_gaussian_standard_errors"],
        "derivative_changes":information["derivative_relative_change_decade"]}),flush=True)
    # Historical inputs are generated after the training fit and never select
    # an initializer. They only expose whether a retained fit generalizes.
    initialization_result = None
    initial_model = None
    full_budget = budget
    if prefix_initialization:
        prefix = training_prefix(data,12.)
        prefix_label = source.stem+"-training-prefix12s"
        initialization_result = fit_group(native,model,"mechanics_and_nuisance",prefix_label,
            prefix,{prefix_label:prefix},output,budget//3)
        initial_model = replace(model,**{key:initialization_result["model"][key] for key in fields})
        full_budget -= budget//3
        print(json.dumps({"prefix_relative":initialization_result["relative_parameter_errors"],
            "prefix_nfev":initialization_result["optimizer"]["evaluations"],
            "prefix_termination":initialization_result["gates"]["optimizer_termination_reason"]}),flush=True)
    moving_result = None
    if moving_initialization:
        initial_model,moving_result = moving_joint_initializer(native,model,data,output,budget//3)
        full_budget -= budget//3
    result = fit_group(native,model,"mechanics_and_nuisance",source.stem,data,
        {source.stem:data},output,full_budget,initial_model=initial_model)
    if initialization_result is not None:
        result["training_prefix_initialization"] = initialization_result
        result["total_optimizer_evaluations"] = result["optimizer"]["evaluations"]+initialization_result["optimizer"]["evaluations"]
        result["total_residual_evaluations_including_jacobian_and_diagnostics"] = sum(
            row["optimizer"]["residual_evaluations_including_jacobian_and_diagnostics"]
            for row in (result,initialization_result))
    if moving_result is not None:
        result["moving_equation_initialization"] = moving_result
        result["total_optimizer_evaluations"] = result["optimizer"]["evaluations"]+moving_result["evaluations"]
        result["total_residual_evaluations_including_jacobian_and_diagnostics"] = (
            result["optimizer"]["residual_evaluations_including_jacobian_and_diagnostics"]+moving_result["residual_evaluations"])
    fitted = replace(model,**{key:result["model"][key] for key in fields})
    historical = []
    for name in ("development-pulses","development-alternating"):
        target,_ = generate(native,model,name,1601,False,"independent")
        pred = native.rollout(fitted,target["t"],target["tx_t"],target["tx_A"],target["initial"])
        np.savez_compressed(output/("mechanics-joint-predict-"+name+".npz"),
            t=target["t"],prediction=pred,q_new=target["q_new"],v_new=target["v_new"],
            current_new=target["current_new"])
        errors = measurement_errors(target,pred)
        historical.append({"case":name,"role":"HISTORICAL_REGRESSION",
            "errors":errors,"trajectory_gate":trajectory_gate(target,errors),
            "forward":forward_gate(native,model,target)})
    result["historical_predictions"] = historical
    result["gates"]["historical_regression"] = "PASS" if all(
        row["trajectory_gate"]["passed"] for row in historical) else "FAIL"
    result["synthetic_scope_verified"] = False  # development only, fresh inverse remains required
    result["role"] = "CONSUMED_DATA_JOINT_DEVELOPMENT; NOT_FRESH_RECOVERY_VERIFICATION"
    save(output/"mechanics-joint-result.json",result)
    print(json.dumps({"relative":result["relative_parameter_errors"],"gates":result["gates"],
        "nfev":result["optimizer"]["evaluations"],"elapsed_s":result["elapsed_s"]}),flush=True)


def mechanics_gauge_probe(native,model,source,output):
    data = load_dataset(source)
    prior = replace(model,q_min=-100.,q_max=100.,**FIT_INITIAL)
    save(output/"mechanics-gauge-contract.json",{"source":str(source),
        "role":"CONSUMED_TRAIN_INFORMATION_DIAGNOSTIC",
        "prior":{key:getattr(prior,key) for key in FIT_INITIAL},
        "analytic_transformations":["actuator/current-filter pole permutation",
            "transport/reported-current delay interchange"],
        "selection_or_future_data_used":False,"fit_performed":False,"physical_actions":False})
    baseline = native.rollout(prior,data["t"],data["tx_t"],data["tx_A"],data["initial"])
    transformations = {
        "cascade_pole_permutation":replace(prior,actuator_tau=prior.current_tau,current_tau=prior.actuator_tau),
        "current_total_delay_interchange":replace(prior,transport_delay=prior.current_delay,
            current_delay=prior.transport_delay)}
    results = []
    for name,candidate in transformations.items():
        prediction = native.rollout(candidate,data["t"],data["tx_t"],data["tx_A"],data["initial"])
        differences = {channel:float(np.sqrt(np.mean((prediction[data[mask],index]-
            baseline[data[mask],index])**2))) for channel,index,mask in
            (("q",0,"q_new"),("gyro",3,"v_new"),("current",4,"current_new"))}
        results.append({"transformation":name,"observation_rms_difference":differences,
            "reported_current_discriminates":differences["current"]>1e-9,
            "motion_discriminates":differences["q"]>RECOVERY_GATES["numerical_q_rms_rad"],
            "parameters":{key:getattr(candidate,key) for key in FIT_INITIAL}})
    save(output/"mechanics-gauge-results.json",{"results":results,
        "interpretation":"current alone cannot distinguish commuting poles or split its total delay; encoder/gyro motion can distinguish these declared prior transformations",
        "limitation":"finite prior transformations plus local rank do not prove global uniqueness with refitted mechanics",
        "scale_gauge":"command/current gains fixed; A-equivalent mechanics retain an explicit current scale",
        "offset_gauge":"load and sensor/actuator biases fixed; no separate unknown load/directional-friction claim"})
    print(json.dumps(results),flush=True)


def mechanics_information_design(native,model,output,duration=242.):
    name = "information-mechanics-octave-"+str(int(duration))+"s"
    fields = GROUPS["mechanics_and_nuisance"]
    prior = replace(model,q_min=-100.,q_max=100.,**FIT_INITIAL)
    save(output/"mechanics-information-contract.json",{"input":name,
        "design_basis":"32 s dwell/reversal supplies slow mechanical-pole information plus the declared octave band; evaluate proposed duration before observations",
        "mechanical_prefix_s":32.,"mechanical_cycle_s":4.,"native_current_cap_A":.32,
        "waveform_role":"SYNTHETIC_TRAIN_DESIGN; NO_PHYSICAL_COMMAND_AUTHORITY",
        "prior":FIT_INITIAL,"free_fields":fields,"gauges":"same declared current-scale/load/static/clock/initial facts",
        "standard_error_target":"5percent of each declared synthetic parameter divided by3",
        "unused_development_seed":7001,"reserved_fresh_train_seeds":[7103,7517,7907],
        "seed_provenance_addendum":"6101 was used in a separate load-structure scope; original unused contracts remain retained, new identifiers avoid ambiguous freshness",
        "native_library":str(native.lib._name),
        "quality_gates":RECOVERY_GATES,"before_measurement_generation":True,"physical_actions":False})
    t = np.arange(int(round(duration/.001))+1)*.001
    tx_t,tx_A = prescribed_input(name)
    data = {"t":t,"tx_t":tx_t,"tx_A":tx_A,"initial":np.zeros(5),
        "q_new":np.ones(len(t),bool),"v_new":np.arange(len(t))%20==0,
        "current_new":np.ones(len(t),bool)}
    result = prior_joint_information(native,prior,data,fields)
    targets = {key:FIT_TRUTH[key]*.05/3 for key in fields}
    result["standard_error_targets_5percent_over3"] = targets
    result["unsupported_fields"] = [key for key in fields if result["local_gaussian_standard_errors"][key]>targets[key]]
    result["information_status"] = "UNSTABLE_DERIVATIVES" if result["derivative_consistency"]!="PASS" else (
        "NEEDS_MORE_INFORMATION" if result["unsupported_fields"] else "PRIOR_PRECISION_TARGET_MET")
    save(output/"mechanics-information-design.json",result)
    print(json.dumps({"information_status":result["information_status"],"normalized_rank":result["normalized_rank"],
        "forecast":result["local_gaussian_standard_errors"],"unsupported_fields":result["unsupported_fields"]}),flush=True)


def mechanics_gyro_derivatives(native,model,output,duration=362.):
    name = "information-mechanics-octave-"+str(int(duration))+"s"
    prior = replace(model,q_min=-100.,q_max=100.,**FIT_INITIAL)
    t = np.arange(int(round(duration/.001))+1)*.001
    tx_t,tx_A = prescribed_input(name)
    mask = np.arange(len(t))%20==0
    steps = (1.1e-8,1e-6,1e-5,1e-4)
    resolutions = (.000125,.0000625,.00003125)
    sigma = sensor_scales({"noisy":True})
    save(output/"gyro-derivative-contract.json",{"input":name,"prior":FIT_INITIAL,
        "physical_gyro_delay_steps_s":steps,"integration_max_steps_s":resolutions,
        "analytic_derivative":"-(v(t-delay)-filtered_gyro(t-delay))/gyro_tau",
        "causal_zero_derivatives":["q versus gyro_delay","current versus gyro_delay"],
        "no_observations_noise_or_inverse_fit_generated":True,
        "diagnostic_derivative_relative_error_target":.01,"physical_actions":False})
    rows,raw = [],{"t_gyro":t[mask]}
    for resolution in resolutions:
        candidate = replace(prior,max_step=resolution)
        base = native.rollout(candidate,t,tx_t,tx_A,np.zeros(5))
        velocity = np.interp(t[mask]-candidate.gyro_delay,t,base[:,1],left=0.)
        analytic = -(velocity-base[mask,3])/candidate.gyro_tau
        raw["analytic_gyro_"+str(resolution)] = analytic
        for h in steps:
            plus = native.rollout(replace(candidate,gyro_delay=candidate.gyro_delay+h),t,tx_t,tx_A,np.zeros(5))
            minus = native.rollout(replace(candidate,gyro_delay=candidate.gyro_delay-h),t,tx_t,tx_A,np.zeros(5))
            difference = (plus-minus)/(2*h)
            gyro = difference[mask,3]
            norms = {"q":float(np.linalg.norm(difference[:,0]/sigma["sigma_q"])),
                "gyro":float(np.linalg.norm(gyro/sigma["sigma_v"])),
                "current":float(np.linalg.norm(difference[:,4]/sigma["sigma_current"]))}
            row = {"max_step_s":resolution,"gyro_delay_step_s":h,
                "gyro_relative_error_vs_filter_equation":float(np.linalg.norm(gyro-analytic)/np.linalg.norm(analytic)),
                "scaled_channel_derivative_norms":norms,
                "spurious_q_over_physical_gyro_norm":norms["q"]/norms["gyro"],
                "q_prediction_difference_rms_rad":float(np.sqrt(np.mean((plus[:,0]-minus[:,0])**2)))}
            rows.append(row)
            raw["gyro_"+str(resolution)+"_"+str(h)] = gyro
            save(output/"gyro-derivative-results.json",{"rows":rows,
                "role":"FORWARD_DERIVATIVE_DIAGNOSTIC; NOT_PARAMETER_RECOVERY_OR_PHYSICAL_UNCERTAINTY"})
            print(json.dumps(row),flush=True)
    np.savez_compressed(output/"gyro-derivatives.npz",**raw)


def mechanics_pristine_probe(native,model,output,budget,duration=362.,*,source=None,loss="huber"):
    model = replace(model,q_min=-100.,q_max=100.)
    if source is None:
        name = "information-mechanics-octave-"+str(int(duration))+"s"
        data = None
    else:
        data = load_dataset(source)
        if data["noisy"] or data["generator"]!="INDEPENDENT_ORACLE":
            raise ValueError("retained pristine loss ablation requires independent noiseless TRAIN")
        name = data["input_name"]
    save(output/"mixed-pristine-contract.json",{"input":name,"development_identifier":7001,
        "generator":"INDEPENDENT_ORACLE","noise":"NONE; original pristine gates",
        "free_fields":GROUPS["mechanics_and_nuisance"],"nontruth_initial":FIT_INITIAL,
        "bounds":FIT_BOUNDS,"max_nfev":budget,"quality_gates":RECOVERY_GATES,
        "method":"full native hybrid "+loss+"/TRF with unchanged bounded central derivatives; no prefix or surrogate initializer",
        "loss":loss,"source":str(source) if source is not None else "NEW INDEPENDENT PRISTINE GENERATION",
        "loss_assumption":"no observation noise in retained pristine data" if loss=="linear" else "unchanged Huber objective",
        "discriminator":"near-fit gyro 95.4% Huber linear-region residuals and lost local curvature" if loss=="linear" else None,
        "gauges":"known current units/gains, biases, constant load, static thresholds, clock mappings and supplied initial state",
        "native_library":str(native.lib._name),"fresh_noise_generated":False,
        "physical_actions":False,"promotion_blocked":True})
    began = time.monotonic()
    label = source.stem if source is not None else name+"-pristine-7001"
    if data is None:
        data,events = generate(native,model,name,7001,False,"independent")
        np.savez_compressed(output/(label+".npz"),**data)
        save(output/(label+"-events.json"),events)
    forward = forward_gate(native,model,data)
    save(output/"mixed-pristine-forward.json",forward)
    if forward["status"]!="PASS":
        print(json.dumps({"forward":forward,"inverse":"NOT_RUN"}),flush=True)
        return
    result = fit_group(native,model,"mechanics_and_nuisance",label,data,{label:data},output,budget,loss=loss)
    fitted = replace(model,**{key:result["model"][key] for key in GROUPS["mechanics_and_nuisance"]})
    historical = []
    for source in ("development-pulses","development-alternating"):
        target,_ = generate(native,model,source,7001,False,"independent")
        prediction = native.rollout(fitted,target["t"],target["tx_t"],target["tx_A"],target["initial"])
        np.savez_compressed(output/("mixed-pristine-predict-"+source+".npz"),
            t=target["t"],prediction=prediction,q_new=target["q_new"],v_new=target["v_new"],current_new=target["current_new"])
        errors = measurement_errors(target,prediction)
        historical.append({"case":source,"role":"HISTORICAL_REGRESSION","errors":errors,
            "trajectory_gate":trajectory_gate(target,errors),"forward":forward_gate(native,model,target)})
    result["historical_predictions"] = historical
    result["gates"]["historical_regression"] = "PASS" if all(row["trajectory_gate"]["passed"] for row in historical) else "FAIL"
    result["pristine_scope_verified"] = result["gates"]["optimizer_converged"] and all(
        result["gates"][key]=="PASS" for key in ("forward_numerics","synthetic_parameter_recovery","training_trajectory","historical_regression"))
    result["synthetic_scope_verified"] = False
    result["role"] = "PRISTINE_DEVELOPMENT; FRESH_NOISY_INVERSE_NOT_RUN"
    result["total_generation_fit_and_validation_elapsed_s"] = time.monotonic()-began
    save(output/"mixed-pristine-result.json",result)
    print(json.dumps({"relative":result["relative_parameter_errors"],"gates":result["gates"],
        "nfev":result["optimizer"]["evaluations"],"elapsed_s":result["total_generation_fit_and_validation_elapsed_s"]}),flush=True)


def conditional_sensor_probe(native,model,data_source,fit_source,output,budget):
    data = load_dataset(data_source)
    retained = json.loads(fit_source.read_text())
    model = replace(model,q_min=-100.,q_max=100.)
    initial = replace(model,**{key:retained["model"][key] for key in FIT_TRUTH})
    save(output/"conditional-sensor-contract.json",{"data_source":str(data_source),
        "fit_source":str(fit_source),"source_benchmark_max_nfev":120,
        "diagnostic_max_nfev":budget,"role":"TRAIN_ONLY_CONDITIONAL_DIAGNOSTIC; NOT_A_LARGER_BUDGET_RECOVERY_BENCHMARK",
        "free_fields":GROUPS["reported_sensors"],
        "initial_estimated_parameters":{key:getattr(initial,key) for key in FIT_TRUTH},
        "fixed_parameter_role":"data-fitted mechanical/actuation estimates, never fixed at truth",
        "quality_gates":RECOVERY_GATES,"new_noise_or_selection_used":False,"physical_actions":False})
    result = fit_group(native,model,"reported_sensors",data_source.stem,data,
        {data_source.stem:data},output,budget,initial_model=initial)
    result["synthetic_scope_verified"] = False
    result["role"] = "CONDITIONAL_SENSOR_DIAGNOSTIC; WHOLE10COORDINATE_RECOVERY_NOT_VERIFIED"
    save(output/"conditional-sensor-result.json",result)
    print(json.dumps({"relative":result["relative_parameter_errors"],"gates":result["gates"],
        "nfev":result["optimizer"]["evaluations"],"elapsed_s":result["elapsed_s"]}),flush=True)


def staged_mechanics_probe(native,model,source,output,budget):
    model = replace(model,q_min=-100.,q_max=100.)
    data = load_dataset(source)
    allocations = (budget//2,budget//8,budget-budget//2-budget//8)
    save(output/"staged-mechanics-contract.json",{"source":str(source),
        "role":"CONSUMED_PRISTINE_TRAIN_METHOD_DEVELOPMENT",
        "stages":[{"fields":GROUPS["mechanics_and_nuisance"],"max_nfev":allocations[0]},
            {"fields":GROUPS["reported_sensors"],"max_nfev":allocations[1]},
            {"fields":GROUPS["mechanics_and_nuisance"],"max_nfev":allocations[2]}],
        "total_max_nfev":budget,"initial":FIT_INITIAL,"bounds":FIT_BOUNDS,
        "stage_initialization":"preceding TRAIN fitted model; no truth or selection initialization",
        "method_basis":"conditional sensor diagnostic converges13eval while full trust region exhausts120; reset joint solve from sensor-optimized data candidate",
        "final_all10_coordinates_free":True,"quality_gates":RECOVERY_GATES,
        "loss":"unchanged native whole-run Huber residuals and masks",
        "fresh_noise_generated":False,"physical_actions":False})
    forward = forward_gate(native,model,data)
    save(output/"staged-mechanics-forward.json",forward)
    if forward["status"]!="PASS":
        raise ValueError("staged inverse requires independent forward prerequisite")
    began = time.monotonic()
    stages = []
    initial = None
    for index,(group,cap) in enumerate(zip(("mechanics_and_nuisance","reported_sensors","mechanics_and_nuisance"),allocations),1):
        label = source.stem+"-stage"+str(index)
        stage = fit_group(native,model,group,label,data,{label:data},output,cap,initial_model=initial)
        stages.append(stage)
        initial = replace(model,**{key:stage["model"][key] for key in FIT_TRUTH})
        print(json.dumps({"stage":index,"nfev":stage["optimizer"]["evaluations"],
            "relative":{key:stage["model"][key]/FIT_TRUTH[key]-1 for key in FIT_TRUTH},
            "termination":stage["gates"]["optimizer_termination_reason"]}),flush=True)
    result = stages[-1].copy()
    result["stages"] = stages
    result["total_optimizer_evaluations"] = sum(stage["optimizer"]["evaluations"] for stage in stages)
    result["total_residual_evaluations_including_jacobian_and_diagnostics"] = sum(
        stage["optimizer"]["residual_evaluations_including_jacobian_and_diagnostics"] for stage in stages)
    historical = []
    for name in ("development-pulses","development-alternating"):
        target,_ = generate(native,model,name,7001,False,"independent")
        pred = native.rollout(initial,target["t"],target["tx_t"],target["tx_A"],target["initial"])
        np.savez_compressed(output/("staged-predict-"+name+".npz"),t=target["t"],prediction=pred,
            q_new=target["q_new"],v_new=target["v_new"],current_new=target["current_new"])
        errors = measurement_errors(target,pred)
        historical.append({"case":name,"role":"HISTORICAL_REGRESSION","errors":errors,
            "trajectory_gate":trajectory_gate(target,errors),"forward":forward_gate(native,model,target)})
    result["historical_predictions"] = historical
    result["gates"] = dict(result["gates"])
    result["gates"]["historical_regression"] = "PASS" if all(row["trajectory_gate"]["passed"] for row in historical) else "FAIL"
    result["pristine_scope_verified"] = result["gates"]["optimizer_converged"] and all(
        result["gates"][key]=="PASS" for key in ("forward_numerics","synthetic_parameter_recovery","training_trajectory","historical_regression"))
    result["synthetic_scope_verified"] = False
    result["role"] = "STAGED_PRISTINE_DEVELOPMENT; FRESH_NOISY_RECOVERY_NOT_RUN"
    result["total_elapsed_s"] = time.monotonic()-began
    save(output/"staged-mechanics-result.json",result)
    print(json.dumps({"relative":result["relative_parameter_errors"],"gates":result["gates"],
        "total_nfev":result["total_optimizer_evaluations"],"elapsed_s":result["total_elapsed_s"]}),flush=True)


def independent_noisy_copy(base,seed):
    """Fresh independent draws in the original generator's declared order."""
    if base["noisy"] or base["generator"]!="INDEPENDENT_ORACLE":
        raise ValueError("independent noiseless latent source required")
    data = {key:value.copy() if isinstance(value,np.ndarray) else value for key,value in base.items()}
    rng = np.random.default_rng(seed)
    n = len(base["t"])
    data["q"] = np.round((base["q"]+rng.normal(0,FIXTURE["encoder_noise"],n))/
        FIXTURE["encoder_quantum"])*FIXTURE["encoder_quantum"]
    data["current"] = base["current"]+rng.normal(0,FIXTURE["current_noise"],n)
    gyro = base["truth"][:,3].copy()
    gyro[base["v_new"]] += rng.normal(0,FIXTURE["gyro_noise"],int(base["v_new"].sum()))
    data["v"] = gyro[(np.arange(n)//20)*20]
    data.update(seed=seed,noisy=True,noise_sources=np.asarray(NOISE_SOURCES,dtype="U32"))
    return data


def bin_likelihood_prerequisite(native,model,source,output):
    data = load_dataset(source)
    quantum,sigma = FIXTURE["encoder_quantum"],FIXTURE["encoder_noise"]
    b = quantum/(2*sigma)
    p0 = float(erf(b/np.sqrt(2.)))
    phi = np.exp(-b*b/2)/np.sqrt(2*np.pi)
    A = b*phi/p0
    center_switch = .01
    M = b+center_switch
    polynomial_bound = M**7+21*M**5+105*M**3+105*M
    density_bound = np.exp(-max(0.,b-center_switch)**2/2)/np.sqrt(2*np.pi)
    probability_remainder_bound = 2*polynomial_bound*density_bound*center_switch**8/math.factorial(8)/p0
    phase_grid = np.array([0.,1e-12,1e-9,1e-6,1e-4,.00999,.01,.01001,.1,1.,b,5.,10.,100.,1e4,1e6])
    reference_tolerance = 1e-12
    standardized = np.r_[-phase_grid[:0:-1],phase_grid]
    steps = sigma*np.array([1e-3,1e-4,1e-5,1e-6])
    save(output/"bin-likelihood-prerequisite-contract.json",{
        "source":str(source),"role":"CONSUMED TRAIN NUMERICAL ENCODER LIKELIHOOD PREREQUISITE; NO FIT OR NEW NOISE",
        "quantum_rad":quantum,"Gaussian_before_rounding_sigma_rad":sigma,
        "standardized_mean_offsets":standardized.tolist(),"absolute_mean_steps_rad":steps.tolist(),
        "center_policy":"Even probability series through d6 and log1p; analytic r(0)=0 and nonzero mean slope",
        "center_switch_abs_d":center_switch,"center_probability_ratio_remainder_bound_at_switch":probability_remainder_bound,
        "bound_policy":"Taylor remainder bounded by absolute Hermite7 polynomial times maximum Gaussian density over b±switch",
        "tail_policy":"Factored erfcx survival difference with log1mexp, no probability/deviance floors",
        "reference":"Independent quadrature of probability deficit near center and factored Gaussian interval integral in tail",
        "reference_quadrature_requested_tolerance":reference_tolerance,
        "residual_relative_error_target":1e-8,"derivative_relative_error_target":1e-5,
        "derivative_plateau":"At least two adjacent physical mean steps pass, retaining all smaller-step roundoff results",
        "native_prior":FIT_INITIAL,"physical_actions":False,"hashes":False})
    prior = replace(model,q_min=-100.,q_max=100.,**FIT_INITIAL)
    pred = native.rollout(prior,data["t"],data["tx_t"],data["tx_A"],data["initial"])
    all_r,all_slope = gaussian_bin_deviance(pred[data["q_new"],0],data["q"][data["q_new"]],quantum,sigma)
    def reference(d):
        h = abs(d)
        if h==0:
            return 0.
        if h<=.1:
            def integrand(x):
                y = b*h*x
                return x*np.exp(-h*h*x*x/2)*(np.sinh(y)/y if y else 1.)
            integral = quad(integrand,0.,1.,epsabs=reference_tolerance,epsrel=reference_tolerance)[0]
            deficit_ratio = 2*b*phi*h*h*integral/p0
            loss = -np.log1p(-deficit_ratio)
        elif h<b:
            probability = quad(lambda z:np.exp(-z*z/2)/np.sqrt(2*np.pi),
                -b-h,b-h,epsabs=reference_tolerance,epsrel=reference_tolerance)[0]
            loss = np.log(p0)-np.log(probability)
        else:
            t,w = h-b,2*b
            if t<1.:
                integral = quad(lambda x:np.exp(-t*x-x*x/2),0.,w,epsabs=reference_tolerance,epsrel=reference_tolerance)[0]
            else:
                integral = quad(lambda y:np.exp(-y-y*y/(2*t*t)),0.,min(t*w,50.),
                    epsabs=reference_tolerance,epsrel=reference_tolerance)[0]/t
            logmass = -t*t/2-.5*np.log(2*np.pi)+np.log(integral)
            loss = np.log(p0)-logmass
        return float(np.sign(d)*np.sqrt(2*loss))
    indices = np.unique(np.r_[np.linspace(0,len(data["t"])-1,17,dtype=int),
        np.argmin(pred[:,0]-data["q"]),np.argmax(pred[:,0]-data["q"])])
    observations = np.r_[data["q"][0]+np.zeros(len(standardized)),data["q"][indices]]
    means = np.r_[data["q"][0]+sigma*standardized,pred[indices,0]]
    r,slope = gaussian_bin_deviance(means,observations,quantum,sigma)
    represented = (means-observations)/sigma
    expected = np.array([reference(value) for value in represented])
    nonzero = expected!=0
    relative = np.zeros_like(r)
    relative[nonzero] = abs(r[nonzero]-expected[nonzero])/abs(expected[nonzero])
    rows = []
    derivatives = []
    for step in steps:
        plus,minus = means+step,means-step
        rp,_ = gaussian_bin_deviance(plus,observations,quantum,sigma)
        rm,_ = gaussian_bin_deviance(minus,observations,quantum,sigma)
        numerical = (rp-rm)/(plus-minus)
        derivatives.append(numerical)
        errors = abs(numerical-slope)/slope
        rows.append({"absolute_mean_step_rad":float(step),"max_relative_derivative_error":float(errors.max()),
            "per_point_relative_errors":errors.tolist()})
    derivative_pass = np.array([np.asarray(row["per_point_relative_errors"])<=1e-5 for row in rows])
    plateau = np.any(derivative_pass[:-1]&derivative_pass[1:],axis=0)
    endpoint_ell = A*center_switch**2
    relative_series_bound = probability_remainder_bound/(1-probability_remainder_bound)/endpoint_ell
    center_r,center_slope = gaussian_bin_deviance(0.,0.,quantum,sigma)
    passed = bool(relative.max()<=1e-8 and plateau.all() and relative_series_bound<1e-10 and
        float(center_r)==0 and abs(float(center_slope)-np.sqrt(2*A)/sigma)<1e-10 and
        np.isfinite(all_r).all() and np.isfinite(all_slope).all() and np.all(all_slope>0))
    result = {"status":"PASS" if passed else "FAIL","max_relative_residual_error_vs_quadrature":float(relative.max()),
        "per_point_residual_errors":relative.tolist(),"physical_mean_derivative_steps":rows,
        "derivative_plateau_passes":int(plateau.sum()),"derivative_points":len(plateau),
        "center_slope_per_rad":float(center_slope),"center_nonzero_slope":bool(center_slope>0),
        "center_series_relative_loss_remainder_bound":relative_series_bound,
        "source_native_encoder_samples":len(all_r),"all_source_prior_residuals_and_slopes_finite":True,
        "actual_source_prior_standardized_offset_range":[float(((pred[:,0]-data["q"])/sigma).min()),float(((pred[:,0]-data["q"])/sigma).max())],
        "new_noise_draws":0,"optimizer_fits":0,"qualification":"KNOWN PRESET ENCODER NOISE/QUANTIZER NUMERICS ONLY"}
    save(output/"bin-likelihood-prerequisite-results.json",result)
    np.savez_compressed(output/"bin-likelihood-prerequisite-raw.npz",mean=means,observed=observations,
        represented_standardized_offset=represented,residual=r,quadrature_residual=expected,
        analytic_mean_slope=slope,physical_fd=np.asarray(derivatives))
    print(json.dumps({key:result[key] for key in ("status","max_relative_residual_error_vs_quadrature",
        "derivative_plateau_passes","derivative_points","center_slope_per_rad",
        "center_series_relative_loss_remainder_bound","actual_source_prior_standardized_offset_range")}),flush=True)


def fresh_linear_mechanics_verification(native,model,source,output,budget):
    model = replace(model,q_min=-100.,q_max=100.)
    base = load_dataset(source)
    if base["input_name"]!="information-mechanics-octave-362s" or base["noisy"] or \
        base["generator"]!="INDEPENDENT_ORACLE":
        raise ValueError("fresh verification reuses the declared independent pristine362s latent fixture")
    n = len(base["t"])
    expected_tx_t,expected_tx_A = prescribed_input(base["input_name"])
    if not np.array_equal(base["t"],np.arange(362001,dtype=float)*.001) or \
        not np.array_equal(base["tx_t"],expected_tx_t) or \
        not np.array_equal(base["tx_A"],expected_tx_A) or \
        not np.array_equal(base["initial"],np.zeros(5)):
        raise ValueError("fresh source must retain exact declared362s times/TX/prehistory/zero initial state")
    if not np.array_equal(base["q_new"],np.ones(n,bool)) or \
        not np.array_equal(base["current_new"],np.ones(n,bool)) or \
        not np.array_equal(base["v_new"],np.arange(n)%20==0):
        raise ValueError("preserve original1kHz encoder/current and50Hz gyro freshness")
    validation_inputs = {name:{"noise_seed":seed,"duration_s":duration_for_input(name),
        "successful_tx_t":prescribed_input(name)[0].tolist(),
        "successful_tx_A":prescribed_input(name)[1].tolist()}
        for name,seed in FRESH_MECHANICS_VALIDATION.items()}
    save(output/"fresh-linear-mechanics-contract.json",{
        "role":"FROZEN INDEPENDENT-NOISE JOINT10 VERIFICATION",
        "latent_train_source":str(source),"train_input":base["input_name"],
        "fresh_train_seeds":FRESH_MECHANICS_SEEDS,"train_excitation_designs":1,
        "method_frozen":"whole-run bounded TRF squared interval residual; original nontruth start every seed",
        "initial":FIT_INITIAL,"bounds":FIT_BOUNDS,"free_fields":GROUPS["mechanics_and_nuisance"],
        "max_nfev_per_seed":budget,"original_derivative_steps":"1e-6 times fixed original characteristic scales",
        "sampling":"Encoder/current1kHz all fresh; gyro50Hz; held gyro samples excluded by native mask",
        "noise_sources":NOISE_SOURCES,"noise_contract":"Independent Gaussian encoder/current/new gyro draws; encoder rounded after additive noise; original draw order and magnitudes",
        "sensor_scales":sensor_scales({"noisy":True}),"quality_gates":RECOVERY_GATES,
        "encoder_objective_caveat":"Original flat encoder-bin squared residual plus combined variance scale is an empirical surrogate, not integrated Gaussian bin likelihood",
        "validation_inputs":validation_inputs,"validation_noise_levels":["pristine","original_full_noise"],
        "validation_order":"Freeze all fitted vectors before generating untouched validation observations; no fit or winner selection uses validation",
        "validation_product":"Product A: prescribed-input held-out plant prediction; Product B pre-run closed-loop forecast NOT_RUN",
        "historical_inputs":["development-pulses","development-alternating"],
        "gauges":"Known synthetic unit gains/current scale, fixed biases/constant load/static thresholds/common clock and one zero acquisition state; not measured physical facts",
        "statistical_scope":"Prescribed input; no closed-loop or universal statistical/unbiasedness/covariance claim",
        "physical_actions":False,"promotion_blocked":True,"hashes":False})
    began = time.monotonic()
    forward = forward_gate(native,model,base)
    save(output/"fresh-linear-mechanics-forward.json",forward)
    if forward["status"]!="PASS":
        raise ValueError("fresh inverse requires unchanged independent forward gates")
    results = []
    for seed in FRESH_MECHANICS_SEEDS:
        data = independent_noisy_copy(base,seed)
        label = base["input_name"]+"-noisy-"+str(seed)
        np.savez_compressed(output/(label+".npz"),**data)
        result = fit_group(native,model,"mechanics_and_nuisance",label,data,
            {label:data},output,budget,loss="linear_independent_noise")
        results.append(result)
        save(output/"fresh-linear-mechanics-train-results.json",results)
        print(json.dumps({"seed":seed,"nfev":result["optimizer"]["evaluations"],
            "relative":result["relative_parameter_errors"],"gates":result["gates"]}),flush=True)
    save(output/"frozen-fitted-vectors-before-validation.json",{
        "all_train_fits_complete":True,"validation_observations_generated":False,
        "vectors":[{"seed":seed,"parameters":{key:result["model"][key] for key in FIT_TRUTH}}
            for seed,result in zip(FRESH_MECHANICS_SEEDS,results)]})
    targets = {}
    for name,seed in FRESH_MECHANICS_VALIDATION.items():
        target,events = generate(native,model,name,0,False,"independent")
        save(output/(name+"-events.json"),events)
        for label,data in ((name+"-pristine",target),
            (name+"-noisy-"+str(seed),independent_noisy_copy(target,seed))):
            targets[label] = ("HELD_OUT_PRESCRIBED_INPUT_PLANT_PREDICTION",data)
            np.savez_compressed(output/(label+".npz"),**data)
    for name in ("development-pulses","development-alternating"):
        data,_ = generate(native,model,name,1601,False,"independent")
        targets[name+"-pristine"] = ("HISTORICAL_REGRESSION",data)
    for result in results:
        fitted = replace(model,**{key:result["model"][key] for key in FIT_TRUTH})
        for label,(role,data) in targets.items():
            prediction = native.rollout(fitted,data["t"],data["tx_t"],data["tx_A"],data["initial"])
            np.savez_compressed(output/(result["training_case"]+"-predict-"+label+".npz"),
                t=data["t"],prediction=prediction,q_new=data["q_new"],v_new=data["v_new"],current_new=data["current_new"])
            errors = measurement_errors(data,prediction)
            result["predictions"].append({"case":label,"role":role,"errors":errors,
                "trajectory_gate":trajectory_gate(data,errors),"forward":forward_gate(native,model,data)})
        for role,flag in (("HISTORICAL_REGRESSION","historical_regression"),
            ("HELD_OUT_PRESCRIBED_INPUT_PLANT_PREDICTION","held_out_prescribed_input_prediction")):
            rows = [row for row in result["predictions"] if row["role"]==role]
            result["gates"][flag] = "PASS" if all(row["trajectory_gate"]["passed"] and
                row["forward"]["status"]=="PASS" for row in rows) else "FAIL"
        result["synthetic_scope_verified"] = result["gates"]["optimizer_converged"] and all(
            result["gates"][key]=="PASS" for key in ("data_integrity","forward_numerics",
            "synthetic_parameter_recovery","training_trajectory","historical_regression","held_out_prescribed_input_prediction"))
        result["inverse_probe_passed"] = result["synthetic_scope_verified"]
    save(output/"fresh-linear-mechanics-results.json",results)
    summary = {"cases":len(results),"train_excitation_designs":1,
        "fresh_validation_excitation_designs":2,"scope_passes":sum(row["synthetic_scope_verified"] for row in results),
        "optimizer_evaluations":[row["optimizer"]["evaluations"] for row in results],
        "residual_evaluations":[row["optimizer"]["residual_evaluations_including_jacobian_and_diagnostics"] for row in results],
        "recovery_passes":sum(row["gates"]["synthetic_parameter_recovery"]=="PASS" for row in results),
        "held_out_prescribed_input_prediction_passes":sum(row["trajectory_gate"]["passed"] for result in results
            for row in result["predictions"] if row["role"]=="HELD_OUT_PRESCRIBED_INPUT_PLANT_PREDICTION"),
        "product_b_pre_run_closed_loop_forecast":"NOT_RUN",
        "total_elapsed_s":time.monotonic()-began,"physical_stage3a":"NOT_RUN",
        "physical_stage3b":"NOT_RUN","deployment_authorized":False}
    save(output/"fresh-linear-mechanics-summary.json",summary)
    print(json.dumps(summary),flush=True)


def bin_likelihood_development(native,model,source,prerequisite,output,budget):
    """One consumed TRAIN loss discriminator; all validation is now regression."""
    model = replace(model,q_min=-100.,q_max=100.)
    data = load_dataset(source)
    numerical_contract = json.loads((prerequisite/"bin-likelihood-prerequisite-contract.json").read_text())
    numerical_result = json.loads((prerequisite/"bin-likelihood-prerequisite-results.json").read_text())
    if numerical_result["status"]!="PASS" or Path(numerical_contract["source"]).resolve()!=source.resolve() or \
        numerical_contract["quantum_rad"]!=FIXTURE["encoder_quantum"] or \
        numerical_contract["Gaussian_before_rounding_sigma_rad"]!=FIXTURE["encoder_noise"]:
        raise ValueError("passing same-source frozen encoder numerical prerequisite required")
    expected_t = np.arange(362001,dtype=float)*.001
    expected_tx_t,expected_tx_A = prescribed_input("information-mechanics-octave-362s")
    if data["seed"]!=7103 or data["input_name"]!="information-mechanics-octave-362s" or \
        not data["noisy"] or data["generator"]!="INDEPENDENT_ORACLE" or \
        set(map(str,data["noise_sources"]))!=set(NOISE_SOURCES) or \
        not np.array_equal(data["t"],expected_t) or \
        not np.array_equal(data["tx_t"],expected_tx_t) or \
        not np.array_equal(data["tx_A"],expected_tx_A) or \
        not np.array_equal(data["initial"],np.zeros(5)) or \
        not np.array_equal(data["q_new"],np.ones(len(expected_t),bool)) or \
        not np.array_equal(data["current_new"],np.ones(len(expected_t),bool)) or \
        not np.array_equal(data["v_new"],np.arange(len(expected_t))%20==0):
        raise ValueError("exact-bin ablation requires unchanged consumed7103 whole362s fixture and native masks")
    target_paths = {name+suffix:source.parent/(name+suffix+".npz")
        for name,seed in FRESH_MECHANICS_VALIDATION.items()
        for suffix in ("-pristine","-noisy-"+str(seed))}
    targets = {label:load_dataset(path) for label,path in target_paths.items()}
    save(output/"bin-likelihood-development-contract.json",{
        "role":"CONSUMED7103 TRAIN EXACT-ENCODER-LIKELIHOOD DEVELOPMENT ABLATION",
        "training_source":str(source),"numerical_prerequisite":str(prerequisite),
        "numerical_prerequisite_status":numerical_result["status"],
        "free_fields":GROUPS["mechanics_and_nuisance"],"initial":FIT_INITIAL,"bounds":FIT_BOUNDS,
        "max_nfev":budget,"whole_train_duration_s":362.,"initial_state_count":1,"initial_state":data["initial"].tolist(),
        "objective":"Signed sqrt(2*(logPbin_at_bin_center-logPbin_at_predicted_mean)) encoder residual; Gaussian gyro/current residuals",
        "encoder_pre_rounding_sigma_rad":FIXTURE["encoder_noise"],"encoder_quantum_rad":FIXTURE["encoder_quantum"],
        "likelihood_scope":"Known preset independent Gaussian-before-rounding encoder and Gaussian gyro/current, fixed quantizer; fixture conditional",
        "method":"Bounded TRF linear loss, original nontruth supplied initializer, fixed original characteristic scales and absolute central derivative steps; no restart",
        "sampling":"Encoder/current1kHz all fresh; gyro50Hz native mask excludes held readings",
        "quality_gates":RECOVERY_GATES,"engineering_encoder_scale":"Original combined encoder scale retained only for unchanged prediction gates",
        "known_gauges":"Unit command/current gain, zero sensor biases, fixed constant load/static thresholds/common clock and stationary prehistory state",
        "regression_sources":{label:str(path) for label,path in target_paths.items()},
        "other_regression_inputs":["development-pulses","development-alternating"],
        "validation_status":"All encountered validation inputs/noise are consumed regression; no fresh final ProductA or ProductB forecast",
        "new_noise_draws":0,"production_objective_default_changed":False,"physical_actions":False,
        "promotion_blocked":True,"hashes":False})
    began = time.monotonic()
    forward = forward_gate(native,model,data)
    save(output/"bin-likelihood-training-forward.json",forward)
    if forward["status"]!="PASS":
        raise ValueError("exact-bin inverse requires unchanged independent forward gates")
    label = source.stem
    try:
        result = fit_group(native,model,"mechanics_and_nuisance",label,data,
            {label:data},output,budget,loss="linear_gaussian_bins")
    except FloatingPointError as exc:
        save(output/"bin-likelihood-development-precision-failure.json",{
            "status":"NUMERICAL_PRECISION_FAILURE","error":str(exc),"elapsed_s":time.monotonic()-began,
            "physical_model_inadequacy":False,"restarted":False,"fresh_noise":False})
        raise
    save(output/"frozen-fit-before-regression.json",{
        "parameters":{key:result["model"][key] for key in FIT_TRUTH},
        "role":"TRAIN ONLY FIT FROZEN; CONSUMED REGRESSION CANNOT SELECT/REFIT VECTOR"})
    for name in ("development-pulses","development-alternating"):
        targets[name+"-pristine"],_ = generate(native,model,name,1601,False,"independent")
    fitted = replace(model,**{key:result["model"][key] for key in FIT_TRUTH})
    for target_label,target in targets.items():
        prediction = native.rollout(fitted,target["t"],target["tx_t"],target["tx_A"],target["initial"])
        np.savez_compressed(output/("predict-"+target_label+".npz"),t=target["t"],prediction=prediction,
            q_new=target["q_new"],v_new=target["v_new"],current_new=target["current_new"])
        errors = measurement_errors(target,prediction)
        result["predictions"].append({"case":target_label,"role":"CONSUMED_REGRESSION",
            "errors":errors,"trajectory_gate":trajectory_gate(target,errors),"forward":forward_gate(native,model,target)})
    regression = [row for row in result["predictions"] if row["role"]=="CONSUMED_REGRESSION"]
    result["gates"]["historical_regression"] = "PASS" if all(row["trajectory_gate"]["passed"] and
        row["forward"]["status"]=="PASS" for row in regression) else "FAIL"
    result["gates"]["held_out_prescribed_input_prediction"] = "NOT_RUN_FRESH_AFTER_REPAIR"
    result["development_quality_passed"] = result["gates"]["optimizer_converged"] and all(
        result["gates"][key]=="PASS" for key in ("data_integrity","forward_numerics",
        "synthetic_parameter_recovery","training_trajectory","historical_regression"))
    result["inverse_probe_passed"] = result["development_quality_passed"]
    result["synthetic_scope_verified"] = False
    result["role"] = "CONSUMED7103 DEVELOPMENT; FRESH FINAL NOISY RECOVERY/PREDICTION NOT_RUN"
    result["total_elapsed_s"] = time.monotonic()-began
    save(output/"bin-likelihood-development-result.json",result)
    compact = {key:result[key] for key in ("role","model","free_fields","relative_parameter_errors",
        "recovery_limit","gates","forward","predictions","development_quality_passed","synthetic_scope_verified","total_elapsed_s")}
    compact["optimizer"] = {key:result["optimizer"][key] for key in ("success","evaluations","jacobian_evaluations",
        "message","termination_status","residual_evaluations_including_jacobian_and_diagnostics","optimality",
        "coordinate_scales","absolute_derivative_steps","coordinate_values","parameter_bound_hits","elapsed_s",
        "loss","loss_role","encoder_objective","encoder_pre_rounding_noise_sigma_rad","encoder_quantum_rad")}
    save(output/"bin-likelihood-development-compact.json",compact)
    print(json.dumps({"relative":result["relative_parameter_errors"],"gates":result["gates"],
        "nfev":result["optimizer"]["evaluations"],"development_quality_passed":result["development_quality_passed"],
        "elapsed_s":result["total_elapsed_s"]}),flush=True)


def expected_encoder_information(mean):
    """Exact preset-bin expected scalar Fisher information, before noise draws."""
    quantum,sigma = FIXTURE["encoder_quantum"],FIXTURE["encoder_noise"]
    phase = np.asarray(mean)-np.round(np.asarray(mean)/quantum)*quantum
    p0 = float(erf(quantum/(2*sigma*np.sqrt(2.))))
    information,mass = np.zeros_like(phase),np.zeros_like(phase)
    for offset in range(-3,4):
        residual,slope = gaussian_bin_deviance(phase,offset*quantum,quantum,sigma)
        probability = np.exp(np.log(p0)-.5*residual**2)
        information += probability*(residual*slope)**2
        mass += probability
    return information,mass


def bin_final_information(native,model,output):
    """One frozen candidate input at a nontruth prior; no observed data or fit."""
    fields = GROUPS["mechanics_and_nuisance"]
    prior = replace(model,q_min=-100.,q_max=100.,**FIT_INITIAL)
    targets = {key:FIT_TRUTH[key]*.05/4 for key in fields}
    save(output/"bin-final-information-contract.json",{
        "method_version":"adr0022.joint10.known-gaussian-bins/1","input_name":BIN_FINAL_TRAIN,
        "candidate_inputs":1,"duration_s":720.,"mechanical_prefix_s":48.,
        "high_frequency_Hz":[4.1,6.7,9.3],"synthetic_current_cap_A":.33,
        "input_reason":"Different slow mechanical dwell/reversal cycle and higher timing-sensitive bands;720s declared before forecast to target4sigma margin at frozen5percent recovery gates",
        "prior":FIT_INITIAL,"free_fields":fields,"standard_error_targets_5percent_over4":targets,
        "expected_encoder_information":"Sum Pbin*(dlogPbin/dmean)^2 over seven bins at each prior quantization phase, known preset Gaussian-before-rounding sigma/Q",
        "other_information":"Original Gaussian native gyro50Hz and current1kHz noise scales",
        "derivative_policy":"Original characteristic×1e-6 central steps compared with×1e-7; same coarse steps at half native max_step",
        "native_max_steps_s":[prior.max_step,prior.max_step/2],"derivative_consistency_target":.01,
        "reserved_final_noise_seeds":BIN_FINAL_SEEDS,"reserved_validation_inputs":BIN_FINAL_VALIDATION,
        "one_train_input_three_noise_realizations":True,"source_admission_required_before_generation":True,
        "information_scope":"Local expected preset-noise information at supplied nontruth prior; no global uniqueness, optimizer convergence or physical confidence proof",
        "no_observations_noise_or_inverse_fit_generated":True,"physical_actions":False})
    began = time.monotonic()
    t = np.arange(720001,dtype=float)*.001
    tx_t,tx_A = prescribed_input(BIN_FINAL_TRAIN)
    mask = np.arange(len(t))%20==0
    center = native.rollout(prior,t,tx_t,tx_A,np.zeros(5))
    encoder_information,mass = expected_encoder_information(center[:,0])
    q_weight = np.sqrt(encoder_information)
    scales = sensor_scales({"noisy":True})
    def columns(candidate,factors):
        jacobians = []
        for factor in factors:
            vectors = []
            for field in fields:
                characteristic = max(abs(getattr(prior,field)),.05*np.ptp(FIT_BOUNDS[field]))
                h = characteristic*factor
                plus = native.rollout(replace(candidate,**{field:getattr(candidate,field)+h}),t,tx_t,tx_A,np.zeros(5))
                minus = native.rollout(replace(candidate,**{field:getattr(candidate,field)-h}),t,tx_t,tx_A,np.zeros(5))
                difference = (plus-minus)/(2*h)
                vectors.append(np.r_[difference[:,0]*q_weight,difference[mask,3]/scales["sigma_v"],
                    difference[:,4]/scales["sigma_current"]])
            jacobians.append(np.column_stack(vectors))
        return jacobians
    original,finer = columns(prior,(1e-6,1e-7))
    refined, = columns(replace(prior,max_step=prior.max_step/2),(1e-6,))
    norms = np.linalg.norm(original,axis=0)
    changes = np.linalg.norm(finer-original,axis=0)/norms
    refinement = np.linalg.norm(refined-original,axis=0)/norms
    def precision(jac):
        column_norms = np.linalg.norm(jac,axis=0)
        normalized = jac/column_norms
        singular = np.linalg.svd(normalized,compute_uv=False)
        covariance = np.linalg.inv(normalized.T@normalized)/column_norms[:,None]/column_norms[None,:]
        errors = np.sqrt(np.diag(covariance))
        return errors,singular,covariance/errors[:,None]/errors[None,:]
    errors,singular,correlation = precision(original)
    refined_errors,_,_ = precision(refined)
    unsupported = [key for key,error in zip(fields,np.maximum(errors,refined_errors)) if error>targets[key]]
    stable = bool(max(changes.max(),refinement.max())<=.01)
    mass_ok = bool(np.max(abs(mass-1))<=1e-12 and np.all(encoder_information>0) and
        np.max(encoder_information)*FIXTURE["encoder_noise"]**2<=1+1e-10)
    result = {"information_status":"UNSTABLE_DERIVATIVES" if not stable else "INVALID_EXPECTED_INFORMATION" if not mass_ok else
        "NEEDS_MORE_INFORMATION" if unsupported else "PRIOR_PRECISION_TARGET_MET",
        "input_name":BIN_FINAL_TRAIN,"prior":FIT_INITIAL,"free_fields":fields,
        "local_expected_standard_errors":dict(zip(fields,map(float,errors))),
        "refined_native_standard_errors":dict(zip(fields,map(float,refined_errors))),
        "standard_error_targets_5percent_over4":targets,"unsupported_fields":unsupported,
        "derivative_consistency":"PASS" if stable else "FAIL",
        "derivative_relative_change_decade":dict(zip(fields,map(float,changes))),
        "derivative_relative_change_native_refinement":dict(zip(fields,map(float,refinement))),
        "normalized_singular_values":singular.tolist(),"normalized_rank":int(np.sum(singular>singular[0]*1e-8)),
        "correlation":correlation.tolist(),"encoder_retained_probability_max_mass_error":float(np.max(abs(mass-1))),
        "encoder_expected_information_range_per_rad2":[float(encoder_information.min()),float(encoder_information.max())],
        "sampling_counts":{"encoder":len(t),"gyro":int(mask.sum()),"current":len(t)},
        "noise_draws":0,"fits":0,"elapsed_s":time.monotonic()-began,
        "interpretation":"Fixture-conditional expected local variance forecast only; empirical fresh inverse recovery and independent forward prerequisites remain pending"}
    save(output/"bin-final-information-result.json",result)
    print(json.dumps({key:result[key] for key in ("information_status","local_expected_standard_errors",
        "refined_native_standard_errors","unsupported_fields","derivative_consistency","elapsed_s")}),flush=True)


def bin_final_source_admission(evidence_root):
    """Reject reserved identities already present in retained dataset metadata."""
    reserved_seeds = set(BIN_FINAL_SEEDS)|set(BIN_FINAL_VALIDATION.values())
    reserved_inputs = {BIN_FINAL_TRAIN,*BIN_FINAL_VALIDATION}
    datasets,with_seed,with_input,collisions = 0,0,0,[]
    for path in evidence_root.rglob("*.npz"):
        datasets += 1
        with np.load(path,allow_pickle=False) as archive:
            seed = int(archive["seed"]) if "seed" in archive.files and archive["seed"].ndim==0 else None
            input_name = str(archive["input_name"]) if "input_name" in archive.files else None
        with_seed += seed is not None
        with_input += input_name is not None
        if seed in reserved_seeds or input_name in reserved_inputs:
            collisions.append({"path":str(path),"seed":seed,"input_name":input_name})
    if collisions:
        raise ValueError("reserved final identities were already retained: "+json.dumps(collisions))
    return {"status":"PASS","evidence_root":str(evidence_root),"retained_npz_files_inspected":datasets,
        "datasets_with_scalar_seed":with_seed,"datasets_with_input_name":with_input,
        "reserved_seeds":sorted(reserved_seeds),"reserved_inputs":sorted(reserved_inputs),
        "retained_dataset_identity_collisions":0,
        "scope":"Retained dataset metadata absence; separate declaration reserves IDs before any new generation"}


def admit_bin_final_dataset(data,input_name,noisy,seed):
    expected_t = np.arange(int(round(duration_for_input(input_name)/.001))+1,dtype=float)*.001
    tx_t,tx_A = prescribed_input(input_name)
    if data["input_name"]!=input_name or data["noisy"]!=noisy or data["seed"]!=seed or \
        data["generator"]!="INDEPENDENT_ORACLE" or \
        set(map(str,data["noise_sources"]))!=set(NOISE_SOURCES if noisy else ()) or \
        not np.array_equal(data["t"],expected_t) or \
        not np.array_equal(data["tx_t"],tx_t) or not np.array_equal(data["tx_A"],tx_A) or \
        not np.array_equal(data["initial"],np.zeros(5)) or \
        not np.array_equal(data["q_new"],np.ones(len(expected_t),bool)) or \
        not np.array_equal(data["current_new"],np.ones(len(expected_t),bool)) or \
        not np.array_equal(data["v_new"],np.arange(len(expected_t))%20==0):
        raise ValueError("final dataset must match frozen input/times/seed/noise/native masks/zero initial state")


def bin_final_verification(native,model,information,protocol,method,output,budget):
    model = replace(model,q_min=-100.,q_max=100.)
    frozen_protocol = json.loads(protocol.read_text())
    frozen_method = json.loads(method.read_text())
    info = json.loads((information/"bin-final-information-result.json").read_text())
    info_contract = json.loads((information/"bin-final-information-contract.json").read_text())
    version = "adr0022.joint10.known-gaussian-bins/1"
    if budget!=120 or frozen_method["method_version"]!=version or frozen_protocol["method_version"]!=version or \
        frozen_method["encoder_pre_rounding_sigma_rad"]!=FIXTURE["encoder_noise"] or \
        frozen_method["encoder_quantum_rad"]!=FIXTURE["encoder_quantum"] or \
        frozen_protocol["fresh_training"]["input_name"]!=BIN_FINAL_TRAIN or \
        frozen_protocol["fresh_training"]["noise_seeds"]!=list(BIN_FINAL_SEEDS) or \
        frozen_protocol["numerical_q_domain_rad"]!=[model.q_min,model.q_max] or \
        {name:row["noise_seed"] for name,row in frozen_protocol["fresh_validation"].items()
            if isinstance(row,dict)}!=BIN_FINAL_VALIDATION or \
        info["information_status"]!="PRIOR_PRECISION_TARGET_MET" or info["derivative_consistency"]!="PASS" or \
        info["prior"]!=FIT_INITIAL or info["noise_draws"]!=0 or info["fits"]!=0 or \
        info["normalized_rank"]!=len(FIT_TRUTH) or \
        info["standard_error_targets_5percent_over4"]!={key:FIT_TRUTH[key]*.05/4 for key in FIT_TRUTH} or \
        info_contract["input_name"]!=BIN_FINAL_TRAIN or \
        info_contract["reserved_final_noise_seeds"]!=list(BIN_FINAL_SEEDS) or \
        info_contract["reserved_validation_inputs"]!=BIN_FINAL_VALIDATION:
        raise ValueError("same frozen method/protocol and passing before-observation prior-information required")
    for key in FIT_TRUTH:
        values = [info["local_expected_standard_errors"][key],info["refined_native_standard_errors"][key]]
        derivative_changes = [info["derivative_relative_change_decade"][key],
            info["derivative_relative_change_native_refinement"][key]]
        if not np.isfinite(values+derivative_changes).all() or min(values)<=0 or \
            max(values)>info["standard_error_targets_5percent_over4"][key] or max(derivative_changes)>.01:
            raise ValueError("finite full-rank prior precision and derivative consistency required")
    evidence_root = Path(__file__).resolve().parents[2]/"run"/"adr0022-stage2"
    admission = bin_final_source_admission(evidence_root)
    save(output/"fresh-source-admission.json",admission)
    save(output/"frozen-final-protocol.json",frozen_protocol)
    save(output/"frozen-final-method.json",frozen_method)
    save(output/"frozen-prior-information.json",info)
    save(output/"frozen-prior-information-contract.json",info_contract)
    save(output/"numerical-domain-and-order-receipt.json",{
        "q_min_rad":model.q_min,"q_max_rad":model.q_max,
        "domain_role":"Synthetic numerical rollout domain; no physical travel qualification",
        "admission_protocol_method_prior_information_saved_before_generation":True,
        "observations_generated":False,"fits_performed":False})
    began = time.monotonic()
    print(json.dumps({"stage":"INDEPENDENT_TRAIN_GENERATION","input":BIN_FINAL_TRAIN,
        "noise_draws":0,"source_admission":admission["status"]}),flush=True)
    base,events = generate(native,model,BIN_FINAL_TRAIN,0,False,"independent")
    admit_bin_final_dataset(base,BIN_FINAL_TRAIN,False,0)
    np.savez_compressed(output/(BIN_FINAL_TRAIN+"-independent-latent.npz"),**base)
    save(output/"independent-train-events.json",events)
    forward = forward_gate(native,model,base)
    save(output/"independent-train-forward.json",forward)
    if forward["status"]!="PASS":
        raise ValueError("fresh TRAIN noise blocked by unchanged independent forward gates")
    results = []
    for seed in BIN_FINAL_SEEDS:
        data = independent_noisy_copy(base,seed)
        admit_bin_final_dataset(data,BIN_FINAL_TRAIN,True,seed)
        label = BIN_FINAL_TRAIN+"-noisy-"+str(seed)
        np.savez_compressed(output/(label+".npz"),**data)
        result = fit_group(native,model,"mechanics_and_nuisance",label,data,
            {label:data},output,budget,loss="linear_gaussian_bins")
        results.append(result)
        save(output/"fresh-final-train-results.json",results)
        save(output/"fresh-final-train-checkpoint.json",{
            "method_version":version,"declared_cases":len(BIN_FINAL_SEEDS),"completed_train_fits":len(results),
            "validation":"NOT_RUN_AT_TRAIN_CHECKPOINT",
            "cases":[{"case":row["training_case"],"model":{key:row["model"][key] for key in FIT_TRUTH},
                "relative_parameter_errors":row["relative_parameter_errors"],"gates":row["gates"],
                "optimizer":{key:row["optimizer"][key] for key in ("evaluations","jacobian_evaluations",
                    "residual_evaluations_including_jacobian_and_diagnostics","message","optimality","elapsed_s")}}
                for row in results]})
        print(json.dumps({"stage":"TRAIN_FIT_COMPLETE","seed":seed,
            "nfev":result["optimizer"]["evaluations"],"relative":result["relative_parameter_errors"],
            "gates":result["gates"]}),flush=True)
    save(output/"frozen-vectors-before-validation.json",{
        "all_train_fits_complete":True,"validation_observations_generated":False,
        "vectors":[{"seed":seed,"parameters":{key:result["model"][key] for key in FIT_TRUTH}}
            for seed,result in zip(BIN_FINAL_SEEDS,results)]})
    for result in results:
        fitted = replace(model,**{key:result["model"][key] for key in FIT_TRUTH})
        for name in BIN_FINAL_VALIDATION:
            t = np.arange(int(round(duration_for_input(name)/.001))+1,dtype=float)*.001
            tx_t,tx_A = prescribed_input(name)
            prediction = native.rollout(fitted,t,tx_t,tx_A,np.zeros(5))
            np.savez_compressed(output/(result["training_case"]+"-preobservation-predict-"+name+".npz"),
                t=t,prediction=prediction,q_new=np.ones(len(t),bool),v_new=np.arange(len(t))%20==0,
                current_new=np.ones(len(t),bool))
    save(output/"predictions-frozen-before-validation-observations.json",{
        "all_fitted_vectors_frozen":True,"predictions_saved":len(results)*len(BIN_FINAL_VALIDATION),
        "validation_observations_generated":False,
        "product":"ProductA prescribed-input plant predictions; ProductB own-future closed-loop forecast NOT_RUN"})
    targets = {}
    for name,seed in BIN_FINAL_VALIDATION.items():
        base_target,events = generate(native,model,name,0,False,"independent")
        admit_bin_final_dataset(base_target,name,False,0)
        save(output/(name+"-events.json"),events)
        target_forward = forward_gate(native,model,base_target)
        for label,data in ((name+"-pristine",base_target),(name+"-noisy-"+str(seed),independent_noisy_copy(base_target,seed))):
            admit_bin_final_dataset(data,name,data["noisy"],data["seed"])
            targets[label] = ("HELD_OUT_PRESCRIBED_INPUT_PLANT_PREDICTION",name,data,target_forward)
            np.savez_compressed(output/(label+".npz"),**data)
    for name in ("development-pulses","development-alternating"):
        data,_ = generate(native,model,name,1601,False,"independent")
        targets[name+"-pristine"] = ("HISTORICAL_REGRESSION",name,data,forward_gate(native,model,data))
    for result in results:
        fitted = replace(model,**{key:result["model"][key] for key in FIT_TRUTH})
        for label,(role,name,data,target_forward) in targets.items():
            if role=="HELD_OUT_PRESCRIBED_INPUT_PLANT_PREDICTION":
                with np.load(output/(result["training_case"]+"-preobservation-predict-"+name+".npz"),allow_pickle=False) as archive:
                    prediction = archive["prediction"]
                    if not np.array_equal(archive["t"],data["t"]) or not np.array_equal(archive["v_new"],data["v_new"]):
                        raise ValueError("frozen prediction must preserve native validation times/masks")
            else:
                prediction = native.rollout(fitted,data["t"],data["tx_t"],data["tx_A"],data["initial"])
                np.savez_compressed(output/(result["training_case"]+"-predict-"+label+".npz"),
                    t=data["t"],prediction=prediction,q_new=data["q_new"],v_new=data["v_new"],current_new=data["current_new"])
            errors = measurement_errors(data,prediction)
            result["predictions"].append({"case":label,"role":role,"errors":errors,
                "trajectory_gate":trajectory_gate(data,errors),"forward":target_forward,
                "prediction_saved_before_observations":role=="HELD_OUT_PRESCRIBED_INPUT_PLANT_PREDICTION"})
        for role,flag in (("HISTORICAL_REGRESSION","historical_regression"),
            ("HELD_OUT_PRESCRIBED_INPUT_PLANT_PREDICTION","held_out_prescribed_input_prediction")):
            rows = [row for row in result["predictions"] if row["role"]==role]
            result["gates"][flag] = "PASS" if all(row["trajectory_gate"]["passed"] and row["forward"]["status"]=="PASS" for row in rows) else "FAIL"
        result["synthetic_scope_verified"] = result["gates"]["optimizer_converged"] and all(
            result["gates"][key]=="PASS" for key in ("data_integrity","forward_numerics",
            "synthetic_parameter_recovery","training_trajectory","historical_regression","held_out_prescribed_input_prediction"))
        result["inverse_probe_passed"] = result["synthetic_scope_verified"]
    save(output/"fresh-final-results.json",results)
    summary = {"method_version":version,"cases":len(results),"train_excitation_designs":1,
        "fresh_validation_inputs":2,"scope_passes":sum(row["synthetic_scope_verified"] for row in results),
        "optimizer_evaluations":[row["optimizer"]["evaluations"] for row in results],
        "residual_evaluations":[row["optimizer"]["residual_evaluations_including_jacobian_and_diagnostics"] for row in results],
        "recovery_passes":sum(row["gates"]["synthetic_parameter_recovery"]=="PASS" for row in results),
        "held_out_prediction_passes":sum(row["trajectory_gate"]["passed"] for result in results for row in result["predictions"]
            if row["role"]=="HELD_OUT_PRESCRIBED_INPUT_PLANT_PREDICTION"),
        "historical_regression_passes":sum(row["trajectory_gate"]["passed"] for result in results for row in result["predictions"]
            if row["role"]=="HISTORICAL_REGRESSION"),"total_elapsed_s":time.monotonic()-began,
        "product_b_pre_run_closed_loop_forecast":"NOT_RUN","physical_stage3a":"NOT_RUN",
        "physical_stage3b":"NOT_RUN","production_objective_default_changed":False,"deployment_authorized":False}
    save(output/"fresh-final-summary.json",summary)
    print(json.dumps(summary),flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--library",type=Path,required=True)
    parser.add_argument("--output-dir",type=Path,required=True)
    parser.add_argument("--generator",choices=("native","independent"),default="independent")
    parser.add_argument("--partition",choices=("development","final"),default="development")
    parser.add_argument("--groups",nargs="+",choices=tuple(GROUPS),default=list(TRUTH))
    parser.add_argument("--max-nfev",type=int,default=120)
    parser.add_argument("--diagnose-gyro-source",type=Path)
    parser.add_argument("--information-design",action="store_true")
    parser.add_argument("--information-training",action="store_true")
    parser.add_argument("--fresh-recovery-verification",action="store_true")
    parser.add_argument("--retained-joint-source",type=Path)
    parser.add_argument("--fresh-joint-verification",action="store_true")
    parser.add_argument("--joint-duration-s",type=float,default=210.)
    parser.add_argument("--joint-prior-derivatives",action="store_true")
    parser.add_argument("--retained-mechanics-source",type=Path)
    parser.add_argument("--mechanics-gauges-source",type=Path)
    parser.add_argument("--prefix-initialization",action="store_true")
    parser.add_argument("--moving-initialization",action="store_true")
    parser.add_argument("--mechanics-information-design",action="store_true")
    parser.add_argument("--mechanics-duration-s",type=float,default=242.)
    parser.add_argument("--mechanics-gyro-derivatives",action="store_true")
    parser.add_argument("--mechanics-pristine-probe",action="store_true")
    parser.add_argument("--sensor-block-source",type=Path)
    parser.add_argument("--sensor-block-fit",type=Path)
    parser.add_argument("--staged-mechanics-source",type=Path)
    parser.add_argument("--linear-mechanics-source",type=Path)
    parser.add_argument("--fresh-linear-mechanics-source",type=Path)
    parser.add_argument("--bin-likelihood-prerequisite-source",type=Path)
    parser.add_argument("--bin-likelihood-source",type=Path)
    parser.add_argument("--bin-likelihood-prerequisite",type=Path)
    parser.add_argument("--bin-final-information",action="store_true")
    parser.add_argument("--bin-final-verification",action="store_true")
    parser.add_argument("--bin-final-information-source",type=Path)
    parser.add_argument("--bin-final-protocol",type=Path)
    parser.add_argument("--bin-final-method",type=Path)
    parser.add_argument("--bin-final-source-admission",action="store_true")
    args = parser.parse_args()
    if args.prefix_initialization and args.moving_initialization:
        parser.error("choose one declared initializer")
    if bool(args.sensor_block_source)!=bool(args.sensor_block_fit):
        parser.error("conditional sensor diagnostic needs both data and retained fit")
    if args.staged_mechanics_source and args.max_nfev<8:
        parser.error("staged procedure requires at least eight total evaluations")
    if bool(args.bin_likelihood_source)!=bool(args.bin_likelihood_prerequisite):
        parser.error("exact-bin development needs consumed7103 source and passing numerical prerequisite")
    if args.bin_final_verification and (not args.bin_final_information_source or not args.bin_final_protocol or not args.bin_final_method):
        parser.error("fresh exact-bin verification needs frozen method/protocol and passing prior information")
    if (args.linear_mechanics_source or args.fresh_linear_mechanics_source or args.bin_likelihood_source) and not 1<=args.max_nfev<=120:
        parser.error("linear fixture procedure retains the original maximum 120-evaluation budget")
    if args.output_dir.exists() and any(args.output_dir.iterdir()):
        parser.error("fresh output directory required")
    args.output_dir.mkdir(parents=True,exist_ok=True)
    model = true_model()
    save(args.output_dir/"predeclared-contract.json",{
        "schema":"adr0022.nuisance-verification/1","argv":sys.argv,"truth":model.document(),
        "initial_free_values":FIT_INITIAL,"free_bounds":FIT_BOUNDS,"groups":GROUPS,
        "development_inputs":["development-pulses","development-alternating"],
        "development_noise_seeds":DEVELOPMENT_SEEDS,
        "final_inputs":["final-asymmetric-bursts","final-dwell-reversal"],"final_noise_seeds":FINAL_SEEDS,
        "quality_gates":RECOVERY_GATES,"numerical_current_gate_A":1e-9,
        "input_contract":"Declared synthetic accepted-TX pulses, capped .33A; no physical controller/slew authorization inferred",
        "sampling":"Encoder/current1kHz; gyro50Hz native masks; one supplied five-state initialization per complete run",
        "promotion_blocked":True,"physical_actions":False,"hashes":False})
    native = FamilyNative(args.library)
    if args.bin_final_source_admission:
        result = bin_final_source_admission(Path(__file__).resolve().parents[2]/"run"/"adr0022-stage2")
        save(args.output_dir/"source-admission-result.json",result)
        print(json.dumps(result),flush=True)
        return
    if args.bin_final_verification:
        bin_final_verification(native,model,args.bin_final_information_source,args.bin_final_protocol,
            args.bin_final_method,args.output_dir,args.max_nfev)
        return
    if args.bin_final_information:
        bin_final_information(native,model,args.output_dir)
        return
    if args.bin_likelihood_source:
        bin_likelihood_development(native,model,args.bin_likelihood_source,args.bin_likelihood_prerequisite,
            args.output_dir,args.max_nfev)
        return
    if args.bin_likelihood_prerequisite_source:
        bin_likelihood_prerequisite(native,model,args.bin_likelihood_prerequisite_source,args.output_dir)
        return
    if args.fresh_linear_mechanics_source:
        fresh_linear_mechanics_verification(native,model,args.fresh_linear_mechanics_source,
            args.output_dir,args.max_nfev)
        return
    if args.linear_mechanics_source:
        mechanics_pristine_probe(native,model,args.output_dir,args.max_nfev,
            source=args.linear_mechanics_source,loss="linear")
        return
    if args.staged_mechanics_source:
        staged_mechanics_probe(native,model,args.staged_mechanics_source,args.output_dir,args.max_nfev)
        return
    if args.sensor_block_source:
        conditional_sensor_probe(native,model,args.sensor_block_source,args.sensor_block_fit,
            args.output_dir,args.max_nfev)
        return
    if args.mechanics_pristine_probe:
        mechanics_pristine_probe(native,model,args.output_dir,args.max_nfev,args.mechanics_duration_s)
        return
    if args.mechanics_gyro_derivatives:
        mechanics_gyro_derivatives(native,model,args.output_dir,args.mechanics_duration_s)
        return
    if args.mechanics_information_design:
        mechanics_information_design(native,model,args.output_dir,args.mechanics_duration_s)
        return
    if args.mechanics_gauges_source:
        mechanics_gauge_probe(native,model,args.mechanics_gauges_source,args.output_dir)
        return
    if args.retained_mechanics_source:
        retained_mechanics_probe(native,model,args.retained_mechanics_source,args.output_dir,
            args.max_nfev,args.prefix_initialization,args.moving_initialization)
        return
    if args.information_design:
        information_design(native,model,args.output_dir)
        return
    if args.information_training:
        information_training(native,model,args.output_dir,args.max_nfev)
        return
    if args.fresh_recovery_verification:
        fresh_recovery_verification(native,model,args.output_dir,args.max_nfev)
        return
    if args.retained_joint_source:
        retained_joint_probe(native,model,args.retained_joint_source,args.output_dir,args.max_nfev)
        return
    if args.fresh_joint_verification:
        fresh_joint_verification(native,model,args.output_dir,args.max_nfev,args.joint_duration_s)
        return
    if args.joint_prior_derivatives:
        joint_prior_derivatives(native,model,args.output_dir)
        return
    if args.diagnose_gyro_source:
        gyro_information_diagnosis(native,model,args.diagnose_gyro_source,args.output_dir)
        return
    names = ("development-pulses","development-alternating") if args.partition=="development" else \
        ("final-asymmetric-bursts","final-dwell-reversal")
    seeds = DEVELOPMENT_SEEDS if args.partition=="development" else FINAL_SEEDS
    datasets = {}
    for noisy in (False,True):
        for seed in (seeds[:1] if not noisy else seeds):
            for input_name in names:
                label = input_name+"-"+("noisy" if noisy else "pristine")+"-"+str(seed)
                data,events = generate(native,model,input_name,seed,noisy,args.generator)
                datasets[label] = data
                np.savez_compressed(args.output_dir/(label+".npz"),**data)
                save(args.output_dir/(label+"-events.json"),events)
    results = []
    for group in args.groups:
        for label,data in datasets.items():
            if data["input_name"]!=names[0]:
                continue
            targets = {key:value for key,value in datasets.items() if value["noisy"]==data["noisy"]}
            result = fit_group(native,model,group,label,data,targets,args.output_dir,args.max_nfev)
            results.append(result)
            save(args.output_dir/"results.json",results)
            print(json.dumps({"group":group,"training":label,"relative":result["relative_parameter_errors"],
                "nfev":result["optimizer"]["evaluations"],"gates":result["gates"]}),flush=True)
    save(args.output_dir/"summary.json",{"partition":args.partition,"generator":args.generator,
        "cases":len(results),"inverse_passes":sum(row["inverse_probe_passed"] for row in results),
        "independent_scope_passes":sum(row["synthetic_scope_verified"] for row in results),
        "promotion_blocked":True,"physical_stage3a":"NOT_RUN","physical_stage3b":"NOT_RUN"})


if __name__=="__main__":
    main()
