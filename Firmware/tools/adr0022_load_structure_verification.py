"""Independent synthetic load-map estimation and blocked structure selection.

Known algebraic actuation/current/sensor/static nuisances are explicit. Constant
and affine candidates share all moving freedoms, bounds and causal initial-state
policies. No physical capture, model promotion or deployment is performed.
"""
from __future__ import annotations

import argparse
from dataclasses import replace
import json
from pathlib import Path
import sys

import numpy as np
from scipy.optimize import lsq_linear
from scipy.signal import savgol_filter

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from Firmware.commissioning.model_family import FamilyModel,FamilyNative,FamilyRun,fit_family,moving_integral_initializer
from Firmware.commissioning.synthetic_family_oracle import independent_rollout


MOVING_BOUNDS={"a":(.06,.15),"viscous":(.015,.12),"coulomb_negative":(.07,.135),"coulomb_positive":(.10,.17)}
SLOPE_BOUNDS=(-.08,.08)
SUPPORT_Q=(-1.5,1.5)
DEVELOPMENT_SEED=53
CONSUMED_DESIGN_SEEDS=(3251,3529,3911)
CONSUMED_COVERAGE_SEEDS=(4127,4591,4909)
FINAL_SEEDS=(5441,5779,6101)
BUDGET=120


def save(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value,indent=2,allow_nan=False)+"\n",encoding="utf-8")


def fixture_model(load="affine"):
    return FamilyModel(a=.1,viscous=.06,coulomb_negative=.1,coulomb_positive=.14,
        static_negative=.14,static_positive=.18,load_offset=0.,load=load,
        load_slope=.04 if load=="affine" else 0.,q_origin=0.,q_min=-20.,q_max=20.,
        actuator_gain=1.,actuator_bias=0.,transport_delay=.008,gyro_bias=0.,gyro_tau=.015,
        gyro_delay=.004,current_gain=1.,current_bias=0.,current_tau=0.,current_delay=0.,max_step=.001)


def schedule(name):
    if name=="development":
        levels=(.285,-.29,.245,-.255,.305,-.27)
        holds=(.63,.68,.54,.59,.61,.72);rest=.95;q0=-.25
    elif name=="train_left":
        levels=(.295,-.285,.265,-.245,.315,-.275)
        holds=(.71,.76,.62,.69,.57,.81);rest=1.03;q0=-.55
    elif name=="train_right":
        levels=(-.305,.275,-.265,.285,-.245,.325)
        holds=(.74,.82,.67,.63,.71,.59);rest=1.11;q0=.60
    elif name=="selection":
        levels=(.305,-.275,-.255,.285,.235,-.315)
        holds=(.68,.83,.64,.79,.74,.61);rest=1.17;q0=.12
    elif name=="holdout":
        levels=(-.295,.315,.245,-.265,-.235,.285)
        holds=(.79,.67,.83,.73,.62,.87);rest=1.23;q0=-.18
    elif name=="train_balanced_left":
        levels=(.305,-.265,.275,-.235,.295,-.255)
        holds=(.70,.72,.66,.65,.75,.73);rest=1.07;q0=-.55
    elif name=="train_balanced_right":
        levels=(-.245,.285,-.255,.295,-.235,.275)
        holds=(.66,.69,.71,.75,.73,.78);rest=1.13;q0=.55
    elif name=="selection_balanced":
        levels=(.285,-.245,-.225,.265,.305,-.265)
        holds=(.64,.70,.58,.63,.69,.74);rest=1.19;q0=.02
    elif name=="holdout_balanced":
        levels=(-.255,.295,.265,-.225,-.245,.285)
        holds=(.68,.76,.61,.67,.71,.74);rest=1.29;q0=-.12
    elif name=="support_train_left":
        levels=(.315,-.275,.285,-.245,.305,-.265)
        holds=(.64,.67,.72,.71,.69,.74);rest=1.17;q0=-.85
    elif name=="support_train_right":
        levels=(-.255,.295,-.245,.285,-.265,.305)
        holds=(.72,.70,.65,.74,.68,.75);rest=1.21;q0=.85
    elif name=="support_selection":
        levels=(.295,-.255,.255,-.215,-.265,.305)
        holds=(.72,.68,.64,.69,.71,.65);rest=1.27;q0=.07
    elif name=="support_holdout":
        levels=(-.265,.305,-.235,.275,.285,-.245)
        holds=(.63,.71,.67,.73,.69,.75);rest=1.33;q0=-.17
    else:raise ValueError("unknown predeclared independent input schedule")
    tx_t,tx_A=[-.1],[0.];now=.5
    for current,hold in zip(levels,holds,strict=True):
        tx_t.extend([now,now+hold]);tx_A.extend([current,0.]);now+=hold+rest
    return np.asarray(tx_t),np.asarray(tx_A),now,q0


def generate(load,name,seed):
    model=fixture_model(load);tx_t,tx_A,duration,q0=schedule(name)
    t=np.arange(int(np.ceil(duration/.005))+1)*.005;initial=np.array([q0,0.,0.,0.,0.])
    oracle=independent_rollout(model,t,tx_t,tx_A,initial,max_step=.001)
    truth=oracle.trace;rng=np.random.default_rng(seed);quantum=2*np.pi/8192
    q=np.round((truth[:,0]+rng.normal(0,.00015,len(t)))/quantum)*quantum
    current=truth[:,4]+rng.normal(0,.002,len(t));v_new=np.arange(len(t))%4==0
    v=np.full(len(t),np.nan);v[v_new]=truth[v_new,3]+rng.normal(0,.005,int(v_new.sum()))
    label=f"{load}-{name}-seed{seed}"
    run=FamilyRun(label,"independent-load-oracle/"+label,t,q,v,current,np.ones(len(t),bool),v_new,
        np.ones(len(t),bool),tx_t,tx_A,initial,.00015,.005,.002,provenance="SYNTHETIC",
        configuration_id="synthetic-known-nuisance-"+load,calibration_revision="synthetic-known-sensors",
        encoder_quantum=quantum).validate()
    if np.min(truth[:,0])<SUPPORT_Q[0] or np.max(truth[:,0])>SUPPORT_Q[1]:
        raise ValueError("declared true synthetic input exceeds supplied load interpolation support")
    return run,truth,oracle.diagnostics


def load_integral_seed(model,train,bounds):
    """Extend the existing TRAIN moving equations with the affine load integral."""
    if model.load=="constant":return moving_integral_initializer(model,train,bounds)
    names=tuple(MOVING_BOUNDS)+("load_slope",);matrix=[];rhs=[]
    for run in train:
        times=run.t[run.v_new]-model.gyro_delay;gyro=run.v[run.v_new]-model.gyro_bias
        spacing=float(np.median(np.diff(times)));window=max(5,int(round(.22/spacing))|1)
        if len(times)<window or np.max(np.abs(np.diff(times)-spacing))>max(1e-8,spacing*.01):
            return None,{"status":"INSUFFICIENT_UNIFORM_NATIVE_GYRO","training_only":True}
        velocity=savgol_filter(gyro,window,3,mode="interp")+model.gyro_tau*savgol_filter(
            gyro,window,3,deriv=1,delta=spacing,mode="interp")
        q_times,q_values=run.t[run.q_new],run.q[run.q_new];command_times=run.tx_t+model.transport_delay
        for count in (max(3,int(round(duration/spacing))) for duration in (.12,.2,.4)):
            for start in range(0,len(times)-count,3):
                end=start+count;section=velocity[start:end+1];minimum=max(.05,10*run.sigma_v)
                if times[start]<max(run.t[0],command_times[0]) or not (
                    np.all(section>minimum) or np.all(section<-minimum)):continue
                direction=1 if section[0]>0 else -1;duration=times[end]-times[start]
                delta_q=np.interp(times[end],q_times,q_values)-np.interp(times[start],q_times,q_values)
                anchors=np.r_[times[start],q_times[(q_times>times[start])&(q_times<times[end])],times[end]]
                load_integral=float(np.trapezoid(np.interp(anchors,q_times,q_values)-model.q_origin,anchors))
                input_edges=np.r_[times[start],command_times[(command_times>times[start])&(command_times<times[end])],times[end]]
                held=np.searchsorted(command_times,input_edges[:-1]+1e-13,side="right")-1
                input_integral=float(np.diff(input_edges)@(model.actuator_gain*run.tx_A[held]+model.actuator_bias))
                matrix.append([velocity[end]-velocity[start],delta_q,-duration if direction<0 else 0.,
                    duration if direction>0 else 0.,load_integral]);rhs.append(input_integral)
    matrix=np.asarray(matrix);rhs=np.asarray(rhs)
    if len(rhs)<5 or np.linalg.matrix_rank(matrix)<5:
        return None,{"status":"INSUFFICIENT_MOVING_LOAD_EQUATIONS","equations":len(rhs),"training_only":True}
    result=lsq_linear(matrix,rhs,bounds=([bounds[k][0] for k in names],[bounds[k][1] for k in names]))
    if not result.success:return None,{"status":"BOUNDED_LINEAR_SEED_FAILED","training_only":True}
    values=dict(zip(names,map(float,result.x)))
    return replace(model,**values),{"status":"COMPUTED_TRAIN_ONLY","parameters":values,"equations":len(rhs),
        "rank":int(np.linalg.matrix_rank(matrix)),"condition":float(np.linalg.cond(matrix)),
        "equation_residual_rms_A_s":float(np.sqrt(np.mean((matrix@result.x-rhs)**2))),
        "moving_interval_s":[.12,.2,.4],"smoothing_window_s":.22,"training_only":True,
        "acceptance":"complete native hybrid observation-error fitting; equation fit is initialization only"}


def prediction_report(native,model,run):
    pred=native.rollout(model,run.t,run.tx_t,run.tx_A,run.initial)
    errors={name:float(np.sqrt(np.mean((pred[getattr(run,mask),column]-getattr(run,channel)[getattr(run,mask)])**2)))
        for name,channel,mask,column in (("q_rms_rad","q","q_new",0),("gyro_rms_rad_s","v","v_new",3),("current_rms_A","current","current_new",4))}
    limits={"q_rms_rad":float(3*np.sqrt(run.sigma_q**2+run.encoder_quantum**2/12)),
        "gyro_rms_rad_s":3*run.sigma_v,"current_rms_A":3*run.sigma_current}
    channel_gates={key:errors[key]<=limits[key] for key in errors}
    support=bool(np.min(pred[:,0])>=SUPPORT_Q[0] and np.max(pred[:,0])<=SUPPORT_Q[1])
    return pred,{"run_id":run.run_id,"errors":errors,"limits":limits,"channel_gates":channel_gates,
        "prediction_inside_declared_load_support":support,"passed":all(channel_gates.values()) and support,
        "initial":run.initial.tolist(),"state_resets":0}


def save_trajectory(path,run,prediction=None,truth=None):
    values={key:getattr(run,key) for key in ("t","q","v","current","q_new","v_new","current_new","tx_t","tx_A","initial")}
    if prediction is not None:values["prediction"]=prediction
    if truth is not None:values["truth"]=truth
    path.parent.mkdir(parents=True,exist_ok=True);np.savez_compressed(path,**values)


def candidate_fit(native,load,train):
    bounds={**MOVING_BOUNDS,**({"load_slope":SLOPE_BOUNDS} if load=="affine" else {})}
    initial=replace(fixture_model(load),a=.085,viscous=.075,coulomb_negative=.09,coulomb_positive=.13,load_slope=0.)
    seed,seed_record=load_integral_seed(initial,train,bounds)
    fit=fit_family(native,seed if seed is not None else initial,train,bounds=bounds,max_nfev=BUDGET)
    fit["load_initializer"]=seed_record
    return fit


def select_candidate(rows):
    """Select using blocked absolute predictions and complexity, never truth."""
    eligible=[row for row in rows if row["optimizer"]["success"] is True
        and row["training"] and all(r["passed"] for r in row["training"])
        and row["selection"] and all(r["passed"] for r in row["selection"])]
    if not eligible:return None
    return min(eligible,key=lambda row:(row["complexity"],sum(
        r["errors"]["q_rms_rad"]**2+r["errors"]["gyro_rms_rad_s"]**2 for r in row["selection"]),row["load"]))


def verify_world(native,true_load,seed,output):
    # All TRAIN and blocked SELECTION inputs are declared before noise/fitting.
    # Their observations never enter another role's seed or objective.
    train_data=[generate(true_load,"support_train_left",seed),generate(true_load,"support_train_right",seed+100)]
    selection_data=[generate(true_load,"support_selection",seed+200)]
    train=[item[0] for item in train_data]
    domain=(max(SUPPORT_Q[0],float(min(np.min(r.q[r.q_new]) for r in train))),
            min(SUPPORT_Q[1],float(max(np.max(r.q[r.q_new]) for r in train))))
    rows=[];forward=[]
    for role,data in (("train",train_data),("selection",selection_data)):
        for run,truth,oracle in data:
            at_truth=native.rollout(fixture_model(true_load),run.t,run.tx_t,run.tx_A,run.initial)
            numerical={"run_id":run.run_id,"role":role,
                "q_rms_rad":float(np.sqrt(np.mean((at_truth[:,0]-truth[:,0])**2))),
                "gyro_rms_rad_s":float(np.sqrt(np.mean((at_truth[:,3]-truth[:,3])**2))),
                "current_max_abs_A":float(np.max(np.abs(at_truth[:,4]-truth[:,4])))}
            numerical["passed"]=numerical["q_rms_rad"]<=1e-5 and numerical["gyro_rms_rad_s"]<=1e-4 and numerical["current_max_abs_A"]<=1e-10
            forward.append(numerical);save_trajectory(output/"observations"/(run.run_id+".npz"),run,truth=truth)
    for load in ("constant","affine"):
        fit=candidate_fit(native,load,train);row={"load":load,"model":fit["model"].document(),
            "optimizer":fit["optimizer"],"load_initializer":fit["load_initializer"],"complexity":4 if load=="constant" else 6,
            "training":[],"selection":[],"actual_training_support_q_rad":list(domain)}
        row["supported_load_interpolation"]={"q_nodes_rad":list(domain),
            "load_nodes_A":[float(fit["model"].load_slope*(q-fit["model"].q_origin)) for q in domain],
            "outside_domain":"UNSUPPORTED; no extrapolation qualification","offset_gauge_A":0.}
        for role,data in (("training",train_data),("selection",selection_data)):
            for run,truth,_ in data:
                prediction,report=prediction_report(native,fit["model"],run)
                report["prediction_inside_training_interpolation_domain"]=bool(
                    np.min(prediction[:,0])>=domain[0] and np.max(prediction[:,0])<=domain[1])
                # No support extrapolation on blocked selection. TRAIN centres
                # carry noise, so its own start/end domain check remains the
                # explicitly supplied support, not a hard observed-range clip.
                if role=="selection":report["passed"] &= report["prediction_inside_training_interpolation_domain"]
                row[role].append(report)
                save_trajectory(output/load/(run.run_id+"-prediction.npz"),run,prediction)
        moving_relative={key:float((getattr(fit["model"],key)-getattr(fixture_model(true_load),key))/getattr(fixture_model(true_load),key))
            for key in MOVING_BOUNDS}
        row["parameter_diagnostics"]={"relative_moving_error":moving_relative,
            "slope_error_A_rad":float(fit["model"].load_slope-fixture_model(true_load).load_slope),
            "moving_recovery_passed":bool(max(map(abs,moving_relative.values()))<=.05),
            "load_slope_recovery_passed":bool(abs(fit["model"].load_slope-fixture_model(true_load).load_slope)<=.002),
            "truth_diagnostics_used_for_selection":False}
        row["gates"]={"data_integrity":"PASS","forward_numerics":"PASS" if all(r["passed"] for r in forward) else "FAIL",
            "optimizer_termination_reason":fit["optimizer"]["message"],"optimizer_converged":fit["optimizer"]["success"],
            "synthetic_parameter_recovery":"PASS" if row["parameter_diagnostics"]["moving_recovery_passed"] and
                row["parameter_diagnostics"]["load_slope_recovery_passed"] else "FAIL",
            "training_trajectory":"PASS" if all(r["passed"] for r in row["training"]) else "FAIL",
            "selection_trajectory":"PASS" if all(r["passed"] for r in row["selection"]) else "FAIL",
            "historical_regression":"NOT_RUN","prospective_prediction":"NOT_RUN",
            "physical_stage3a":"NOT_RUN","physical_stage3b":"NOT_RUN","deployment_authorized":False}
        rows.append(row);save(output/(load+"-candidate.json"),row)
    selected=select_candidate(rows)
    # Save the exact choice before generating or reading the untouched whole-run
    # holdout. A failed holdout rejects promotion; it never changes this choice.
    frozen={"selected_load":selected["load"] if selected else None,"selected_model":selected["model"] if selected else None,
        "support_q_rad":list(domain),"holdout_observations":"NOT_GENERATED_OR_READ",
        "selection_rule":"least complex optimizer+TRAIN+blockedSELECTION absolute passing candidate",
        "truth_used_for_choice":False,"deployment_authorized":False}
    save(output/"frozen-selection-before-holdout.json",frozen)
    final=[]
    if selected is not None:
        run,truth,oracle=generate(true_load,"support_holdout",seed+300)
        chosen=FamilyModel(**{name:selected["model"][name] for name in fixture_model().__dataclass_fields__})
        prediction,report=prediction_report(native,chosen,run)
        report["prediction_inside_training_interpolation_domain"]=bool(np.min(prediction[:,0])>=domain[0] and np.max(prediction[:,0])<=domain[1])
        report["passed"] &= report["prediction_inside_training_interpolation_domain"]
        report["choice_refitted_or_changed_after_holdout"]=False
        save_trajectory(output/"selected-holdout.npz",run,prediction,truth);final.append(report)
    selection_correct=selected is not None and selected["load"]==true_load
    result={"true_fixture_load":true_load,"seed":seed,"candidates":rows,"forward_oracle":forward,
        "selected_load":selected["load"] if selected else None,"final_holdout":final,
        "synthetic_structure_selection_correct":selection_correct,
        "selected_parameter_recovery_passed":bool(selected and selected["gates"]["synthetic_parameter_recovery"]=="PASS"),
        "passed":bool(selection_correct and all(r["passed"] for r in final) and final and all(r["passed"] for r in forward)
            and selected["gates"]["synthetic_parameter_recovery"]=="PASS"),
        "scope":"known static/input/sensor nuisance, scalar affine interpolation on training q support, prescribed independent inputs",
        "rejected_alternatives_see_holdout":False,"physical_qualification":False,"deployment_authorized":False}
    save(output/"result.json",result)
    return result


def gauge_report(native):
    base=fixture_model("affine");delta=.025
    transformed=replace(base,load_offset=delta,coulomb_negative=base.coulomb_negative+delta,
        coulomb_positive=base.coulomb_positive-delta,static_negative=base.static_negative+delta,
        static_positive=base.static_positive-delta)
    run,_,_=generate("affine","development",97)
    first=native.rollout(base,run.t,run.tx_t,run.tx_A,run.initial)
    second=native.rollout(transformed,run.t,run.tx_t,run.tx_A,run.initial)
    difference=float(np.max(np.abs(first-second)))
    return {"gauge":"L'=L+d,Fc+'=Fc+-d,Fc-'=Fc-+d; analogous Fs totals",
        "delta_A":delta,"maximum_all_native_channel_difference":difference,"passed":difference<1e-10,
        "estimating_separate_offset_and_directional_friction_supported":False,
        "fit_reference":"L(q_origin=0)=0; directional Fc/Fs are total-load coefficients at this reference"}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--library",type=Path,required=True);parser.add_argument("--output-dir",type=Path,required=True)
    parser.add_argument("--partition",choices=("development","final"),default="development")
    parser.add_argument("--probe-only",action="store_true")
    args=parser.parse_args()
    if args.output_dir.exists() and any(args.output_dir.iterdir()):parser.error("use a fresh retained evidence directory")
    args.output_dir.mkdir(parents=True,exist_ok=True);native=FamilyNative(args.library)
    save(args.output_dir/"predeclared-contract.json",{"schema":"adr0022.load-structure-verification/1","partition":args.partition,
        "development_seed":DEVELOPMENT_SEED,"final_seeds":list(FINAL_SEEDS),"optimizer_budget":BUDGET,
        "consumed_design_seed_partition":list(CONSUMED_DESIGN_SEEDS),
        "consumed_coverage_seed_partition":list(CONSUMED_COVERAGE_SEEDS),
        "final_input_schedules":["support_train_left","support_train_right","support_selection","support_holdout"],
        "input_design_revision":"Original TRAIN-left violated supplied support; balanced partition failed holdoutTRAINhull gate; broader TRAIN q0anchors and new schedules/seeds frozen without changing gates or supplied support",
        "moving_bounds":MOVING_BOUNDS,"affine_slope_bounds_A_rad":SLOPE_BOUNDS,
        "load_interpolation_support_q_rad":SUPPORT_Q,"load_reference":"load_offset fixed zero at q_origin=0; directional totals",
        "numerical_rollout_guard_q_rad":[-20,20],"quality_gates":"unchanged original3sigma native channels; 5% parameter diagnostic only",
        "load_slope_diagnostic_absolute_gate_A_rad":.002,
        "load_slope_diagnostic_basis":"5% of declared .04A/rad characteristic, also meaningful at zero true slope; never used to select",
        "nuisance":"supplied known algebraic current map/delay, gyro/current sensors/filter/delay, directional static totals, one initial state",
        "selection":"least complex optimizer+TRAIN+blockedSELECTION absolute trajectory passing candidate; truth parameters never select",
        "holdout":"created/evaluated only after selected model frozen; rejected alternatives never see it",
        "initial_moving_parameters":{"a":.085,"viscous":.075,"coulomb_negative":.09,"coulomb_positive":.13,"load_slope":0.},
        "library":str(args.library),"physical_actions":False,"deployment_authorized":False})
    if args.probe_only:
        run,truth,oracle=generate("affine","development",DEVELOPMENT_SEED)
        exact=native.rollout(fixture_model(),run.t,run.tx_t,run.tx_A,run.initial)
        fit=candidate_fit(native,"affine",[run]);prediction,report=prediction_report(native,fit["model"],run)
        record={"forward_q_rms_rad":float(np.sqrt(np.mean((exact[:,0]-truth[:,0])**2))),
            "forward_gyro_rms_rad_s":float(np.sqrt(np.mean((exact[:,3]-truth[:,3])**2))),
            "oracle":oracle,"fitted_model":fit["model"].document(),"optimizer":fit["optimizer"],
            "load_initializer":fit["load_initializer"],"training_prediction":report,"gauge":gauge_report(native),
            "relative_parameters":{key:(getattr(fit["model"],key)-getattr(fixture_model(),key))/getattr(fixture_model(),key)
                for key in (*MOVING_BOUNDS,"load_slope")},"physical_qualification":False,"deployment_authorized":False}
        save(args.output_dir/"probe-result.json",record);save_trajectory(args.output_dir/"probe-trajectory.npz",run,prediction,truth)
        print(json.dumps({"optimizer":record["optimizer"]["message"],"training_prediction":report,
            "relative_parameters":record["relative_parameters"],"forward_q_rms_rad":record["forward_q_rms_rad"]}),flush=True)
        return
    if args.partition!="final":parser.error("full blocked verification uses frozen fresh final schedules; development uses --probe-only")
    results=[]
    for true_load in ("constant","affine"):
        for seed in FINAL_SEEDS:
            result=verify_world(native,true_load,seed,args.output_dir/(true_load+f"-seed{seed}"));results.append(result)
            print(json.dumps({"truth":true_load,"seed":seed,"selected":result["selected_load"],"passed":result["passed"],
                "candidate_gates":{r["load"]:r["gates"] for r in result["candidates"]},"holdout":result["final_holdout"]}),flush=True)
    summary={"schema":"adr0022.load-structure-verification-summary/1","partition":args.partition,
        "worlds":results,"gauge":gauge_report(native),"passed":all(r["passed"] for r in results),
        "physical_stage3a":"NOT_RUN","physical_stage3b":"NOT_RUN","deployment_authorized":False}
    save(args.output_dir/"summary.json",summary)


if __name__=="__main__":main()
