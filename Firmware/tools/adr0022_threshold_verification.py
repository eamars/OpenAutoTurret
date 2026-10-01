"""Offline synthetic censored-threshold verification; no station operations."""
from __future__ import annotations

import argparse
from dataclasses import asdict, replace
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from Firmware.commissioning.model_family import FamilyModel, FamilyNative, FamilyRun
from Firmware.commissioning.synthetic_family_oracle import independent_rollout
from Firmware.commissioning.threshold_identification import (
    PlateauTrial, ThresholdPolicy, censored_threshold_intervals, fit_threshold_family,
    threshold_outcome_support,RestSupport)


MOVING_BOUNDS={"a":(.06,.15),"viscous":(.015,.12),
    "coulomb_negative":(.07,.115),"coulomb_positive":(.11,.155)}
THRESHOLD_BOUNDS={"negative":(.12,.21),"positive":(.16,.24)}
DEVELOPMENT_SEED=43
CONSUMED_FINAL_SEEDS=(1543,1697,1871)
FINAL_SEEDS=(2017,2371,2801)


def save(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value,indent=2,allow_nan=False)+"\n",encoding="utf-8")


def model():
    # Same lumped mechanics as the original oracle, under an explicit total-load
    # gauge: Fc- = .12-.02, Fc+ = .12+.02; Fs-=.16-.02, Fs+=.16+.02.
    return FamilyModel(a=.1,viscous=.06,coulomb_negative=.10,coulomb_positive=.14,
        static_negative=.14,static_positive=.18,load_offset=0.,q_min=-20.,q_max=20.,
        actuator_gain=1.,actuator_bias=0.,transport_delay=.008,
        gyro_bias=0.,gyro_tau=.015,gyro_delay=.004,current_gain=1.,current_bias=0.,
        current_tau=0.,current_delay=0.,max_step=.001)


def schedule(name):
    # Predeclared excitation families. Each starts from actual rest and contains
    # short threshold censored trials plus larger moving identification steps.
    if name == "development":
        levels=(.175,-.135,.185,-.145,.30,-.28,.24,-.23)
        holds=(.65,.65,.65,.65,.85,.85,.70,.70); rest=.95
    elif name == "final_balanced":
        levels=(-.136,.176,-.144,.184,-.27,.31,-.225,.25)
        holds=(.72,.61,.68,.77,.82,.93,.75,.88); rest=1.05
    elif name == "final_dwell":
        levels=(.176,-.136,.184,-.144,.265,-.305,.235,-.245)
        holds=(.81,.73,.63,.69,1.02,.78,.91,.84); rest=1.17
    elif name == "final_offsets":
        levels=(.177,-.137,-.143,.183,.285,-.295,-.255,.225)
        holds=(.83,.67,.74,.79,.96,.86,.73,.91); rest=1.31
    elif name == "final_long_dwell":
        levels=(-.1365,.1765,.1845,-.1445,-.315,.275,.245,-.235)
        holds=(.91,.86,.73,.82,1.06,.94,.89,.77); rest=1.43
    elif name == "final_reordered":
        levels=(-.134,.174,.187,-.147,.325,-.265,.235,-.285)
        holds=(.78,.93,.86,.71,.87,1.13,.97,.82); rest=1.52
    elif name == "interval_ambiguity":
        levels=(-.139,.179);holds=(.8,.8);rest=1.31
    elif name == "supported_training":
        levels=(.1762,-.1362,-.1442,.1842,.295,-.285,.235,-.255)
        holds=(.88,.79,.69,.82,.91,1.01,.83,.97);rest=1.47
    elif name == "supported_longrest":
        levels=(-.1372,.1772,.1832,-.1432,-.275,.325,.255,-.245)
        holds=(.92,.87,.78,.85,1.07,.89,.98,.81);rest=1.63
    elif name == "supported_reordered":
        levels=(-.1342,.1742,.1872,-.1472,.315,-.305,-.225,.265)
        holds=(.86,.94,.73,.91,.99,1.02,.87,.93);rest=1.79
    else:
        raise ValueError("unknown predeclared schedule")
    tx_t,tx_A=[-.10],[0.]; trials=[]; now=.5
    for index,(level,hold) in enumerate(zip(levels,holds,strict=True)):
        tx_t.extend([now,now+hold]); tx_A.extend([level,0.])
        trials.append((f"plateau-{index}",1 if level>0 else -1,now,now+hold,now-.2))
        now += hold+rest
    return np.asarray(tx_t),np.asarray(tx_A),trials,now


def generate(name,seed,*,noisy=True):
    truth_model=model(); tx_t,tx_A,trial_defs,duration=schedule(name)
    n=int(np.ceil(duration/.005)); t=np.arange(n+1)*.005
    oracle=independent_rollout(truth_model,t,tx_t,tx_A,np.zeros(5),max_step=.001)
    truth=oracle.trace; rng=np.random.default_rng(seed)
    quantum=2*np.pi/8192 if noisy else 0.
    q_noise=.00015 if noisy else 0.; v_noise=.005 if noisy else 0.; i_noise=.002 if noisy else 0.
    q=truth[:,0]+rng.normal(0,q_noise,len(t))
    if noisy:q=np.round(q/quantum)*quantum
    current=truth[:,4]+rng.normal(0,i_noise,len(t))
    v_new=np.arange(len(t))%4 == 0
    v=np.full(len(t),np.nan);v[v_new]=truth[v_new,3]+rng.normal(0,v_noise,int(v_new.sum()))
    label=f"{name}-seed{seed}-{'noisy' if noisy else 'pristine'}"
    run=FamilyRun(label,"independent-analytic-threshold/"+label,t,q,v,current,
        np.ones(len(t),bool),v_new,np.ones(len(t),bool),tx_t,tx_A,np.zeros(5),
        max(q_noise,1e-7),max(v_noise,1e-7),max(i_noise,1e-7),provenance="SYNTHETIC",
        configuration_id="synthetic-total-load-gauge",calibration_revision="known-synthetic-nuisance",
        encoder_quantum=quantum).validate()
    trials=[PlateauTrial(label,*item) for item in trial_defs]
    return run,trials,truth,oracle.diagnostics


def synthetic_rest_support(run,trials,truth):
    if run.provenance != "SYNTHETIC":
        raise ValueError("independent synthetic oracle cannot certify physical rest")
    supports=[]
    for trial in trials:
        section=(run.t >= trial.rest_start-1e-12)&(run.t <= trial.command_time+1e-12)
        # Constant pre-command zero input and the oracle's actual sticking mode
        # support continuous rest. Noise bands are not used to issue this support.
        indices=np.searchsorted(run.tx_t,run.t[section]+1e-12,side="right")-1
        before=run.t[section] < trial.command_time-1e-12
        established=bool(section.any() and np.all(truth[section,5]==1) and
            np.all(np.abs(truth[section,1])<=1e-12) and np.all(run.tx_A[indices[before]]==0.))
        supports.append(RestSupport(run.run_id,trial.trial_id,"SYNTHETIC","INDEPENDENT_SYNTHETIC_ORACLE",
            trial.rest_start,trial.command_time,established,"independent linear-mode oracle trace/"+run.run_id))
    return supports


def metrics(run,prediction):
    return {"q_rms_rad":float(np.sqrt(np.mean((prediction[run.q_new,0]-run.q[run.q_new])**2))),
        "gyro_rms_rad_s":float(np.sqrt(np.mean((prediction[run.v_new,3]-run.v[run.v_new])**2))),
        "current_rms_A":float(np.sqrt(np.mean((prediction[run.current_new,4]-run.current[run.current_new])**2)))}


def trajectory_gates(run,errors):
    # Preserve the original noisy synthetic three-sigma bands, including encoder
    # noise plus uniform quantization variance. No threshold-specific loosening.
    q_limit=3*np.sqrt(run.sigma_q**2+run.encoder_quantum**2/12)
    return {"q":bool(errors["q_rms_rad"] <= q_limit),"gyro":bool(errors["gyro_rms_rad_s"] <= 3*run.sigma_v),
        "current":bool(errors["current_rms_A"] <= 3*run.sigma_current),
        "limits":{"q_rms_rad":float(q_limit),"gyro_rms_rad_s":3*run.sigma_v,"current_rms_A":3*run.sigma_current}}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--library",type=Path,required=True)
    parser.add_argument("--output-dir",type=Path,required=True)
    parser.add_argument("--probe-only",action="store_true")
    parser.add_argument("--partition",choices=("development","final"),default="development")
    args=parser.parse_args()
    if args.output_dir.exists() and any(args.output_dir.iterdir()):parser.error("use fresh retained evidence directory")
    args.output_dir.mkdir(parents=True,exist_ok=True)
    policy=ThresholdPolicy()
    save(args.output_dir/"predeclared-contract.json",{"schema":"adr0022.threshold-verification/1",
        "partition":args.partition,"development_seed":DEVELOPMENT_SEED,"final_seeds":list(FINAL_SEEDS),
        "train_schedules":["development"] if args.partition=="development" else ["supported_training"],
        "cross_schedules":["final_balanced","final_dwell"] if args.partition=="development" else ["supported_longrest","supported_reordered"],
        "partition_provenance":"quiet-rest-only procedure superseded; supported final schedules first generated after explicit-rest-support method freeze; repeats are regressions",
        "consumed_final_seeds_before_rest_repair":list(CONSUMED_FINAL_SEEDS),
        "moving_bounds":MOVING_BOUNDS,
        "threshold_bounds":THRESHOLD_BOUNDS,"policy":asdict(policy),"fixture":model().document(),
        "supplied_nuisance":"Known algebraic input map/delay and sensor map/filter/delay; independent prescribed inputs",
        "rest_support":"Separate independent synthetic actual-sticking certificate; cannot qualify MEASURED rest or physical deployment",
        "initial_free_coordinates":{"a":.085,"viscous":.075,"coulomb_negative":.09,"coulomb_positive":.13},
        "threshold_identification_target":"Directional total breakaway brackets, not exact unique Fs or calibrated confidence bounds",
        "numerical_gates":{"q_rms_rad":1e-5,"gyro_rms_rad_s":1e-4,"current_max_abs_A":1e-10},
        "synthetic_moving_relative_gate":.05,"synthetic_trajectory_gates":"original3sigma with encoder quantization variance; no quality threshold changed",
        "physical_actions":False,"deployment_authorized":False})
    native=FamilyNative(args.library)
    seeds=(DEVELOPMENT_SEED,) if args.partition=="development" else FINAL_SEEDS
    train_name="development" if args.partition=="development" else "supported_training"
    cases=[]
    for seed in seeds:
        run,trials,truth,oracle=generate(train_name,seed)
        rest_support=synthetic_rest_support(run,trials,truth)
        initial=replace(model(),a=.085,viscous=.075,coulomb_negative=.09,coulomb_positive=.13,
                        static_negative=.21,static_positive=.23)
        at_truth=native.rollout(model(),run.t,run.tx_t,run.tx_A,run.initial)
        numerical={"q_rms_rad":float(np.sqrt(np.mean((at_truth[:,0]-truth[:,0])**2))),
            "gyro_rms_rad_s":float(np.sqrt(np.mean((at_truth[:,3]-truth[:,3])**2))),
            "current_max_abs_A":float(np.max(np.abs(at_truth[:,4]-truth[:,4])))}
        intervals=censored_threshold_intervals(initial,[run],trials,bounds=MOVING_BOUNDS,
            threshold_bounds=THRESHOLD_BOUNDS,policy=policy,rest_support=rest_support)
        record={"case":run.run_id,"numerical_oracle":numerical,"oracle_diagnostics":oracle,
            "censored_threshold_evidence":intervals}
        np.savez_compressed(args.output_dir/(run.run_id+"-observation.npz"),t=run.t,q=run.q,v=run.v,
            current=run.current,q_new=run.q_new,v_new=run.v_new,current_new=run.current_new,
            tx_t=run.tx_t,tx_A=run.tx_A,initial=run.initial,truth=truth)
        if not args.probe_only:
            fit=fit_threshold_family(native,initial,[run],trials,bounds=MOVING_BOUNDS,
                threshold_bounds=THRESHOLD_BOUNDS,policy=policy,rest_support=rest_support)
            fitted=fit["model"]
            relative={k:(getattr(fitted,k)-getattr(model(),k))/getattr(model(),k) for k in MOVING_BOUNDS}
            predictions=[]
            cross_names=("final_balanced","final_dwell") if args.partition=="development" else ("supported_longrest","supported_reordered")
            targets=[(run,truth)]+[(item[0],item[2]) for item in
                (generate(cross_names[0],seed+10000),generate(cross_names[1],seed+20000))]
            for target,target_truth in targets:
                pred=native.rollout(fitted,target.t,target.tx_t,target.tx_A,target.initial)
                errors=metrics(target,pred); gates=trajectory_gates(target,errors)
                predictions.append({"run_id":target.run_id,"errors":errors,"gates":gates})
                np.savez_compressed(args.output_dir/(run.run_id+"-predict-"+target.run_id+".npz"),
                    t=target.t,prediction=pred,q=target.q,v=target.v,current=target.current,
                    q_new=target.q_new,v_new=target.v_new,current_new=target.current_new,
                    tx_t=target.tx_t,tx_A=target.tx_A,initial=target.initial)
            interval_gates={d:intervals["intervals"][d]["lower_A"] <= getattr(model(),"static_"+d)
                < intervals["intervals"][d]["upper_A"] for d in ("negative","positive")}
            ambiguity_run,ambiguity_trials,ambiguity_truth,_=generate("interval_ambiguity",seed+30000)
            edge_models=[replace(fitted,**{"static_"+d:intervals["intervals"][d][edge] + (1e-8 if edge=="lower_A" else -1e-8)
                for d in ("negative","positive")}) for edge in ("lower_A","upper_A")]
            ambiguity_predictions=[native.rollout(m,ambiguity_run.t,ambiguity_run.tx_t,ambiguity_run.tx_A,ambiguity_run.initial)
                for m in edge_models]
            ambiguous_commands=[{"trial":trial.trial_id,"direction":trial.direction,
                "support":threshold_outcome_support(intervals["intervals"]["positive" if trial.direction>0 else "negative"],
                    abs(float(ambiguity_run.tx_A[np.searchsorted(ambiguity_run.tx_t,trial.command_time+1e-12,side="right")-1])))}
                for trial in ambiguity_trials]
            ambiguity={"commands":ambiguous_commands,"q_span_max_rad":float(np.max(np.abs(ambiguity_predictions[0][:,0]-ambiguity_predictions[1][:,0]))),
                "decision":"interval-interior rest commands remain unqualified; retain bracket or acquire finer authorized threshold evidence",
                "representative_selected_using_ambiguity_data":False,"calibrated_confidence_ensemble":False}
            np.savez_compressed(args.output_dir/(run.run_id+"-interval-ambiguity.npz"),t=ambiguity_run.t,
                lower_threshold_prediction=ambiguity_predictions[0],upper_threshold_prediction=ambiguity_predictions[1],
                q=ambiguity_run.q,v=ambiguity_run.v,current=ambiguity_run.current,tx_t=ambiguity_run.tx_t,tx_A=ambiguity_run.tx_A,
                q_new=ambiguity_run.q_new,v_new=ambiguity_run.v_new,current_new=ambiguity_run.current_new)
            record.update(fitted_model=fitted.document(),relative_moving_error=relative,interval_ambiguity=ambiguity,
                optimizer=fit["optimizer"],threshold_identification=fit["threshold_identification"],predictions=predictions,
                gates={"data_integrity":"PASS","forward_numerics":"PASS" if numerical["q_rms_rad"]<=1e-5 and numerical["gyro_rms_rad_s"]<=1e-4 and numerical["current_max_abs_A"]<=1e-10 else "FAIL",
                    "optimizer_termination_reason":fit["optimizer"]["message"],"optimizer_converged":fit["optimizer"]["success"],
                    "synthetic_moving_parameter_recovery":"PASS" if max(map(abs,relative.values()))<=.05 else "FAIL",
                    "synthetic_static_interval_coverage":"PASS" if all(interval_gates.values()) else "FAIL",
                    "training_trajectory":"PASS" if all(predictions[0]["gates"][k] for k in ("q","gyro","current")) else "FAIL",
                    "selection_trajectory":"PASS" if all(p["gates"][k] for p in predictions[1:] for k in ("q","gyro","current")) else "FAIL",
                    "historical_regression":"NOT_RUN","prospective_prediction":"NOT_RUN",
                    "physical_stage3a":"NOT_RUN","physical_stage3b":"NOT_RUN","deployment_authorized":False})
        save(args.output_dir/(run.run_id+"-result.json"),record); cases.append(record)
        print(json.dumps({"case":run.run_id,"numerical":numerical,"intervals":intervals["intervals"],
            "gates":record.get("gates","PROBE_ONLY")}),flush=True)
    save(args.output_dir/"summary.json",{"schema":"adr0022.threshold-verification-summary/1",
        "partition":args.partition,"cases":cases,"physical_stage3a":"NOT_RUN","physical_stage3b":"NOT_RUN",
        "exact_static_friction_identified":False,"deployment_authorized":False})


if __name__ == "__main__":main()
