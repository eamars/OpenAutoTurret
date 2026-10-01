"""Bounded nonqualifying physical-data crossed ablation on existing journals.

Only TRAIN and SELECTION observations are loaded. Unknown pooled configuration,
current meaning and gyro support remain unknown; numerical improvements do not
qualify a physical structure or controller. No station connection is performed.
"""
from __future__ import annotations

import argparse
from dataclasses import replace
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import time

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from Firmware.commissioning.contracts import Rejected
from Firmware.commissioning.model_family import FamilyModel,FamilyNative,fit_family,family_reports
from Firmware.commissioning.recovery import comparison_recovery,diagnostic_report
from Firmware.commissioning.yaw_events import load_yaw_journal
from Firmware.tools.adr0022_yaw_compare import whole_run,diagnostic_summary,write_json


CELL_NAMES=("constant-current-fixed","affine-current-fixed",
            "constant-current-free","affine-current-free")


def read_plans(directory):
    plans={}; selected={}
    for name in CELL_NAMES:
        plan=json.loads((directory/(name+".json")).read_text(encoding="utf-8"))
        if plan.get("fair_ablation_contract",{}).get("full_optimizer_budget") != 200:
            raise ValueError("source crossed plan must retain its declared 200-evaluation budget")
        candidates=[item for item in plan["candidates"] if
            item["initial_model"]["actuator"] == "algebraic" and item["initial_model"]["friction"] == "coulomb"]
        if len(candidates)!=1:raise ValueError("exactly one least-complex algebraic-Coulomb candidate required")
        plans[name]=plan;selected[name]=candidates[0]
    common=("schema","splits","frozen_calibration_json","calibration_revision","encoder_datum_count",
        "maximum_timing_search_s","training_noise","calibration_scope","numerical_domain_contract")
    baseline=plans[CELL_NAMES[0]]
    if any(plan[key]!=baseline[key] for plan in plans.values() for key in common):
        raise ValueError("crossed plans differ in input, split, noise, support or numerical domain")
    checks=[]
    for measurement in ("fixed","free"):
        left,right=(selected[f"{load}-current-{measurement}"] for load in ("constant","affine"))
        model_diff=sorted(k for k in set(left["initial_model"])|set(right["initial_model"])
            if left["initial_model"].get(k)!=right["initial_model"].get(k))
        bound_diff=sorted(k for k in set(left["bounds"])|set(right["bounds"]) if left["bounds"].get(k)!=right["bounds"].get(k))
        if model_diff != ["load"] or bound_diff != ["load_slope"]:
            raise ValueError("load comparison has a confounding parameter freedom or seed")
        checks.append({"factor":"load","measurement":measurement,"model_differences":model_diff,"bound_differences":bound_diff})
    for load in ("constant","affine"):
        left,right=(selected[f"{load}-current-{measurement}"] for measurement in ("fixed","free"))
        bound_diff=sorted(k for k in set(left["bounds"])|set(right["bounds"]) if left["bounds"].get(k)!=right["bounds"].get(k))
        if left["initial_model"] != right["initial_model"] or bound_diff != ["current_delay","current_tau"]:
            raise ValueError("current-measurement comparison has an unrelated seed or freedom")
        checks.append({"factor":"current_measurement","load":load,"model_differences":[],"bound_differences":bound_diff})
    # Reject split metadata leakage without opening any historical holdout file.
    ids=set(); sources=set()
    for role in ("train","selection","holdout"):
        for record in baseline["splits"][role]:
            source=str(Path(record["journal"]).resolve())
            if source in sources or record["physical_run_id"] in ids:
                raise ValueError("predeclared physical acquisition split leakage")
            sources.add(source);ids.add(record["physical_run_id"])
    return plans,selected,checks


def load_groups(plan):
    calibration_path=Path(plan["frozen_calibration_json"])
    calibration=json.loads(calibration_path.read_text(encoding="utf-8"))
    calibration=calibration.get("gyro_calibration",calibration)
    groups={};inventory={}
    for role in ("train","selection"):
        groups[role]=[];inventory[role]=[]
        for record in plan["splits"][role]:
            journal=load_yaw_journal(str(Path(record["journal"]).resolve()),gyro_calibration=calibration,
                encoder_datum_count=plan["encoder_datum_count"],physical_run_id=record["physical_run_id"],
                configuration_id=record["configuration_id"],calibration_revision=plan["calibration_revision"])
            run,summary=whole_run(journal,plan,calibration,plan["training_noise"])
            if role == "train" and journal.manifest.get("schema") == "adr0022.yaw-control/1":
                raise ValueError("closed-loop training requires separate estimator qualification")
            groups[role].append(run);inventory[role].append(summary)
    if not all(groups.values()):raise ValueError("nonempty TRAIN and SELECTION required")
    return groups,inventory


def save_predictions(native,model,groups,optimizer,directory):
    diagnostics=[]; retained=[]
    for role in ("train","selection"):
        for index,run in enumerate(groups[role]):
            initial=np.asarray(optimizer["initial_latent_states"][index]) if role=="train" else run.initial.copy()
            diagnostic_run=replace(run,initial=initial)
            target=directory/"trajectories"/(run.run_id+".npz")
            target.parent.mkdir(parents=True,exist_ok=True)
            try:
                prediction=native.rollout(model,run.t,run.tx_t,run.tx_A,initial)
            except Rejected as exc:
                rejected=target.with_name(run.run_id+"-rejected.json")
                write_json(rejected,{"role":role,"run_id":run.run_id,"reason":str(exc),"initial":initial.tolist(),
                    "prediction":"REJECTED","source_id":run.source_id})
                retained.append({"role":role,"run_id":run.run_id,"status":"REJECTED","file":str(rejected)})
                continue
            np.savez_compressed(target,t=run.t,prediction=prediction,encoder_q=run.q,gyro_v=run.v,
                decoded_current=run.current,q_new=run.q_new,v_new=run.v_new,current_new=run.current_new,
                tx_t=run.tx_t,tx_A=run.tx_A,initial=initial,prediction_columns=np.asarray(
                    ["q_rad","v_rad_s","effective_current_A","gyro_rad_s","reported_current_A","stick"]))
            report=diagnostic_report(diagnostic_run,prediction)
            filename=directory/"diagnostics"/(run.run_id+".json")
            write_json(filename,report)
            diagnostics.append({"role":role,"report":report,"file":str(filename)})
            retained.append({"role":role,"run_id":run.run_id,"status":"RETAINED_CONTINUOUS","file":str(target)})
    return diagnostics,retained


def summarize_candidate(row):
    selection_q=[r["q_rms_deg"] for r in row.get("selection",[]) if "q_rms_deg" in r]
    return {"cell":row["cell"],"optimizer_converged":row.get("optimizer",{}).get("success","NOT_RUN"),
        "optimizer_termination_reason":row.get("optimizer",{}).get("message",row.get("reason","NOT_RUN")),
        "nfev":row.get("optimizer",{}).get("evaluations"),"training_cost":row.get("optimizer",{}).get("cost"),
        "training_pass_count":sum(r["passed"] for r in row.get("training",[])),
        "training_run_count":len(row.get("training",[])),
        "selection_pass_count":sum(r["passed"] for r in row.get("selection",[])),
        "selection_run_count":len(row.get("selection",[])),
        "maximum_selection_q_rms_deg":max(selection_q,default=None),
        "no_motion_contradictions":sum(r["report"]["no_motion_model_on_moving_data"] for r in row.get("diagnostic_records",[])),
        "current_tau_s":row.get("model",{}).get("current_tau"),"current_delay_s":row.get("model",{}).get("current_delay"),
        "load_slope_A_rad":row.get("model",{}).get("load_slope"),"recovery":row.get("recovery"),
        "configuration_support":"UNKNOWN","physical_model_qualified":False,"deployment_authorized":False}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plans",type=Path,required=True)
    parser.add_argument("--library",type=Path,required=True)
    parser.add_argument("--output-dir",type=Path,required=True)
    parser.add_argument("--probe-only",action="store_true")
    args=parser.parse_args()
    if args.output_dir.exists() and any(args.output_dir.iterdir()):parser.error("choose fresh retained output directory")
    plans,candidates,checks=read_plans(args.plans)
    args.output_dir.mkdir(parents=True,exist_ok=True)
    base=plans[CELL_NAMES[0]]
    write_json(args.output_dir/"frozen-ablation-plan.json",{"schema":"adr0022.fair-physical-diagnostic/1",
        "created_at":datetime.now(timezone.utc).isoformat(),"source_plan_directory":str(args.plans),
        "crossed_factor_checks":checks,"cells":{name:{"source_plan":str(args.plans/(name+".json")),
            "candidate":candidates[name],"plan":plans[name]} for name in CELL_NAMES},
        "native_library":str(args.library),"max_nfev_each_cell":200,"loss":"unchanged native normalized Huber f_scale=1",
        "single_initial_state_per_whole_run":True,"train_only_fitting":True,
        "historical_holdout_observations":"NOT_LOADED","controller_synthesis":"NOT_RUN","physical_actions":False,
        "diagnostic_scope":"Confound removal only; pooled configuration/current/gyro support and expanded estimator scope unqualified",
        "deployment_authorized":False})
    groups,inventory=load_groups(base)
    write_json(args.output_dir/"input-inventory.json",{"splits":inventory,"historical_holdout":"NOT_LOADED",
        "noise":base["training_noise"],"configuration_support":"UNKNOWN","source_mode":"owner-confirmed CAN current control"})
    native=FamilyNative(args.library)
    # Early executable probe: all four declared initial models cross the actual
    # native boundary on one complete TRAIN acquisition before fitting.
    probe=[];run=groups["train"][0]
    for name in CELL_NAMES:
        prediction=native.rollout(FamilyModel(**candidates[name]["initial_model"]),run.t,run.tx_t,run.tx_A,run.initial)
        probe.append({"cell":name,"run_id":run.run_id,"native_rollout_finite":bool(np.isfinite(prediction).all()),
            "shape":list(prediction.shape),"state_resets":0})
    write_json(args.output_dir/"native-initial-probe.json",{"probe":probe,"passing_probe_does_not_qualify_model":True})
    if args.probe_only:
        print(json.dumps({"factor_checks":"PASS","initial_native_probes":len(probe),"fit":"NOT_RUN"}),flush=True)
        return
    results=[]
    for name in CELL_NAMES:
        candidate=candidates[name]; cell=args.output_dir/name
        cell.mkdir()
        write_json(cell/"exact-fit-plan.json",{"candidate":candidate,"max_nfev":200,"loss":"huber f_scale=1",
            "input_inventory":"../input-inventory.json","source_plan":str(args.plans/(name+".json")),
            "loaded_roles":["train","selection"],"holdout":"NOT_LOADED","physical_actions":False})
        progress_path=cell/"fit-progress.jsonl"
        def progress(stage,detail):
            with progress_path.open("a",encoding="utf-8") as stream:
                stream.write(json.dumps({"stage":stage,"at":datetime.now(timezone.utc).isoformat(),**detail},allow_nan=False)+"\n")
        began=time.monotonic()
        print(json.dumps({"stage":"fit_started","cell":name,"max_nfev":200}),flush=True)
        row={"cell":name,"label":candidate["label"]}
        try:
            fit=fit_family(native,FamilyModel(**candidate["initial_model"]),groups["train"],
                bounds={key:tuple(value) for key,value in candidate["bounds"].items()},max_nfev=200,progress=progress)
            selection=family_reports(native,fit["model"],groups["selection"])
            reports,retained=save_predictions(native,fit["model"],groups,fit["optimizer"],cell)
            row.update(model=fit["model"].document(),optimizer=fit["optimizer"],training=fit["training"],selection=selection,
                configuration_assessment=fit.get("configuration_assessment",{"support_status":"UNKNOWN"}),
                diagnostic_records=reports,retained_predictions=retained,objective_breakdown=diagnostic_summary(reports),
                training_passed=all(r["passed"] for r in fit["training"]),selection_passed=False)
            row["gates"]={"native_array_integrity":"PASS" if len(reports)==sum(map(len,groups.values())) else "FAIL",
                "physical_data_integrity":"UNKNOWN","forward_numerics":"UNKNOWN",
                "optimizer_termination_reason":fit["optimizer"]["message"],"optimizer_converged":fit["optimizer"]["success"],
                "synthetic_parameter_recovery":"NOT_RUN","training_trajectory":"PASS" if row["training_passed"] else "FAIL",
                "selection_trajectory":"PASS" if all(r["passed"] for r in selection) else "FAIL",
                "physical_configuration_support":"UNKNOWN","current_and_gyro_contract":"UNKNOWN",
                "historical_regression":"NOT_RUN","prospective_prediction":"NOT_RUN",
                "physical_stage3a":"NOT_RUN","physical_stage3b":"NOT_RUN","deployment_authorized":False}
            row["recovery"]=comparison_recovery(row,[r["report"] for r in reports])
        except Rejected as exc:
            row.update(reason=str(exc),training=[],selection=[],recovery=comparison_recovery(row))
        row.update(elapsed_s=time.monotonic()-began,qualification="NONQUALIFYING_DIAGNOSTIC",physical_actions=False,
            holdout_observations="NOT_LOADED",deployment_authorized=False)
        write_json(cell/"result.json",{k:v for k,v in row.items() if k!="diagnostic_records"})
        results.append(row)
        print(json.dumps({"stage":"fit_completed",**{k:v for k,v in summarize_candidate(row).items() if k!="recovery"}}),flush=True)
        write_json(args.output_dir/"checkpoint-summary.json",{"completed_cells":[summarize_candidate(r) for r in results],
            "unrun_cells":[x for x in CELL_NAMES if x not in [r["cell"] for r in results]],"deployment_authorized":False})
    summary=[summarize_candidate(r) for r in results]
    contrasts=[]
    costs={r["cell"]:r.get("optimizer",{}).get("cost") for r in results}
    for factor,fixed,a,b in (("load","current_fixed","constant-current-fixed","affine-current-fixed"),
            ("load","current_free","constant-current-free","affine-current-free"),
            ("current_measurement","constant","constant-current-fixed","constant-current-free"),
            ("current_measurement","affine","affine-current-fixed","affine-current-free")):
        before,after=costs[a],costs[b]
        contrasts.append({"factor":factor,"fixed_factor":fixed,"from":a,"to":b,
            "training_cost_delta":after-before if before is not None and after is not None else None,
            "identifiable_causal_effect":"UNQUALIFIED; bounded local solvers and physical support assumptions remain"})
    decision={"schema":"adr0022.fair-diagnostic-decision/1","crossed_factors_verified":checks,
        "cells":summary,"contrasts":contrasts,"changed_conclusion":"Load and current-measurement freedoms are now varied independently in four retained fits; their numerical effects can be inspected without the old structural confound",
        "interpretation":"No physical model or universal structure rejection follows from unqualified pooled context and nuisance/optimizer support",
        "next_action":"Use limiting event/objective and threshold diagnostics to choose estimator/input/configuration recovery; no identical rerun loop",
        "historical_holdout_observations":"NOT_LOADED","synthesis":"NOT_RUN","physical_actions":False,
        "physical_model_qualified":False,"WP4_passed":False,"deployment_authorized":False}
    write_json(args.output_dir/"comparison-decision.json",decision)
    print(json.dumps({"completed_cells":len(results),"WP4_passed":False,"historical_holdout":"NOT_LOADED"}),flush=True)


if __name__ == "__main__":main()
