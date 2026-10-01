"""Calculate a deterministic measured yaw update; this program never drives CAN."""
from __future__ import annotations

import argparse
from copy import deepcopy
import json
import math
from pathlib import Path
import sys

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from Firmware.commissioning.contracts import Rejected
from Firmware.commissioning.native import Native
from Firmware.commissioning.yaw_local_update import feedback_windows, update
from adr0022_yaw_information import fitted_training_runs, read_stream


def save(path,value):
    path.write_text(json.dumps(value,indent=2,allow_nan=False)+"\n",encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fit-json",type=Path,required=True)
    parser.add_argument("--journal",type=Path,action="append",required=True)
    parser.add_argument("--output-dir",type=Path,required=True)
    parser.add_argument("--native-library",type=Path)
    parser.add_argument("--measured-model-json",type=Path,action="append",default=[],
                        help="Additional existing measured model context to preserve as a provisional stress hypothesis")
    parser.add_argument("--delay-bound-s",type=float,help="Existing recorded finite timing search bound; subsequent update contexts retain this field")
    parser.add_argument("--acceleration-cap-rad-s2",type=float,default=math.radians(30),
                        help="Owner-declared planning limit, default 30 degrees/s²; this is an envelope binding, not a fitted gain")
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True,exist_ok=True)
    prior = json.loads(args.fit_json.read_text())
    native_path = args.native_library or Path(prior["native_library"])
    delay_bound = args.delay_bound_s if args.delay_bound_s is not None else prior["delay_search_bound_s"]
    descriptions, captures = [], []
    selection = {"source_prior":str(args.fit_json),"source_journals":[str(path) for path in args.journal],
                 "holdout_used":False,"gyro_calibration_changed":False,"station_actions":False}
    try:
        for journal in args.journal:
            stream = read_stream(journal)
            rows = [json.loads(line) for line in journal.read_text().splitlines()]
            windows, capture = feedback_windows(stream,rows,prior["gyro_calibration"])
            descriptions.extend(windows)
            captures.append(capture)
        selection.update(training_run_descriptions=descriptions,captures=captures)
        save(args.output_dir/"actual-window-selection.json",selection)
        reconstruction = deepcopy(prior)
        reconstruction["training_journals"] = [str(path) for path in args.journal]
        reconstruction["training_run_descriptions"] = descriptions
        _,runs,support = fitted_training_runs(reconstruction,args.fit_json,journals=args.journal)
        for description,actual in zip(descriptions,support):
            description.update(actual)
        save(args.output_dir/"actual-window-selection.json",selection)
        def progress(stage,value):
            save(args.output_dir/("information-report.json" if stage == "information" else "initializer-report.json"),value)
            print(json.dumps({"stage":stage,"branch":value["branch"],"periodic_information":value["periodic_information"],
                              "local_information":value["local_information"],
                              "observed_periodic_columns":value["observed_periodic_columns"],
                              "observed_periodic_information":value["observed_periodic_information"],
                              "local_dynamic_information":value["local_dynamic_information"]}),flush=True)
        report,context = update(prior,runs,descriptions,Native(native_path),delay_bound_s=delay_bound,
                               source_path=args.fit_json,native_path=native_path,progress=progress,
                               additional_model_paths=args.measured_model_json)
        report["capture_observations"] = captures
        last = read_stream(args.journal[-1])["pitch"][-1]
        context["fixed_pitch_rad"] = last["value"]
        context["current_planning_pose"] = {"journal":str(args.journal[-1]),"actual_MechPos_rad":last["value"],
            "receive_ns":last["receive_ns"],"scope":"Last measured MechPos is the current planning pose; time-varying training readbacks remain intact"}
        context["measured_gyro_observation_metadata"] = [row["gyro_observation_metadata"] for row in captures]
        context["actual_controller_parameter_readbacks"] = [row["actual_controller_parameters"] for row in captures]
        context["training_capture_support"] = captures
        context["all_windows"] = [window for row in captures for window in row["all_motion_decisions"]]
        observed = captures[-1]["gyro_observation_metadata"]
        context["acceleration_guard_observation"] = {"acceleration_cap_rad_s2":args.acceleration_cap_rad_s2,
            "acceleration_noise_sigma_rad_s2":observed["baseline_acceleration_sigma_rad_s2"],
            "acceleration_sample_period_s":observed["sample_period_s"],
            "cap_source":"Explicit owner 30 degrees/s² planning envelope or CLI binding",
            "noise_and_period_source":"Recorded actual gyro sample timestamps, frozen yaw projection, measured pre-control baseline"}
        save(args.output_dir/"numerical-fit.json",report)
        save(args.output_dir/"candidate-context.json",context)
        print(json.dumps({"status":"ACTUAL_YAW_UPDATE_COMPLETED_UNQUALIFIED","branch":report["branch"],
            "local_h_offsets_negative_positive_A":report.get("local_h_offsets_negative_positive_A"),
            "local_a_b":report.get("local_a_b"),
            "optimizer_evaluations":report["optimizer_evaluations"],"failure_classification":report["failure_classification"],
            "local_q_support_rad":report["local_q_support_rad"],"actual_posture_support_rad":report["actual_posture_support_rad"]}),flush=True)
        return 0
    except Rejected as exc:
        failure = {"status":"ACTUAL_YAW_UPDATE_BLOCKER","reason":exc.reason.value,"detail":exc.detail,
                   "selection":selection,"formal_qualification":False}
        save(args.output_dir/"failure.json",failure)
        print(json.dumps({key:failure[key] for key in ("status","reason","detail")}),flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
