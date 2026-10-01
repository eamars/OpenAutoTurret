"""Reproduce recorded yaw velocity validation; reporting only, no motor action."""
from pathlib import Path
import argparse
import csv
import json
import sys

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from Firmware.commissioning.yaw_validation import (
    TRACE_FIELDS, analyze_yaw_velocity, compare_recorded_metrics)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--journal",type=Path,required=True)
    parser.add_argument("--manifest",type=Path,help="otherwise use the actual embedded journal manifest")
    parser.add_argument("--candidate-context",type=Path,required=True,
                        help="existing candidate context containing measured_sensor_bandwidth_context")
    parser.add_argument("--expected-parameters",type=Path,
                        help="existing calculated controller_parameters or prior exact_parameters JSON")
    parser.add_argument("--compare-report",type=Path,help="existing actual-velocity-analysis.json for exact replay proof")
    parser.add_argument("--compare-transitions",type=Path,help="existing actual-motion-transitions.json for exact replay proof")
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--trace-output",type=Path)
    args = parser.parse_args()
    trace = None
    try:
        rows = [json.loads(line) for line in args.journal.read_text().splitlines() if line.strip()]
        manifest = json.loads(args.manifest.read_text()) if args.manifest else None
        context = json.loads(args.candidate_context.read_text())
        expected = None
        if args.expected_parameters:
            asset = json.loads(args.expected_parameters.read_text())
            expected = asset.get("controller_parameters",asset.get("exact_parameters"))
            if expected is None:
                raise ValueError("expected parameter asset has no controller_parameters or exact_parameters")
        report,trace = analyze_yaw_velocity(rows,manifest=manifest,
            bandwidth_context=context["measured_sensor_bandwidth_context"],expected_parameters=expected)
        if args.compare_report:
            previous = json.loads(args.compare_report.read_text())
            report["recorded_metric_reproduction"] = compare_recorded_metrics(report,previous)
        if args.compare_transitions:
            previous = json.loads(args.compare_transitions.read_text())
            current = json.loads(json.dumps(report["motion_transitions"],allow_nan=False))
            keys = ("starts","START_to_MOVE_releases","all_motion_groups","plateau_quiet_samples",
                    "plateau_quiet_timestamp_exposure_s","plateau_fastest_200ms_encoder_velocity_sample",
                    "plateau_fastest_independent_gyro_sample","sampled_low_motion_convention","controlled_stop")
            matches = {key:current[key]==previous[key] for key in keys}
            report["recorded_transition_reproduction"] = {"all_fields_match":all(matches.values()),"field_matches":matches}
    except Exception as exc:
        report = {"schema":"adr0022.actual-yaw-velocity-analysis/1","analysis_status":"REPORT_INCOMPLETE",
                  "retained_analysis_failure":{"reason":type(exc).__name__,"detail":str(exc)},
                  "motor_action":"NONE","physical_qualification":False}
    report.update(source_journal=str(args.journal),
        source_manifest=str(args.manifest) if args.manifest else "embedded_journal_header",
        source_candidate_context=str(args.candidate_context),
        source_expected_parameters=str(args.expected_parameters) if args.expected_parameters else None,
        raw_journal_modified=False)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    if args.trace_output and trace is not None:
        with args.trace_output.open("w",newline="") as output:
            writer = csv.writer(output)
            writer.writerow(TRACE_FIELDS)
            writer.writerows(trace)
    args.output.write_text(json.dumps(report,indent=2,allow_nan=False)+"\n")
    print(json.dumps({"analysis_status":report["analysis_status"],"output":str(args.output),
        "frozen_motion_metrics":report.get("frozen_motion_metrics"),
        "numerical_readback_matches_captured_manifest":report.get("numerical_readback_matches_captured_manifest"),
        "exact_parameters_match_expected_export":report.get("exact_parameters_match_expected_export"),
        "recorded_metric_reproduction":report.get("recorded_metric_reproduction"),
        "recorded_transition_reproduction":report.get("recorded_transition_reproduction"),
        "retained_analysis_failure":report.get("retained_analysis_failure"),"motor_action":"NONE"},indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
