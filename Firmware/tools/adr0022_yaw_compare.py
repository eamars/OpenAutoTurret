"""Compare predeclared yaw structures on complete, blocked physical journals.

This offline entry never generates gains or writes station configuration. Existing
data are retrospective evidence; prospective qualification remains a separate step.
"""
from __future__ import annotations

import argparse
from dataclasses import fields
import json
from pathlib import Path
import sys
from datetime import datetime, timezone

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from Firmware.commissioning.model_family import FamilyModel, FamilyNative, FamilyRun, compare_families
from Firmware.commissioning.recovery import comparison_recovery, diagnostic_report
from Firmware.commissioning.yaw_events import load_yaw_journal


def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def diagnostic_summary(records):
    """Retain run/channel/regime information without changing fitting weights."""
    result = {}
    for role in ("train", "selection"):
        rows = [r for r in records if r["role"] == role]
        total = sum(r["report"]["objective"]["total_huber_cost"] for r in rows)
        result[role] = {"total_huber_cost": total, "used_to_fit": role == "train", "runs": []}
        for row in rows:
            report = row["report"]
            cost = report["objective"]["total_huber_cost"]
            result[role]["runs"].append({"run_id": report["run_id"], "huber_cost": cost,
                "fraction_of_role_cost": cost / total if total else 0.,
                "objective_channels": report["objective"]["channels"],
                "baselines": report["baselines"], "displacement": report["displacement"],
                "no_motion_model_on_moving_data": report["no_motion_model_on_moving_data"],
                "observed_moving_duration_s": report["observed_motion"]["moving_duration_s"],
                "predicted_moving_duration_s": report["predicted_motion"]["moving_duration_s"],
                "event_count": {kind: sum(e["kind"] == kind for e in report["observed_motion"]["events"])
                    for kind in ("observed_motion_onset", "observed_reversal", "observed_stop")},
                "diagnostic_file": row["file"]})
    return result


def diagnose_retained(comparison_paths, output):
    """Analyze frozen complete predictions; no solver, native library or holdout."""
    output = Path(output)
    if output.exists() and any(output.iterdir()):
        raise ValueError("diagnostic output contains retained evidence; choose a fresh directory")
    result = {"schema": "adr0022.retained-family-diagnostics/1", "comparisons": {},
        "refit": "NOT_RUN", "holdout": "NOT_RUN", "physical_trials": "NOT_RUN",
        "acceptance_criteria_changed": False, "deployment_authorized": False}
    for source in comparison_paths:
        source = Path(source)
        name = source.name
        if name in result["comparisons"]:
            raise ValueError("retained comparison directory names must be distinct")
        comparison = json.loads((source / "comparison-result.json").read_text(encoding="utf-8"))
        plan = json.loads((source / "comparison-plan.json").read_text(encoding="utf-8"))
        noise = plan["training_noise"]
        roles = {r["physical_run_id"]: role for role in ("train", "selection")
                 for r in plan["splits"][role]}
        candidates = []
        for candidate in comparison["comparisons"]:
            records = []
            for trajectory in sorted((source / "trajectories" / candidate["label"]).glob("*.npz")):
                if trajectory.stem not in roles:
                    raise ValueError("retained diagnostics cannot inspect final holdout trajectories")
                with np.load(trajectory, allow_pickle=False) as values:
                    run = FamilyRun(run_id=trajectory.stem, physical_run_id=trajectory.stem,
                        source_id=str(trajectory.resolve()), t=values["t"], q=values["encoder_q"],
                        v=values["gyro_v"], current=values["decoded_current"], q_new=values["q_new"],
                        v_new=values["v_new"], current_new=values["current_new"],
                        tx_t=values["tx_t"], tx_A=values["tx_A"], initial=values["initial"],
                        sigma_q=noise["sigma_q_rad"], sigma_v=noise["sigma_v_rad_s"],
                        sigma_current=noise["sigma_current_A"], encoder_quantum=2*np.pi/8192)
                    report = diagnostic_report(run, values["prediction"])
                filename = Path(name) / candidate["label"] / (run.run_id + ".json")
                write_json(output / filename, report)
                records.append({"role": roles[trajectory.stem], "report": report, "file": str(filename)})
            expected = len(roles) if "model" in candidate else 0
            if len(records) != expected:
                raise ValueError("missing frozen train/selection predictions; preserve failure rather than omit a run")
            candidates.append({"label": candidate["label"],
                "optimizer_termination": candidate.get("optimizer", {}).get("message", "UNKNOWN"),
                "objective_breakdown": diagnostic_summary(records),
                "recovery": comparison_recovery(candidate, [r["report"] for r in records])})
        result["comparisons"][name] = {"source": str(source), "candidates": candidates,
            "source_outcome": comparison["outcome"], "selection_changed": False}
    write_json(output / "diagnostic-summary.json", result)
    return result


def whole_run(journal, config, calibration, noise):
    arrays = journal.fitter_arrays()
    # Only a leading baseline can initialize unavailable delayed state history.
    # Retain it in the canonical journal; no motion/regime window is removed.
    delay = float(config["maximum_timing_search_s"])
    observations = journal.union_observations(delay_bound_s=delay)
    start = observations["initial_support"]["start_s_from_session_begin"]
    originals = [e["raw"] for e in journal.events if e["channel"] == "raw"]
    excitation = next(r["time_ns"] for r in originals
                      if r.get("kind") in ("excitation_begin", "yaw_control_begin"))
    if journal.origin_ns + round(start * 1e9) >= excitation:
        raise ValueError("latent-state initialization would remove motion rather than baseline")
    times = observations["t"]
    end = start + times[-1]
    values = [observations[k] for k in ("q_obs", "v_obs", "current_obs")]
    masks = [observations[k] for k in ("q_new", "v_new", "current_new")]
    initial = observations["initial"]
    # Initial latent states are estimated once per training run inside measured
    # baseline noise bounds. Held-out runs keep the causal seeds unchanged.
    widths = np.array([3*noise["sigma_q_rad"], 3*noise["sigma_v_rad_s"],
                       3*noise["sigma_current_A"], 3*noise["sigma_v_rad_s"], 3*noise["sigma_current_A"]])
    run = FamilyRun.from_union(observations, sigma_q=noise["sigma_q_rad"],
        sigma_v=noise["sigma_v_rad_s"], sigma_current=noise["sigma_current_A"],
        initial_bounds=(initial-widths, initial+widths), encoder_quantum=2*np.pi/8192)
    summary = {
        "physical_run_id": journal.physical_run_id, "source_journal": journal.source_journal,
        "configuration_id": journal.configuration_id, "calibration_revision": journal.calibration_revision,
        "original_record_count": len(originals), "retained_native_observations": {
            "encoder": int(masks[0].sum()), "gyro": int(masks[1].sum()), "current": int(masks[2].sum())},
        "total_native_observations": {"encoder": len(arrays["encoder_t_s"]),
                                      "gyro": len(arrays["gyro_t_s"]), "current": len(arrays["current_t_s"])},
        "successful_tx_events_with_prehistory": len(arrays["tx_t_s"]),
        "initialization_baseline_s": float(start), "excitation_from_origin_s": (excitation-journal.origin_ns)*1e-9,
        "prediction_span_s": float(end-start), "initial_state": initial.tolist(),
        "initial_support": observations["initial_support"],
        "initial_state_source": "last observations at or before baseline start; no future motion",
        "initial_current_interpretation": "reported-current scale is a hypothesis, not calibrated torque current",
        "registration": arrays["registration"], "clock_status": arrays["clock_status"],
        "all_motion_and_stop_regimes_retained": True,
        "encoder_range_rad": [float(np.nanmin(values[0])), float(np.nanmax(values[0]))],
        "posture_receipts": len(arrays["pitch_t_s"]),
        "posture_range_rad": [float(arrays["pitch_rad"].min()), float(arrays["pitch_rad"].max())]
            if len(arrays["pitch_rad"]) else None,
    }
    return run, summary


def execute(config_path, library, output, max_nfev):
    output = Path(output)
    if output.exists() and any(output.iterdir()):
        raise ValueError("comparison output contains retained evidence; choose a fresh directory")
    config = json.loads(Path(config_path).read_text(encoding="utf-8"))
    if config.get("schema") != "adr0022.yaw-family-comparison/1":
        raise ValueError("unsupported comparison config schema")
    calibration_path = Path(config["frozen_calibration_json"])
    calibration = json.loads(calibration_path.read_text(encoding="utf-8"))
    calibration = calibration.get("gyro_calibration", calibration)
    native = FamilyNative(library)
    groups, inventory = {}, {}
    sources, identities = {}, {}
    for role in ("train", "selection", "holdout"):
        groups[role], inventory[role] = [], []
        for record in config["splits"][role]:
            source = str(Path(record["journal"]).resolve())
            physical_id = record["physical_run_id"]
            if source in sources or physical_id in identities:
                raise ValueError("split leakage: a physical journal/run occurs more than once")
            sources[source], identities[physical_id] = role, role
            journal = load_yaw_journal(source, gyro_calibration=calibration,
                encoder_datum_count=config["encoder_datum_count"], physical_run_id=physical_id,
                configuration_id=record["configuration_id"], calibration_revision=config["calibration_revision"])
            run, summary = whole_run(journal, config, calibration, config["training_noise"])
            if role == "train" and journal.manifest.get("schema") == "adr0022.yaw-control/1":
                raise ValueError("closed-loop training requires a qualified estimator; use current-excitation runs")
            groups[role].append(run); inventory[role].append(summary)
    if any(not group for group in groups.values()):
        raise ValueError("nonempty whole-run train/selection/holdout blocks required")
    write_json(Path(output)/"input-inventory.json", {"schema": config["schema"], "splits": inventory,
               "calibration_source": str(calibration_path), "split_policy": "whole physical journals before fit",
               "calibration_scope": config["calibration_scope"],
               "historical_holdout_exposure": "all archived data were previously examined; prospective runs remain required"})
    candidates = []
    for candidate in config["candidates"]:
        candidates.append((candidate["label"], FamilyModel(**candidate["initial_model"]),
                           {k: tuple(v) for k,v in candidate["bounds"].items()}))
    # Save the exact numerical plan before fitting. Final holdout is not fitted.
    write_json(Path(output)/"comparison-plan.json", {**config, "started_at": datetime.now(timezone.utc).isoformat(),
               "max_nfev": max_nfev, "controller_synthesis": "NOT_RUN", "physical_trials": "NOT_RUN"})
    progress_path = Path(output)/"fit-progress.jsonl"
    def progress(stage, detail):
        record = {"stage": stage, "at": datetime.now(timezone.utc).isoformat(), **detail}
        with progress_path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(record, allow_nan=False) + "\n")
        if stage in ("family_started", "family_completed"):
            print(json.dumps({"stage": stage, "label": detail.get("label"),
                              "selection_passed": detail.get("selection_passed")}), flush=True)
        if stage == "family_completed" and "model" in detail:
            model = FamilyModel(**{f.name:detail["model"][f.name] for f in fields(FamilyModel)})
            prediction_dir = Path(output)/"trajectories"/detail["label"]
            prediction_dir.mkdir(parents=True, exist_ok=True)
            diagnostics = []
            for role in ("train", "selection"):
                for index, run in enumerate(groups[role]):
                    initial = (detail["optimizer"]["initial_latent_states"][index]
                               if role == "train" else run.initial)
                    try:
                        prediction = native.rollout(model, run.t, run.tx_t, run.tx_A, initial)
                    except ValueError as exc:
                        write_json(prediction_dir/(run.run_id+"-rejected.json"),
                                   {"role":role, "source_id":run.source_id, "reason":str(exc)})
                        continue
                    # Sparse observation arrays retain native masks; these values
                    # are saved for analysis and never fed into the forward rollout.
                    np.savez_compressed(prediction_dir/(run.run_id+".npz"),
                        t=run.t, prediction=prediction, encoder_q=run.q, gyro_v=run.v,
                        decoded_current=run.current, q_new=run.q_new, v_new=run.v_new,
                        current_new=run.current_new, tx_t=run.tx_t, tx_A=run.tx_A,
                        initial=np.asarray(initial), prediction_columns=np.array(
                            ["q_rad","v_rad_s","effective_current_A","gyro_rad_s","reported_current_A","stick"]))
                    diagnostic_run = FamilyRun(**{**vars(run), "initial": np.asarray(initial)})
                    report = diagnostic_report(diagnostic_run, prediction)
                    filename = Path("diagnostics")/detail["label"]/(run.run_id+".json")
                    write_json(Path(output)/filename, report)
                    diagnostics.append({"role": role, "report": report, "file": str(filename)})
            detail["objective_breakdown"] = diagnostic_summary(diagnostics)
            detail["recovery"] = comparison_recovery(detail, [r["report"] for r in diagnostics])
    result = compare_families(native, candidates, groups["train"], groups["selection"], groups["holdout"],
                              max_nfev=max_nfev, progress=progress)
    result.update(schema="adr0022.yaw-family-comparison-result/1", completed_at=datetime.now(timezone.utc).isoformat(),
                  input_inventory="input-inventory.json", model_plan="comparison-plan.json",
                  calibration_scope=config["calibration_scope"],
                  deployment_eligible=False, qualification="UNQUALIFIED",
                  candidate14="NONDEPLOYABLE; NEVER_PHYSICALLY_RUN",
                  physical_trials="NOT_RUN", controller_synthesis="NOT_RUN",
                  remaining_verification=["physical current meaning and torque relation",
                     "absolute sensor timing/filter and cross-session coordinate qualification",
                     "synthetic closed-loop estimator validity", "prospective frozen forecasts and physical Stage 3a/3b",
                     "both axes and complete configuration/domain coverage"])
    write_json(Path(output)/"comparison-result.json", result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path)
    parser.add_argument("--library", type=Path)
    parser.add_argument("--diagnose-retained", type=Path, action="append",
                        help="frozen comparison directory; repeat for additional retained comparisons")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-nfev", type=int, default=200)
    args = parser.parse_args()
    if args.max_nfev < 1:
        parser.error("positive optimizer budget required")
    if args.diagnose_retained:
        if args.config or args.library:
            parser.error("retained diagnostics do not use a fit config or native library")
        result = diagnose_retained(args.diagnose_retained, args.output)
        print(json.dumps({"diagnosed_comparisons": len(result["comparisons"]),
            "refit": "NOT_RUN", "holdout": "NOT_RUN", "deployment_authorized": False}, indent=2))
    else:
        if not args.config or not args.library:
            parser.error("a fit requires both --config and --library")
        result = execute(args.config, args.library, args.output, args.max_nfev)
        print(json.dumps({k:result.get(k) for k in ("outcome", "selected_label", "qualification", "deployment_eligible")}, indent=2))


if __name__ == "__main__":
    main()
