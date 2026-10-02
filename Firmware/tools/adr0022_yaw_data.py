"""Describe a yaw control journal without qualifying motion or sensor noise."""
from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path
import statistics


def describe(values):
    values = list(values)
    if not values:
        return {"count": 0}
    return {"count": len(values), "minimum": min(values), "maximum": max(values),
            "mean": statistics.fmean(values), "median": statistics.median(values),
            "standard_deviation": statistics.pstdev(values)}


def load_capture(journal: Path, manifest: Path | None = None):
    journal = Path(journal)
    if journal.is_dir():
        directory = journal
        journal = directory / "capture.jsonl"
        if not journal.exists():
            journal = directory / "yaw-control.jsonl"
        if manifest is None and (directory / "manifest.json").exists():
            manifest = directory / "manifest.json"
    rows = [json.loads(line) for line in journal.read_text().splitlines() if line.strip()]
    header = rows[0] if rows else {}
    config = json.loads(Path(manifest).read_text()) if manifest else json.loads(header.get("manifest_yaml", "{}"))
    return journal, config, rows


def summarize(journal: Path, manifest: Path | None = None) -> dict:
    journal, config, rows = load_capture(journal, manifest)
    footer = rows[-1] if rows else {}
    yaw = [row for row in rows if row.get("kind") == "yaw_feedback"]
    tx = [row for row in rows if row.get("kind") == "yaw_current_tx"]
    successful = [row for row in tx if row.get("success")]
    imu = [json.loads(row["raw_json"]) for row in rows if row.get("kind") == "imu_raw"]
    samples = [row for row in imu if row.get("kind") == "sample"]
    phase_commands = {}
    for phase in ("baseline", "excitation", "stop"):
        commands = [row for row in successful if row.get("phase") == phase]
        phase_commands[phase] = {
            "successful_transmissions": len(commands),
            "actual_current_A": describe(row["successful_tx_A"] for row in commands),
            "first_kernel_accepted_ns": commands[0]["kernel_accepted_ns"] if commands else None,
            "last_kernel_accepted_ns": commands[-1]["kernel_accepted_ns"] if commands else None,
        }
    stamps = [row["kernel_monotonic_ns"] for row in yaw]
    excitation = phase_commands["excitation"]["first_kernel_accepted_ns"]
    stopped = phase_commands["stop"]["first_kernel_accepted_ns"]
    before_motion = [row for row in yaw if excitation is not None and row["kernel_monotonic_ns"] <= excitation]
    before_stop = [row for row in yaw if stopped is not None and row["kernel_monotonic_ns"] <= stopped]
    after_stop = [row for row in yaw if stopped is not None and row["kernel_monotonic_ns"] >= stopped]
    first = yaw[0] if yaw else None
    motion_start = before_motion[-1] if before_motion else first
    motion_end = before_stop[-1] if before_stop else None
    final = yaw[-1] if yaw else None
    displacement = {}
    if first and final:
        displacement = {
            "first_encoder_raw": first["encoder_raw"], "final_encoder_raw": final["encoder_raw"],
            "total_relative_rad": final["q_relative_rad"] - first["q_relative_rad"],
            "first_receipt_ns": first["kernel_monotonic_ns"],
            "final_receipt_ns": final["kernel_monotonic_ns"],
        }
        if motion_start and motion_end:
            displacement["excitation_relative_rad"] = motion_end["q_relative_rad"] - motion_start["q_relative_rad"]
        if motion_end:
            displacement["drift_after_zero_rad"] = final["q_relative_rad"] - motion_end["q_relative_rad"]
            displacement["observed_after_zero_s"] = max(0, (final["kernel_monotonic_ns"] - stopped) / 1e9)
    return {
        "schema": "adr0022.yaw-data-summary/1", "provenance": config.get("provenance"),
        "journal": str(journal.resolve()), "manifest": str(Path(manifest).resolve()) if manifest else "capture header",
        "capture_complete": footer.get("status") == "COMPLETE" and footer.get("sequence_complete") is True,
        "capture_footer_status": footer.get("status"), "capture_footer_detail": footer.get("detail"),
        "footer": footer, "record_counts": dict(Counter(row.get("kind", "unknown") for row in rows)),
        "imu_sample_counts": dict(Counter(row["sensor"] for row in samples)),
        "gyro_accuracy_counts": dict(Counter(str(row["status"]) for row in samples if row["sensor"] == "gyro")),
        "yaw_feedback": {
            "encoder_raw": describe(row["encoder_raw"] for row in yaw),
            "current_raw": describe(row["current_raw"] for row in yaw),
            "reported_current_A": describe(row["current_A"] for row in yaw),
            "temperature_raw": describe(row["temperature_raw"] for row in yaw),
            "speed_rpm": describe(row["speed_rpm"] for row in yaw),
            "receipt_interval_s": describe((b-a)/1e9 for a, b in zip(stamps, stamps[1:])),
            "dequeue_delay_s": describe((row["dequeue_ns"]-row["kernel_monotonic_ns"])/1e9 for row in yaw),
            "stop_feedback_samples": len(after_stop),
        },
        "current_commands": phase_commands,
        "tx_duration_s": describe((row["kernel_accepted_ns"]-row["begin_ns"])/1e9 for row in successful),
        "displacement": displacement,
        "qualification": {"dynamics": False, "current_mapping": False, "stopping": False},
        "interpretation": "Acquisition completion records the requested sequence and zero-current observation. Dynamics, current mapping and stopping require subsequent physical analysis; raw noise and gyro accuracy zero are retained.",
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--journal", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, help="optional: otherwise use the journal header manifest")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = summarize(args.journal, args.manifest)
    serialized = json.dumps(result, indent=2) + "\n"
    if args.output:
        args.output.write_text(serialized)
    print(serialized, end="")


if __name__ == "__main__":
    main()
