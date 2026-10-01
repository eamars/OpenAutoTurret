"""Save a reporting-only actual yaw IMU acceleration/vibration observation."""
from pathlib import Path
import argparse
import json
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--journal", type=Path)
    parser.add_argument("--manifest", type=Path)
    parser.add_argument("--calibration", type=Path, help="existing frozen calibration JSON; never fitted here")
    parser.add_argument("--acceleration-cap-deg-s2", type=float, help="explicit owner analysis constraint for historical captures")
    parser.add_argument("--sustained-s", type=float, help="actual runtime/analysis support, e.g. existing0.06s")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.journal is None and args.manifest is None:
        parser.error("--journal or --manifest is required")
    journal = args.journal
    manifest = None
    try:
        if args.manifest:
            manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
            if journal is None:
                journal = Path(manifest["output"])
        output = args.output or journal.with_suffix(".vibration.json")
        rows = [json.loads(line) for line in journal.read_text(encoding="utf-8").splitlines() if line.strip()]
        calibration = json.loads(args.calibration.read_text(encoding="utf-8")) if args.calibration else None
        from Firmware.commissioning.yaw_vibration import analyze_yaw_vibration
        import math
        report = analyze_yaw_vibration(rows, manifest=manifest, calibration=calibration,
            acceleration_cap=math.radians(args.acceleration_cap_deg_s2) if args.acceleration_cap_deg_s2 is not None else None,
            sustained_s=args.sustained_s,
            constraint_source="explicit_owner_historical_analysis_constraint" if args.acceleration_cap_deg_s2 is not None else None)
    except Exception as exc:
        output = args.output or (journal.with_suffix(".vibration.json") if journal else args.manifest.with_suffix(".vibration.json"))
        report = {"schema": "adr0022.yaw-acceleration-vibration-report/1", "status": "REPORT_INCOMPLETE",
                  "retained_measurement_failures": [{"reason": type(exc).__name__, "detail": str(exc)}],
                  "motor_action": "NONE", "qualification": False}
    report.update(source_journal=str(journal) if journal else None,
                  source_manifest=str(args.manifest) if args.manifest else "embedded_journal_header",
                  source_calibration=str(args.calibration) if args.calibration else "manifest_gyro_calibration",
                  raw_journal_modified=False)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    print(json.dumps({"status": report["status"], "output": str(output),
                      "measurement_failure_records": len(report.get("retained_measurement_failures", [])),
                      "acceleration_cap_available": report.get("constraints", {}).get("acceleration_cap_rad_s2") is not None,
                      "cap_exceedance_observed_count": (report.get("capture_phases", {}).get("command_body", {}).get("cap_exceedance_observed_count")
                          if report.get("constraints", {}).get("acceleration_cap_rad_s2") is not None else None),
                      "motor_action": "NONE"}))
    # A failed or unavailable measurement remains a report, not a motor/session gate.
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
