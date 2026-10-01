"""Generate finite yaw acquisition using the owner's maximum-first signal rule."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from Firmware.commissioning.adaptation import first_stimulus


def initial_plan(*, label, approved_minimum_a, current_bound_a, factor_index,
                 pulse_s, baseline_s, zero_between_s, startup_s, duration_s,
                 hold_s=2., maximum_first=True):
    amplitude = float(first_stimulus([approved_minimum_a],
                      SimpleNamespace(current_a=current_bound_a), previous_factor=factor_index,
                      maximum_first=maximum_first)[0])
    half = pulse_s / 2
    segments = [
        {"duration_s": half, "start_A": 0., "end_A": amplitude},
        {"duration_s": hold_s, "start_A": amplitude, "end_A": amplitude},
        {"duration_s": half, "start_A": amplitude, "end_A": 0.},
        {"duration_s": zero_between_s, "start_A": 0., "end_A": 0.},
        {"duration_s": half, "start_A": 0., "end_A": -amplitude},
        {"duration_s": hold_s, "start_A": -amplitude, "end_A": -amplitude},
        {"duration_s": half, "start_A": -amplitude, "end_A": 0.},
    ]
    return {
        "schema": "adr0022.yaw-acquisition/1", "purpose": "yaw_current_identification",
        "provenance": "MEASURED", "transport": "socketcan", "session_label": label,
        "source_description": "ADR-002.2 yaw acquisition; workstation ARM64 build; current working-tree source",
        "yaw": {"interface": "can0"}, "pitch": {"interface": "can1"},
        "expected_pitch_uid": "7216313130333105", "pitch_supported_when_disabled": True,
        "baseline_s": baseline_s, "stop_observation_s": 2.,
        "yaw_current_bound_A": current_bound_a, "pitch_maximum_temperature_C": 45.,
        "current_segments": segments,
        "signal_selection": {"method": "first_stimulus", "maximum_first": maximum_first,
                             "factor_index": factor_index, "selected_amplitude_A": amplitude,
                             "purpose": "actual body rotation and dynamic information in both directions",
                             "plant_prediction_available": False,
                             "capability_basis": "owner maximum-first finite acquisition; initial stationary-condition allowance follows the continuous stall rating",
                             "manufacturer_maximum_continuous_rating_A": 1.62,
                             "continuous_stall_rating_A": .9,
                             "rating_current_definition": "manufacturer labels; rating-to-command-Iq equivalence not independently calibrated",
                             "feedback_current_conversion_qualified": False,
                             "fixed_displacement_required": False},
        "operator_attendance": {"present_at_manual_cutoff": None, "operator_identity": "owner",
                                "manual_cutoff_evidence_identity": "attendance not observed by remote operator"},
        "session_authorization": {"purpose": "yaw_current_identification", "yaw_acquisition_authorized": True,
                                  "authorization_identity": "owner instruction in this chat, 2026-09-30",
                                  "unattended_operation_authorized": True, "presence_required": False},
        "limits": {"clock_uncertainty_s": .001, "dequeue_age_s": .1,
                   "can_gap_s": .1, "imu_gap_s": .2, "startup_s": startup_s,
                   "duration_s": duration_s, "minimum_imu_status": 0,
                   "read_timeout_s": .2, "read_period_s": .02, "stop_period_s": .02},
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--session-label", required=True)
    parser.add_argument("--approved-minimum-A", type=float, default=.25)
    parser.add_argument("--current-bound-A", type=float, default=.9)
    parser.add_argument("--factor-index", type=int, default=0)
    parser.add_argument("--pulse-s", type=float, default=1.)
    parser.add_argument("--hold-s", type=float, default=2.)
    parser.add_argument("--minimum-first", action="store_true", help="explicit legacy initial-stimulus policy")
    parser.add_argument("--baseline-s", type=float, default=2.)
    parser.add_argument("--zero-between-s", type=float, default=2.)
    parser.add_argument("--startup-s", type=float, default=20.)
    parser.add_argument("--duration-s", type=float, default=40.)
    parser.add_argument("--displacement-deg", type=float)
    parser.add_argument("--maximum-excitation-s", type=float, default=15.)
    args = parser.parse_args()
    config = initial_plan(label=args.session_label, approved_minimum_a=args.approved_minimum_A,
                          current_bound_a=args.current_bound_A, factor_index=args.factor_index,
                          pulse_s=args.pulse_s, baseline_s=args.baseline_s, zero_between_s=args.zero_between_s,
                          startup_s=args.startup_s, duration_s=args.duration_s,
                          hold_s=args.hold_s, maximum_first=not args.minimum_first)
    if args.displacement_deg is not None:
        import math
        if not math.isfinite(args.displacement_deg) or args.displacement_deg == 0:
            raise ValueError("displacement must be finite and nonzero")
        if not math.isfinite(args.maximum_excitation_s) or args.maximum_excitation_s <= .5:
            raise ValueError("maximum excitation must exceed the initial ramp")
        amplitude = math.copysign(config["signal_selection"]["selected_amplitude_A"], args.displacement_deg)
        config["yaw_displacement_target_deg"] = args.displacement_deg
        config["current_segments"] = [
            {"duration_s": .5, "start_A": 0., "end_A": amplitude},
            {"duration_s": args.maximum_excitation_s - .5, "start_A": amplitude, "end_A": amplitude},
        ]
        config["signal_selection"].update({
            "purpose": "owner-requested additional yaw displacement sample before information selection",
            "requested_displacement_deg": args.displacement_deg,
            "capability_basis": "GM6020 continuous current allowance; existing program-selected starting current",
        })
    from adr0022_capture_launch import validate_yaw_contract
    validate_yaw_contract(config)
    with args.output.open("x", encoding="utf-8") as target:
        json.dump(config, target, indent=2, allow_nan=False)
        target.write("\n")
    print(json.dumps({"manifest": str(args.output), "signal_selection": config["signal_selection"]}))


if __name__ == "__main__":
    main()
