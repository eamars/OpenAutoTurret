"""Report sensorless homing measurements without numerical acceptance gates.

Raw command and feedback observations remain distinct from executor annotations,
completion and physical qualification. No hashes or certificates are required.
"""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
from adr0022_capture_review import need
from adr0022_current_review import _wire_report, summary

MODE, SPEED, POSITION, SPEED_LIMIT = 0x7005, 0x700A, 0x7016, 0x7017
CURRENT_LIMIT, MECH_POSITION = 0x7018, 0x7019


def review(path):
    report, config, rows, commands, frames = _wire_report(path)
    need(report["capture_schema"] == "adr0022.sensorless-homing/1", "unsupported homing capture schema")
    pitch = [c for c in commands if c["axis"] == "pitch" and c["success"] is True]
    feedback = report["pitch_feedback_observations"]
    observations = report["register_observations"]
    approaches = []
    active = None
    for command in pitch:
        speed = command.get("value") if command["kind"] == 18 and command.get("index") == SPEED else None
        ending = command["kind"] == 4 or (command["kind"] == 18 and command.get("index") == MODE) or speed == 0
        if active is not None and ending:
            measured = [f for f in feedback if active["capture_row_index"] < f["capture_row_index"] < command["capture_row_index"]]
            approaches.append({"commanded_speed_rad_s": active["value"], "begin_ns": active["begin_ns"],
                "end_ns": command["begin_ns"], "feedback_count": len(measured),
                "terminal_angle_raw": measured[-1]["angle_raw"] if measured else None,
                "terminal_protocol_angle_rad": measured[-1]["angle_rad"] if measured else None,
                "terminal_receive_ns": measured[-1]["receive_ns"] if measured else None,
                "observed_protocol_angle": summary(f["angle_rad"] for f in measured),
                "observed_protocol_torque": summary(f["protocol_torque_Nm"] for f in measured),
                "contact_qualified": False})
            active = None
        if active is None and speed is not None and speed != 0:
            active = command
    if active is not None:
        measured = [f for f in feedback if f["capture_row_index"] > active["capture_row_index"]]
        approaches.append({"commanded_speed_rad_s": active["value"], "begin_ns": active["begin_ns"],
            "end_ns": None, "feedback_count": len(measured), "terminal_angle_raw": measured[-1]["angle_raw"] if measured else None,
            "terminal_protocol_angle_rad": measured[-1]["angle_rad"] if measured else None,
            "terminal_receive_ns": measured[-1]["receive_ns"] if measured else None, "contact_qualified": False})
    groups = []
    for approach in approaches:
        direction = 1 if approach["commanded_speed_rad_s"] > 0 else -1
        if not groups or groups[-1]["direction"] != direction:
            groups.append({"direction": direction, "observations": []})
        groups[-1]["observations"].append(approach)
    endpoints = []
    for index, group in enumerate(groups):
        terminal = [a["terminal_protocol_angle_rad"] for a in group["observations"] if a["terminal_protocol_angle_rad"] is not None]
        pair = terminal[-2:]
        endpoints.append({"endpoint": "a" if index == 0 else "b" if index == 1 else str(index),
            "direction": group["direction"], "terminal_protocol_angles_rad": terminal,
            "angle_rad": sum(pair)/len(pair) if pair else None,
            "repeatability_rad": abs(pair[1]-pair[0]) if len(pair) == 2 else None,
            "meaning": "Last received poses before each commanded approach ended; no contact or repeatability criterion is applied",
            "qualified": False})
    geometry = [e["angle_rad"] for e in endpoints[:2]]
    available = len(geometry) == 2 and all(value is not None for value in geometry)
    midpoint = sum(geometry)/2 if available else None
    span = abs(geometry[1]-geometry[0])*180/math.pi if available else None
    position_pairs = []
    for observation in observations:
        if observation.get("index") != MECH_POSITION or observation.get("value") is None:
            continue
        pin = next((c for c in pitch if c["kind"] == 18 and c.get("index") == POSITION and
                    c["capture_row_index"] > observation["capture_row_index"]), None)
        boundary = pin["capture_row_index"] if pin else observation["capture_row_index"]
        processed = [f for f in feedback if f["capture_row_index"] < boundary]
        encoder = processed[-1] if processed else None
        position_pairs.append({"read_receive_ns": observation["receive_ns"], "native_mechpos_rad": observation["value"],
            "raw_readback_correlated": observation["raw_readback_correlated"], "device_sample_ns": observation["device_sample_ns"],
            "type2_pose_rad": encoder["angle_rad"] if encoder else None,
            "status_receive_ns": encoder["receive_ns"] if encoder else None,
            "status_dequeue_ns": encoder["dequeue_ns"] if encoder else None,
            "register_minus_type2_rad": observation["value"]-encoder["angle_rad"] if encoder else None,
            "pin_write_ns": pin["begin_ns"] if pin else None,
            "pin_reference_rad": pin.get("value") if pin else None,
            "pin_minus_native_mechpos_rad": pin["value"]-observation["value"] if pin and pin.get("value") is not None else None,
            "mapping_qualified": False})
    reference_observations = []
    for observation in observations:
        if observation.get("index") != POSITION or observation.get("value") is None:
            continue
        writes = [c for c in pitch if c["kind"] == 18 and c.get("index") == POSITION and
                  c["capture_row_index"] < observation["capture_row_index"]]
        write = writes[-1] if writes else None
        reference_observations.append({"receive_ns": observation["receive_ns"], "observed_reference_rad": observation["value"],
            "commanded_reference_rad": write.get("value") if write else None,
            "difference_rad": observation["value"]-write["value"] if write and write.get("value") is not None else None,
            "raw_readback_correlated": observation["raw_readback_correlated"]})
    first_write = next((c for c in pitch if c["kind"] == 18), None)
    original = {o["index"]: o["value"] for o in observations if o["raw_readback_correlated"] and o.get("value") is not None and
                first_write and o["capture_row_index"] < first_write["capture_row_index"] and
                o["index"] in (MODE, CURRENT_LIMIT, 0x701E, 0x701F, 0x7020)}
    restores = []
    for register, value in original.items():
        reads = [o for o in observations if o["index"] == register and o["raw_readback_correlated"]]
        actual = reads[-1] if reads else None
        restores.append({"index": register, "original_value": value,
            "last_observed_value": actual["value"] if actual else None,
            "difference": actual["value"]-value if actual and actual.get("value") is not None else None,
            "last_read_receive_ns": actual["receive_ns"] if actual else None,
            "observed_after_final_reset": bool(actual and report["final_disabled_observed_ns"] is not None and
                                                actual["receive_ns"] > report["final_disabled_observed_ns"])})
    annotations = {kind: [r for r in rows if r.get("kind") == kind] for kind in
        ("homing_endpoints", "homing_endpoint_observation", "homing_midpoint_dwell", "homing_position_observation", "homing_reference_observation")}
    report.update(schema="adr0022.homing_review/1", capability_scope="sensorless_measurement_observations_only",
        homing_observed=bool(approaches or annotations["homing_endpoints"] or annotations["homing_endpoint_observation"]),
        homing_executor_reported_observed=report["capture_footer"].get("homing_observed"),
        approaches=approaches, endpoints=endpoints, midpoint_rad=midpoint, measured_travel_deg=span,
        geometry_meaning="Opposite-direction terminal-pose candidates; raw annotations and completion are reported separately",
        executor_measurement_annotations=annotations, encoder_resolution_rad=25/65535,
        encoder_mechpos_receipt_pairs=position_pairs, encoder_mechpos_bias_correction=None,
        native_reference_readback_observations=reference_observations,
        original_settings={str(k):v for k,v in original.items()}, restored_setting_observations=restores,
        original_mode=original.get(MODE), restored_mode=next((r["last_observed_value"] for r in restores if r["index"] == MODE), None),
        restored_settings_verified=bool(restores and all(r["observed_after_final_reset"] and r["difference"] == 0 for r in restores)),
        mode_transitions=sum(c["kind"] == 3 for c in pitch),
        homing_command_current_observations=[c for c in pitch if c["kind"] == 18 and c.get("index") == CURRENT_LIMIT],
        position_speed_limit_observations=[c for c in pitch if c["kind"] == 18 and c.get("index") == SPEED_LIMIT],
        encoder_zero_command_sent=any(c["kind"] == 6 for c in pitch))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        result = review(args.capture)
    except (ValueError, KeyError, TypeError, OSError, IndexError, OverflowError) as exc:
        result = {"schema": "adr0022.homing_review/1", "capture_complete": False, "homing_observed": False,
            "capture_integrity": "DATA_INVALID", "reason": "DATA_INVALID", "detail": str(exc),
            "physical_capabilities_qualified": False, "physical_parameters_qualified": False,
            "calibration_qualified": False, "protection_qualified": False, "controller_qualified": False}
    with args.output.open("x", encoding="utf-8") as output:
        json.dump(result, output, indent=2, allow_nan=False)
        output.write("\n")
    print(json.dumps(result, indent=2, allow_nan=False))
    return 0 if result["capture_complete"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
