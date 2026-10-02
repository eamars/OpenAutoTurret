"""Report sensorless homing measurements without numerical acceptance gates.

Raw command and feedback observations remain distinct from executor annotations,
completion and physical qualification. No hashes or certificates are required.
The raw CAN/IMU wire decode below is shared by the launcher's homing review.
"""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
import statistics
import struct

MODE, IQREF, IQF = 0x7005, 0x7006, 0x701A
SPEED, POSITION, SPEED_LIMIT = 0x700A, 0x7016, 0x7017
CURRENT_LIMIT, MECH_POSITION = 0x7018, 0x7019
COUNT_RAD = 25 / 65535  # CyberGear type-2 feedback angle resolution
SENSORS = ("accel", "gyro", "rv", "game_rv")
FLOAT_REGISTERS = {0x7006, 0x700A, 0x7010, 0x7011, 0x7014, 0x7016,
                   0x7017, 0x7018, 0x7019, 0x701A, 0x701C, 0x701E, 0x701F, 0x7020}


def strict_json(text):
    def unique(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError(f"duplicate JSON key: {key}")
            result[key] = value
        return result
    def invalid(value):
        raise ValueError(f"nonfinite JSON value: {value}")
    return json.loads(text, object_pairs_hook=unique, parse_constant=invalid)


def need(ok, detail):
    if not ok:
        raise ValueError(detail)


def integer(value, detail, minimum=0):
    need(type(value) is int and value >= minimum, detail)
    return value


def word(data, offset):
    return int.from_bytes(data[offset:offset + 2], "big")


def timing_observations(times):
    times = list(times)
    gaps = [(b-a)/1e9 for a, b in zip(times, times[1:])]
    return {"count": len(times), "first_ns": times[0] if times else None,
            "last_ns": times[-1] if times else None,
            "max_gap_s": max(gaps) if gaps else None,
            "min_gap_s": min(gaps) if gaps else None,
            "median_gap_s": statistics.median(gaps) if gaps else None,
            "nonincreasing_intervals": sum(g <= 0 for g in gaps),
            "observed_hz": (len(times)-1)*1e9/(times[-1]-times[0])
                           if len(times)>1 and times[-1]>times[0] else None}


def summary(values):
    values = list(values)
    return {"count": len(values), "min": min(values) if values else None,
            "max": max(values) if values else None,
            "mean": statistics.fmean(values) if values else None,
            "sample_std": statistics.stdev(values) if len(values)>1 else None,
            "sample_variance": statistics.variance(values) if len(values)>1 else None}


def register_value(wire, index):
    if index == MODE:
        return wire[4]
    if index in FLOAT_REGISTERS:
        value = struct.unpack("<f", wire[4:8])[0]
        return value if math.isfinite(value) else None
    return None


def _wire_report(path):
    data = Path(path).read_bytes()
    rows = [strict_json(line) for line in data.decode("utf-8-sig").splitlines() if line.strip()]
    need(rows and all(type(row) is dict for row in rows), "capture records must be JSON objects")
    need(rows[0].get("kind") == "header", "capture header missing")
    header = rows[0]
    footers = [row for row in rows if row.get("kind") == "footer"]
    footer = footers[-1] if footers else {}
    config = strict_json(header.get("manifest_yaml", "{}"))
    need(type(config) is dict, "manifest must be a JSON object")
    integrity = []
    if not data.endswith(b"\n"):
        integrity.append("Capture has no terminal newline")
    if len(footers) != 1 or rows[-1].get("kind") != "footer":
        integrity.append("Capture footer missing, duplicated or followed by more records")
    if sum(row.get("kind") == "header" for row in rows) != 1:
        integrity.append("Capture header duplicated")
    commands, frames, issues = [], {"yaw": [], "pitch": []}, []
    for position, row in enumerate(rows):
        kind = row.get("kind")
        if kind in ("neutral_tx", "homing_tx"):
            need(type(row.get("data_hex")) is str, "TX payload must be hexadecimal")
            try:
                wire = bytes.fromhex(row["data_hex"])
            except ValueError:
                raise ValueError("TX payload is not hexadecimal") from None
            need(len(wire) == 8, "CAN TX payload must contain eight bytes")
            cid = integer(row.get("id"), "invalid TX CAN ID")
            need(cid <= 0x1fffffff, "TX CAN ID exceeds protocol width")
            axis = row.get("axis")
            item = {"capture_row_index": position, "axis": axis, "begin_ns": row.get("begin_ns"),
                    "kernel_accepted_ns": row.get("kernel_accepted_ns"), "success": row.get("success"),
                    "operation": row.get("operation"), "id": cid, "data_hex": wire.hex(),
                    "kind": (cid >> 24) & 31 if axis == "pitch" else None}
            if axis == "pitch" and item["kind"] in (17, 18):
                item.update(index=int.from_bytes(wire[:2], "little"), reserved_bytes_hex=wire[2:4].hex())
                item["value"] = register_value(wire, item["index"]) if item["kind"] == 18 else None
            if axis == "yaw":
                item["neutral_payload"] = wire == bytes(8)
            commands.append(item)
        elif kind == "can_rx":
            axis = row.get("axis")
            need(type(row.get("bytes")) is list and all(type(v) is int and 0 <= v <= 255 for v in row["bytes"]),
                 "CAN payload bytes invalid")
            wire = bytes(row["bytes"])
            cid = integer(row.get("id"), "invalid RX CAN ID")
            need(cid <= 0x1fffffff, "RX CAN ID exceeds protocol width")
            item = {"capture_row_index": position, "receive_ns": row.get("kernel_monotonic_ns"),
                    "dequeue_ns": row.get("dequeue_ns"), "sequence": row.get("sequence"),
                    "generation": row.get("generation"), "socket_drops": row.get("socket_drops"),
                    "drop_delta": row.get("drop_delta"), "clock_uncertainty_ns": row.get("clock_uncertainty_ns"),
                    "error": row.get("error"), "rtr": row.get("rtr"), "id": cid,
                    "raw_hex": wire.hex(), "kind": None}
            if len(wire) != 8 or row.get("dlc") != 8 or row.get("error") or row.get("rtr"):
                issues.append({"capture_row_index": position, "detail": "Frame has no decodable eight-byte data payload"})
            elif axis == "pitch" and row.get("extended") is True:
                item["kind"] = (cid >> 24) & 31
                if item["kind"] == 2:
                    angle, velocity, torque, temperature = [word(wire, offset) for offset in (0, 2, 4, 6)]
                    item.update(angle_raw=angle, angle_rad=angle*25/65535-12.5,
                        velocity_raw=velocity, protocol_velocity_rad_s=velocity*60/65535-30,
                        torque_raw=torque, protocol_torque_Nm=torque*24/65535-12,
                        temperature_raw=temperature, temperature_C=temperature/10,
                        host_id=cid & 255, motor_id=(cid >> 8) & 255,
                        fault_bits=(cid >> 16) & 63, drive_state=(cid >> 22) & 3)
                elif item["kind"] in (17, 18):
                    index = int.from_bytes(wire[:2], "little")
                    item.update(index=index, value=register_value(wire, index),
                        response_status_hex=wire[2:4].hex(), readback_status_ok=wire[2:4] == bytes(2))
                elif item["kind"] == 0:
                    item["uid_hex"] = wire.hex()
            elif axis == "yaw" and row.get("extended") is False and cid == 0x205:
                item.update(angle_raw=word(wire, 0), speed_raw=int.from_bytes(wire[2:4], "big", signed=True),
                    current_raw=int.from_bytes(wire[4:6], "big", signed=True), temperature_raw=wire[6])
            if axis in frames:
                frames[axis].append(item)
            else:
                issues.append({"capture_row_index": position, "detail": "Frame axis is not identified"})
    feedback = [f for f in frames["pitch"] if f.get("kind") == 2 and "drive_state" in f]
    faults = [f for f in feedback if f["fault_bits"]]
    reads = [f for f in frames["pitch"] if f.get("kind") == 17]
    requests = [c for c in commands if c["axis"] == "pitch" and c["kind"] == 17 and c["success"] is True]
    observations = []
    for reply in reads:
        annotations = [(i, r) for i, r in enumerate(rows) if r.get("kind") == "register_read" and
                       r.get("index") == reply["index"] and r.get("receive_ns") == reply["receive_ns"]]
        annotation_position, record = annotations[0] if len(annotations) == 1 else (None, {})
        candidates = [c for c in requests if c.get("index") == reply["index"] and
            c["capture_row_index"] < reply["capture_row_index"] and
            type(c["begin_ns"]) is int and type(reply["receive_ns"]) is int and c["begin_ns"] <= reply["receive_ns"] and
            (not record or (type(record.get("request_begin_ns")) is int and record["request_begin_ns"] <= c["begin_ns"] and
             type(c["kernel_accepted_ns"]) is int and type(record.get("request_accepted_ns")) is int and
             c["kernel_accepted_ns"] <= record["request_accepted_ns"]))]
        request = candidates[-1] if candidates else None
        correlated = bool(len(annotations) == 1 and request and record.get("source") == "type17_readback" and
            reply.get("readback_status_ok") and reply.get("value") is not None and record.get("value") == reply["value"] and
            reply["capture_row_index"] < annotation_position and
            record["request_accepted_ns"] <= reply.get("dequeue_ns", -1))
        observation = dict(reply)
        observation.update(request_sequence=record.get("request_sequence"),
            request_begin_ns=record.get("request_begin_ns"), request_accepted_ns=record.get("request_accepted_ns"),
            command_begin_ns=request["begin_ns"] if request else None,
            kernel_accepted_ns=request["kernel_accepted_ns"] if request else None,
            device_sample_ns=record.get("device_sample_ns"), raw_readback_correlated=correlated,
            annotation_value=record.get("value"),
            annotation_minus_raw_value=record["value"]-reply["value"] if type(record.get("value")) in (int,float) and reply.get("value") is not None else None)
        observations.append(observation)
    stops = [c for c in commands if c["axis"] == "pitch" and c["kind"] == 4 and c["success"] is True]
    enables = [c for c in commands if c["axis"] == "pitch" and c["kind"] == 3 and c["success"] is True]
    stop_observations = []
    for stop in stops:
        sent = stop["kernel_accepted_ns"] if type(stop["kernel_accepted_ns"]) is int else stop["begin_ns"]
        later_enable = next((c for c in enables if c["capture_row_index"] > stop["capture_row_index"]), None)
        candidates = [f for f in feedback if f["capture_row_index"] > stop["capture_row_index"] and
            type(f["receive_ns"]) is int and type(sent) is int and f["receive_ns"] >= sent and
            (later_enable is None or f["capture_row_index"] < later_enable["capture_row_index"])]
        reset = next((f for f in candidates if f["drive_state"] == 0 and f["fault_bits"] == 0 and
                      f["host_id"] == 0 and f["motor_id"] == 127), None)
        stop_observations.append({"stop_tx_ns": stop["begin_ns"], "kernel_accepted_ns": sent,
            "reset_observed_ns": reset["receive_ns"] if reset else None,
            "fault_free_reset_after_stop": reset is not None,
            "receipt_latency_s": (reset["receive_ns"]-sent)/1e9 if reset else None,
            "reset_payload_hex": reset["raw_hex"] if reset else None})
    last_enable_position = enables[-1]["capture_row_index"] if enables else -1
    final_stops = [(c, o) for c, o in zip(stops, stop_observations) if c["capture_row_index"] > last_enable_position]
    final_stop, final_observation = final_stops[0] if final_stops else (None, None)
    first_reset_ns = final_observation["reset_observed_ns"] if final_observation else None
    after_reset = [f for f in feedback if first_reset_ns is not None and f["receive_ns"] >= first_reset_ns]
    final_confirmed = bool(first_reset_ns is not None and after_reset and
                           all(f["drive_state"] == 0 and f["fault_bits"] == 0 for f in after_reset))
    iqf = [o for o in observations if o.get("index") == IQF and o.get("value") is not None]
    currents = []
    for observation in iqf:
        earlier = [c for c in enables if c["begin_ns"] <= observation["receive_ns"]]
        enable = earlier[-1] if earlier else None
        current = dict(observation)
        current.update(value_A=observation["value"], raw_register_response_hex=observation["raw_hex"],
            receipt_from_enable_s=(observation["receive_ns"]-enable["begin_ns"])/1e9 if enable else None)
        currents.append(current)
    imu = {sensor: [] for sensor in SENSORS}
    imu_events = []
    for position, row in enumerate(rows):
        if row.get("kind") != "imu_raw":
            continue
        raw = strict_json(row["raw_json"])
        need(type(raw) is dict, "IMU record must be a JSON object")
        if raw.get("kind") == "sample" and raw.get("sensor") in imu:
            values = raw.get("values")
            need(type(values) is list and all(type(v) in (int,float) and math.isfinite(v) for v in values), "invalid IMU values")
            imu[raw["sensor"]].append({"capture_row_index": position, "dequeue_ns": row.get("dequeue_ns"), **raw})
        else:
            imu_events.append({"capture_row_index": position, "raw": raw})
    streams = {}
    for axis, source in frames.items():
        observed = [f for f in source if "angle_raw" in f]
        streams[axis+"_feedback"] = timing_observations(f["receive_ns"] for f in observed if type(f["receive_ns"]) is int)
    streams["pitch_iqf"] = {**timing_observations(o["receive_ns"] for o in iqf),
                            "device_sample_ns": None, "sample_clock_calibrated": False}
    imu_summary = {}
    for sensor, samples in imu.items():
        sample_times = [s["sample_ns"] for s in samples if type(s.get("sample_ns")) is int]
        streams[sensor] = timing_observations(sample_times)
        imu_summary[sensor] = {"sample_clock": streams[sensor],
            "dequeue_clock": timing_observations(s["dequeue_ns"] for s in samples if type(s.get("dequeue_ns")) is int),
            "status_counts": {str(status): sum(s.get("status") == status for s in samples) for status in sorted({s.get("status") for s in samples if type(s.get("status")) is int})},
            "generation_observed": sorted({s.get("generation") for s in samples if type(s.get("generation")) is int}),
            "sequence_wrap_count": sum(b.get("sequence",0)<a.get("sequence",0) for a,b in zip(samples,samples[1:])),
            "modulo_sequence_discontinuities": sum((b.get("sequence",0)-a.get("sequence",0))%256 != 1 for a,b in zip(samples,samples[1:])),
            "axes": [summary(s["values"][i] for s in samples) for i in range(min((len(s["values"]) for s in samples), default=0))],
            "mounting_calibrated": False, "sample_clock_calibrated": False}
    yaw = [f for f in frames["yaw"] if "angle_raw" in f]
    displacement, count = [0.], 0
    for a, b in zip(yaw, yaw[1:]):
        count += (b["angle_raw"]-a["angle_raw"]+4096)%8192-4096
        displacement.append(count*2*math.pi/8192)
    pitch_angles = [f["angle_rad"] for f in feedback]
    identities = [f["uid_hex"] for f in frames["pitch"] if "uid_hex" in f]
    start_rows = [r for r in rows if r.get("kind") == "session_begin"]
    status = footer.get("status")
    report = {"schema": "adr0022.raw_measurement_review/1", "raw_capture_path": str(Path(path)),
        "provenance": header.get("provenance"), "capture_schema": header.get("schema"),
        "purpose": header.get("purpose"), "capture_footer_status": status, "capture_footer_detail": footer.get("detail"),
        "capture_footer": footer, "capture_complete": status == "COMPLETE" and not integrity,
        "capture_integrity": "PASS" if not integrity else "PARTIAL", "integrity_observations": integrity,
        "decode_observations": issues, "start_ns": start_rows[0].get("time_ns") if start_rows else None,
        "end_ns": footer.get("end_ns"), "streams": streams,
        "commands": commands, "register_observations": observations, "register_reads": len(observations),
        "register_rejected_records": [r for r in rows if r.get("kind") == "register_rejected"],
        "write_echoes_ignored": sum(f.get("kind") == 18 for f in frames["pitch"]),
        "write_echoes_qualify_readback": False, "pitch_uid_observed": identities[-1] if identities else None,
        "drive_faults": faults, "drive_fault_seen": bool(faults),
        "drive_fault_active_at_last_feedback": bool(feedback[-1]["fault_bits"]) if feedback else None,
        "pitch_feedback_observations": feedback, "stop_observations": stop_observations,
        "final_stop_confirmed": final_confirmed, "normal_stop_confirmed": final_confirmed and status == "COMPLETE",
        "abort_stop_confirmed": final_confirmed and status not in (None,"COMPLETE"),
        "final_disabled_observed_ns": first_reset_ns,
        "final_disabled_observation_meaning": "Fault-free Reset after STOP and every subsequent recorded pitch status remains Reset; no unobserved interval is certified",
        "final_socket_drops": footer.get("socket_drops"), "interface_loss_deltas": footer.get("interface_loss_deltas"),
        "loss_observations": {a: [{k:f[k] for k in ("capture_row_index","sequence","generation","socket_drops","drop_delta","error")} for f in fs if f.get("drop_delta") or f.get("error")] for a,fs in frames.items()},
        "clock_uncertainty_ns": {a: summary(f["clock_uncertainty_ns"] for f in fs if type(f.get("clock_uncertainty_ns")) is int) for a,fs in frames.items()},
        "imu_observations": imu_summary, "imu_events": imu_events,
        "displacement": {"pitch": {"protocol_min_rad": min(pitch_angles) if pitch_angles else None,
             "protocol_max_rad": max(pitch_angles) if pitch_angles else None,
             "protocol_range_rad": max(pitch_angles)-min(pitch_angles) if pitch_angles else None},
             "yaw": {"protocol_max_abs_rad": max(map(abs,displacement)) if yaw else None}},
        "yaw_max_abs_displacement_rad": max(map(abs,displacement)) if yaw else None,
        "temperatures": {"pitch": {"min_C": min((f["temperature_C"] for f in feedback), default=None),
                                    "max_C": max((f["temperature_C"] for f in feedback), default=None)},
             "yaw": {"raw_min": min((f["temperature_raw"] for f in yaw), default=None),
                     "raw_max": max((f["temperature_raw"] for f in yaw), default=None), "Celsius_mapping": "UNKNOWN"}},
        "pitch_temperature_max_C": max((f["temperature_C"] for f in feedback), default=None),
        "current_observations": currents, "current_statistics": summary(o["value_A"] for o in currents),
        "pitch_current_max_abs_A": max((abs(o["value_A"]) for o in currents), default=None),
        "current_bias_correction": None, "current_feedback_scale_calibrated": False,
        "current_feedback_unit": "DOCUMENTED_AMPERE_UNCALIBRATED",
        "current_units": {"yaw": {"raw_unit": "SIGNED_PROTOCOL_COUNT", "scale_A_per_count": None, "calibrated": False}},
        "manufacturer_limits_reported_by_capture": {k:v for k,v in config.get("protection_limit_basis",{}).items() if k not in ("sha256","hash")},
        "neutral_transition_qualified": False, "neutral_current_qualified": False, "physical_current_mode_qualified": False,
        "motion_authorized": False, "motion_qualified": False, "current_mode_qualified": False,
        "physical_capabilities_qualified": False, "physical_parameters_qualified": False,
        "calibration_qualified": False, "protection_qualified": False, "controller_qualified": False,
        "encoder_mechpos_agreement_qualified": False, "plant_snapshot": None, "controller_candidate": None,
        "measurement_limitations": ["Observations retain raw timing, loss, scatter and operating context without numerical quality rejection.",
            "Pitch angles/torque use documented protocol mapping; physical scale, sign, ratio and reference remain to be calibrated.",
            "Iqf mean is an observed response, not a sensor-bias estimate; device sample time and filter response remain unknown.",
            "STOP conclusions describe received status after transmission, not a stopping-distance or protection qualification."]}
    return report, config, rows, commands, frames


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
        executor_measurement_annotations=annotations, encoder_resolution_rad=COUNT_RAD,
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
