"""Review sensorless homing from raw commands, readbacks and encoder feedback.

Endpoint geometry and phase order are reconstructed independently. Journal
annotations are cross-checks, never substitutes for observed motion or STOP.
No current-mode, dynamics, controller or encoder/mechPos qualification is made.
"""
from __future__ import annotations

import argparse
import bisect
import hashlib
import json
import math
from pathlib import Path
import struct

from adr0022_capture_review import need, statistics_ns, strict_json
from adr0022_current_review import integer, manifest_number, number, stream, word

MODE, IQREF, SPEED, POSITION, SPEED_LIMIT, CURRENT_LIMIT = 0x7005, 0x7006, 0x700A, 0x7016, 0x7017, 0x7018
POSITION_KP, SPEED_KP, SPEED_KI, IQF = 0x701E, 0x701F, 0x7020, 0x701A
MECH_POSITION = 0x7019
SETTINGS = (CURRENT_LIMIT, SPEED_KP, SPEED_KI, POSITION_KP)
REGISTERS = {MODE, IQREF, SPEED, POSITION, SPEED_LIMIT, *SETTINGS, IQF, MECH_POSITION}
SENSORS = ("accel", "gyro", "rv", "game_rv")
COUNT_RAD = 25 / 65535


def close(value, expected):
    return math.isclose(value, expected, rel_tol=1e-6, abs_tol=1e-6)


def pose_close(value, expected):
    return abs(value - expected) <= COUNT_RAD + 1e-6


def numeric(document):
    return {key: manifest_number(document, key) for key in document}


def review(path: Path):
    data = Path(path).read_bytes()
    need(data.endswith(b"\n"), "truncated homing capture")
    rows = [strict_json(line) for line in data.decode().splitlines()]
    need(len(rows) > 2 and all(type(r) is dict for r in rows) and
         rows[0].get("kind") == "header" and rows[-1].get("kind") == "footer", "missing homing capture boundaries")
    need(sum(r.get("kind") == "header" for r in rows) == sum(r.get("kind") == "footer" for r in rows) == 1,
         "duplicate homing boundaries")
    header, footer = rows[0], rows[-1]
    need(header.get("schema") == "adr0022.sensorless-homing/1" and header.get("purpose") == "pitch_sensorless_homing",
         "unsupported homing schema/purpose")
    provenance = header.get("provenance")
    need(provenance in ("SYNTHETIC", "MEASURED") and header.get("parameter_qualified") is False,
         "invalid homing provenance/qualification")
    need(footer.get("status") == "COMPLETE" and footer.get("detail") == "" and
         all(footer.get(field) is False for field in ("parameter_qualified", "motion_qualified", "current_mode_qualified", "encoder_mechpos_agreement_qualified")),
         "homing capture failed/incomplete or claims qualification")
    config = strict_json(header["manifest_yaml"])
    need(config.get("schema") == header["schema"] and config.get("purpose") == header["purpose"] and
         config.get("provenance") == provenance and config.get("transport") == ("loopback_udp" if provenance == "SYNTHETIC" else "socketcan"),
         "homing manifest identity/transport differs")
    support = config.get("pitch_supported_when_disabled")
    need(support is True or (type(support) is str and support == "true"), "pitch support undeclared")
    if provenance == "MEASURED":
        need(config["yaw"].get("interface") == "can0" and config["pitch"].get("interface") == "can1", "station topology differs")
        loss = footer.get("interface_loss_deltas")
        need(type(loss) is dict and set(loss) == {"yaw", "pitch"} and all(type(loss[a]) is dict and
             set(loss[a]) == {"rx_dropped", "rx_errors"} and all(type(v) is int and v == 0 for v in loss[a].values()) for a in loss),
             "interface loss evidence missing/nonzero")
    drops = footer.get("socket_drops")
    need(type(drops) is dict and set(drops) == {"yaw", "pitch"} and all(type(v) is int and v == 0 for v in drops.values()),
         "final socket loss evidence missing/nonzero")
    uid = config["expected_pitch_uid"]
    need(type(uid) is str and len(uid) == 16 and all(c in "0123456789abcdef" for c in uid), "invalid pitch UID")
    guards = numeric(config["guards"])
    homing = config["homing"]
    parameters = {key: manifest_number(homing, key) for key in ("coarse_speed_rad_s", "fine_speed_rad_s", "backoff_speed_rad_s",
        "backoff_rad", "small_backoff_rad", "repeatability_rad", "settle_time_s", "approach_timeout_s", "max_rotation_rad",
        "backoff_timeout_s", "backoff_arrival_tol_rad", "backoff_arrive_vel_rad_s", "limit_cur_initial_a", "limit_cur_max_a", "torque_safety_nm")}
    need(homing.get("motion_checks_abort") in (True, "true") and homing.get("rearm_before_start") in (True, "true") and
         float(homing["limit_cur_step_a"]) == 0 and parameters["limit_cur_initial_a"] == parameters["limit_cur_max_a"] <= 5 and
         guards["current_bound_A"] <= 5, "homing fixed-current/abort/rearm contract differs")
    retries = int(homing["repeatability_retries"])
    direction = int(homing["dir_endpoint_a"])
    need(0 <= retries <= 2 and direction in (-1, 1) and int(homing["dir_endpoint_b"]) == -direction, "invalid homing directions/retries")
    contact = {key: manifest_number(homing["contact"], key) for key in ("v_stall_threshold_rad_s", "q_stall_threshold_rad", "progress_window_s",
        "effort_contact_threshold_nm", "effort_hard_contact_nm", "motion_history_vel_rad_s", "contact_dwell_ms", "min_command_active_ms", "jitter_window_ms", "v_move_threshold_rad_s")}
    native = config["native_settings"]
    position_readback_tolerance = manifest_number(native, "position_reference_readback_tolerance_rad")
    need(position_readback_tolerance <= .0004 and
         guards["encoder_mechpos_agreement_bound_rad"] <= guards["mode_transition_displacement_rad"],
         "native reference tolerance or measured sensor agreement bound exceeds transition contract")
    original = {MODE: float(native["expected_original_mode"]), CURRENT_LIMIT: manifest_number(native, "original_limit_cur_A"),
        SPEED_KP: float(native["original_speed_kp"]), SPEED_KI: float(native["original_speed_ki"]),
        POSITION_KP: float(native["original_position_kp"])}
    need(all(math.isfinite(original[reg]) and original[reg] >= 0 for reg in (SPEED_KP, SPEED_KI, POSITION_KP)),
         "invalid original native gains")
    desired = {CURRENT_LIMIT: manifest_number(native, "homing_limit_cur_A"), SPEED_KP: manifest_number(native, "homing_speed_kp"),
        SPEED_KI: manifest_number(native, "homing_speed_ki"), POSITION_KP: manifest_number(native, "homing_position_kp")}
    need(original[MODE] in (1, 2, 3) and desired[CURRENT_LIMIT] == parameters["limit_cur_initial_a"], "native mode/current differs")
    band = numeric(config["expected_span"])
    need(band["operator_reported_deg"] == 60 and 0 < band["minimum_deg"] < 60 < band["maximum_deg"] <= 90, "invalid expected span")
    timing = config["limits"]
    ns = {key: int(manifest_number(timing, key + "_s") * 1e9) for key in ("clock_uncertainty", "dequeue_age", "can_gap", "imu_gap", "startup", "duration", "read_timeout", "stop_period")}
    need(all(1 <= n <= 3600 * 10**9 for n in ns.values()) and ns["duration"] > ns["startup"], "invalid homing timing")
    minimum_status = int(timing["minimum_imu_status"])
    need(0 <= minimum_status <= 3, "invalid IMU status bound")
    begins = [r for r in rows if r.get("kind") == "session_begin"]
    need(len(begins) == 1, "session begin missing/duplicated")
    start, end = integer(begins[0]["time_ns"], "invalid session begin", 1), integer(footer["end_ns"], "invalid session end", 1)
    need(0 < end - start < ns["duration"], "homing deadline exceeded")
    allowed = {"header", "footer", "session_begin", "can_rx", "imu_raw", "homing_tx", "homing_executor_state", "homing_desired_state",
               "homing_original_setting", "homing_endpoints", "homing_midpoint_dwell", "homing_position_observation",
               "pitch_identity", "register_read", "write_echo"}
    need(all(r.get("kind") in allowed for r in rows), "unexpected/rejected homing evidence")
    frames, commands = {"yaw": [], "pitch": []}, []
    for position, row in enumerate(rows):
        if row["kind"] == "homing_tx":
            t = integer(row["begin_ns"], "invalid TX time", 1)
            accepted = integer(row["kernel_accepted_ns"], "invalid TX acceptance", 1)
            need(start <= t <= accepted <= end and row.get("success") is True, "TX acceptance/time differs")
            hex_data = row["data_hex"]
            need(type(hex_data) is str and len(hex_data) == 16 and all(c in "0123456789abcdef" for c in hex_data), "invalid TX bytes")
            wire = bytes.fromhex(hex_data)
            cid = integer(row["id"], "invalid TX CAN ID")
            axis = row.get("axis")
            need(axis in frames, "unknown TX axis")
            item = dict(row=row, position=position, time=t, accepted=accepted, wire=wire, kind=None, index=None, value=None)
            if axis == "yaw": need(cid == 0x1FE and wire == bytes(8), "nonneutral yaw homing command")
            else:
                kind = (cid >> 24) & 31
                need(cid <= 0x1FFFFFFF and cid & 255 == 127 and (cid >> 8) & 65535 == 0 and kind in (0, 3, 4, 17, 18), "TX outside homing contract")
                item["kind"] = kind
                if kind in (17, 18):
                    index = int.from_bytes(wire[:2], "little")
                    need(index in REGISTERS and wire[2:4] == bytes(2), "unrelated/malformed register command")
                    item["index"] = index
                    if kind == 17: need(wire[4:] == bytes(4), "malformed read request")
                    else:
                        need(index not in (IQF, MECH_POSITION), "write to read-only current/position feedback")
                        value = wire[4] if index == MODE else struct.unpack("<f", wire[4:])[0]
                        number(value, "nonfinite TX value")
                        need(index != MODE or (value in (1, 2, 3) and wire[5:] == bytes(3)), "unqualified native mode")
                        need(index != IQREF or value == 0., "nonzero pitch Iq command")
                        item["value"] = value
                else: need(wire == bytes(8), "simple homing command clears faults or has nonzero payload")
            commands.append(item)
        elif row["kind"] == "can_rx":
            axis = row.get("axis")
            need(axis in frames and row.get("generation") == 1 and type(row.get("generation")) is int and
                 row.get("socket_drops") == 0 and type(row.get("socket_drops")) is int and row.get("drop_delta") == 0 and
                 type(row.get("drop_delta")) is int and row.get("error") is False and row.get("rtr") is False and row.get("dlc") == 8,
                 "CAN loss/invalidity")
            need(type(row["bytes"]) is list and len(row["bytes"]) == 8 and all(type(v) is int and 0 <= v <= 255 for v in row["bytes"]), "invalid CAN bytes")
            wire = bytes(row["bytes"])
            t, dq = integer(row["kernel_monotonic_ns"], "invalid CAN receive", 1), integer(row["dequeue_ns"], "invalid CAN dequeue", 1)
            uncertainty = integer(row["clock_uncertainty_ns"], "invalid CAN clock uncertainty")
            integer(row["kernel_realtime_ns"], "missing kernel timestamp", 1)
            need(uncertainty <= ns["clock_uncertainty"] and -uncertainty <= dq - t <= ns["dequeue_age"] and dq <= end, "CAN timestamp/age differs")
            cid = integer(row["id"], "invalid RX CAN ID")
            item = dict(row=row, position=position, wire=wire, time=t, kind=None)
            if axis == "yaw":
                need(row.get("extended") is False and cid == 0x205 and word(wire, 0) <= 8191, "unexpected yaw traffic")
                item.update(count=word(wire, 0), current=int.from_bytes(wire[4:6], "big", signed=True), temperature=wire[6])
                need(row.get("angle_raw") == item["count"] and row.get("current_raw") == item["current"] and
                     row.get("temperature_raw") == item["temperature"] and row.get("temperature_C") is None, "yaw wire values differ")
            else:
                kind = (cid >> 24) & 31
                need(row.get("extended") is True and cid <= 0x1FFFFFFF and kind in (0, 2, 17, 18), "unexpected pitch traffic")
                item["kind"] = kind
                if kind == 2:
                    need(cid & 255 == 0 and (cid >> 8) & 255 == 127 and (cid >> 16) & 63 == 0 and (cid >> 22) & 3 in (0, 2), "pitch fault/identity/state differs")
                    fields = [word(wire, at) for at in (0, 2, 4, 6)]
                    need(all(row.get(key) == value for key, value in zip(("angle_raw", "velocity_raw", "torque_raw", "temperature_raw"), fields)) and
                         close(number(row["temperature_C"], "missing pitch temperature"), fields[3] / 10), "pitch wire values differ")
                    item.update(count=fields[0], angle=fields[0] * COUNT_RAD - 12.5, torque=fields[2] * 24 / 65535 - 12,
                                temperature=fields[3] / 10, mode=(cid >> 22) & 3, velocity=0.)
                    need(item["temperature"] < guards["pitch_temperature_C"] and abs(item["torque"]) <= guards["torque_bound_Nm"], "pitch temperature/torque guard exceeded")
                elif kind == 0: need(cid == 0x7FFE and wire.hex() == uid, "pitch identity differs")
                else:
                    need(cid == ((kind << 24) | 0x7F00) and wire[2:4] == bytes(2), "register reply identity/status differs")
                    index = int.from_bytes(wire[:2], "little")
                    need(index in REGISTERS, "unexpected register reply")
                    item.update(index=index, value=wire[4] if index == MODE else struct.unpack("<f", wire[4:])[0])
                    number(item["value"], "nonfinite readback")
            frames[axis].append(item)
    need(commands and all(a["time"] < b["time"] for a, b in zip(commands, commands[1:])), "TX timestamps missing/reordered")
    report = {"schema": "adr0022.homing_review/1", "capture_sha256": hashlib.sha256(data).hexdigest(), "provenance": provenance,
        "capture_complete": True, "homing_observed": True, "capability_scope": "sensorless_geometry_observation_only",
        "motion_authorized": False, "motion_qualified": False, "current_mode_qualified": False, "physical_capabilities_qualified": False,
        "physical_parameters_qualified": False, "encoder_mechpos_agreement_qualified": False, "plant_snapshot": None,
        "controller_candidate": None, "encoder_zero_command_sent": False, "streams": {}, "final_socket_drops": drops}
    feedback = [f for f in frames["pitch"] if f["kind"] == 2]
    for axis, observed in frames.items():
        need([f["row"]["sequence"] for f in observed] == list(range(1, len(observed) + 1)), "CAN sequence missing/reordered")
        statistics_ns([f["time"] for f in observed])
        report["streams"][axis + "_feedback"] = stream([f["time"] for f in (observed if axis == "yaw" else feedback)],
            start, end, min(ns["can_gap"], 80_000_000) if axis == "yaw" else ns["can_gap"], ns["startup"], axis)
    counts, yaw_displacement = 0, [0.]
    for a, b in zip(frames["yaw"], frames["yaw"][1:]):
        delta = b["count"] - a["count"]
        if delta > 4096: delta -= 8192
        if delta < -4096: delta += 8192
        scale = 2 * math.pi / 8192
        need(abs(delta) * scale <= 40 * (b["time"] - a["time"]) / 1e9 + 2 * scale, "yaw encoder discontinuity")
        counts += delta; yaw_displacement.append(counts * scale)
    need(max(map(abs, yaw_displacement)) <= guards["yaw_displacement_rad"], "yaw displacement guard exceeded")
    anchor, velocity = feedback[0], 0.
    for b in feedback[1:]:
        dt = (b["time"] - anchor["time"]) / 1e9
        displacement = (b["count"] - anchor["count"]) * COUNT_RAD
        need(abs(displacement) <= guards["maximum_encoder_speed_rad_s"] * dt + COUNT_RAD,
             "pitch encoder speed guard exceeded")
        if b["time"] - anchor["time"] >= ns["stop_period"]:
            velocity = displacement / dt
            anchor = b
        b["velocity"] = velocity
    need(max(f["angle"] for f in feedback) - min(f["angle"] for f in feedback) <= guards["total_displacement_rad"], "pitch total displacement guard exceeded")
    stamps = [f["time"] for f in feedback]
    def latest(t):
        index = bisect.bisect_right(stamps, t) - 1
        need(index >= 0 and t - stamps[index] <= ns["can_gap"], "command lacks fresh pitch feedback")
        return feedback[index]
    def interval(a, b):
        return feedback[bisect.bisect_left(stamps, a):bisect.bisect_left(stamps, b)]
    imu = {sensor: [] for sensor in SENSORS}
    generations = set()
    for row in rows:
        if row["kind"] != "imu_raw": continue
        raw = strict_json(row["raw_json"])
        need(type(raw) is dict and raw.get("kind") not in ("gap", "summary", "trace_reset"), "IMU history discarded/reset/ended")
        dq = integer(row["dequeue_ns"], "invalid IMU dequeue", 1)
        need(dq <= end, "IMU beyond capture")
        if raw.get("kind") != "sample": continue
        sensor = raw.get("sensor"); need(sensor in imu, "unknown IMU stream")
        t, rx = integer(raw["sample_ns"], "invalid IMU sample", 1), integer(raw["rx_ns"], "invalid IMU receive", 1)
        need(t <= rx <= dq and rx - t <= ns["imu_gap"] and dq - rx <= ns["dequeue_age"], "IMU clock/age differs")
        generations.add(integer(raw["generation"], "invalid IMU generation"))
        need(integer(raw["sequence"], "invalid IMU sequence") <= 255 and minimum_status <= integer(raw["status"], "invalid IMU status") <= 3, "IMU sequence/status differs")
        values = raw["values"]
        need(type(values) is list and len(values) == (3 if sensor in ("accel", "gyro") else 4), "invalid IMU dimensions")
        for v in values: number(v, "nonfinite IMU value")
        if len(values) == 4: need(.9801 <= sum(v * v for v in values) <= 1.0201, "invalid quaternion norm")
        imu[sensor].append(raw)
    need(len(generations) == 1, "IMU generation changed/missing")
    for sensor, observed in imu.items():
        need(all(((a["sequence"] + 1) & 255) == b["sequence"] and a["rx_ns"] <= b["rx_ns"] for a, b in zip(observed, observed[1:])), "IMU sequence loss/reordering")
        report["streams"][sensor] = stream([r["sample_ns"] for r in observed], start, end, ns["imu_gap"], ns["startup"], sensor)
        report["streams"][sensor].update(mounting_calibrated=False, sample_clock_calibrated=False)
    pitch_commands = [c for c in commands if c["row"]["axis"] == "pitch"]
    discovery = [c for c in pitch_commands if c["kind"] == 0]
    identities = [f for f in frames["pitch"] if f["kind"] == 0]
    identity_rows = [r for r in rows if r["kind"] == "pitch_identity"]
    need(len(discovery) == len(identities) == len(identity_rows) == 1 and discovery[0]["time"] <= identities[0]["time"] < discovery[0]["time"] + ns["read_timeout"] and
         identity_rows[0].get("uid_hex") == uid and identity_rows[0].get("receive_ns") == identities[0]["time"], "discovery correlation missing/invalid")
    need(all(c["kind"] == 0 or c["time"] >= identities[0]["time"] for c in pitch_commands), "pitch command before identity")
    requests = [c for c in pitch_commands if c["kind"] == 17]
    replies = [f for f in frames["pitch"] if f["kind"] == 17]
    records = [(i, r) for i, r in enumerate(rows) if r["kind"] == "register_read"]
    need(len(requests) == len(replies) == len(records) and [r["request_sequence"] for _, r in records] == list(range(1, len(records) + 1)), "register evidence incomplete/reordered")
    observations = []
    for tx, rx, (position, row) in zip(requests, replies, records):
        begin = integer(row["request_begin_ns"], "invalid read begin", 1)
        accepted = integer(row["request_accepted_ns"], "invalid read acceptance", 1)
        need(tx["index"] == rx["index"] == row["index"] and row.get("axis") == "pitch" and row.get("source") == "type17_readback" and
             "device_sample_ns" in row and row["device_sample_ns"] is None and row.get("receive_ns") == rx["time"] and
             number(row["value"], "invalid read value") == rx["value"] and begin <= tx["time"] <= begin + ns["dequeue_age"] and
             tx["accepted"] <= accepted <= rx["row"]["dequeue_ns"] and begin <= rx["time"] < begin + ns["read_timeout"] and
             tx["position"] < rx["position"] < position, "register raw correlation/value differs")
        if observations: need(observations[-1]["time"] <= begin, "overlapping read transactions")
        observations.append(dict(index=rx["index"], value=rx["value"], time=rx["time"], request=tx, position=position))
    position_observations = []
    for observation in observations:
        if observation["index"] != MECH_POSITION: continue
        pins = [c for c in pitch_commands if c["kind"] == 18 and c["index"] == POSITION and c["time"] > observation["time"]]
        need(pins and pins[0]["time"] - observation["time"] <= ns["read_timeout"], "native position read lacks a fresh pin")
        # Kernel arrival can precede TX while the queued frame is processed
        # afterward. Reconstruct the executor's observed state from the raw
        # journal order, preserving receipt and dequeue times separately.
        processed = [f for f in feedback if f["position"] < pins[0]["position"]]
        need(processed and pins[0]["time"] - processed[-1]["time"] <= ns["can_gap"],
             "native position pin lacks fresh processed feedback")
        encoder = processed[-1]
        position_observations.append({"request_begin_ns": observation["request"]["time"],
            "receive_ns": observation["time"], "device_sample_ns": None, "mechpos_rad": observation["value"],
            "encoder_receive_ns": encoder["time"], "encoder_raw": encoder["count"], "encoder_rad": encoder["angle"],
            "encoder_dequeue_ns": encoder["row"]["dequeue_ns"],
            "receipt_gap_s": (observation["time"] - encoder["time"]) / 1e9,
            "pin_write_ns": pins[0]["time"],
            "difference_rad": observation["value"] - encoder["angle"]})
    need(all(abs(o["difference_rad"]) <= guards["encoder_mechpos_agreement_bound_rad"] + 1e-6
             for o in position_observations), "native/encoder receipt difference exceeds declared measured bound")
    position_by_time = {o["receive_ns"]: o for o in position_observations}
    for position, row in enumerate(rows):
        if row["kind"] != "homing_position_observation": continue
        observed = position_by_time.get(row.get("read_receive_ns"))
        raw_read = next((o for o in observations if o["index"] == MECH_POSITION and o["time"] == row.get("read_receive_ns")), None)
        encoder = next((f for f in feedback if f["time"] == row.get("status_receive_ns")), None)
        need(observed is not None and raw_read is not None and encoder is not None and
             raw_read["position"] < position and encoder["position"] < position and
             row.get("mapping_qualified") is False and
             close(number(row["native_mechpos_rad"], "invalid native position annotation"), observed["mechpos_rad"]) and
             close(number(row["type2_pose_rad"], "invalid encoder position annotation"), encoder["angle"]) and
             close(number(row["register_minus_type2_rad"], "invalid position residual annotation"), observed["difference_rad"]) and
             close(number(row["agreement_bound_rad"], "invalid position bound annotation"), guards["encoder_mechpos_agreement_bound_rad"]) and
             row["status_receive_ns"] == observed["encoder_receive_ns"],
             "native position annotation differs from raw read/feedback/pin evidence")
    echoes = [f for f in frames["pitch"] if f["kind"] == 18]
    echo_rows = [r for r in rows if r["kind"] == "write_echo"]
    need(len(echoes) == len(echo_rows), "echo provenance incomplete")
    for echo, row in zip(echoes, echo_rows):
        need(any(c["kind"] == 18 and c["wire"] == echo["wire"] and c["time"] <= echo["time"] < c["time"] + ns["read_timeout"] for c in pitch_commands) and
             row.get("readback_verified") is False and row.get("receive_ns") == echo["time"] and row.get("index") == echo["index"], "uncorrelated write echo")
    first_write = next(c for c in pitch_commands if c["kind"] == 18)
    snapshot = [o for o in observations if o["index"] != IQF and o["time"] < first_write["time"]]
    need([o["index"] for o in snapshot] == list(original) and
         all(close(o["value"], original[o["index"]]) for o in snapshot) and
         all(latest(o["time"])["mode"] == 0 for o in snapshot), "original native snapshot differs/missing")
    original = {o["index"]: o["value"] for o in snapshot}
    iqf = [o for o in observations if o["index"] == IQF]
    need(iqf and all(abs(o["value"]) <= guards["current_bound_A"] for o in iqf), "measured pitch current guard exceeded/missing")
    report["streams"]["pitch_iqf"] = statistics_ns([o["time"] for o in iqf])
    report["streams"]["pitch_iqf"].update(device_sample_ns=None, sample_clock_calibrated=False)
    writes = [c for c in pitch_commands if c["kind"] == 18]
    stops = [c for c in pitch_commands if c["kind"] == 4]
    enables = [c for c in pitch_commands if c["kind"] == 3]
    modes = [c for c in writes if c["index"] == MODE]
    need(enables and len(modes) == len(enables) + 1 and close(modes[-1]["value"], original[MODE]), "mode transition/restore commands missing")
    for write in writes:
        if write["index"] in (MODE, *SETTINGS): need(latest(write["time"])["mode"] == 0, "native settings changed while enabled")
        if write["index"] == SPEED and write["value"] != 0:
            need(latest(write["time"])["mode"] == 2 and abs(write["value"]) <= max(parameters["coarse_speed_rad_s"], parameters["fine_speed_rad_s"]) + 1e-6, "approach reference outside contract")
        if write["index"] == SPEED_LIMIT: need(0 <= write["value"] <= max(parameters["backoff_speed_rad_s"], guards["midpoint_speed_rad_s"]) + 1e-6, "position speed cap differs")
    def check_reads(observed, expected, detail):
        need([o["index"] for o in observed] == list(expected) and all(
            abs(o["value"] - expected[o["index"]]) <= position_readback_tolerance
            if o["index"] == POSITION else close(o["value"], expected[o["index"]]) for o in observed), detail)
    epochs = []
    enabled_acknowledgements = []
    previous_enable = start
    for index, (select, enable) in enumerate(zip(modes, enables)):
        next_select = modes[index + 1]
        need(select["value"] in (1, 2) and select["time"] < enable["time"] < next_select["time"], "homing mode/enable order invalid")
        preceding = [s for s in stops if previous_enable < s["time"] < select["time"]]
        need(preceding, "transition STOP missing")
        # Repeated inert STOP polls can occur while a read is outstanding.
        # Correlate the transition with its first STOP and continuously Reset
        # feedback, rather than requiring every read to follow the last poll.
        stop = preceding[0]
        reset = [f for f in interval(stop["time"], select["time"]) if f["mode"] == 0]
        need(reset and reset[0]["time"] - stop["time"] < ns["read_timeout"], "fresh transition STOP acknowledgement missing")
        before = [o for o in observations if o["index"] != IQF and select["time"] < o["request"]["time"] and o["time"] < enable["time"]]
        expected = {**desired, MODE: select["value"], SPEED: 0, IQREF: 0}
        setup = [c for c in writes if select["time"] < c["time"] < enable["time"]]
        for reg, value in desired.items(): need(any(c["index"] == reg and close(c["value"], value) for c in setup), "homing setting write missing")
        pin = None
        if select["value"] == 1:
            pins = [c for c in setup if c["index"] == POSITION]
            caps = [c for c in setup if c["index"] == SPEED_LIMIT]
            need(len(pins) == len(caps) == 1, "position enable lacks measured pin/speed limit")
            if caps[0]["value"] == 0:
                measured = [o for o in observations if o["index"] == MECH_POSITION and stop["time"] < o["request"]["time"] and o["time"] < select["time"]]
                need(len(measured) == 1 and close(pins[0]["value"], measured[0]["value"]) and
                     abs(position_by_time[measured[0]["time"]]["difference_rad"]) <= guards["encoder_mechpos_agreement_bound_rad"] + 1e-6,
                     "disabled position pin lacks fresh native MechPos evidence")
            else:
                # Historical positive-speed setup used the type-2 pose. The
                # zero-speed native-pin path above preserves measured sensor
                # differences rather than assuming the two coordinates match.
                need(pose_close(pins[0]["value"], latest(pins[0]["time"])["angle"]),
                     "position enable not pinned to measured pose")
            pin = pins[0]["value"]
            expected.update({POSITION: pin, SPEED_LIMIT: caps[0]["value"]})
        check_reads(before, expected, "native mode/neutral readback missing before enable")
        next_stop = next((s for s in stops if s["time"] > enable["time"]), None)
        need(next_stop is not None and next_stop["time"] < next_select["time"], "enabled epoch lacks closing STOP")
        epoch_writes = [c for c in writes if enable["time"] < c["time"] < next_stop["time"]]
        actions = []
        active_position_cap = expected.get(SPEED_LIMIT, 0)
        for c in epoch_writes:
            if c["index"] == SPEED_LIMIT: active_position_cap = c["value"]
            if ((c["index"] == SPEED and c["value"] != 0) or
                (c["index"] == POSITION and active_position_cap > 0)):
                actions.append(c)
        need(actions or (index == 0 and select["value"] == 2), "enabled homing epoch has no observed approach/backoff")
        action = actions[0] if actions else None
        releases = [c for c in epoch_writes if (c["index"] == SPEED and c["value"] != 0) or
                    (c["index"] == SPEED_LIMIT and c["value"] > 0)]
        # A positive speed limit can release the native position controller
        # before a subsequent target write, so verification must precede it.
        action_time = releases[0]["time"] if releases else (action["time"] if action else next_stop["time"])
        after = [o for o in observations if o["index"] != IQF and enable["time"] < o["request"]["time"] and o["time"] < action_time]
        enabled_expected = dict(expected); enabled_expected.pop(SPEED_LIMIT, None)
        if pin is not None and expected[SPEED_LIMIT] == 0:
            enabled_expected.pop(POSITION)
            enabled_expected[SPEED_LIMIT] = 0
            prefix = after[:len(enabled_expected)]
            check_reads(prefix, enabled_expected, "native mode/neutral readback missing after enable")
            repin_reads = after[len(enabled_expected):]
            need([o["index"] for o in repin_reads] == [MECH_POSITION, POSITION, SPEED_LIMIT] and
                 abs(position_by_time[repin_reads[0]["time"]]["difference_rad"]) <= guards["encoder_mechpos_agreement_bound_rad"] + 1e-6 and
                 abs(repin_reads[1]["value"] - repin_reads[0]["value"]) <= position_readback_tolerance and
                 repin_reads[2]["value"] == 0,
                 "enabled position repin lacks fresh MechPos/LocRef/zero-limit readback")
            repins = [c for c in epoch_writes if c["index"] == POSITION and repin_reads[0]["time"] < c["time"] < repin_reads[1]["request"]["time"]]
            need(len(repins) == 1 and close(repins[0]["value"], repin_reads[0]["value"]), "enabled measured position was not repinned")
        else:
            check_reads(after, enabled_expected, "native mode/neutral readback missing after enable")
        enabled_feedback = interval(enable["time"], next_stop["time"])
        motor_index = next((i for i, f in enumerate(enabled_feedback) if f["mode"] == 2), None)
        need(motor_index is not None, "fresh Motor feedback absent before homing motion")
        first_motor = enabled_feedback[motor_index]
        need(first_motor["time"] < action_time and all(f["mode"] == 2 for f in enabled_feedback[motor_index:]),
             "enabled feedback absent/lost after first Motor acknowledgement")
        pending_writes = [c for c in epoch_writes if c["time"] < first_motor["time"]]
        need(all(c["index"] in (SPEED, IQREF, SPEED_LIMIT) and c["value"] == 0 for c in pending_writes),
             "command was not neutral while awaiting first Motor acknowledgement")
        enabled_acknowledgements.append({"enable_tx_ns": enable["time"], "first_motor_feedback_ns": first_motor["time"],
            "receipt_latency_s": (first_motor["time"] - enable["time"]) / 1e9,
            "pending_reset_feedback_count": motor_index, "neutral_until_first_motor": True})
        disabled_feedback = interval(reset[0]["time"], enable["time"])
        need(disabled_feedback and all(f["mode"] == 0 for f in disabled_feedback), "reenabled during native setup")
        # The initial STOP itself may produce the first type-2 feedback.
        origin = latest(stop["time"])["angle"] if bisect.bisect_right(stamps, stop["time"]) else reset[0]["angle"]
        need(all(abs(f["angle"] - origin) <= guards["mode_transition_displacement_rad"] for f in interval(stop["time"], action_time)), "mode transition displacement guard exceeded")
        current_begin = action_time if action else after[-1]["time"]
        readings = [o for o in iqf if current_begin <= o["time"] < next_stop["time"]]
        neutral_rearm = action is None and not readings and next_stop["time"] - current_begin < ns["read_timeout"]
        need(neutral_rearm or (readings and readings[0]["time"] - current_begin <= ns["read_timeout"] and
             next_stop["time"] - readings[-1]["time"] <= ns["read_timeout"] and
             all(b["time"] - a["time"] <= ns["read_timeout"] for a, b in zip(readings, readings[1:]))), "enabled current readback coverage missing")
        epochs.append(dict(mode=int(select["value"]), enable=enable, select=select, stop=next_stop,
                           action=action, writes=epoch_writes, feedback=enabled_feedback))
        previous_enable = enable["time"]
    if epochs[0]["action"] is None:
        epochs = epochs[1:]
    restore = modes[-1]
    final_stop = epochs[-1]["stop"]
    final_reset = [f for f in interval(final_stop["time"], restore["time"]) if f["mode"] == 0]
    need(final_reset and final_reset[0]["time"] - final_stop["time"] < ns["read_timeout"] and
         all(f["mode"] == 0 for f in feedback if f["time"] >= final_reset[0]["time"]), "fresh final STOP acknowledgement missing before restore")
    restored = [c for c in writes if c["time"] >= restore["time"] and c["index"] in original]
    need([c["index"] for c in restored] == list(original) and all(close(c["value"], original[c["index"]]) for c in restored), "original settings restore writes differ")
    check_reads([o for o in observations if o["request"]["time"] > restore["time"]], original, "restored settings readback differs/missing")

    # Recover coarse/fine/repeat phases from native mode and actual reference
    # writes. A contact requires persistent raw encoder stall plus effort in
    # the commanded direction; annotations and type-2 torque alone do not pass.
    contacts, backoffs = [], []
    def approach(epoch, expected_speed, label):
        need(epoch["mode"] == 2 and epoch["action"]["index"] == SPEED and close(epoch["action"]["value"], expected_speed), "approach phase/reference order differs")
        moving = [c for c in epoch["writes"] if c["index"] == SPEED and c["value"] != 0]
        need(all(close(c["value"], expected_speed) for c in moving), "approach speed changed unexpectedly")
        zero = next((c for c in epoch["writes"] if c["index"] == SPEED and c["value"] == 0 and c["time"] > moving[0]["time"]), epoch["stop"])
        observed = interval(moving[0]["time"], zero["time"])
        need(observed and observed[-1]["time"] - moving[0]["time"] < parameters["approach_timeout_s"] * 1e9, "approach evidence absent/timed out")
        finish = observed[-1]
        window_s = max(contact["contact_dwell_ms"] / 1000, contact["progress_window_s"])
        window = [f for f in observed if f["time"] >= finish["time"] - window_s * 1e9 - ns["can_gap"]]
        # Choose the longest trailing raw plateau satisfying all independent gates.
        plateau = []
        direction = 1 if expected_speed > 0 else -1
        for f in reversed(window):
            if abs(f["velocity"]) >= contact["v_stall_threshold_rad_s"] or direction * f["torque"] <= contact["effort_contact_threshold_nm"]: break
            candidate = [f, *plateau]
            if max(x["angle"] for x in candidate) - min(x["angle"] for x in candidate) >= contact["q_stall_threshold_rad"]: break
            plateau = candidate
        ever_moved, previously_stalled, recovered = False, True, False
        for f in observed:
            stalled = abs(f["velocity"]) < contact["v_move_threshold_rad_s"]
            if previously_stalled and not stalled and ever_moved and f["time"] >= finish["time"] - contact["jitter_window_ms"] * 1e6:
                recovered = True
            ever_moved = ever_moved or not stalled
            previously_stalled = stalled
        need(len(plateau) >= 3 and finish["time"] - plateau[0]["time"] >= window_s * 1e9 and
             not recovered and
             finish["time"] - moving[0]["time"] >= contact["min_command_active_ms"] * 1e6 and
             (max(abs(f["velocity"]) for f in observed) > contact["motion_history_vel_rad_s"] or
              min(direction * f["torque"] for f in plateau) > contact["effort_hard_contact_nm"]), "contact lacks persistent encoder/effort/motion evidence")
        need(abs(finish["angle"] - observed[0]["angle"]) <= parameters["max_rotation_rad"], "approach rotation bound exceeded")
        result = dict(phase=label, angle_rad=finish["angle"], angle_raw=finish["count"], receive_ns=finish["time"],
                      evidence_dwell_s=(finish["time"] - plateau[0]["time"]) / 1e9, begin_ns=moving[0]["time"])
        contacts.append(result)
        return result, zero["time"]
    def backoff(epoch, endpoint, clearance, direction, settle, label):
        need(epoch["mode"] == 1 and epoch["action"]["index"] == POSITION and
             pose_close(epoch["action"]["value"], endpoint["angle_rad"] - clearance * direction), "backoff target/direction/clearance differs")
        target = epoch["action"]["value"]
        caps = [c for c in epoch["writes"] if c["index"] == SPEED_LIMIT and c["time"] <= epoch["action"]["time"]]
        need(caps and close(caps[-1]["value"], parameters["backoff_speed_rad_s"]), "backoff native speed cap differs")
        hold_writes = [c for c in epoch["writes"] if c["index"] == POSITION and c["time"] > epoch["action"]["time"]]
        previous_hold = epoch["action"]["time"]
        for c in hold_writes:
            measured = [o for o in observations if o["index"] == MECH_POSITION and
                        previous_hold < o["request"]["time"] and o["time"] < c["time"]]
            verified = [o for o in observations if o["index"] == POSITION and
                        c["time"] < o["request"]["time"] and o["time"] < epoch["stop"]["time"]]
            need(measured and c["time"] - measured[-1]["time"] <= ns["read_timeout"] and
                 close(c["value"], measured[-1]["value"]) and verified and
                 abs(verified[0]["value"] - c["value"]) <= position_readback_tolerance,
                 "position hold lacks fresh native MechPos pin and correlated LocRef verification")
            previous_hold = verified[0]["time"]
        tolerance = min(math.pi / 720, parameters["backoff_arrival_tol_rad"], clearance * .1)
        observed = interval(epoch["action"]["time"], epoch["stop"]["time"])
        trailing = []
        for f in reversed(observed):
            if abs(f["angle"] - target) >= tolerance or abs(f["velocity"]) >= parameters["backoff_arrive_vel_rad_s"]: break
            trailing.insert(0, f)
        dwell = .15 + (parameters["settle_time_s"] if settle else 0)
        need(len(trailing) >= 3 and trailing[-1]["time"] - trailing[0]["time"] >= dwell * 1e9 and
             max(f["angle"] for f in trailing) - min(f["angle"] for f in trailing) <= .04 * math.pi / 180 and
             trailing[-1]["time"] - epoch["action"]["time"] < parameters["backoff_timeout_s"] * 1e9,
             "backoff near-target arrival/stationary dwell missing")
        backoffs.append(dict(phase=label, target_rad=target, observed_rad=trailing[-1]["angle"], clearance_rad=clearance,
                             stationary_dwell_s=(trailing[-1]["time"] - trailing[0]["time"]) / 1e9))
    cursor = 0
    results = []
    def endpoint(direction, label):
        nonlocal cursor
        need(cursor + 4 < len(epochs), "endpoint approach/backoff/repeat phases missing")
        coarse, zero = approach(epochs[cursor], direction * parameters["coarse_speed_rad_s"], label + "_coarse")
        next_epoch = epochs[cursor + 1]
        need(next_epoch["select"]["time"] - zero >= parameters["settle_time_s"] * 1e9, "coarse contact settle missing")
        backoff(next_epoch, coarse, parameters["backoff_rad"], direction, True, label + "_backoff")
        first, zero = approach(epochs[cursor + 2], direction * parameters["fine_speed_rad_s"], label + "_fine_1")
        need(abs(first["angle_rad"] - coarse["angle_rad"]) <= parameters["repeatability_rad"], "fine contact differs from coarse endpoint")
        cursor += 3
        attempts = 0
        while True:
            need(cursor + 1 < len(epochs) and attempts <= retries, "repeatability phases/budget missing")
            need(epochs[cursor]["select"]["time"] - zero >= parameters["settle_time_s"] * 1e9 if attempts == 0 else True, "fine contact settle missing")
            backoff(epochs[cursor], first, max(parameters["small_backoff_rad"], parameters["backoff_rad"]), direction, False, label + "_repeat_backoff")
            second, zero = approach(epochs[cursor + 1], direction * parameters["fine_speed_rad_s"], label + "_fine_repeat")
            cursor += 2; attempts += 1
            repeatability = abs(second["angle_rad"] - first["angle_rad"])
            if repeatability <= parameters["repeatability_rad"]: break
        results.append(dict(endpoint=label, angle_rad=.5 * (first["angle_rad"] + second["angle_rad"]),
                            repeatability_rad=repeatability, repeatability_retries=attempts - 1))
    endpoint(direction, "a")
    endpoint(-direction, "b")
    need(cursor + 1 == len(epochs) and epochs[cursor]["mode"] == 1, "unexpected/missing midpoint epoch")
    midpoint_epoch = epochs[cursor]
    midpoint = .5 * (results[0]["angle_rad"] + results[1]["angle_rad"])
    span_deg = abs(results[0]["angle_rad"] - results[1]["angle_rad"]) * 180 / math.pi
    repeatability = max(r["repeatability_rad"] for r in results)
    need(band["minimum_deg"] <= span_deg <= band["maximum_deg"] and pose_close(midpoint_epoch["action"]["value"], midpoint), "endpoint span/midpoint target differs")
    centered = interval(midpoint_epoch["action"]["time"], final_stop["time"])
    dwell = []
    for f in reversed(centered):
        if abs(f["angle"] - midpoint) > guards["midpoint_tolerance_rad"] or abs(f["velocity"]) > parameters["backoff_arrive_vel_rad_s"]: break
        dwell.insert(0, f)
    need(len(dwell) >= 3 and dwell[-1]["time"] - dwell[0]["time"] >= guards["midpoint_dwell_s"] * 1e9 and
         final_stop["time"] - midpoint_epoch["action"]["time"] < guards["midpoint_timeout_s"] * 1e9, "measured midpoint dwell missing/timed out")
    endpoint_rows = [r for r in rows if r["kind"] == "homing_endpoints"]
    midpoint_rows = [r for r in rows if r["kind"] == "homing_midpoint_dwell"]
    need(len(endpoint_rows) == len(midpoint_rows) == 1, "homing endpoint/midpoint annotations missing/duplicated")
    for doc in (endpoint_rows[0], footer):
        need(pose_close(number(doc["endpoint_a_rad"], "invalid endpoint a"), results[0]["angle_rad"]) and
             pose_close(number(doc["endpoint_b_rad"], "invalid endpoint b"), results[1]["angle_rad"]) and
             pose_close(number(doc["midpoint_rad"], "invalid midpoint"), midpoint) and abs(number(doc["measured_travel_deg"], "invalid span") - span_deg) <= 2 * COUNT_RAD * 180 / math.pi and
             abs(number(doc["repeatability_rad"], "invalid repeatability") - repeatability) <= 2 * COUNT_RAD, "homing annotation/footer differs from raw geometry")
    center_row = midpoint_rows[0]
    need(pose_close(number(center_row["target_rad"], "invalid midpoint dwell target"), midpoint) and
         integer(center_row["begin_ns"], "invalid midpoint begin", 1) >= dwell[0]["time"] and
         center_row["end_ns"] - center_row["begin_ns"] >= guards["midpoint_dwell_s"] * 1e9 and center_row["end_ns"] < final_stop["time"] and
         pose_close(number(center_row["observed_rad"], "invalid midpoint observed pose"), latest(center_row["end_ns"])["angle"]),
         "midpoint dwell annotation differs from raw observations")
    need(footer.get("normal_stop_confirmed") is True and footer.get("abort_stop_confirmed") is False and footer.get("homing_observed") is True and
         footer.get("expected_original_mode") == original[MODE] and footer.get("observed_original_mode") == original[MODE], "STOP/original-mode footer differs from raw evidence")
    report.update(endpoints=results, contacts=contacts, backoffs=backoffs, midpoint_rad=midpoint, measured_travel_deg=span_deg,
        repeatability_rad=repeatability, encoder_resolution_rad=COUNT_RAD, midpoint_observed_rad=dwell[-1]["angle"], midpoint_dwell_s=(dwell[-1]["time"] - dwell[0]["time"]) / 1e9,
        normal_stop_confirmed=True, final_disabled_observed_ns=final_reset[0]["time"], original_settings={str(k):v for k,v in original.items()},
        restored_settings_verified=True, mode_transitions=len(enables), register_reads=len(observations), write_echoes_ignored=len(echoes),
        enabled_acknowledgements=enabled_acknowledgements,
        pitch_current_max_abs_A=max(abs(o["value"]) for o in iqf), pitch_temperature_max_C=max(f["temperature"] for f in feedback),
        encoder_mechpos_receipt_pairs=position_observations,
        encoder_mechpos_max_abs_receipt_difference_rad=max((abs(o["difference_rad"]) for o in position_observations), default=None),
        encoder_mechpos_declared_guard_rad=guards["encoder_mechpos_agreement_bound_rad"],
        encoder_mechpos_bias_correction=None,
        yaw_max_abs_displacement_rad=max(map(abs, yaw_displacement)), current_units={"yaw":{"raw_unit":"SIGNED_PROTOCOL_COUNT", "scale_A_per_count":None, "calibrated":False}},
        measurement_limitations=["Sensorless geometry does not qualify encoder/mechPos agreement or absolute mechanical coordinates.",
            "Encoder/MechPos differences pair receipts with unknown native sample time; observed offsets remain uncorrected.",
            "Motor-reported Iqf has unknown device sample time and no bound measurement calibration.",
            "IMU mounting/time mapping and yaw current/temperature scales remain unqualified.",
            "No current-mode stopping, dynamics, PlantSnapshot or controller qualification follows from homing."])
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        result = review(args.capture)
    except (ValueError, KeyError, TypeError, OSError, IndexError, OverflowError) as exc:
        result = {"schema":"adr0022.homing_review/1", "capture_complete":False, "homing_observed":False,
            "motion_authorized":False, "motion_qualified":False, "current_mode_qualified":False,
            "physical_capabilities_qualified":False, "physical_parameters_qualified":False,
            "encoder_mechpos_agreement_qualified":False, "reason":"DATA_INVALID", "detail":str(exc)}
    with args.output.open("x", encoding="utf-8") as output:
        json.dump(result, output, indent=2, allow_nan=False); output.write("\n")
    print(json.dumps(result, indent=2, allow_nan=False))
    return 0 if result["capture_complete"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
