"""Independently review neutral current-mode evidence; never authorize motion.

Commands, type-17 replies and type-2 status are decoded from the wire. A
COMPLETE footer, write echo, or state label cannot substitute for readback.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import struct

from adr0022_capture_review import need, statistics_ns, strict_json

SCHEMA = "adr0022.current-preparation/1"
MODE, IQREF, IQF = 0x7005, 0x7006, 0x701A
SENSORS = ("accel", "gyro", "rv", "game_rv")


def integer(value, detail, minimum=0):
    need(type(value) is int and value >= minimum, detail)
    return value


def number(value, detail):
    need(type(value) in (int, float) and math.isfinite(value), detail)
    return value


def manifest_number(config, key):
    value = config[key]
    need(type(value) in (str, int, float), f"invalid manifest {key}")
    try:
        result = float(value)
    except (ValueError, OverflowError):
        raise ValueError(f"invalid manifest {key}") from None
    need(math.isfinite(result) and result > 0, f"invalid manifest {key}")
    return result


def word(data, offset):
    return int.from_bytes(data[offset:offset + 2], "big")


def stream(times, start, end, gap, startup, detail):
    result = statistics_ns(times)
    need(result["max_gap_s"] <= gap / 1e9, f"{detail} observation gap")
    need(times[0] <= start + startup and end - times[-1] <= gap,
         f"{detail} does not cover the transition")
    return result


def review(path: Path):
    data = Path(path).read_bytes()
    need(data.endswith(b"\n"), "truncated capture")
    rows = [strict_json(line) for line in data.decode("utf-8").splitlines()]
    need(len(rows) > 2 and all(type(r) is dict for r in rows), "missing capture records")
    need(rows[0].get("kind") == "header" and rows[-1].get("kind") == "footer", "missing capture boundaries")
    need(sum(r.get("kind") == "header" for r in rows) == 1 and
         sum(r.get("kind") == "footer" for r in rows) == 1, "duplicate capture boundaries")
    header, footer = rows[0], rows[-1]
    need(header.get("schema") == SCHEMA and header.get("purpose") == "neutral_current_mode_verification",
         "unsupported preparation schema/purpose")
    provenance = header.get("provenance")
    need(provenance in ("SYNTHETIC", "MEASURED") and header.get("parameter_qualified") is False,
         "invalid provenance or qualification claim")
    need(footer.get("status") == "COMPLETE" and footer.get("detail") == "" and
         footer.get("parameter_qualified") is False and footer.get("motion_qualified") is False,
         "capture failed/incomplete or qualification claim invalid")
    config = strict_json(header["manifest_yaml"])
    need(config.get("schema") == SCHEMA and config.get("provenance") == provenance and
         config.get("purpose") == header["purpose"], "manifest identity differs")
    need(config.get("transport") == ("loopback_udp" if provenance == "SYNTHETIC" else "socketcan"),
         "manifest transport/provenance differs")
    support = config.get("pitch_supported_when_disabled")
    need(support is True or (type(support) is str and support == "true"), "disabled pitch support not declared")
    if provenance == "MEASURED":
        need(config["yaw"].get("interface") == "can0" and config["pitch"].get("interface") == "can1",
             "station topology differs")
        loss = footer.get("interface_loss_deltas")
        need(type(loss) is dict and set(loss) == {"yaw", "pitch"} and
             all(type(loss[a]) is dict and set(loss[a]) == {"rx_dropped", "rx_errors"} and
                 all(type(v) is int and v == 0 for v in loss[a].values()) for a in loss),
             "interface receive loss evidence missing/nonzero")
    uid = config["expected_pitch_uid"]
    need(type(uid) is str and len(uid) == 16 and all(c in "0123456789abcdef" for c in uid), "invalid expected UID")
    current_bound = manifest_number(config, "neutral_current_bound_A")
    displacement_bound = manifest_number(config, "transition_displacement_bound_rad")
    temperature_bound = manifest_number(config, "pitch_maximum_temperature_C")
    limits = config["limits"]
    ns = {key: int(manifest_number(limits, key + "_s") * 1e9) for key in
          ("clock_uncertainty", "dequeue_age", "can_gap", "imu_gap", "startup", "duration", "read_timeout", "read_period", "stop_period")}
    need(all(1 <= v <= 3600 * 10**9 for v in ns.values()) and ns["duration"] > ns["startup"], "invalid timing limits")
    try:
        minimum_status = int(limits["minimum_imu_status"])
    except (ValueError, TypeError):
        raise ValueError("invalid minimum IMU status") from None
    need(str(minimum_status) == str(limits["minimum_imu_status"]) and 0 <= minimum_status <= 3,
         "invalid minimum IMU status")
    drops = footer.get("socket_drops")
    need(type(drops) is dict and set(drops) == {"yaw", "pitch"} and
         all(type(v) is int and v == 0 for v in drops.values()), "final socket loss counters missing/nonzero")
    end = integer(footer["end_ns"], "invalid capture end", 1)
    allowed = {"header", "footer", "session_begin", "neutral_tx", "can_rx", "imu_raw", "preparation_state",
               "pitch_identity", "register_read", "register_rejected", "write_echo"}
    need(all(r.get("kind") in allowed for r in rows), "unexpected journal record")
    need(not any(r.get("kind") == "register_rejected" for r in rows), "required register rejected")

    commands = []
    for position, row in enumerate(rows):
        if row["kind"] != "neutral_tx":
            continue
        begin = integer(row["begin_ns"], "invalid command begin", 1)
        accepted = integer(row["kernel_accepted_ns"], "invalid command acceptance", 1)
        need(begin <= accepted <= end and row.get("success") is True, "command not accepted or time invalid")
        cid = integer(row["id"], "invalid command CAN ID")
        hex_data = row["data_hex"]
        need(type(hex_data) is str and len(hex_data) == 16 and all(c in "0123456789abcdef" for c in hex_data),
             "invalid command payload")
        wire = bytes.fromhex(hex_data)
        axis = row.get("axis")
        need(axis in ("yaw", "pitch"), "unknown command axis")
        kind, index, value = None, None, None
        if axis == "yaw":
            need(cid == 0x1FE and wire == bytes(8), "nonneutral yaw command")
        else:
            need(cid <= 0x1FFFFFFF and cid & 255 == 127 and (cid >> 8) & 65535 == 0,
                 "pitch command identity differs")
            kind = (cid >> 24) & 31
            need(kind in (0, 3, 4, 17, 18), "pitch command outside neutral contract")
            if kind in (17, 18):
                index = int.from_bytes(wire[:2], "little")
                need(wire[2:4] == bytes(2), "register command reserved bytes differ")
                if kind == 17:
                    need(index in (MODE, IQREF, IQF) and wire[4:] == bytes(4), "unexpected register read command")
                else:
                    need(index in (MODE, IQREF), "write outside neutral register contract")
                    value = wire[4] if index == MODE else struct.unpack("<f", wire[4:])[0]
                    need((index == IQREF and value == 0.) or
                         (index == MODE and value in range(4) and wire[5:] == bytes(3)), "nonneutral register write")
            else:
                need(wire == bytes(8), "simple command payload must be zero (no fault clearing)")
        commands.append(dict(row=row, position=position, time=begin, accepted=accepted,
                             wire=wire, kind=kind, index=index, value=value))
    need(commands and all(a["time"] < b["time"] for a, b in zip(commands, commands[1:])),
         "missing/reordered command timestamps")
    start = commands[0]["time"]
    begins = [r for r in rows if r["kind"] == "session_begin"]
    if begins:
        need(len(begins) == 1 and 0 < integer(begins[0]["time_ns"], "invalid session begin", 1) <= start,
             "invalid session begin")
        start = begins[0]["time_ns"]
    need(start < end and end - start < ns["duration"], "capture deadline exceeded")
    report = {"schema": "adr0022.current_review/1", "capture_sha256": hashlib.sha256(data).hexdigest(),
              "provenance": provenance, "capture_complete": True, "capability_scope": "neutral_transition_only",
              "neutral_transition_verified": True, "neutral_transition_qualified": provenance == "MEASURED",
              "physical_capabilities_qualified": False, "physical_parameters_qualified": False,
              "motion_authorized": False, "streams": {}, "final_socket_drops": drops,
              "measurement_limitations": ["Yaw current and temperature retain protocol units without a bound calibration.",
                  "Register device sample times are unknown; receipt timing does not qualify current sample timing.",
                  "IMU mounting and device-to-host sample time mapping remain unqualified.",
                  "Neutral transition evidence does not qualify excitation, dynamics, homing, or a controller."]}

    frames = {axis: [] for axis in ("yaw", "pitch")}
    for position, row in enumerate(rows):
        if row["kind"] != "can_rx":
            continue
        axis = row.get("axis")
        need(axis in frames, "unknown received CAN axis")
        need(row.get("generation") == 1 and type(row.get("generation")) is int and
             row.get("socket_drops") == 0 and type(row.get("socket_drops")) is int and
             row.get("drop_delta") == 0 and type(row.get("drop_delta")) is int and
             row.get("error") is False and row.get("rtr") is False and row.get("dlc") == 8,
             "CAN loss/invalidity")
        need(type(row["bytes"]) is list and len(row["bytes"]) == 8 and
             all(type(b) is int and 0 <= b <= 255 for b in row["bytes"]), "invalid received CAN bytes")
        wire = bytes(row["bytes"])
        stamp = integer(row["kernel_monotonic_ns"], "invalid CAN receive timestamp", 1)
        dequeue = integer(row["dequeue_ns"], "invalid CAN dequeue timestamp", 1)
        uncertainty = integer(row["clock_uncertainty_ns"], "invalid clock uncertainty")
        integer(row["kernel_realtime_ns"], "missing CAN kernel timestamp", 1)
        need(uncertainty <= ns["clock_uncertainty"] and -uncertainty <= dequeue - stamp <= ns["dequeue_age"] and
             dequeue <= end, "CAN clock mapping/dequeue age invalid")
        cid = integer(row["id"], "invalid received CAN ID")
        frame = dict(row=row, position=position, wire=wire, time=stamp, kind=None)
        if axis == "yaw":
            need(row.get("extended") is False and cid == 0x205 and word(wire, 0) <= 8191, "unexpected yaw traffic")
            angle, speed, current, temp = word(wire, 0), int.from_bytes(wire[2:4], "big", signed=True), int.from_bytes(wire[4:6], "big", signed=True), wire[6]
            need(row.get("angle_raw") == angle and row.get("speed_rpm") == speed and
                 row.get("current_raw") == current and row.get("temperature_raw") == temp and
                 row.get("temperature_C") is None, "yaw raw fields differ from wire/unqualified temperature")
            frame.update(angle=angle, current=current, temperature=temp)
        else:
            need(row.get("extended") is True and cid <= 0x1FFFFFFF, "invalid pitch CAN frame")
            kind = (cid >> 24) & 31
            need(kind in (0, 2, 17, 18), "unexpected pitch traffic")
            frame["kind"] = kind
            if kind == 2:
                need(cid & 255 == 0 and (cid >> 8) & 255 == 127 and (cid >> 16) & 63 == 0,
                     "pitch feedback identity/fault differs")
                mode = (cid >> 22) & 3
                need(mode in (0, 2), "pitch feedback not disabled/enabled")
                fields = [word(wire, at) for at in (0, 2, 4, 6)]
                need(all(row.get(name) == value for name, value in zip(
                    ("angle_raw", "velocity_raw", "torque_raw", "temperature_raw"), fields)) and
                     math.isclose(number(row.get("temperature_C"), "pitch temperature missing"), fields[3] / 10),
                     "pitch raw fields differ from wire")
                need(fields[3] / 10 < temperature_bound, "pitch temperature bound exceeded")
                frame.update(mode=mode, angle=fields[0] * (8 * math.pi) / 65535 - 4 * math.pi,
                             temperature=fields[3] / 10)
            elif kind == 0:
                need(cid == 0x7FFE and wire.hex() == uid, "pitch discovery identity differs")
            else:
                need(cid & 255 == 0 and (cid >> 8) & 65535 == 127 and wire[2:4] == bytes(2),
                     "register response identity/status differs")
                frame["index"] = int.from_bytes(wire[:2], "little")
                need(frame["index"] in (MODE, IQREF, IQF), "unexpected register response")
                frame["value"] = wire[4] if frame["index"] == MODE else struct.unpack("<f", wire[4:])[0]
                number(frame["value"], "nonfinite register wire value")
        frames[axis].append(frame)
    for axis in frames:
        observed = frames[axis]
        need(len(observed) == integer(footer[axis + "_frames"], "invalid final CAN count") and
             [f["row"]["sequence"] for f in observed] == list(range(1, len(observed) + 1)), "CAN count/sequence differs")
        statistics_ns([f["time"] for f in observed])
        feedback = observed if axis == "yaw" else [f for f in observed if f["kind"] == 2]
        report["streams"][axis + "_feedback"] = stream([f["time"] for f in feedback], start, end,
            min(ns["can_gap"], 80_000_000) if axis == "yaw" else ns["can_gap"], ns["startup"], axis + " feedback")
        report["streams"][axis + "_feedback"]["max_dequeue_delay_s"] = max(f["row"]["dequeue_ns"] - f["time"] for f in observed) / 1e9
    yaw = frames["yaw"]
    cumulative, yaw_displacement = 0, [0.]
    scale = 2 * math.pi / 8192
    for a, b in zip(yaw, yaw[1:]):
        delta = b["angle"] - a["angle"]
        if delta > 4096: delta -= 8192
        if delta < -4096: delta += 8192
        need(abs(delta) * scale <= 40 * (b["time"] - a["time"]) / 1e9 + 2 * scale, "yaw encoder discontinuity")
        cumulative += delta
        yaw_displacement.append(cumulative * scale)
    pitch_feedback = [f for f in frames["pitch"] if f["kind"] == 2]
    pitch_displacement = [f["angle"] - pitch_feedback[0]["angle"] for f in pitch_feedback]
    need(max(map(abs, yaw_displacement)) <= displacement_bound and max(map(abs, pitch_displacement)) <= displacement_bound,
         "neutral transition displacement bound exceeded")
    report["displacement"] = {"bound_rad": displacement_bound, "yaw_max_abs_rad": max(map(abs, yaw_displacement)),
                              "pitch_max_abs_rad": max(map(abs, pitch_displacement)), "mechanically_homed": False}
    report["current_units"] = {"yaw": {"raw_unit": "SIGNED_PROTOCOL_COUNT", "scale_A_per_count": None,
        "calibrated": False, "raw_min": min(f["current"] for f in yaw), "raw_max": max(f["current"] for f in yaw),
        "legacy_derived_ampere_fields_ignored": sum(f["row"].get("current_A") is not None for f in yaw)}}
    report["temperatures"] = {"yaw": {"raw_min": min(f["temperature"] for f in yaw),
        "raw_max": max(f["temperature"] for f in yaw), "Celsius_mapping": "UNKNOWN"},
        "pitch": {"min_C": min(f["temperature"] for f in pitch_feedback), "max_C": max(f["temperature"] for f in pitch_feedback)}}

    imu = {sensor: [] for sensor in SENSORS}
    generations = set()
    for row in rows:
        if row["kind"] != "imu_raw":
            continue
        raw = strict_json(row["raw_json"])
        need(type(raw) is dict and raw.get("kind") not in ("trace_reset", "gap", "summary"), "IMU reset/discard/end within capture")
        dequeue = integer(row["dequeue_ns"], "invalid IMU dequeue", 1)
        need(dequeue <= end, "IMU dequeue beyond capture")
        if raw.get("kind") != "sample":
            continue
        sensor = raw.get("sensor")
        need(sensor in imu, "unknown IMU sensor")
        sample = integer(raw["sample_ns"], "invalid IMU sample time", 1)
        receive = integer(raw["rx_ns"], "invalid IMU receive time", 1)
        need(sample <= receive <= dequeue and receive - sample <= ns["imu_gap"] and dequeue - receive <= ns["dequeue_age"],
             "IMU clock/order/dequeue age invalid")
        generations.add(integer(raw["generation"], "invalid IMU generation"))
        seq = integer(raw["sequence"], "invalid IMU sequence")
        status = integer(raw["status"], "invalid IMU status")
        need(seq <= 255 and minimum_status <= status <= 3, "IMU sequence/status invalid")
        values = raw["values"]
        need(type(values) is list and len(values) == (3 if sensor in ("accel", "gyro") else 4), "invalid IMU dimensions")
        for v in values: number(v, "nonfinite IMU value")
        if len(values) == 4: need(.9801 <= sum(v * v for v in values) <= 1.0201, "invalid IMU quaternion norm")
        imu[sensor].append(raw)
    need(len(generations) == 1, "IMU generation changed or missing")
    for sensor, observations in imu.items():
        need(all(((a["sequence"] + 1) & 255) == b["sequence"] and a["rx_ns"] <= b["rx_ns"]
                 for a, b in zip(observations, observations[1:])), "IMU sequence loss/reordering")
        report["streams"][sensor] = stream([r["sample_ns"] for r in observations], start, end, ns["imu_gap"], ns["startup"], sensor)
        report["streams"][sensor].update(mounting_calibrated=False, sample_clock_calibrated=False,
            status_counts={str(status): sum(r["status"] == status for r in observations) for status in range(4)})

    pitch_commands = [c for c in commands if c["row"]["axis"] == "pitch"]
    discoveries = [c for c in pitch_commands if c["kind"] == 0]
    discovery_frames = [f for f in frames["pitch"] if f["kind"] == 0]
    identities = [r for r in rows if r["kind"] == "pitch_identity"]
    need(len(discoveries) == len(discovery_frames) == len(identities) == 1, "discovery evidence missing/duplicated")
    discover, identity_frame = discoveries[0], discovery_frames[0]
    need(discover["time"] <= identity_frame["time"] < discover["time"] + ns["read_timeout"] and
         discover["position"] < identity_frame["position"] and identities[0].get("uid_hex") == uid and
         identities[0].get("receive_ns") == identity_frame["time"], "discovery correlation differs")
    need(all(c["kind"] == 0 or c["time"] >= identity_frame["time"] for c in pitch_commands), "pitch operation before identity")

    requests = [c for c in pitch_commands if c["kind"] == 17]
    replies = [f for f in frames["pitch"] if f["kind"] == 17]
    reads = [(i, r) for i, r in enumerate(rows) if r["kind"] == "register_read"]
    need(len(requests) == len(replies) == len(reads) and
         [r["request_sequence"] for _, r in reads] == list(range(1, len(reads) + 1)), "register read evidence incomplete/reordered")
    observations = []
    for request, reply, (position, record) in zip(requests, replies, reads):
        rb_begin = integer(record["request_begin_ns"], "invalid register request begin", 1)
        rb_accepted = integer(record["request_accepted_ns"], "invalid register acceptance", 1)
        need(request["index"] == reply["index"] == record["index"] and
             record.get("source") == "type17_readback" and record.get("axis") == "pitch" and
             "device_sample_ns" in record and record["device_sample_ns"] is None and record.get("receive_ns") == reply["time"] and
             number(record["value"], "invalid register value") == reply["value"] and
             rb_begin <= request["time"] <= rb_begin + ns["dequeue_age"] and
             request["accepted"] <= rb_accepted <= reply["row"]["dequeue_ns"] and
             rb_begin <= reply["time"] < rb_begin + ns["read_timeout"] and
             request["position"] < reply["position"] < position, "register raw correlation/value differs")
        if observations: need(observations[-1]["time"] <= rb_begin, "overlapping register transaction")
        observations.append(dict(index=reply["index"], value=reply["value"], time=reply["time"], request=request, position=position))
    # Echoes retain provenance but are never used as readback observations.
    echoes = [f for f in frames["pitch"] if f["kind"] == 18]
    echo_records = [r for r in rows if r["kind"] == "write_echo"]
    need(len(echoes) == len(echo_records), "write echo evidence incomplete")
    for echo, record in zip(echoes, echo_records):
        writes = [c for c in pitch_commands if c["kind"] == 18 and c["wire"] == echo["wire"] and
                  c["time"] <= echo["time"] < c["time"] + ns["read_timeout"] and c["position"] < echo["position"]]
        need(writes and record.get("index") == echo["index"] and record.get("receive_ns") == echo["time"] and
             record.get("readback_verified") is False, "uncorrelated write echo")

    mode_writes = [c for c in pitch_commands if c["kind"] == 18 and c["index"] == MODE]
    enable = [c for c in pitch_commands if c["kind"] == 3]
    zero_writes = [c for c in pitch_commands if c["kind"] == 18 and c["index"] == IQREF]
    stops = [c for c in pitch_commands if c["kind"] == 4]
    need(len(mode_writes) == 2 and len(enable) == 1 and zero_writes and stops, "neutral transition commands missing/duplicated")
    select, restore = mode_writes
    enable = enable[0]
    original = observations[0] if observations else None
    need(original and original["index"] == MODE and original["value"] in range(4), "initial RunMode read missing/invalid")
    original_mode = int(original["value"])
    need(select["value"] == 3 and restore["value"] == original_mode == footer.get("original_mode"), "mode selection/restore differs")
    need(original["time"] < zero_writes[0]["time"] < select["time"] < enable["time"] < restore["time"],
         "neutral transition ordering invalid")
    initial_stops = [c for c in stops if c["time"] < original["request"]["time"]]
    need(initial_stops, "initial STOP missing before mode read")
    initial_stop = initial_stops[-1]
    initial_disabled = [f for f in pitch_feedback if initial_stop["time"] <= f["time"] < original["request"]["time"] and f["mode"] == 0]
    need(initial_disabled, "initial STOP disabled acknowledgement missing")
    need(zero_writes[0]["time"] - initial_disabled[0]["time"] >= 2 * 10**9, "disabled observation dwell missing")
    before = [o for o in observations if select["time"] < o["request"]["time"] and o["time"] < enable["time"]]
    after = [o for o in observations if enable["time"] < o["request"]["time"] and o["time"] < restore["time"]]
    need([(o["index"], o["value"]) for o in before] == [(MODE, 3), (IQREF, 0)], "mode/zero readback missing before enable")
    need(len(after) >= 3 and [(o["index"], o["value"]) for o in after[:2]] == [(MODE, 3), (IQREF, 0)] and
         all(o["index"] == IQF for o in after[2:]), "enabled mode/zero/current readback missing")
    final_stops = [c for c in stops if after[1]["time"] < c["time"] < restore["time"]]
    need(len(final_stops) == 1, "final STOP missing/duplicated")
    final_stop = final_stops[0]
    need([c for c in stops if enable["time"] < c["time"]] == [final_stop], "STOP interrupted enabled observation or followed restore")
    need(final_stop["time"] - after[1]["time"] >= 2 * 10**9, "enabled observation dwell missing")
    disabled_before = [f for f in pitch_feedback if initial_disabled[0]["time"] <= f["time"] < enable["time"]]
    enabled_feedback = [f for f in pitch_feedback if enable["time"] <= f["time"] < final_stop["time"]]
    final_disabled = [f for f in pitch_feedback if final_stop["time"] <= f["time"] < restore["time"]]
    need(disabled_before and all(f["mode"] == 0 for f in disabled_before), "pitch enabled before verified transition")
    need(enabled_feedback and all(f["mode"] == 2 for f in enabled_feedback) and
         enabled_feedback[0]["time"] <= after[1]["time"], "enabled feedback status absent/lost")
    need(final_disabled and final_disabled[-1]["mode"] == 0 and
         final_disabled[-1]["time"] < final_stop["time"] + ns["read_timeout"], "fresh STOP acknowledgement missing before restore")
    need(all(f["mode"] == 0 for f in pitch_feedback if f["time"] >= final_disabled[-1]["time"]), "pitch reenabled after final STOP")
    restored = [o for o in observations if o["request"]["time"] > restore["time"]]
    need(len(restored) == 1 and restored[0]["index"] == MODE and restored[0]["value"] == original_mode,
         "restored mode readback missing")
    need(len(observations) == len(before) + len(after) + len(restored) + 1, "unexpected register observations")
    current = after[2:]
    need(all(abs(o["value"]) <= current_bound and o["time"] < final_stop["time"] for o in current), "neutral pitch current bound exceeded")
    need(current[0]["time"] - after[1]["time"] < ns["read_timeout"] and
         final_stop["time"] - current[-1]["time"] < ns["read_timeout"], "neutral current stream coverage missing")
    report["streams"]["pitch_iqf"] = statistics_ns([o["time"] for o in current])
    need(report["streams"]["pitch_iqf"]["max_gap_s"] < ns["read_timeout"] / 1e9, "neutral current readback gap")
    report["streams"]["pitch_iqf"].update(device_sample_ns=None, sample_clock_calibrated=False)
    need(footer.get("normal_stop_confirmed") is True and footer.get("abort_stop_confirmed") is False,
         "normal STOP footer differs from evidence")
    report.update(pitch_uid_observed=uid, original_mode=original_mode, restored_mode=original_mode,
        final_disabled_observed_ns=final_disabled[-1]["time"], normal_stop_confirmed=True,
        pitch_current_max_abs_A=max(abs(o["value"]) for o in current), pitch_neutral_current_bound_A=current_bound,
        register_reads=len(observations), write_echoes_ignored=len(echoes), neutral_commands=len(commands))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        result = review(args.capture)
    except (ValueError, KeyError, TypeError, OSError, IndexError, OverflowError) as exc:
        result = {"schema": "adr0022.current_review/1", "capture_complete": False,
                  "capability_scope": "neutral_transition_only", "neutral_transition_verified": False,
                  "neutral_transition_qualified": False, "physical_capabilities_qualified": False,
                  "motion_authorized": False, "physical_parameters_qualified": False,
                  "reason": "DATA_INVALID", "detail": str(exc)}
    with args.output.open("x", encoding="utf-8") as output:
        json.dump(result, output, indent=2, allow_nan=False)
        output.write("\n")
    print(json.dumps(result, indent=2, allow_nan=False))
    return 0 if result["capture_complete"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
