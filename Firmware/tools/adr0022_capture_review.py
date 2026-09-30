"""Strict offline review of a baseline capture; never authorizes motor output."""
from __future__ import annotations
import argparse
import hashlib
import json
import math
from pathlib import Path
import statistics


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


def statistics_ns(times):
    need(len(times) >= 3 and all(type(t) is int and t > 0 for t in times), "insufficient/invalid timestamps")
    gaps = [b - a for a, b in zip(times, times[1:])]
    need(min(gaps) > 0, "timestamps duplicate or reverse")
    return {"count": len(times), "observed_hz": (len(times) - 1) * 1e9 / (times[-1] - times[0]),
            "median_gap_s": statistics.median(gaps) / 1e9, "max_gap_s": max(gaps) / 1e9}


def review(path: Path):
    data = path.read_bytes()
    need(data.endswith(b"\n"), "truncated capture")
    rows = [strict_json(line) for line in data.decode("utf-8").splitlines()]
    need(len(rows) > 2 and rows[0].get("kind") == "header" and rows[-1].get("kind") == "footer", "missing capture boundaries")
    header, footer = rows[0], rows[-1]
    need(header.get("schema") == "adr0022.capture/1", "unsupported capture schema")
    need(header.get("provenance") in ("SYNTHETIC", "MEASURED"), "unknown provenance")
    need(footer.get("status") == "COMPLETE" and footer.get("parameter_qualified") is False, "capture failed/incomplete")
    need(sum(r.get("kind") == "header" for r in rows) == sum(r.get("kind") == "footer" for r in rows) == 1, "duplicate boundaries")
    report = {"schema": "adr0022.capture_review/1", "capture_sha256": hashlib.sha256(data).hexdigest(),
              "provenance": header["provenance"], "capture_complete": True,
              "physical_parameters_qualified": False, "motion_authorized": False, "streams": {}, "temperatures": {}}
    for axis in ("yaw", "pitch"):
        frames = [r for r in rows if r.get("kind") == "can_rx" and r.get("axis") == axis]
        need(len(frames) == footer[axis + "_frames"], "CAN count differs from durable footer")
        need([r["sequence"] for r in frames] == list(range(1, len(frames) + 1)), "missing/reordered capture frame")
        need(all(r["generation"] == 1 and r["socket_drops"] == 0 and r["drop_delta"] == 0
                 and not r["error"] and not r["rtr"] and r["dlc"] == 8 for r in frames), "CAN loss/invalidity")
        statistics_ns([r["kernel_monotonic_ns"] for r in frames])
        need(all(r["dequeue_ns"] + r["clock_uncertainty_ns"] >= r["kernel_monotonic_ns"]
                 and r["clock_uncertainty_ns"] >= 0 for r in frames), "CAN clock mapping invalid")
        feedback = [r for r in frames if "angle_raw" in r]
        report["streams"][axis + "_feedback"] = statistics_ns([r["kernel_monotonic_ns"] for r in feedback])
        report["streams"][axis + "_feedback"]["max_dequeue_delay_s"] = max(r["dequeue_ns"] - r["kernel_monotonic_ns"] for r in frames) / 1e9
        temps = [r["temperature_raw"] for r in feedback]
        need(all(type(t) is int for t in temps), "missing temperature wire values")
        if axis == "yaw":
            need(all(r["temperature_C"] is None for r in feedback), "unqualified yaw temperature conversion")
            report["temperatures"][axis] = {"raw_min": min(temps), "raw_max": max(temps), "Celsius_mapping": "UNKNOWN"}
        else:
            need(all(math.isclose(r["temperature_C"], r["temperature_raw"] / 10) for r in feedback), "pitch temperature scale differs")
            report["temperatures"][axis] = {"min_C": min(temps) / 10, "max_C": max(temps) / 10}
    imu = []
    for row in rows:
        if row.get("kind") != "imu_raw":
            continue
        raw = strict_json(row["raw_json"])
        need(raw.get("kind") not in ("trace_reset", "gap", "summary"), "IMU reset/discard/end within capture")
        if raw.get("kind") == "sample":
            need(all(math.isfinite(v) for v in raw["values"]), "invalid IMU value")
            need(row["dequeue_ns"] >= raw["rx_ns"] >= raw["sample_ns"], "IMU time order")
            imu.append(raw)
    need(len({r["generation"] for r in imu}) == 1, "IMU generation changed")
    for sensor in ("accel", "gyro", "rv", "game_rv"):
        stream = [r for r in imu if r["sensor"] == sensor]
        need(stream, f"required IMU stream missing: {sensor}")
        need(all(0 <= r["sequence"] <= 255 for r in stream), "IMU sequence range")
        need(all((a["sequence"] + 1) & 255 == b["sequence"] for a, b in zip(stream, stream[1:])), "IMU sequence loss")
        report["streams"][sensor] = statistics_ns([r["sample_ns"] for r in stream])
    requests = [r for r in rows if r.get("kind") == "register_request"]
    reads = [r for r in rows if r.get("kind") == "register_read"]
    need(len(reads) == footer["register_reads"] == len(requests), "register capture incomplete")
    need([r["request_sequence"] for r in reads] == [r["request_sequence"] for r in requests]
         == list(range(1, len(reads) + 1)), "register transaction sequence differs")
    for request, reply in zip(requests, reads):
        need(request["index"] == reply["index"] and request["begin_ns"] == reply["request_begin_ns"]
             and reply["receive_ns"] >= request["begin_ns"] and reply["device_sample_ns"] is None
             and reply["source"] == "type17_readback", "register correlation invalid")
        need(math.isfinite(reply["value"]), "nonfinite register read")
    iqf = [r for r in reads if r["index"] == 0x701A]
    if iqf:
        report["streams"]["pitch_iqf"] = statistics_ns([r["receive_ns"] for r in iqf])
        report["streams"]["pitch_iqf"]["sample_clock_calibrated"] = False
    report["register_values_first_observed"] = {str(index): next(r["value"] for r in reads if r["index"] == index)
                                                for index in sorted({r["index"] for r in reads})}
    identities = [r for r in rows if r.get("kind") == "pitch_identity"]
    need(len(identities) == 1, "pitch identity observation missing/duplicated")
    report["pitch_uid_observed"] = identities[0]["uid_hex"]
    stops = [r for r in rows if r.get("kind") == "pitch_stop_request"]
    disabled = [r for r in rows if r.get("kind") == "pitch_stop_confirmed"]
    need(len(stops) == len(disabled) == footer["pitch_stop_confirmed"], "STOP observations incomplete")
    for request, response in zip(stops, disabled):
        need(request["sequence"] == response["sequence"] and request["clear_fault"] is False
             and response["disabled"] is True and response["receive_ns"] >= request["begin_ns"], "STOP correlation invalid")
    report["pitch_disabled_observations"] = len(disabled)
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        result = review(args.capture)
    except (ValueError, KeyError, TypeError, OSError) as exc:
        result = {"capture_complete": False, "motion_authorized": False, "physical_parameters_qualified": False,
                  "reason": "DATA_INVALID", "detail": str(exc)}
    with args.output.open("x", encoding="utf-8") as output:
        json.dump(result, output, indent=2, allow_nan=False)
        output.write("\n")
    print(json.dumps(result, indent=2, allow_nan=False))
    raise SystemExit(0 if result["capture_complete"] else 1)
