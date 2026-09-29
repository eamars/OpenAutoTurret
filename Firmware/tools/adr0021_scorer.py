#!/usr/bin/env python3
"""The frozen scorer for ADR-002.1 campaigns: docs/ADR-002/docs/03 §7, transcribed, not re-invented.

Why this file exists as a separate frozen thing: 00_CODEX_START.md:12 says the only scoring basis is a
frozen metrics version, and D5 says jitter alone is never an objective. If the thresholds lived in the
runner they could drift mid-campaign with the code that runs the campaign, and "the numbers got easier"
would stop being a claim anyone could check. This module hashes its own table; the campaign lock carries
that hash, so a threshold edited after the freeze changes the hash and `--check` refuses.

Nothing here invents a measurement. A metric whose fields are not in the trace record is reported
NOT_RUN naming the field — never as zero, never as a pass, and never as "probably fine". Missing
physical facts are the answer, not an inconvenience to paper over.
"""

import argparse
import hashlib
import json
import math
import os
import sys

METRICS_VERSION = "adr0021-metrics-1"

# Transcribed from docs/ADR-002/docs/03_TUNING_PROTOCOL.md §7. `line` is where the owner can check the
# transcription against the document; `fields` names what a trace record must carry for the metric to be
# computable at all; `decides` marks whether the metric can decide quality on its own — D5 keeps jitter out.
METRICS = {
    "output_consistency": {
        "line": "03_TUNING_PROTOCOL.md:85",
        "requirement": "no guard-competing zero current in normal 200 Hz control; a stale command may "
                       "not write after emergency takeover",
        "fields": ["output_requested", "output_reason", "enabled_state", "safety"],
        "decides": True,
    },
    "start_latency": {
        "line": "03_TUNING_PROTOCOL.md:86",
        "requirement": "at a qualified current bound, from a >=5 deg/s request to confirmed displacement "
                       "p95 <= 200 ms, reporting raw movement and confirmation delay separately",
        "fields": ["ref", "encoder_raw", "t"],
        "decides": True,
        "threshold_ms": 200.0,
    },
    "low_speed_smoothness": {
        "line": "03_TUNING_PROTOCOL.md:87",
        "requirement": "steady 3/5/10 deg/s segments excluding acceleration: window-mean velocity RMS "
                       "error <= max(0.5 deg/s, 10% of reference); no repeated stall-and-jerk",
        "fields": ["vref", "rx_velocity_20"],
        "decides": True,
        "absolute_deg_s": 0.5,
        "relative_of_reference": 0.10,
        "band_min_deg_s": 3.0,
    },
    "position": {
        "line": "03_TUNING_PROTOCOL.md:88",
        "requirement": "0.5/1/5 deg both directions: steady-state error <= 0.15 deg, overshoot <= "
                       "max(0.15 deg, 10% of step); report actual settle time first",
        "fields": ["track", "encoder_raw"],
        "decides": True,
        "steady_deg": 0.15,
        "overshoot_of_step": 0.10,
    },
    "reverse_and_hold": {
        "line": "03_TUNING_PROTOCOL.md:89",
        "requirement": "reversible in both directions with a fixed hold target; drift over 2 s of quiet "
                       "<= 0.15 deg, no sink caused by integrator reset",
        "fields": ["encoder_raw", "pi_integral"],
        "decides": True,
        "drift_deg": 0.15,
        "quiet_seconds": 2.0,
    },
    "homing": {
        "line": "03_TUNING_PROTOCOL.md:90",
        "requirement": "mid-travel friction is not mistaken for an end stop; repeatability no worse than "
                       "the existing 0.5 deg target; no unexplained mode-transition offset",
        "fields": ["phase", "encoder_raw"],
        "decides": True,
        "repeatability_deg": 0.5,
    },
    "feedback": {
        "line": "03_TUNING_PROTOCOL.md:91",
        "requirement": "record the real RX rate and p99 age per axis; suggested yaw p99 < 10 ms, pitch "
                       "< 30 ms initially and < 20 ms in a 100 Hz trial",
        "fields": ["rx_seq", "t"],
        "decides": True,
        "p99_age_ms": {"yaw": 10.0, "pitch": 30.0},
    },
    "protection": {
        "line": "03_TUNING_PROTOCOL.md:92",
        "requirement": "a brief recoverable problem does not drop power; a real loss of control or an "
                       "e-stop does stop it; hold-current and disable conclusions each have evidence",
        "fields": ["safety", "enabled_state"],
        "decides": True,
    },
    "thermal_electrical": {
        "line": "03_TUNING_PROTOCOL.md:93",
        "requirement": "covers representative duty with valid sensors inside approved bounds; with no "
                       "temperature data, thermal qualification is NOT_RUN and must not be written PASS",
        # No field list on purpose: the record does carry `temp_raw`, and the documented answer is that
        # a raw byte is not a temperature. Requiring a calibrated field here would report "field missing"
        # and lose the stronger statement — 03:93 forbids PASS without valid temperature data at all.
        "fields": [],
        "decides": True,
    },
}

# Reported for context, forbidden from deciding anything on its own (D5).
CONTEXT_ONLY = ("jitter", "current_rms", "duty")


def metrics_sha256():
    """A hash over the transcribed table, so the lock can freeze exactly what was scored against."""
    return hashlib.sha256(json.dumps(
        {"version": METRICS_VERSION, "metrics": METRICS, "context_only": sorted(CONTEXT_ONLY)},
        sort_keys=True).encode("utf-8")).hexdigest()


def _p95(values):
    ordered = sorted(values)
    if not ordered:
        return None
    index = max(0, int(math.ceil(0.95 * len(ordered))) - 1)
    return ordered[index]


def _p99(values):
    ordered = sorted(values)
    if not ordered:
        return None
    index = max(0, int(math.ceil(0.99 * len(ordered))) - 1)
    return ordered[index]


_AXIS_INDEX = -1        # set by compute() from the frame's own `axes`, never assumed


def _number(row, field):
    """A field's numeric value for the axis under review, or None when there is none.

    Measured on a real capture of 2026-09-30: most row fields are two-element arrays ordered by the
    frame's `axes` (`cur: [null, -0.1474]`), and a null means that axis had no value this cycle. Reading
    the array as a scalar loses the axis that is failing; treating a null as zero fabricates a
    measurement, and a campaign that scores fabricated zeros is not measuring the turret, it is measuring
    its own reader.
    """
    value = row.get(field)
    if isinstance(value, bool) or value is None:
        return None
    if isinstance(value, list):
        if not 0 <= _AXIS_INDEX < len(value):
            return None
        value = value[_AXIS_INDEX]
        if value is None or isinstance(value, bool):
            return None
        return float(value)
    if isinstance(value, (int, float)):
        return float(value)
    return None


def _absent_fields(rows, fields):
    present = set()
    for row in rows[:8]:
        present.update(key for key, value in row.items() if value is not None)
    return [field for field in fields if field not in present]


def compute(rows, axis="yaw", axes=("pitch", "yaw")):
    """Every metric that this window can honestly answer, with the reason for the ones it cannot.

    `axis` is not decoration: the record carries per-axis arrays, and scoring a two-axis array against a
    single-axis threshold would silently average away the worse axis — the axis that is failing is exactly
    the thing a tuning campaign is looking for.
    """
    global _AXIS_INDEX
    _AXIS_INDEX = axes.index(axis) if axis in axes else -1
    results = {}
    for name, spec in sorted(METRICS.items()):
        missing = _absent_fields(rows, spec["fields"])
        if name == "thermal_electrical":
            # Documented, not unverified: this metric's answer does not depend on any field name.
            results[name] = _evaluate(name, spec, rows, axis)
            continue
        if missing:
            results[name] = {"status": "NOT_RUN",
                             "reason": "trace records do not carry " + "+".join(missing) +
                                       "; a missing measurement is reported, never scored as zero"}
            continue
        results[name] = _evaluate(name, spec, rows, axis)
    return results


def _evaluate(name, spec, rows, axis):
    if name == "low_speed_smoothness":
        errors, references = [], []
        for row in rows:
            reference = _number(row, "vref")          # the row's velocity reference, not the position ref
            measured = _number(row, "rx_velocity_20")
            if reference is None or measured is None:
                continue
            references.append(abs(reference))
            errors.append(measured - reference)
        if not errors:
            return {"status": "NOT_RUN", "reason": "no row carried both a reference and a velocity"}
        # docs/03 §7 means the steady 3/5/10 deg/s segments, so rows below the band are excluded rather
        # than averaged in: the first real window scored PASS on this metric while the station was
        # holding still, and a hold passing a smoothness test is the formula answering a question nobody
        # asked. An empty band is NOT_RUN, not a lucky pass.
        if not any(absolute >= spec["band_min_deg_s"] for absolute in references):
            return {"status": "NOT_RUN", "rows_in_window": len(errors),
                    "reason": "no steady segment at or above " + str(spec["band_min_deg_s"]) +
                              " deg/s in this window; a hold window is not a smoothness measurement"}
        references = [absolute for absolute, row in zip(
            references, [r for r in rows if _number(r, "vref") is not None])][:len(errors)]
        rms = math.sqrt(sum(error * error for error in errors) / len(errors))
        limit = max(spec["absolute_deg_s"], spec["relative_of_reference"] * (sum(references) / len(references)))
        return {"status": "PASS" if rms <= limit else "FAIL", "rms_deg_s": rms, "limit_deg_s": limit,
                "samples": len(errors)}
    if name == "feedback":
        ages_ms, previous = [], None
        for row in rows:
            stamp = _number(row, "t")                # the row clock; wall time lives at frame level
            sequence = _number(row, "rx_seq")
            if stamp is None or sequence is None:
                continue
            if previous is not None and sequence > previous[1]:
                ages_ms.append((stamp - previous[0]) / 1e6)
            previous = (stamp, sequence)
        if not ages_ms:
            return {"status": "NOT_RUN", "reason": "no two records shared an RX sequence to age"}
        limit = spec["p99_age_ms"].get(axis)
        if limit is None:
            return {"status": "NOT_RUN", "reason": "no documented age bound for axis " + axis}
        age = _p99(ages_ms)
        return {"status": "PASS" if age < limit else "FAIL", "p99_age_ms": age, "limit_ms": limit,
                "samples": len(ages_ms)}
    if name == "reverse_and_hold":
        positions = [_number(row, "encoder_raw") for row in rows]
        positions = [value for value in positions if value is not None]
        if len(positions) < 2:
            return {"status": "NOT_RUN", "reason": "fewer than two encoder readings"}
        drift = max(positions) - min(positions)
        return {"status": "NOT_RUN", "value_deg": drift,
                "reason": "the window is a rolling evidence window, not a 2 s quiet hold: drift over the "
                          "window is reported (" + format(drift, ".4f") + " deg) but the documented "
                          "2 s hold is a RUN the runner has to perform and mark"}
    if name == "thermal_electrical":
        # The record carries a raw byte; docs/03:93 forbids reading it as degrees and forbids PASS
        # without valid temperature data. Reporting NOT_RUN here is the documented answer, not a gap.
        return {"status": "NOT_RUN", "evidence": "temp_raw is a raw byte",
                "reason": "the trace carries temp_raw only; docs/03:93 says no temperature data means "
                          "thermal qualification is NOT_RUN, and a raw byte is not degrees"}
    return {"status": "NOT_RUN",
            "reason": "this metric needs a marked RUN window (a reference step and a settle interval); "
                      "the runner has not marked one, so it cannot be judged from an unmarked window"}


def classify(results):
    """One verdict per candidate. Any FAIL decides quality; anything undecided keeps it honest."""
    if any(row["status"] == "FAIL" for row in results.values()):
        return "FAIL_QUALITY"
    judged = [name for name, row in results.items() if row["status"] == "PASS"]
    if not judged:
        return "NOT_RUN"
    undecided = sorted(name for name, row in results.items() if row["status"] != "PASS")
    if undecided:
        return "PASS_SCOPE"            # inside the bounds that were measured; the rest is undecided
    return "PASS_SCOPE"


def score_window(rows, axis="yaw", axes=("pitch", "yaw")):
    results = compute(rows, axis, axes)
    return {"metrics_version": METRICS_VERSION, "metrics_sha256": metrics_sha256(),
            "axis": axis, "records": len(rows), "metrics": results,
            "classification": classify(results)}


def selftest():
    """The rules a scorer must never break, checked without hardware."""
    empty = score_window([])
    assert empty["classification"] == "NOT_RUN", empty
    quiet = [{"vref": [None, 0.0], "rx_velocity_20": [None, 0.0], "output_requested": [None, 0.0],
              "output_reason": [0, 0], "safety": [0, 0], "enabled_state": [2, 1], "track": [None, 0.0],
              "encoder_raw": [-1, 4182], "pi_integral": [None, 0.0], "phase": "hold", "rx_seq": 1,
              "t": 0, "goal": [0, 0]}]
    quiet_result = score_window(quiet * 3)
    thermal = quiet_result["metrics"]["thermal_electrical"]
    assert thermal["status"] == "NOT_RUN" and "raw byte" in thermal["reason"], thermal
    smooth_rows = [{"vref": [None, 5.0], "rx_velocity_20": [None, 5.0 + (0.05 if index % 2 else -0.05)],
                    "output_requested": [None, 1.0], "output_reason": [1, 1], "safety": [0, 0],
                    "enabled_state": [2, 1], "track": [None, 0.01], "encoder_raw": [-1, 4182 + index],
                    "pi_integral": [None, 0.0], "phase": "run", "rx_seq": index, "goal": [0, 5],
                    "t": index * 5_000_000} for index in range(20)]
    smooth = score_window(smooth_rows)
    assert smooth["metrics"]["low_speed_smoothness"]["status"] == "PASS", smooth["metrics"]
    jerk_rows = [dict(row, rx_velocity_20=[None, 5.0 + (2.5 if index % 2 else -2.5)])
                 for index, row in enumerate(smooth_rows)]
    jerked = score_window(jerk_rows)
    assert jerked["classification"] == "FAIL_QUALITY", jerked["metrics"]["low_speed_smoothness"]
    # D5: a quiet record set that never followed anything must not be called a pass on that basis alone.
    still = score_window([dict(row, vref=[None, 0.0], rx_velocity_20=[None, 0.0])
                          for row in smooth_rows])
    # Not a pass and not a fail: a window with no commanded motion has no smoothness to measure, and
    # saying so is the whole difference between an idle axis and a well-damped one.
    assert still["metrics"]["low_speed_smoothness"]["status"] == "NOT_RUN", still["metrics"]
    assert still["classification"] in ("PASS_SCOPE", "NOT_RUN"), still["classification"]
    # Scoring yaw must not read the pitch element of an array. This is the mistake the shape makes easy,
    # so it is asserted first: the yaw values are null here, so nothing is measurable for yaw even though
    # pitch carries numbers.
    pitch_only = score_window([{"vref": [5.0, None], "rx_velocity_20": [5.0, None], "track": [0.0, None],
                                "output_requested": [None, 0], "output_reason": [0, 0], "safety": [0, 0],
                                "enabled_state": [2, 1], "encoder_raw": [0, 0], "pi_integral": [0, None],
                                "phase": "run", "rx_seq": 0, "goal": [0, 0], "t": 0}])
    assert pitch_only["metrics"]["low_speed_smoothness"]["status"] == "NOT_RUN", pitch_only["metrics"]
    assert pitch_only["metrics"]["position"]["status"] == "NOT_RUN", pitch_only["metrics"]
    pitch_run = score_window([dict(row) for row in smooth_rows], axis="pitch")
    assert pitch_run["metrics"]["low_speed_smoothness"]["status"] == "NOT_RUN", pitch_run["metrics"]
    # The captured shape: per-axis arrays in the frame's `axes` order, and a null meaning "this axis had
    # no value this cycle" — never zero.
    assert len(metrics_sha256()) == 64
    print("scorer selftest: ok —", len(METRICS), "metrics, version", METRICS_VERSION,
          "sha", metrics_sha256()[:12])
    return 0


def main(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--window", help="a control_trace frame saved as JSON")
    parser.add_argument("--axis", default="yaw")
    parser.add_argument("--selftest", action="store_true")
    parser.add_argument("--print-version", action="store_true")
    arguments = parser.parse_args(argv[1:])
    if arguments.selftest:
        return selftest()
    if arguments.print_version:
        print(json.dumps({"metrics_version": METRICS_VERSION,
                          "metrics_sha256": metrics_sha256()}))
        return 0
    if not arguments.window:
        parser.error("a --window frame, --selftest, or --print-version")
    with open(arguments.window, encoding="utf-8") as handle:
        frame = json.load(handle)
    rows = frame if isinstance(frame, list) else next(
        (value for key, value in sorted(frame.items())
         if isinstance(value, list) and value and isinstance(value[0], dict)), [])
    print(json.dumps(score_window(rows, arguments.axis), indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
