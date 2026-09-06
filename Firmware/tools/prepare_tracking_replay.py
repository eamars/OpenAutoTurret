#!/usr/bin/env python3
"""Prepare selected native measurements for the offline C++ boundary probe.

This uses capture timestamps and interpolated recorded encoder poses. The API
pose sampling rate is lower than the controller's internal history: this is a
component replay, not a byte-exact replay of the complete running controller.
Invalid observations are retained as gaps; they never become measurements.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("native", type=Path)
    parser.add_argument("states", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    frames = json.loads(args.native.read_text(encoding="utf-8"))
    states = sorted({s["ts_ns"]: s for s in json.loads(
        args.states.read_text(encoding="utf-8"))}.values(), key=lambda s: s["ts_ns"])
    times = np.array([s["ts_ns"] for s in states])
    poses = {axis: np.array([s[f"q_{axis}_rad"] for s in states]) for axis in ("yaw", "pitch")}
    seen = set()
    identities = {}
    rows = []
    for frame in frames:
        docs = frame["documents"]
        obs = docs["selected_target.json"]
        captured = obs["sensor_timestamp_ns"]
        key = (obs["session_uuid"], captured)
        if key in seen:
            continue
        seen.add(key)
        if captured < times[0] or captured > times[-1]:
            continue
        identity = (obs["session_uuid"], obs["track_uuid"])
        identities.setdefault(identity, len(identities) + 1)
        track_set = docs["track_set.json"]
        width, height = track_set["stream_width"], track_set["stream_height"]
        anchor, box = obs["measured_anchor"], obs["bbox"]
        rows.append([
            captured, obs["publish_timestamp_ns"], identities[identity], int(obs["measurement_valid"]),
            anchor["x"] * width, anchor["y"] * height,
            (box["x_max"] - box["x_min"]) * width, (box["y_max"] - box["y_min"]) * height,
            obs["detector_score"], obs["association_quality"], obs["identity_confidence"],
            *[float(np.interp(captured, times, poses[a])) for a in ("yaw", "pitch")],
        ])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["capture_ns", "arrival_ns", "identity", "valid", "u_px", "v_px",
                         "width_px", "height_px", "confidence", "association", "continuity",
                         "capture_yaw_rad", "capture_pitch_rad"])
        writer.writerows(sorted(rows))
    summary = {"rows": len(rows), "valid_measurements": sum(row[3] for row in rows),
               "identities": len(identities),
               "sources": {str(p): hashlib.sha256(p.read_bytes()).hexdigest()
                           for p in (args.native, args.states)},
               "limitation": "API-sampled pose interpolation; arrival uses perception publication time"}
    args.output.with_suffix(".meta.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
