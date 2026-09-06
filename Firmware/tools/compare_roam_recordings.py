#!/usr/bin/env python3
"""Compare constant-speed horizontal roam telemetry from /api/state recordings.

Run with a project venv containing numpy and matplotlib:
  python tools/compare_roam_recordings.py output_prefix label=recording.json ...
Use --reference-speed for sweeps other than 3 degrees/s. Excludes startup and turns;
encoder rate uses a 200 ms difference to reduce encoder quantization noise.
"""
import argparse
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def measure(path, reference_speed=3):
    raw = path.read_bytes()
    rows = [x for x in json.loads(raw) if x.get("operating_mode") == "AUTO_ROAM"]
    rows = sorted({x["ts_ns"]: x for x in rows}.values(), key=lambda x: x["ts_ns"])
    if len(rows) < 30:
        raise ValueError(f"{path}: insufficient roam samples")
    t = np.array([x["ts_ns"] for x in rows]) * 1e-9
    t -= t[0]
    yaw = np.rad2deg([x["q_yaw_rad"] for x in rows])
    ref = np.rad2deg([x["q_ref_yaw_rad"] for x in rows])
    rate = np.rad2deg([x["q_ref_rate_yaw_rad_s"] for x in rows])
    accel = np.rad2deg([x["q_ref_accel_yaw_rad_s2"] for x in rows])
    g = np.arange(t[0] + .15, t[-1] - .15, .05)
    measured = (np.interp(g + .1, t, yaw) - np.interp(g - .1, t, yaw)) / .2
    commanded = np.interp(g, t, rate)
    steady = (g > 15) & (abs(abs(commanded) - reference_speed) < .05) & (abs(np.interp(g, t, accel)) < .5)
    for edge in t[np.r_[True, np.sign(rate[1:]) != np.sign(rate[:-1])]]:
        steady &= abs(g - edge) > 1
    if not steady.any():
        raise ValueError(f"{path}: no steady {reference_speed} degrees/s interval")
    error = measured[steady] - commanded[steady]
    summary = {
        "source": str(path), "sha256": hashlib.sha256(raw).hexdigest(),
        "duration_s": float(t[-1]), "unique_samples": len(rows),
        "steady_samples": int(steady.sum()), "velocity_window_s": .2,
        "reference_speed_deg_s": reference_speed,
        "observed_reference_max_deg_s": float(max(abs(rate))),
        "rate_error_rms_deg_s": float(np.sqrt(np.mean(error ** 2))),
        "absolute_reference_error_p95_deg": float(np.percentile(abs(ref - yaw), 95)),
        "faults": sorted({x.get("fault", "") for x in rows}),
        "safety_actions": sorted({x.get("safety_action", "") for x in rows}),
    }
    return summary, g, measured, commanded, steady


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output_prefix", type=Path)
    parser.add_argument("recordings", nargs="+", help="label=path")
    parser.add_argument("--reference-speed", type=float, default=3)
    args = parser.parse_args()
    if not np.isfinite(args.reference_speed) or args.reference_speed <= 0:
        parser.error("reference speed must be finite and positive")
    results = [(label, measure(Path(path), args.reference_speed)) for label, path in (x.split("=", 1) for x in args.recordings)]
    plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False})
    fig, axes = plt.subplots(2, 1, figsize=(10, 6.5), layout="constrained")
    labels = [label for label, _ in results]
    axes[0].bar(labels, [r[0]["rate_error_rms_deg_s"] for _, r in results], color="#246b8b")
    axes[0].set_ylabel("Speed error RMS (degrees/s)")
    axes[0].set_title(f"Steady intervals at {args.reference_speed:g} degrees/s; lower rate error is better")
    for label, (summary, g, measured, commanded, steady) in results:
        if label not in labels[:2]:
            continue
        # Show a representative interval, not the best-looking segment.
        center = g[np.flatnonzero(steady)[int(steady.sum()) // 2]]
        part = abs(g - center) <= 4
        shown = np.where(steady, measured * np.sign(commanded), np.nan)
        axes[1].plot(g[part] - center, shown[part], label=label, linewidth=1)
    axes[1].axhline(args.reference_speed, color="#a66612", linestyle="--", label="Reference")
    axes[1].set(xlabel="Seconds within a representative straight interval", ylabel="Encoder speed (degrees/s)")
    axes[1].legend(ncol=3)
    for ax in axes:
        ax.grid(axis="y", alpha=.2)
    fig.suptitle("Unloaded station: uninterrupted horizontal roam", fontsize=14)
    args.output_prefix.parent.mkdir(parents=True, exist_ok=True)
    for ext in ("png", "svg"):
        fig.savefig(args.output_prefix.with_suffix("." + ext), dpi=150)
    summaries = {label: r[0] for label, r in results}
    args.output_prefix.with_suffix(".json").write_text(json.dumps(summaries, indent=2), encoding="utf-8")
    print(json.dumps(summaries, indent=2))


if __name__ == "__main__":
    main()
