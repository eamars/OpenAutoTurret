"""Plot measured before/after reference trials; run in a project analysis venv.

Requires matplotlib, numpy and PyYAML. Arguments: evidence directory, output stem.
"""
import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml


def plot(evidence: Path, output: Path):
    fig, axes = plt.subplots(2, 2, figsize=(11, 6.6), sharex=True, sharey=True)
    summaries = []
    for row, axis in enumerate(("yaw", "pitch")):
        for col, revision in enumerate(("01", "02")):
            source = evidence / f"position-step-{axis}-{revision}"
            result = yaml.safe_load((source / "result.yaml").read_text())
            if not all(result.get(k) for k in ("ok", "restoration_verified", "ended_disabled")):
                raise ValueError(f"{source}: trial did not complete and restore")
            with (source / "samples.csv").open() as handle:
                records = list(csv.DictReader(handle))
            t_ns = np.array([int(r["t_ns"]) for r in records], dtype=np.int64)
            t = (t_ns - t_ns[0]) * 1e-9
            reference = np.rad2deg([float(r["q_ref_rad"]) - result["q_hold_rad"] for r in records])
            encoder = np.rad2deg([float(r["q_rad"]) - result["q_hold_rad"] for r in records])
            middle = (t >= 4) & (t < 15)
            late = (t >= 12) & (t < 15)
            summaries.append({"axis": axis, "revision": revision,
                              "reference_p2p_4_to_15s_deg": float(np.ptp(reference[middle])),
                              "encoder_median_12_to_15s_deg": float(np.median(encoder[late])),
                              "encoder_final_deg": float(np.median(encoder[t >= 19])),
                              "watchdog_reason": result.get("watchdog_reason"),
                              "restoration_verified": result["restoration_verified"],
                              "ended_disabled": result["ended_disabled"]})
            ax = axes[row, col]
            ax.plot(t, reference, color="#D98024", lw=1.5, label="Host reference")
            ax.plot(t, encoder, color="#225E95", lw=1.3, label="Motor encoder")
            ax.axvline(2, color="#999999", lw=.7, ls=":")
            ax.axvline(15, color="#999999", lw=.7, ls=":")
            ax.set_title(f"{axis.title()} · {'Before' if col == 0 else 'After'} reference fix", loc="left", fontsize=11)
            ax.grid(axis="y", color="#e6e6e6", lw=.6)
            ax.spines[["top", "right"]].set_visible(False)
            ax.set_xlim(0, 20)
            ax.set_ylim(-.3, .9)
            if row == 1:
                ax.set_xlabel("Time (s)")
            if col == 0:
                ax.set_ylabel("Angle from starting pose (°)")
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper right", bbox_to_anchor=(.97, .965), ncol=2, frameon=False)
    fig.suptitle("Half-degree step trials on the unloaded station", x=.07, ha="left", fontsize=15)
    fig.text(.07, .025, "Original gains · 2°/s trial limit · pitch 3 A / yaw 1 A · each trial restored and disabled\n"
             "Encoder angles are motor feedback; external load angle and a motor-delay model are unverified.",
             fontsize=9, color="#555555")
    fig.subplots_adjust(left=.075, right=.97, top=.87, bottom=.15, hspace=.35)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output.with_suffix(".png"), dpi=180)
    fig.savefig(output.with_suffix(".svg"))
    output.with_suffix(".json").write_text(json.dumps(summaries, indent=2), encoding="utf-8")
    print(json.dumps(summaries, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("evidence", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    plot(args.evidence, args.output)
