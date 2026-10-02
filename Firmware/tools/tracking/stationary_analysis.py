"""ADR-003 3b stationary trials: acquisition (T01), hold (T02), near-zero speed (T09), the
camera-turning check, and the frame input, from stationary_trial.py's record plus controld's
per-tick tracking trace (OTA_TRACKING_TRACE). Both carry the station's CLOCK_MONOTONIC.

    stationary_analysis.py TRIALS.jsonl TRACE.jsonl [--out SUMMARY.json]

Pointing error is the core's target joint pose minus the measured axis (goal_q - q), in tracker
pixels (fx, fy of the 1920x1080 tracker frame). The image-domain check is the detected aim point
against the frame centre at 10 Hz, independent of the estimator.
"""
import argparse
import json
import math
import re
import statistics as st

FX, FY = 1389.0, 1467.0          # tracker intrinsics, px/rad
REST_LIMIT_PX = 10.7             # yaw_accuracy.json rest_error 0.44 deg in tracker pixels
HOLD_TAIL_S = 15.0


def load_trace(path):
    text = open(path, encoding="utf-8", errors="replace").read()
    rows = []
    for m in re.finditer(r'\{"t_ns".*?"(?:reacquired|rate_limited)":\d+\}', text, flags=re.S):
        try:
            rows.append(json.loads(m.group(0).replace("\n", "")))
        except json.JSONDecodeError:
            pass
    rows.sort(key=lambda r: r["t_ns"])
    return rows


def pct(xs, p):
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(round(p / 100 * (len(xs) - 1))))] if xs else float("nan")


def trial_metrics(trial, ticks, states):
    t0 = ticks[0]["t_ns"]
    ey = [(r["goal_q"][0] - r["q"][0]) * FX for r in ticks]          # yaw px (sign: joint)
    ep = [(r["goal_q"][1] - r["q"][1]) * FY for r in ticks]
    t = [(r["t_ns"] - t0) * 1e-9 for r in ticks]
    mag = [math.hypot(a, b) for a, b in zip(ey, ep)]
    # Acquisition: the last time the error was outside the rest limit.
    outside = [i for i, m in enumerate(mag) if m > REST_LIMIT_PX]
    settle = t[outside[-1] + 1] if outside and outside[-1] + 1 < len(t) else (0.0 if not outside else float("nan"))
    axis = 0 if trial["axis"] == "yaw" else 1
    e_axis = ey if axis == 0 else ep
    sign0 = math.copysign(1, e_axis[0]) if e_axis[0] else 1
    overshoot = max([0.0] + [-sign0 * e for e in e_axis])
    tail = [i for i, x in enumerate(t) if x >= t[-1] - HOLD_TAIL_S]
    hold = {
        "yaw_mean_px": st.fmean(ey[i] for i in tail), "yaw_std_px": st.pstdev([ey[i] for i in tail]),
        "pitch_mean_px": st.fmean(ep[i] for i in tail), "pitch_std_px": st.pstdev([ep[i] for i in tail]),
        "p95_px": pct([mag[i] for i in tail], 95), "max_px": max(mag[i] for i in tail),
        "ffw_mean": st.fmean(max(ticks[i]["ffw"]) for i in tail),
        # The pitch axis itself (what a fixed pitch must show as ~0) and its peak-to-peak.
        "pitch_q_std_deg": st.pstdev([math.degrees(ticks[i]["q"][1]) for i in tail]),
        "pitch_q_pp_deg": math.degrees(max(ticks[i]["q"][1] for i in tail) - min(ticks[i]["q"][1] for i in tail)),
        "rate_dps_mean": [st.fmean(math.degrees(ticks[i]["est"][2 + k]) for i in tail) for k in (0, 1)],
        "rate_over_sigma_p95": pct([max(abs(ticks[i]["est"][2 + k]) / max(ticks[i]["sig_w"][k], 1e-9)
                                        for k in (0, 1)) for i in tail], 95),
    }
    # Camera turning, target still: the slew is where the axis moves fastest.
    speeds = [abs(ticks[i]["q"][axis] - ticks[i - 1]["q"][axis]) / max((ticks[i]["t_ns"] - ticks[i - 1]["t_ns"]) * 1e-9, 1e-4)
              for i in range(1, len(ticks))]
    slew = [i for i, v in enumerate(speeds, start=1) if v > math.radians(3)]
    ego = {
        "peak_axis_dps": math.degrees(max(speeds)) if speeds else 0.0,
        "ticks_moving": len(slew),
        "rate_over_sigma_max": max((abs(ticks[i]["est"][2 + axis]) / max(ticks[i]["sig_w"][axis], 1e-9) for i in slew), default=float("nan")),
        "rate_dps_max": max((abs(math.degrees(ticks[i]["est"][2 + axis])) for i in slew), default=float("nan")),
    }
    # Frame input: a tick whose accepted counter rose received a frame.
    arrivals = [(r["t_ns"], r["age_s"]) for p, r in zip(ticks, ticks[1:]) if r["accepted"] > p["accepted"]]
    gaps = [(b[0] - a[0]) * 1e-6 for a, b in zip(arrivals, arrivals[1:])]
    frames = {
        "n": len(arrivals), "interval_ms_median": pct(gaps, 50), "interval_ms_p95": pct(gaps, 95),
        "interval_ms_max": max(gaps, default=float("nan")),
        "age_ms_median": pct([a * 1e3 for _, a in arrivals], 50), "age_ms_p95": pct([a * 1e3 for _, a in arrivals], 95),
        "rejected": ticks[-1]["rejected"] - ticks[0]["rejected"], "nis_mean": st.fmean(r["nis"] for r in ticks),
    }
    # Image domain: the detected aim point against the frame centre, during the hold tail.
    t_end = ticks[-1]["t_ns"]
    img = [(s["state"]["target_aim_x_norm"] - .5) * 1920 for s in states
           if s["rx_ns"] >= t_end - HOLD_TAIL_S * 1e9 and s["state"].get("target_aim_valid")]
    imgy = [(s["state"]["target_aim_y_norm"] - .5) * 1080 for s in states
            if s["rx_ns"] >= t_end - HOLD_TAIL_S * 1e9 and s["state"].get("target_aim_valid")]
    image = {"n": len(img), "x_mean_px": st.fmean(img) if img else float("nan"), "x_std_px": st.pstdev(img) if len(img) > 1 else float("nan"),
             "y_mean_px": st.fmean(imgy) if imgy else float("nan"), "y_std_px": st.pstdev(imgy) if len(imgy) > 1 else float("nan")}
    return {"trial": trial["trial"], "block": trial["block"], "axis": trial["axis"], "deg": trial["deg"],
            "initial_error_px": e_axis[0], "settle_s": settle, "overshoot_px": overshoot,
            "hold": hold, "camera_turning": ego, "frames": frames, "image": image}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("trials")
    ap.add_argument("trace")
    ap.add_argument("--out")
    args = ap.parse_args()
    rows = [json.loads(l) for l in open(args.trials, encoding="utf-8")]
    ticks = load_trace(args.trace)
    trials, cur = [], None
    for d in rows:
        m = d.get("mark", {})
        if m.get("event") == "offset":
            cur = {"trial": m["trial"], "axis": m["axis"], "deg": m["deg"], "block": d["block"]}
        elif m.get("event") == "track" and cur:
            cur["t0"] = d["rx_ns"]
        elif m.get("event") == "end" and cur and "t0" in cur:
            cur["t1"] = d["rx_ns"]
            trials.append(cur)
            cur = None
    results = []
    for tr in trials:
        tk = [r for r in ticks if tr["t0"] <= r["t_ns"] <= tr["t1"]]
        ss = [d for d in rows if "state" in d and d.get("trial") == tr["trial"]]
        if len(tk) < 100:
            results.append({"trial": tr["trial"], "block": tr["block"], "error": f"only {len(tk)} trace ticks"})
            continue
        results.append(trial_metrics(tr, tk, ss))
    for r in results:
        if "error" in r:
            print(f"{r['trial']:16s} {r['error']}")
            continue
        h, e, f, i = r["hold"], r["camera_turning"], r["frames"], r["image"]
        print(f"{r['trial']:16s} start {r['initial_error_px']:+7.1f}px settle {r['settle_s']:5.2f}s over {r['overshoot_px']:5.1f}px | "
              f"hold yaw {h['yaw_mean_px']:+5.2f}+-{h['yaw_std_px']:.2f} pitch {h['pitch_mean_px']:+5.2f}+-{h['pitch_std_px']:.2f} "
              f"p95 {h['p95_px']:.2f}px ffw {h['ffw_mean']:.2f} | pitch axis sd {h['pitch_q_std_deg']:.3f} pp {h['pitch_q_pp_deg']:.2f}deg | "
              f"img x {i['x_mean_px']:+5.1f}+-{i['x_std_px']:.1f} y {i['y_mean_px']:+5.1f}+-{i['y_std_px']:.1f} | "
              f"slew {e['peak_axis_dps']:4.1f}dps w/sig {e['rate_over_sigma_max']:.1f} | frames {f['interval_ms_median']:.0f}/"
              f"{f['interval_ms_p95']:.0f}/{f['interval_ms_max']:.0f}ms age {f['age_ms_median']:.0f}/{f['age_ms_p95']:.0f}ms")
    if args.out:
        json.dump(results, open(args.out, "w", encoding="utf-8"), indent=1)


if __name__ == "__main__":
    main()
