"""Score a station journal against the use-case segments of its manifest.

    python score.py yaw   JOURNAL MANIFEST
    python score.py pitch JOURNAL MANIFEST

Yaw truth is the 1 kHz GM6020 encoder with the manifest's calibrated current
crosstalk removed (the same model the servo uses). Pitch truth is the CyberGear
type-2 position. Per segment: tracking error RMS/max (deg); ramps add speed
ratio and jitter of 20 ms-averaged speed and detrended position P95-P5; steps
add overshoot and settling (0.05 deg band); holds/steps report final error.
"""
import json, math, sys
import numpy as np
from scipy.signal import savgol_filter
import usecase

DEG = math.pi / 180
Q = 2 * math.pi / 8192


def yaw(path, manifest):
    m = json.load(open(manifest, encoding="utf-8"))
    cyc, enc, txs, begin, first_raw = [], [], [], None, None
    for line in open(path, encoding="utf-8"):
        k = line[7:35]
        if '"servo_cycle"' in k:
            r = json.loads(line)
            cyc.append((r["time_ns"], r["qr"], r["vr"], r["ar"], r["q"], r["v"], r["u"]))
        elif '"yaw_feedback"' in k:
            r = json.loads(line); enc.append((r["kernel_monotonic_ns"], r["encoder_unwrapped_counts"]))
            if first_raw is None: first_raw = r["encoder_raw"]
        elif '"yaw_current_tx"' in k:
            r = json.loads(line)
            if r["success"]: txs.append((r["kernel_accepted_ns"], r["successful_tx_A"]))
        elif '"yaw_control_begin"' in k:
            begin = json.loads(line)["time_ns"]
    C, E, T = np.array(cyc), np.array(enc, float), np.array(txs)
    res = {n: C[:, j] for j, n in enumerate("t qr vr ar q v u".split())}
    res["t"] = (res["t"] - begin) * 1e-9
    et = (E[:, 0] - begin) * 1e-9
    eq = E[:, 1] * Q + first_raw * Q  # servo coordinates are the absolute motor angle
    sp = m["servo_parameters"]
    if sp.get("crosstalk_map"):
        tt = (T[:, 0] - begin) * 1e-9
        k = np.searchsorted(tt, et - sp.get("crosstalk_delay_s", 0.0), side="right") - 1
        cur = np.where(k >= 0, T[np.clip(k, 0, None), 1], 0.0)
        table = np.array(sp["crosstalk_map"]); n = len(table)
        x = np.mod(eq, 2 * math.pi) / (2 * math.pi) * n
        k0 = np.floor(x).astype(int) % n; w = x - np.floor(x)
        eq = eq - ((1 - w) * table[k0] + w * table[(k0 + 1) % n]) * cur
    tu = np.arange(max(0, et[0]), et[-1], 0.001)
    qu = np.interp(tu, et, eq)
    res["true_t"], res["true_q"] = tu, qu
    res["true_v"] = savgol_filter(qu, 21, 2, deriv=1, delta=0.001)
    segs = [(s["begin_s"], s["end_s"], s["kind"], s["value"], s["label"]) for s in m["reference_segments_labels"]]
    print(f"cycles {len(C)}  duration {res['t'][-1]:.1f}s  peak|u| {np.max(np.abs(res['u'])):.3f} A")
    usecase.show(usecase.metrics(res, segs))


def pitch(path, manifest):
    m = json.load(open(manifest, encoding="utf-8"))
    rows, app = [], None
    for line in open(path, encoding="utf-8"):
        if line.startswith('{"kind":"pitch_trial"'): rows.append(json.loads(line))
        elif line.startswith('{"kind":"pitch_trial_approach"'): app = json.loads(line)
    t0 = rows[0]["time_ns"] + (int(app["duration_s"] * 1e9) if app else 0)
    t = np.array([(r["time_ns"] - t0) * 1e-9 for r in rows])
    qr, q = np.array([r["qr"] for r in rows]), np.array([r["q"] for r in rows])
    print(f"cycles {len(t)}  duration {t[-1]:.1f}s  peak|torque| {max(abs(r['torque']) for r in rows):.2f} Nm")
    for s in m["reference_segments_labels"]:
        k = (t >= s["begin_s"]) & (t < s["end_s"])
        if k.sum() < 10: continue
        e = (qr[k] - q[k]) / DEG
        tail = f" final={e[-1]:+.3f}" if s["kind"] in ("hold", "step") else ""
        print(f"{s['label']:>10}  err_rms={np.sqrt(np.mean(e**2)):.3f}  err_max={np.max(np.abs(e)):.3f}{tail}")


if __name__ == "__main__":
    {"yaw": yaw, "pitch": pitch}[sys.argv[1]](sys.argv[2], sys.argv[3])
