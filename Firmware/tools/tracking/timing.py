"""ADR-003 stage 2: the tracking camera's optical time on the control clock, from ego-motion.

The yaw axis follows a known smooth motion (two sines, both directions, about 15 deg/s peak) under
the commissioned servo while the tracking camera records whatever is in front of it. Between two
frames a static scene moves only because the camera turned. The horizontal image translation
between consecutive frames, dx_k (global Lucas-Kanade on strong-gradient pixels at half the lores
resolution, iterated), is therefore

    dx_k = s * (q(t_k + delta) - q(t_(k-1) + delta)) + c,    t_k = SensorTimestamp_k + ExposureTime_k / 2

fitted against the encoder q (kernel receive time, crosstalk removed) by a dense then a fine
scan of R^2; the signed reversals of the sines pin delta. The scale s is a weighted mean over the
depths in view and is not used. Methods that failed on this station's scene and why (2026-10-02):
phase correlation locks on a zero-shift peak (low-texture wall, 33 ms blur, a near stand with large
parallax); frame-difference energy is biased by about a third of the exposure, because consecutive
blurred frames also differ by their blur width (a term in quadrature with the motion).

Six bands of rows are fitted separately; their delays rise linearly down the frame (the rolling
shutter), so delta(v) = a + row_time * v at full-frame row v, and the scatter about that line is
the timestamp uncertainty. Two sessions at different exposures separate the exposure term:
a(E) = d0 + k*E, so t_o = SensorTimestamp + d0 + (0.5 + k) * E + row_time * v.
"""
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "servo_commission"))
import journal  # noqa: E402

DEG = math.pi / 180
FULL_ROWS = 1080
BANDS = 6
GATES = {"r2_min": 0.6, "row_fit_residual_ms_max": 3.0, "exposure_model_residual_ms_max": 2.0}


def load_camera(stem):
    stem = Path(stem)
    rows = [json.loads(l) for l in open(stem.with_suffix(".camera.jsonl"), encoding="utf-8")]
    header = rows[0]
    frames = [r for r in rows if r.get("kind") == "frame"]
    w, h = header["lores"]
    data = np.fromfile(stem.with_suffix(".camera.bin"), dtype=np.uint8)
    n = min(len(frames), data.size // (w * h))
    images = data[: n * w * h].reshape(n, h, w)
    sensor = np.array([f["sensor_ns"] for f in frames[:n]], dtype=np.int64)
    exposure = np.array([(f.get("ExposureTime") or 0) * 1000 for f in frames[:n]], dtype=np.int64)  # us -> ns
    return images, sensor, exposure, header


def flow_x(a, b, iterations=4, keep=0.3):
    """Global horizontal translation of b relative to a (px of a): Lucas-Kanade on the strongest
    `keep` fraction of horizontal gradients, re-warping b each iteration."""
    gx = np.gradient(a, axis=1)
    mask = np.abs(gx) >= np.quantile(np.abs(gx), 1 - keep)
    mask[:, :2] = mask[:, -2:] = False
    g = gx[mask]
    w = a.shape[1]
    cols = np.arange(w, dtype=np.float64)
    d = 0.0
    for _ in range(iterations):
        x = np.clip(cols + d, 0, w - 1.000001)
        i0 = np.floor(x).astype(int)
        f = x - i0
        warped = b[:, i0] * (1 - f) + b[:, i0 + 1] * f  # every row shifted by the same d
        d += -float(np.sum(g * (warped - a)[mask]) / np.sum(g * g))
    return d


def flows(images, rows=None):
    """dx between consecutive frames at half resolution (optionally a band of rows)."""
    small = images[:, ::2, ::2].astype(np.float64) if rows is None else images[:, rows[0]:rows[1]:2, ::2].astype(np.float64)
    return np.array([flow_x(small[k - 1], small[k]) for k in range(1, len(small))])


def fit_delay(t_ns, dx, enc_t_ns, enc_q, scan_ms=100.0):
    """delta (s) maximising R^2 of dx_k = s*(q(t_k+delta) - q(t_(k-1)+delta)) + c; returns (delta, R^2)."""
    def score(delta_ns):
        dq = np.interp(t_ns[1:] + delta_ns, enc_t_ns, enc_q) - np.interp(t_ns[:-1] + delta_ns, enc_t_ns, enc_q)
        a = np.vstack([dq, np.ones_like(dq)]).T
        coef, *_ = np.linalg.lstsq(a, dx, rcond=None)
        return 1 - float(np.var(dx - a @ coef)) / max(float(np.var(dx)), 1e-12)
    coarse = np.arange(-scan_ms, scan_ms + 1e-9, 0.5) * 1e6
    best = max(coarse, key=score)
    fine = np.arange(best - 1e6, best + 1e6 + 1, 0.02e6)
    best = max(fine, key=score)
    return best * 1e-9, score(best)


def analyse(session_dir):
    """Delay of one recorded timing session (whole frame and row bands), with its exposure."""
    d = Path(session_dir)
    j = journal.yaw(d / "yaw-control.jsonl")
    asset = json.load(open(HERE.parents[1] / "config/servo/yaw_servo.json", encoding="utf-8"))
    x = asset["identified"]["crosstalk"]
    q = journal.true_position(j, x["table"], x["delay_s"])
    enc_t = j["begin_ns"] + np.round(j["feedback"]["t"] * 1e9).astype(np.int64)
    images, sensor, exposure, header = load_camera(d / "yaw-control")
    t_mid = sensor + exposure // 2
    use = np.where((t_mid > enc_t[0] + 0.1e9) & (t_mid < enc_t[-1] - 0.1e9))[0]
    tt = t_mid[use]
    delta, r2 = fit_delay(tt, flows(images[use]), enc_t, q)
    h = images.shape[1]
    bands = []
    for b in range(BANDS):
        r0, r1 = b * h // BANDS, (b + 1) * h // BANDS
        db, rb = fit_delay(tt, flows(images[use], (r0, r1)), enc_t, q)
        bands.append({"rows_full": [r0 * FULL_ROWS // h, r1 * FULL_ROWS // h], "delta_s": db, "r2": rb})
    rows = np.array([(b["rows_full"][0] + b["rows_full"][1]) / 2 for b in bands])
    deltas = np.array([b["delta_s"] for b in bands])
    row_time, at_row0 = (float(v) for v in np.polyfit(rows, deltas, 1))
    scatter = float(np.sqrt(np.mean((deltas - (at_row0 + row_time * rows)) ** 2))) * 1e3
    exposures = np.unique(exposure[use])
    fails = []
    if r2 < GATES["r2_min"]:
        fails.append(f"whole-frame R^2 {r2:.2f} < {GATES['r2_min']}")
    if scatter > GATES["row_fit_residual_ms_max"]:
        fails.append(f"row bands scatter {scatter:.1f} ms about the rolling-shutter line")
    if len(exposures) != 1:
        fails.append(f"exposure varied during the session ({sorted(exposures / 1e6)} ms)")
    return {"frames": int(len(use)), "exposure_s": float(np.median(exposure[use])) * 1e-9,
            "frame_interval_ms_median": float(np.median(np.diff(sensor)) / 1e6),
            "peak_reference_speed_dps": float(np.max(np.abs(j["cycles"]["vr"]))) / DEG,
            "delta_s": delta, "r2": r2, "bands": bands, "row_time_s": row_time, "delta_row0_s": at_row0,
            "row_fit_residual_ms": scatter, "fails": fails, "valid": not fails}


CENTRE_ROW = FULL_ROWS / 2


def combine(sessions):
    """TimingParameters from sessions at different exposures.

    The row time is a property of the readout, not of the exposure: it is taken from the session
    with the shortest exposure (the least motion blur, the cleanest bands). Each session's delay is
    read at the centre row on its own band line (insensitive to that line's slope), and those
    delays give the exposure model delta_c(E) = c + k*E. Then t_o(v) = SensorTimestamp + fixed_offset
    + (0.5 + k)*E + row_time*v with fixed_offset = c - row_time*CENTRE_ROW. The uncertainty is the
    worst band scatter plus half the row-time disagreement across the frame."""
    fails = [f"session {i}: {f}" for i, s in enumerate(sessions) for f in s["fails"]]
    e = np.array([s["exposure_s"] for s in sessions])
    centre = np.array([s["delta_row0_s"] + s["row_time_s"] * CENTRE_ROW for s in sessions])
    sharpest = sessions[int(np.argmin(e))]
    row_time = sharpest["row_time_s"]
    if len(set(np.round(e, 4))) < 2:
        fails.append("one exposure only: the exposure term is not identified")
        k, c, resid = 0.0, float(centre.mean()), 0.0
    else:
        k, c = (float(v) for v in np.polyfit(e, centre, 1))
        resid = float(np.sqrt(np.mean((centre - (c + k * e)) ** 2))) * 1e3 if len(e) > 2 else 0.0
        if resid > GATES["exposure_model_residual_ms_max"]:
            fails.append(f"exposure model residual {resid:.2f} ms")
    disagreement = max(abs(s["row_time_s"] - row_time) for s in sessions) * CENTRE_ROW / 2
    uncertainty = max(s["row_fit_residual_ms"] for s in sessions) * 1e-3 + disagreement
    if not -1 <= 0.5 + k <= 1:
        fails.append(f"exposure fraction {0.5 + k:.2f} outside [-1, 1]")
    return {"timing": {"fixed_offset_s": c - row_time * CENTRE_ROW, "exposure_fraction": 0.5 + k, "row_time_s": row_time},
            "exposure_coefficient": k, "delay_at_centre_row_s": [float(v) for v in centre],
            "row_times_s": [s["row_time_s"] for s in sessions], "timestamp_uncertainty_s": uncertainty,
            "exposure_model_residual_ms": resid, "gates": GATES, "fails": fails, "valid": not fails}
