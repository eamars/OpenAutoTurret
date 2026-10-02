"""ADR-003-shaped use-case references and the metrics that matter for framing."""
import math
import numpy as np

DEG = math.pi / 180


def level1(targets, dt=0.001, lam=4.0, vmax=20 * DEG, amax=60 * DEG, jmax=300 * DEG, q0=0.0):
    """ADR-003 Level-1 generator: a_req = lam^2 (q_t-q_r) + 2 lam (v_t-v_r), then a/j/v limits.
    targets: list of (duration_s, kind, value, label) where kind 'hold'/'step' sets q_t (value=delta),
    'ramp' moves q_t at value rad/s, 'sine' adds value=(amp, w) around current q_t."""
    t_all, qt_all, labels = [], [], []
    qt = q0
    t = 0.0
    seg_bounds = []
    for dur, kind, val, label in targets:
        n = int(round(dur / dt))
        start_t = t; base = qt
        for k in range(n):
            if kind == "step":
                q_target, v_target = base + val, 0.0
            elif kind == "ramp":
                q_target, v_target = base + val * k * dt, val
            elif kind == "sine":
                amp, w = val
                q_target = base + sum(A * math.sin(W * k * dt) for A, W in zip(amp, w))
                v_target = sum(A * W * math.cos(W * k * dt) for A, W in zip(amp, w))
            else:
                q_target, v_target = base, 0.0
            t_all.append(t); qt_all.append((q_target, v_target)); t += dt
        if kind == "step":
            qt = base + val
        elif kind == "ramp":
            qt = base + val * n * dt
        seg_bounds.append((start_t, t, kind, val, label))
    qr = vr = ar = 0.0
    qr = q0
    R = np.zeros((len(t_all), 4))
    for k, (q_target, v_target) in enumerate(qt_all):
        a_req = lam * lam * (q_target - qr) + 2 * lam * (v_target - vr)
        a_req = max(-amax, min(amax, a_req))
        da = max(-jmax * dt, min(jmax * dt, a_req - ar))
        ar += da
        vr_new = vr + ar * dt
        if abs(vr_new) > vmax:
            vr_new = math.copysign(vmax, vr_new); ar = (vr_new - vr) / dt
        qr += 0.5 * (vr + vr_new) * dt; vr = vr_new
        R[k] = (t_all[k], qr, vr, ar)
    return R[:, 0], R[:, 1], R[:, 2], R[:, 3], seg_bounds, np.array([x[0] for x in qt_all])


def script(scale=1.0):
    S = []
    S.append((2.0, "hold", 0, "hold"))
    for d in (0.1, 0.2, 0.5, 1.0, 2.0):
        S.append((1.5, "step", d * DEG, f"step+{d}"))
        S.append((1.5, "step", -d * DEG, f"step-{d}"))
    for s in (1, 2, 5, 10, 20):
        dur = max(3.0, min(8.0, 40.0 / s))
        S.append((dur, "ramp", s * DEG, f"ramp+{s}"))
        S.append((1.5, "hold", 0, f"stop+{s}"))
        S.append((dur, "ramp", -s * DEG, f"ramp-{s}"))
        S.append((1.5, "hold", 0, f"stop-{s}"))
    S.append((3.0, "ramp", 10 * DEG, "rev-a"))
    S.append((3.0, "ramp", -10 * DEG, "rev-b"))
    S.append((2.0, "hold", 0, "rev-stop"))
    S.append((7.0, "step", 90 * DEG, "long+90"))
    S.append((7.0, "step", -90 * DEG, "long-90"))
    S.append((20.0, "sine", ((6 * DEG, 1.5 * DEG, 0.5 * DEG), (0.5, 1.7, 4.1)), "walker"))
    S.append((2.0, "hold", 0, "final"))
    return S


def metrics(res, segs, true=True):
    """Per segment: servo error RMS/max (deg), actual speed mean/jitter (deg/s), step overshoot."""
    t, qr, vr = res["t"], res["qr"], res["vr"]
    if true:
        tq, q, v = res["true_t"], res["true_q"], res["true_v"]
        qa = np.interp(t, tq, q)
        # 20 ms moving average velocity (what a camera exposure integrates)
        vs = np.convolve(v, np.ones(20) / 20, mode="same")
        va = np.interp(t, tq, vs)
    else:
        qa, va = res["q"], res["v"]
    rows = []
    for (a, b, kind, val, label) in segs:
        m = (t >= a) & (t < b)
        if m.sum() < 5:
            continue
        e = (qr[m] - qa[m]) / DEG
        row = dict(label=label, err_rms=np.sqrt(np.mean(e**2)), err_max=np.max(np.abs(e)))
        if kind == "ramp":
            # Steady: the reference within 1% of the segment's rate (a 10 deg/s ramp never comes
            # within 1e-6 of it inside its 4 s, so an exact test left that speed unscored).
            mm = m & (t > a + 1.0) & (np.abs(vr - val) <= 0.01 * abs(val))
            if mm.sum() > 20:
                row["v_mean"] = np.mean(va[mm]) / DEG
                row["v_ratio"] = np.mean(va[mm]) / val
                row["v_jit"] = np.std(va[mm] - vr[mm]) / DEG
                ee = (qr[mm] - qa[mm]) / DEG
                ee = ee - np.polyval(np.polyfit(t[mm], ee, 1), t[mm])
                row["pos_p95_5"] = np.percentile(ee, 95) - np.percentile(ee, 5)
        if kind in ("step", "hold"):
            fin = m & (t > b - 0.3)
            row["final_err"] = np.mean(e[-max(1, fin.sum()):]) if fin.sum() else np.nan
            if kind == "step" and val != 0:
                travel = (qa[m] - qa[m][0]) / val
                row["overshoot_pct"] = max(0.0, (np.max(travel) - 1) * 100)
                ok = np.abs(qr[m] - qa[m]) < 0.05 * DEG
                tm = t[m]
                bad = np.where(~ok)[0]
                row["settle_s"] = (tm[bad[-1]] - a) if len(bad) else 0.0
        rows.append(row)
    return rows


def show(rows, keys=("err_rms", "err_max", "v_ratio", "v_jit", "pos_p95_5", "overshoot_pct", "settle_s", "final_err")):
    for r in rows:
        print(f"{r['label']:>10}  " + "  ".join(f"{k}={r[k]:7.3f}" for k in keys if k in r))
