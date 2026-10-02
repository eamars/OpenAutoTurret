"""Identification from station journals: friction, crosstalk, inertia (yaw); speed-loop response (pitch).

Every estimator is a fixed computation on recorded data (no random starts, no
operator choices), so the same journals always give the same asset.
"""
import math

import numpy as np
from scipy.optimize import least_squares

import journal

DEG = math.pi / 180
TWO_PI = 2 * math.pi


# ---------------------------------------------------------------- friction (yaw)
def friction_plateaus(j, settle=0.5):
    """Constant-speed reference stretches -> (speed, direction, angle, current) samples.

    Uses servo_cycle rows where the reference velocity is constant (zero reference
    acceleration) after `settle` seconds; the applied current there is friction
    plus the (tiny) inertial term, which is subtracted with the servo's inertia.
    """
    c = j["cycles"]
    t, vr, ar = c["t"], c["vr"], c["ar"]
    steady = (np.abs(ar) < 1e-6) & (np.abs(vr) > 1e-4) & (c["rock"] == 0) & (c["sat"] == 0)
    # time since the reference velocity last changed
    change = np.concatenate([[True], np.abs(np.diff(vr)) > 1e-6])
    since = t - np.maximum.accumulate(np.where(change, t, -np.inf))
    k = steady & (since > settle)
    a = j["manifest"]["servo_parameters"]["inertia"]
    return {"v": vr[k], "q": c["q"][k], "u": c["u"][k] - a * ar[k], "t": t[k]}


def friction_by_speed(samples):
    """Mean |current| per (direction, reference speed): the friction-speed curve."""
    rows = []
    for direction in (1, -1):
        m = np.sign(samples["v"]) == direction
        for speed in np.unique(np.round(np.abs(samples["v"][m]), 5)):
            k = m & (np.abs(np.abs(samples["v"]) - speed) < 1e-5)
            if k.sum() < 300:
                continue
            u = direction * samples["u"][k]
            rows.append({"direction": direction, "speed": float(speed), "current": float(np.mean(u)),
                         "spread": float(np.std(u)), "n": int(k.sum())})
    return rows


def servo_friction_curve(v, coulomb, stribeck, speed, viscous, creep_drop=0.0, creep_speed=0.01):
    """Magnitude of Servo::friction above friction_band (axis_control_core/servo.cpp)."""
    return np.maximum(0.0, coulomb + stribeck * np.exp(-v / speed) - creep_drop * np.exp(-v / creep_speed)) + viscous * v


def fit_friction(rows):
    """Friction-speed curve: per direction a Coulomb level, a Stribeck excess and viscous
    slope; shared by both directions (one bearing) the Stribeck speed and the low-speed
    creep term (the level falls again below ~1 deg/s).

    Bounded least squares on the per-speed means from a fixed start.
    """
    by = {d: [x for x in rows if x["direction"] == d] for d in (1, -1)}
    for d, r in by.items():
        if len(r) < 3:
            raise ValueError(f"friction: too few constant-speed stretches ({'positive' if d > 0 else 'negative'})")
    slow = min(x["speed"] for x in rows) < 0.6 * DEG  # creep is only identifiable with sub-1 deg/s plateaus
    v = {d: np.array([x["speed"] for x in r]) for d, r in by.items()}
    f = {d: np.array([x["current"] for x in r]) for d, r in by.items()}

    def unpack(p):
        per = {1: (p[0], p[1], p[6], p[2]), -1: (p[3], p[4], p[6], p[5])}  # coulomb, stribeck, speed, viscous
        return per, (p[7], p[8]) if slow else (0.0, 0.01)

    def resid(p):
        per, (drop, cspeed) = unpack(p)
        return np.concatenate([servo_friction_curve(v[d], *per[d], drop, cspeed) - f[d] for d in (1, -1)])

    x0 = [0.3, 0.3, 0.0, 0.3, 0.3, 0.0, 0.3] + ([0.3, 0.01] if slow else [])
    lo = [0, 0, 0, 0, 0, 0, 0.08] + ([0, 0.002] if slow else [])
    hi = [2, 2, 1, 2, 2, 1, 1.0] + ([2, 0.05] if slow else [])
    fit = least_squares(resid, x0=x0, bounds=(lo, hi))
    per, (drop, cspeed) = unpack(fit.x)
    out = {"creep_drop": float(drop), "creep_speed": float(cspeed),
           "rms_residual": float(np.sqrt(np.mean(fit.fun ** 2))), "speeds": len(rows)}
    for d, name in ((1, "positive"), (-1, "negative")):
        out[name] = dict(zip(("coulomb", "stribeck", "stribeck_speed", "viscous"), map(float, per[d])))
    return out


def friction_maps(samples, curve, bins=24):
    """Per-angle residual friction (A) per direction above the fitted curve: the servo's friction maps."""
    maps = {}
    for direction, name in ((1, "positive"), (-1, "negative")):
        k = np.sign(samples["v"]) == direction
        p = curve[name]
        speed = np.abs(samples["v"][k])
        residual = direction * samples["u"][k] - servo_friction_curve(speed, p["coulomb"], p["stribeck"], p["stribeck_speed"],
                                                                      p["viscous"], curve["creep_drop"], curve["creep_speed"])
        b = (np.mod(samples["q"][k], TWO_PI) / TWO_PI * bins).astype(int) % bins
        values = np.full(bins, np.nan)
        for i in range(bins):
            if (b == i).sum() >= 100:
                values[i] = np.median(residual[b == i])
        # unobserved bins take the circular interpolation of their neighbours (0 if none observed)
        if np.all(np.isnan(values)):
            values[:] = 0.0
        else:
            x = np.arange(bins); ok = ~np.isnan(values)
            values = np.interp(x, np.concatenate([x[ok] - bins, x[ok], x[ok] + bins]), np.tile(values[ok], 3))
        maps[name] = [round(float(v), 4) for v in values]
        maps[name + "_observed"] = int(np.sum(~np.isnan(values)))
    return maps


# ---------------------------------------------------------------- crosstalk (yaw)
def crosstalk_windows(j, begin_s, duration_s, frequency, window_s=0.5):
    """Encoder response to the probe tone per window: (angle, complex gain rad/A, applied amplitude).

    The ratio of the encoder's and the transmitted current's components at the
    probe frequency is the reading's current sensitivity at that angle (the
    mechanical response at 80 Hz is ~30x smaller and is removed later).
    """
    f, tx = j["feedback"], j["tx"]
    t = f["t"]; q = f["q"]
    k = (t > begin_s + 0.5) & (t < begin_s + duration_s - 0.5)
    t, q = t[k], q[k]
    u = journal.applied(tx, t)
    width = 25  # 25 ms moving average removes the slow motion, keeps 80 Hz
    kern = np.ones(width) / width
    zq = q - np.convolve(q, kern, mode="same")
    zu = u - np.convolve(u, kern, mode="same")
    n = int(round(window_s / 1e-3))
    rows = []
    for i in range(width, len(t) - n - width, n):
        s = slice(i, i + n)
        ph = np.exp(-1j * TWO_PI * frequency * t[s])
        cq = np.mean(zq[s] * ph); cu = np.mean(zu[s] * ph)
        if abs(cu) < 1e-3:
            continue
        rows.append((float(np.mod(np.mean(q[s]), TWO_PI)), complex(cq / cu), float(2 * abs(cu))))
    return rows


def crosstalk_table(rows, frequency, inertia=None, bins=120, reject=0.012):
    """Rows -> (delay_s, table rad/A over absolute angle, rms of the fit residual)."""
    rows = [r for r in rows if abs(r[1]) < reject]  # a slip inside a window is not crosstalk
    if len(rows) < bins // 3:
        raise ValueError(f"crosstalk: only {len(rows)} usable probe windows")
    angle = np.array([r[0] for r in rows]); c = np.array([r[1] for r in rows])
    if inertia:
        # rigid-body response to the probe: -1/(a w^2), in phase with the current
        c = c + 1.0 / (inertia * (TWO_PI * frequency) ** 2)
    # one delay for all angles: the phase that makes the response real
    delays = np.arange(0, 5e-3, 2e-5)
    cost = [np.sum((c * np.exp(1j * TWO_PI * frequency * d)).imag ** 2) for d in delays]
    delay = float(delays[int(np.argmin(cost))])
    g = (c * np.exp(1j * TWO_PI * frequency * delay)).real
    centres = (np.arange(bins) + 0.5) * TWO_PI / bins
    values = np.zeros(bins)
    width = TWO_PI / bins
    for i, centre in enumerate(centres):
        d = np.angle(np.exp(1j * (angle - centre)))
        k = np.abs(d) < 1.5 * width
        values[i] = np.median(g[k]) if k.sum() >= 2 else np.nan
    ok = ~np.isnan(values)
    x = np.arange(bins)
    values = np.interp(x, np.concatenate([x[ok] - bins, x[ok], x[ok] + bins]), np.tile(values[ok], 3))
    # the table is indexed at bin starts (Servo::crosstalk_gain); shift half a bin
    table = np.interp(np.arange(bins) * width, np.concatenate([centres - TWO_PI, centres, centres + TWO_PI]), np.tile(values, 3))
    residual = g - np.interp(angle, np.concatenate([np.arange(bins) * width - TWO_PI, np.arange(bins) * width, np.arange(bins) * width + TWO_PI]), np.tile(table, 3))
    return delay, table, float(np.sqrt(np.mean(residual ** 2))), int(ok.sum())


# ---------------------------------------------------------------- inertia (yaw)
def sweep_value(sweep, t):
    """The session's LogSweep (axis_control_core/session_parts.hpp) at script times t."""
    t = np.asarray(t, float)
    e = t - sweep["begin_s"]
    k = math.log(sweep["f1_hz"] / sweep["f0_hz"]) / sweep["duration_s"]
    amp = sweep.get("amplitude_A", sweep.get("amplitude_rad_s"))
    inside = (e >= 0) & (e < sweep["duration_s"])
    return np.where(inside, amp * np.sin(TWO_PI * sweep["f0_hz"] * (np.exp(k * np.clip(e, 0, None)) - 1) / k), 0.0)


def frf(t, x, u, y, f0, f1, bands=16):
    """Band-averaged response y/u over [f0, f1] Hz using the excitation x as the
    instrument (Syx/Sux): unbiased by the loop's own feedback. Also the coherence of y with x."""
    dt = np.median(np.diff(t))
    w = np.hanning(len(t))
    X = np.fft.rfft((x - np.mean(x)) * w)
    U = np.fft.rfft((u - np.mean(u)) * w)
    Y = np.fft.rfft((y - np.polyval(np.polyfit(t - t[0], y, 2), t - t[0])) * w)
    f = np.fft.rfftfreq(len(t), dt)
    edges = np.geomspace(f0, f1, bands + 1)
    out = []
    for lo, hi in zip(edges[:-1], edges[1:]):
        k = (f >= lo) & (f < hi)
        if k.sum() < 3:
            continue
        syx = np.sum(Y[k] * np.conj(X[k])); sux = np.sum(U[k] * np.conj(X[k]))
        coherence = abs(syx) ** 2 / (np.sum(np.abs(Y[k]) ** 2) * np.sum(np.abs(X[k]) ** 2))
        out.append((math.sqrt(lo * hi), syx / sux, float(coherence)))
    return out


def inertia_response(j, sweep, table, delay, margin=0.3):
    """Current-to-angle response during the sweep, with the measured crosstalk removed."""
    f, tx = j["feedback"], j["tx"]
    a, b = sweep["begin_s"] + margin, sweep["begin_s"] + sweep["duration_s"] - margin
    t = np.arange(max(a, f["t"][0]), min(b, f["t"][-1]), 1e-3)
    y = np.interp(t, f["t"], journal.true_position(j, table, delay))
    return frf(t, sweep_value(sweep, t), journal.applied(tx, t), y, sweep["f0_hz"], sweep["f1_hz"])


def inertia_from_frf(points, residual_bound=0.0005, min_coherence=0.5):
    """Fit H(w) = (-1/(a w^2) + r) * exp(-j w d): inertia a, loop delay d and the
    crosstalk left after removing the measured table, r (bounded by the table's
    uncertainty: at high inertia the rigid-body term is small and could otherwise
    trade against r). Returns (a, d, r, rms)."""
    pts = [p for p in points if p[2] >= min_coherence]
    if len(pts) < 4:
        raise ValueError(f"inertia: only {len(pts)} coherent frequency bands")
    w = np.array([TWO_PI * p[0] for p in pts]); H = np.array([p[1] for p in pts])

    def resid(x):
        a, d, r = x
        e = ((-1.0 / (a * w ** 2) + r) * np.exp(-1j * w * d) - H) / np.abs(H)
        return np.concatenate([e.real, e.imag])

    fit = least_squares(resid, x0=[0.05, 0.002, 0.0], bounds=([1e-3, 0.0, -residual_bound], [2.0, 0.02, residual_bound]))
    return float(fit.x[0]), float(fit.x[1]), float(fit.x[2]), float(np.sqrt(np.mean(fit.fun ** 2)))


# ---------------------------------------------------------------- speed loop (pitch)
def pitch_speed_loop(p, begin_s, duration_s, f0, f1):
    """FRF from SpdRef to measured speed during a speed-reference sweep -> (gain, delay, tau, fit error, points).

    Model: v/v_cmd = g * exp(-s d) / (tau s + 1). With the 3 kg payload the drive's own
    speed loop passes only part of a small speed command (g < 1: friction and gravity
    against its weak integral, station 2026-10-02), so the gain is part of the model.
    Position response to the command (avoids differentiating the quantized position).
    """
    tr = p["trial"]
    k = (tr["t"] >= begin_s + 0.3) & (tr["t"] < begin_s + duration_s - 0.3)
    t, q, cmd = tr["t"][k], tr["q"][k], tr["cmd"][k]
    tu = np.arange(t[0], t[-1], 1e-3)
    qu = np.interp(tu, t, q); cu = np.interp(tu, t, cmd)
    xu = np.interp(tu, t, tr["x"][k])
    pts = [x for x in frf(tu, xu, cu, qu, f0, f1) if x[2] >= 0.6]
    if len(pts) < 4:
        raise ValueError("pitch: too few coherent frequency bands")
    w = np.array([TWO_PI * x[0] for x in pts]); H = np.array([x[1] for x in pts])

    def resid(x):
        g, d, tau = x
        e = (g * np.exp(-1j * w * d) / ((tau * 1j * w + 1) * 1j * w) - H) / np.abs(H)
        return np.concatenate([e.real, e.imag])

    fit = least_squares(resid, x0=[0.5, 0.003, 0.01], bounds=([0.05, 0.0, 0.0], [2.0, 0.05, 0.5]))
    g, d, tau = map(float, fit.x)
    return g, d, tau, float(np.sqrt(np.mean(fit.fun ** 2))), pts
