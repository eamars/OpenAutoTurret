"""GM6020 encoder current-crosstalk calibration from a probe-tone scan journal.

    python crosstalk.py JOURNAL [JOURNAL...] --begin 66 --duration 125 --asset ../../config/servo/yaw_servo.json

The scan session (manifests.py yaw --script crosstalk_scan with an 80 Hz, 0.15 A
servo_excitation) rotates slowly while a constant probe tone is added to the
current. Mechanical motion at 80 Hz is ~30x smaller than the crosstalk, so the
encoder component at the probe frequency is the crosstalk: q_meas = q + g*i(t-d).
Each 1 s window gives g at one angle; the windows become a 120-bin table. Windows
disturbed by a slip are dropped. The stored table is biased by -2 mrad/A: the
stable side is over-compensation (residual of the same sign as g).
"""
import argparse, json, math
import numpy as np

PROBE_HZ, AMPLITUDE, BIAS = 80.0, 0.15, 0.002


def demodulate(path, begin_s, duration_s):
    begin = None; fb = []
    for line in open(path, encoding="utf-8"):
        k = line[7:35]
        if '"yaw_control_begin"' in k: begin = json.loads(line)["time_ns"]
        elif '"yaw_feedback"' in k:
            r = json.loads(line); fb.append((r["kernel_monotonic_ns"], r["encoder_unwrapped_counts"], r["encoder_raw"]))
    F = np.array(fb, float)
    t = (F[:, 0] - begin) * 1e-9; q = F[:, 1] * 2 * math.pi / 8192; deg = (F[0, 2] + F[:, 1]) * 360 / 8192 % 360
    k = (t > begin_s + 0.5) & (t < begin_s + duration_s - 0.5)
    t, q, deg = t[k], q[k], deg[k]
    z = q - np.convolve(q, np.ones(25) / 25, mode="same")  # remove the slow scan motion
    ph = 2 * math.pi * PROBE_HZ * (t - begin_s)
    rows = []
    for i in range(0, len(t) - 1000, 1000):
        s = slice(i, i + 1000)
        c = 2 * np.mean(z[s] * (np.sin(ph[s]) + 1j * np.cos(ph[s]))) / AMPLITUDE
        rows.append((deg[i], c))
    return rows


def table(rows):
    rows = [(d, c) for d, c in rows if abs(c) < 0.010]  # a slip inside a window is not crosstalk
    th = np.array([d for d, _ in rows]); c = np.array([c for _, c in rows])
    delay = min(np.arange(0, 4e-3, 2e-5), key=lambda d: np.sum((c * np.exp(1j * 2 * math.pi * PROBE_HZ * d)).imag ** 2))
    g = (c * np.exp(1j * 2 * math.pi * PROBE_HZ * delay)).real
    o = np.argsort(th)
    centers = np.arange(120) * 3.0
    values = np.interp(centers, np.concatenate([th[o] - 360, th[o], th[o] + 360]), np.tile(g[o], 3))
    return delay, values


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("journals", nargs="+"); p.add_argument("--begin", type=float, default=66.0)
    p.add_argument("--duration", type=float, default=125.0); p.add_argument("--asset")
    a = p.parse_args()
    rows = [r for j in a.journals for r in demodulate(j, a.begin, a.duration)]
    delay, values = table(rows)
    print(f"{len(rows)} windows, delay {delay*1e3:.2f} ms, |g| max {np.max(np.abs(values))*1e3:.2f} mrad/A")
    if a.asset:
        asset = json.load(open(a.asset, encoding="utf-8"))
        asset["servo_parameters"]["crosstalk_delay_s"] = round(float(delay), 5)
        asset["servo_parameters"]["crosstalk_map"] = [round(float(v) - BIAS, 7) for v in values]
        json.dump(asset, open(a.asset, "w", encoding="utf-8", newline="\n"), indent=1)
        print("updated", a.asset)
