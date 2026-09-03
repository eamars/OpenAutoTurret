#!/usr/bin/env python3
"""Encoder-as-theodolite camera-geometry probe (v3.2 centring/frame-exit prerequisite).

The axis encoder supplies an ANGLE WE KNOW. If a scene feature moves du pixels while the
yaw encoder moves dq radians, then du/dq is the plate scale, and from it the effective
horizontal field of view — the number that makes "will the target fall out of frame" a
computation instead of a hope.

Method: step the axis in small increments, let it settle, grab a frame at each dwell, and
measure the image displacement between consecutive frames by phase correlation. Forward and
back are walked so hysteresis/backlash shows up rather than hiding. Run with the webd preview
holding the camera and controld stepping the axis, so nothing has to hand the sensor over.

    python3 tools/probe_theodolite.py [--step-deg 2] [--samples 8] [--pitch]

Self-test first, always: the correlation sign convention is verified against a known
synthetic shift on the live frame before any measurement is reported. A calibration with the
wrong sign looks exactly like a calibration with the right sign until the turret drives the
wrong way.

Honest limits of this method, stated up front:
  * Principal point and camera-to-axis boresight are NOT separable at the small angular
    spans available here — both enter as a constant pixel offset, and the tan() curvature
    that would separate them is a few percent over +-16 deg. What IS identifiable: the plate
    scale (hence HFOV), the image roll of the yaw axis, and the repeatability of both.
  * JPEG compression and the 640x480 downscale of a 12MP sensor bound the precision; the
    spread across steps is reported so the reader can see that bound instead of trusting a
    single fitted number.
"""
from __future__ import annotations

import argparse
import io
import json
import threading
import time
import urllib.request

import numpy as np
from PIL import Image

BASE = "http://127.0.0.1:8080"


def state() -> dict:
    return json.load(urllib.request.urlopen(BASE + "/api/state", timeout=4))


def command(name: str, arg: str = "") -> dict:
    req = urllib.request.Request(
        BASE + "/api/command",
        data=json.dumps({"command": name, "arg": arg}).encode(),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    return json.loads(urllib.request.urlopen(req, timeout=4).read())


class Stream:
    """Background MJPEG reader keeping the newest decoded frame available on demand."""

    def __init__(self, url: str = BASE + "/api/video") -> None:
        self.url = url
        self.latest = None  # (t_monotonic, float64 gray array)
        self.frames = 0
        self.error = ""
        self._stop = threading.Event()
        self._t = threading.Thread(target=self._run, daemon=True)
        self._t.start()

    def _run(self) -> None:
        buf = b""
        while not self._stop.is_set():
            try:
                r = urllib.request.urlopen(self.url, timeout=8)
                while not self._stop.is_set():
                    buf += r.read(65536)
                    if len(buf) > 1 << 22:  # a stalled stream must not eat the box
                        buf = buf[-200000:]
                    while True:
                        i = buf.find(b"\xff\xd8")
                        j = buf.find(b"\xff\xd9", i) if i >= 0 else -1
                        if i < 0 or j < 0:
                            break
                        part = buf[i : j + 2]
                        buf = buf[j + 2 :]
                        try:
                            g = np.asarray(Image.open(io.BytesIO(part)).convert("L"), "float64")
                        except Exception:
                            continue
                        self.latest = (time.monotonic(), g)
                        self.frames += 1
            except Exception as exc:  # stream restarts; the probe survives a dropped part
                self.error = str(exc)
                time.sleep(0.2)

    def grab(self, after: float = 0.0) -> np.ndarray:
        """A frame DECODED AFTER the given clock value — i.e. one that could not have been
        exposed while the axis was still moving.

        The first draft asked for the newest frame older than 250 ms instead, which a healthy
        15 Hz stream never yields: the newest sample is only ever ~60 ms old, so the probe
        timed out on a perfectly good camera and blamed the stream for being silent. Freshness
        has to be expressed against the moment the motion stopped, not as an absolute age.
        """
        deadline = time.monotonic() + 6.0
        while time.monotonic() < deadline:
            got = self.latest
            if got and got[0] > after:
                return got[1]
            time.sleep(0.02)
        raise RuntimeError("no video frame after settle within 6 s: " + (self.error or "stream silent"))

    def stop(self) -> None:
        self._stop.set()


def measure_shift(a: np.ndarray, b: np.ndarray, max_dx: int = 160, max_dy: int = 40):
    """Displacement (dx, dy) in pixels such that b shows a's scene shifted by +(dx,dy).

    Phase correlation with a search window: without the window, a repeating scene (a shelf,
    a door frame) will happily report a peak 300 px away, and it will look confident.
    """
    h, w = a.shape
    win = np.outer(np.hanning(h), np.hanning(w))
    fa = np.fft.rfft2((a - a.mean()) * win)
    fb = np.fft.rfft2((b - b.mean()) * win)
    # conj on the FIRST frame, not the second: fa*conj(fb) reports the shift that maps b back
    # onto a, which is the negative of what this function promises. The self-test below caught
    # it on the first run — magnitude correct, sign inverted, and every downstream number
    # would have looked immaculate.
    cross = np.conj(fa) * fb
    cross /= np.abs(cross) + 1e-9
    corr = np.fft.irfft2(cross, s=a.shape)
    corr = np.fft.fftshift(corr)
    cy, cx = h // 2, w // 2
    window = np.full_like(corr, -1e18)
    window[cy - max_dy : cy + max_dy + 1, cx - max_dx : cx + max_dx + 1] = corr[
        cy - max_dy : cy + max_dy + 1, cx - max_dx : cx + max_dx + 1
    ]
    py, px = np.unravel_index(int(np.argmax(window)), window.shape)

    # Parabolic subpixel refinement on each axis, guarded against the window edge.
    # (strip mode: callers crop to one row/column before calling, see main())
    def parabola(vals) -> float:
        l, c, r = vals
        denom = (l - 2.0 * c + r)
        return 0.0 if abs(denom) < 1e-12 else 0.5 * (l - r) / denom

    dy_sub, dx_sub = 0.0, 0.0
    if 0 < py < window.shape[0] - 1:
        dy_sub = parabola((window[py - 1, px], window[py, px], window[py + 1, px]))
    if 0 < px < window.shape[1] - 1:
        dx_sub = parabola((window[py, px - 1], window[py, px], window[py, px + 1]))
    return (px + dx_sub - cx), (py + dy_sub - cy), float(window[py, px])


def settle_and_grab(s: Stream, axis: str, sign: float, deg: float, samples: list) -> None:
    before = state()
    key = "q_yaw_rad" if axis == "yaw" else "q_pitch_rad"
    q0 = before[key]
    seq0 = before.get("cmd_ack_seq", 0)
    r = command("manual_step", "%s%+g" % (axis, sign * deg))

    # The HTTP response is NOT the verdict. `manual_step yaw+2` came back {"ok": true} while
    # the control thread was refusing it with 'step size must be one of 0.5, 1, or 5 degrees':
    # the web thread answers for shape, controld answers for substance, and the two are joined
    # only by cmd_ack_seq (the 52 gap, which has now cost a real run rather than a theory).
    verdict = None
    deadline = time.monotonic() + 4.0
    while time.monotonic() < deadline:
        st = state()
        if st.get("cmd_ack_seq", 0) != seq0:
            verdict = st
            break
        time.sleep(0.05)
    if verdict is None:
        raise RuntimeError("no fresh cmd_ack after manual_step (HTTP said %s)" % json.dumps(r)[:80])
    if not verdict.get("cmd_ack_accepted", 0):
        raise RuntimeError("controld refused the step: %r" % verdict.get("cmd_ack_reason", ""))

    # Settle on POSITION alone. The drive-reported velocity carries +-0.16 rad/s of noise at
    # rest (measured: 0.32 rad/s peak-to-peak on yaw while holding position to 0.02 deg), so a
    # test that waits for |v| under 0.02 waits forever on a still, healthy axis.
    seen: list = []
    settled_at = 0.0
    deadline = time.monotonic() + 25.0
    while time.monotonic() < deadline:
        st = state()
        seen.append(st[key])
        if len(seen) > 8:
            seen.pop(0)
        if len(seen) == 8 and (max(seen) - min(seen)) < 6e-4:
            settled_at = time.monotonic()  # frames exposed after this are motion-free
            break
        time.sleep(0.1)
    else:
        raise RuntimeError("axis never settled after %s%+g (last q=%.5f)" % (axis, sign * deg, seen[-1]))
    frame = s.grab(after=settled_at)
    q1 = state()[key]
    moved = abs(np.degrees(q1 - q0))
    if moved < 0.55 * deg:
        raise RuntimeError(
            "step acknowledged but the axis only moved %.3f deg of %.3f commanded" % (moved, deg))
    samples.append({"q_rad": q1, "frame": frame})


def main() -> int:
    ap = argparse.ArgumentParser()
    # 0.5, 1 and 5 degrees are the sizes controld sanctions for a step; anything else is
    # refused on the control thread (and, until 52 is fixed, reported as ok by the web one).
    ap.add_argument("--step-deg", type=float, default=5.0, choices=[0.5, 1.0, 5.0])
    ap.add_argument("--samples", type=int, default=7)
    ap.add_argument("--axis", default="yaw", choices=["yaw", "pitch"])
    ap.add_argument("--neg-first", action="store_true",
                    help="walk the negative direction first (needed near the upper pitch limit)")
    args = ap.parse_args()

    s = Stream()
    while s.frames < 3 and not s.error:
        time.sleep(0.3)
    if s.frames < 3:
        print("no video stream: " + (s.error or "silent"))
        return 1
    f0 = s.grab()
    print("live frame %dx%d, %d frames read" % (f0.shape[1], f0.shape[0], s.frames))

    # ---- self-test: does the estimator agree with a shift we caused on purpose? ----
    dx_true, dy_true = 23.0, 7.0
    moved = np.roll(f0, (int(dy_true), int(dx_true)), axis=(0, 1))
    dx_hat, dy_hat, peak = measure_shift(f0, moved)
    ok = abs(dx_hat - dx_true) < 0.6 and abs(dy_hat - dy_true) < 0.6
    print("SELF-TEST  commanded shift (+%.0f,+%.0f) px -> measured (%+.2f,%+.2f) peak %.3f : %s"
          % (dx_true, dy_true, dx_hat, dy_hat, peak, "PASS" if ok else "FAIL"))
    if not ok:
        print("Refusing to calibrate with an estimator whose sign convention is unproven.")
        s.stop()
        return 2

    print("\nwalking %s: %+g deg x %d out, then back" % (args.axis, args.step_deg, args.samples))
    fwd: list = []
    samples: list = []
    st0 = state()
    samples.append({"q_rad": st0["q_yaw_rad"] if args.axis == "yaw" else st0["q_pitch_rad"],
                    "frame": s.grab()})
    first, second = (-1.0, +1.0) if args.neg_first else (+1.0, -1.0)
    for _ in range(args.samples):
        settle_and_grab(s, args.axis, first, args.step_deg, samples)
    fwd = list(samples)
    for _ in range(args.samples):
        settle_and_grab(s, args.axis, second, args.step_deg, samples)

    print("\n  step   dq(deg)    du(px)    dv(px)   deg per normalized-px")
    per_step = []
    for i in range(1, len(samples)):
        axis = args.axis
        dq = np.degrees(samples[i]["q_rad"] - samples[i - 1]["q_rad"])
        A, B = samples[i - 1]["frame"], samples[i]["frame"]
        if axis == "pitch":
            cols = np.abs(np.diff(B, axis=0)).sum(axis=0)  # vertical change per column
            c = int(np.argsort(cols)[-int(B.shape[1] * 0.06):].mean())
            w = max(48, int(B.shape[1] * 0.06))
            A, B = A[:, max(0, c - w // 2) : c + w // 2], B[:, max(0, c - w // 2) : c + w // 2]
            dx, dy, _ = measure_shift(A, B, max_dx=12, max_dy=140)
        else:
            rows = np.abs(np.diff(B, axis=1)).sum(axis=1)  # horizontal change per row
            r = int(np.argsort(rows)[-int(B.shape[0] * 0.06):].mean())
            h = max(40, int(B.shape[0] * 0.06))
            A, B = A[max(0, r - h // 2) : r + h // 2, :], B[max(0, r - h // 2) : r + h // 2, :]
            dx, dy, _ = measure_shift(A, B, max_dx=260, max_dy=12)
        w = samples[i]["frame"].shape[1]
        deg_per_normpx = abs(dq) / (abs(dx) / w) if abs(dx) > 0.5 else float("nan")
        per_step.append((dq, dx, dy, deg_per_normpx))
        print("  %4d  %+7.3f  %+8.2f  %+7.2f   %8.3f" % (i, dq, dx, dy, deg_per_normpx))

    outbound = per_step[: args.samples]
    oq = sum(abs(p[0]) for p in outbound)
    ou = sum(p[1] for p in outbound)
    ov = sum(p[2] for p in outbound)
    span_norm = (ou if args.axis == "yaw" else ov) / samples[0]["frame"].shape[1 if args.axis == "yaw" else 0]
    if abs(span_norm) > 1e-6:
        print("\n  OUTBOUND LEG ONLY (%.1f deg of commanded travel): %.3f deg per normalized %s"
              % (oq, oq / abs(span_norm), "width" if args.axis == "yaw" else "height"))
        print("  (the return leg is excluded from the scale: at reversal the encoder moves while"
              "\n   the mechanism takes up slack, so it measures backlash, not geometry)")
    finite = [p for p in per_step if np.isfinite(p[3]) and abs(p[0]) > 0.2]
    if len(finite) >= 3:
        vals = np.array([p[3] for p in finite])
        span_q = np.degrees(fwd[-1]["q_rad"] - fwd[0]["q_rad"])
        du_total = sum(p[1] for p in per_step[: args.samples])
        hfov = abs(span_q) / (abs(du_total) / samples[0]["frame"].shape[1])
        # |dx|, not dx: the return leg travels with the opposite sign, and atan2(dy, dx) on
        # that reads as ~180 deg instead of ~1 deg. The first version of this printed a
        # "camera roll of -2.67 deg" that was pure arithmetic; the pitch walk (which travels
        # along its own axis) says the true misalignment is ~0.1 deg.
        rolls = [np.degrees(np.arctan2(p[2], abs(p[1]))) for p in finite if abs(p[1]) > 3]
        print("\nIDENTIFIABLE RESULT")
        print("  plate scale over |dq| span: %.3f deg per full-width (median of %d steps, spread %.3f, min %.3f max %.3f)"
              % (np.median(vals), len(vals), vals.max() - vals.min(), vals.min(), vals.max()))
        print("  implied effective HFOV (linear estimate over %.1f deg of travel): %.2f deg"
              % (abs(span_q), hfov))
        if rolls:
            print("  image drift across the row (camera roll about the optical axis): %.2f deg (median of %d)"
                  % (np.median(rolls), len(rolls)))
        back = per_step[args.samples:]
        comp = 1 if args.axis == "yaw" else 2
        f_sum = sum(p[comp] for p in per_step[: args.samples])
        b_sum = sum(p[comp] for p in back)
        print("  out-and-back closure on the driven axis: out %+.1f px, back %+.1f px, residual %+.1f px (%.1f%% of the walk)"
              % (f_sum, b_sum, f_sum + b_sum, 100.0 * abs(f_sum + b_sum) / max(1.0, abs(f_sum))))
    print("\nNOT identifiable from this walk: the principal point separately from the camera\n"
          "boresight (both are constant pixel offsets; the tan curvature that separates them is\n"
          "a few percent over this span). Say so in the record rather than fitting three\n"
          "unknowns to two.")
    s.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
