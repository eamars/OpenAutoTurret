#!/usr/bin/env python3
"""Camera zero from full-resolution snapshots (ADR-006): where a round target sits in each camera's
picture, as the `zero_bore` point in the wide frame, and how far each crosshair is from it.

The operator's route is the page (MENU > CALIBRATION > ZERO: tap the target on each picture). This
is the same measurement made from the pictures instead of a finger: a round target of one colour is
found near each picture's centre, a circle is fitted to its outline, and the centre is averaged over
many snapshot pairs (docs/operations/camera-zero.md).

    python Firmware/tools/camera_zero.py check     [--pairs 6]
    python Firmware/tools/camera_zero.py measure   [--pairs 20] [--distance 11] [--save DIR]
    python Firmware/tools/camera_zero.py measure   --pairs 20 --distance 11 --save DIR --apply
    python Firmware/tools/camera_zero.py send      set_bore_mode on

Runs on the workstation, against http://<station>:8080 only (--address, default 192.168.2.103):
GET /api/video/snapshot and /api/state. `send` posts one command to POST /api/command and prints
controld's answer. --apply sends `zero_bore detail ...` then `zero_bore wide ...` from the
measurement just made, which the station accepts only in a MANUAL hold, with the bore on, on a
record other than #1. It refuses a measurement that is noisy or was taken while the turret moved,
prints the record's zero before it changes it, and checks the result from fresh snapshots. Needs
numpy, scipy and Pillow.
"""
from __future__ import annotations

import argparse
import io
import json
import sys
import time
import urllib.request
from pathlib import Path

import numpy as np

# The target's colour, in HSV: hue in degrees, saturation and value 0..1, per camera. The defaults are
# the 2026-10-25 zero's yellow printed part: bright in the wide picture (the RGB rule that found it
# there, R > 180, G > 160, B < 0.45 G, is saturation >= ~0.55 at value >= ~0.7), dark in the detail one
# (hue 43-52, saturation 0.79-0.98, value 0.15-0.22; its rule B < 0.25 R is saturation >= 0.75). A tan
# or grey background stays under the saturation floor.
HUE_DEG = (35.0, 75.0)
COLOUR = {"wide": {"min_s": 0.55, "min_v": 0.70}, "detail": {"min_s": 0.75, "min_v": 0.12}}
# Where to look, as a radius in pixels around the expected point (default: the picture's centre).
WINDOW_PX = {"wide": 60, "detail": 150}
# What --apply accepts (mrad): one pair's spread on 2026-10-25 was 0.02-0.04 (detail) and 0.1-0.15
# (wide); a measurement noisier than this, or a turret that moved more than its settle threshold
# (the page's 0.3 mrad), is not applied. A check is done when the target sits this close to each
# crosshair.
MAX_SPREAD_MRAD = 0.3
MAX_FIT_RMS_PX = 1.0
MAX_MOTION_MRAD = 0.3
DONE_MRAD = {"detail": 0.1, "wide": 0.2}


def hsv_mask(rgb: np.ndarray, hue=HUE_DEG, min_s=0.55, min_v=0.10) -> np.ndarray:
    """Pixels of the target's colour. rgb: H x W x 3, 0..255."""
    x = rgb.astype(float) / 255.0
    mx, mn = x.max(axis=2), x.min(axis=2)
    delta = mx - mn
    r, g, b = x[..., 0], x[..., 1], x[..., 2]
    with np.errstate(invalid="ignore", divide="ignore"):
        h = np.where(mx == r, (g - b) / delta % 6, np.where(mx == g, (b - r) / delta + 2, (r - g) / delta + 4)) * 60.0
        s = np.where(mx > 0, delta / mx, 0.0)
    return (delta > 0) & (h >= hue[0]) & (h <= hue[1]) & (s >= min_s) & (mx >= min_v)


def fit_circle(x: np.ndarray, y: np.ndarray):
    """Least-squares circle through points: (cx, cy, r, rms of the radial residual)."""
    a = np.column_stack([x, y, np.ones_like(x)])
    d, e, f = np.linalg.lstsq(a, -(x * x + y * y), rcond=None)[0]
    cx, cy = -d / 2, -e / 2
    r = float(np.sqrt(max(cx * cx + cy * cy - f, 0.0)))
    rms = float(np.sqrt(np.mean((np.hypot(x - cx, y - cy) - r) ** 2)))
    return float(cx), float(cy), r, rms


def find_target(rgb: np.ndarray, at=None, window=60, exclude_bottom=0.0, **colour):
    """The target near `at` (x, y pixels; default the centre): the largest blob of its colour in a
    window, holes filled, and a circle fitted to its outline -- above the bottom `exclude_bottom`
    fraction of it when something cuts its lower edge (a chair's seat did, 2026-10-25: the plain
    centroid then sat 0.66 mrad high). Returns a dict in picture pixels, or raises ValueError."""
    from scipy import ndimage
    hgt, wid = rgb.shape[:2]
    gx, gy = at if at is not None else (wid / 2.0, hgt / 2.0)
    x0, y0 = max(0, int(round(gx - window))), max(0, int(round(gy - window)))
    x1, y1 = min(wid, int(round(gx + window))), min(hgt, int(round(gy + window)))
    mask = hsv_mask(rgb[y0:y1, x0:x1], **colour)
    labels, n = ndimage.label(mask)
    if n == 0:
        raise ValueError(f"no target-coloured pixels within {window} px of ({gx:.0f}, {gy:.0f})")
    sizes = ndimage.sum(mask, labels, range(1, n + 1))
    blob = ndimage.binary_fill_holes(labels == 1 + int(np.argmax(sizes)))
    ys, xs = np.nonzero(blob)
    if blob.sum() < 12:
        raise ValueError(f"the target is {int(blob.sum())} px: too small to fit (closer, or bigger)")
    if xs.min() == 0 or ys.min() == 0 or xs.max() == blob.shape[1] - 1 or ys.max() == blob.shape[0] - 1:
        raise ValueError("the target touches the search window's edge: give its position, or a larger window")
    edge = blob & ~ndimage.binary_erosion(blob)
    ey, ex = np.nonzero(edge)
    keep = ey <= ys.min() + (1.0 - exclude_bottom) * (ys.max() - ys.min())
    cx, cy, r, rms = fit_circle(ex[keep] + 0.5, ey[keep] + 0.5)
    return {"x": x0 + cx, "y": y0 + cy, "radius_px": r, "fit_rms_px": rms, "pixels": int(blob.sum()),
            "size_px": (int(xs.max() - xs.min() + 1), int(ys.max() - ys.min() + 1)), "window": (x0, y0, x1, y1),
            "blob": blob}


def save_crop(path, rgb: np.ndarray, t: dict, zoom: int = 4) -> None:
    """The search window, enlarged, with the fitted pixels tinted and the fitted circle and centre
    drawn: what the measurement saw, to look at before trusting it."""
    from PIL import Image, ImageDraw
    x0, y0, x1, y1 = t["window"]
    crop = rgb[y0:y1, x0:x1].copy()
    crop[t["blob"]] = (0.5 * crop[t["blob"]] + 0.5 * np.array([255, 0, 255])).astype(np.uint8)
    im = Image.fromarray(crop).resize(((x1 - x0) * zoom, (y1 - y0) * zoom), Image.NEAREST)
    d = ImageDraw.Draw(im)
    cx, cy, r = (t["x"] - x0) * zoom, (t["y"] - y0) * zoom, t["radius_px"] * zoom
    d.ellipse([cx - r, cy - r, cx + r, cy + r], outline=(0, 255, 255))
    d.line([cx - 6, cy, cx + 6, cy], fill=(0, 255, 255))
    d.line([cx, cy - 6, cx, cy + 6], fill=(0, 255, 255))
    im.save(path)


def wide_frame(camera: str, x: float, y: float, width: int, height: int, scale: float):
    """A picture point as (u, v) of the wide frame, where zero_bore and the crosshairs live: the
    detail picture is a centred window `scale` times smaller (ADR-006 D3)."""
    u, v = x / width, y / height
    return (u, v) if camera == "wide" else (0.5 + (u - 0.5) / scale, 0.5 + (v - 0.5) / scale)


def mrad(du: float, dv: float, intrinsics: dict):
    """A wide-frame difference in mrad (right, down), through the wide camera's focal lengths."""
    return (du * intrinsics["width"] / intrinsics["fx"] * 1e3, dv * intrinsics["height"] / intrinsics["fy"] * 1e3)


def apply_refusal(state: dict) -> str:
    """Why the station would refuse a zero now, or "" (controld says the same, execute_bore_command)."""
    rec = state.get("calibration_record") or {}
    if not rec.get("id"):
        return "no calibration record is in force"
    if rec["id"] == 1:
        return "#1 is the shipped backup and stays as shipped: NEW makes a copy to zero"
    if (state.get("alignment") or {}).get("mode") != "manual_depth":
        return "turn the bore on first (MENU > CALIBRATION > BORE ON, or set_bore_mode on)"
    if state.get("calibration_switching"):
        return "a record switch is holding the turret: wait for it"
    if not (state.get("phase") == "hold" and state.get("operating_mode") == "MANUAL") or state.get("rest_park"):
        return "zero in a MANUAL hold: HOLD HERE TO TRIM, or set_mode MANUAL, with the payload on the target"
    return ""


class Station:
    """The station's web API: snapshots, state, commands."""

    def __init__(self, address: str):
        self.base = f"http://{address}:8080"

    def state(self) -> dict:
        with urllib.request.urlopen(self.base + "/api/state", timeout=5) as r:
            return json.loads(r.read())

    def snapshot(self, camera: str):
        from PIL import Image
        with urllib.request.urlopen(f"{self.base}/api/video/snapshot?camera={camera}", timeout=10) as r:
            meta = json.loads(r.headers.get("X-OTA-Snapshot") or "{}")
            return np.asarray(Image.open(io.BytesIO(r.read())).convert("RGB")), meta

    def command(self, name: str, arg: str, timeout_s=5.0):
        """(accepted, controld's reason), from the ack in the telemetry."""
        seq = self.state().get("cmd_ack_seq")
        req = urllib.request.Request(self.base + "/api/command", data=json.dumps({"command": name, "arg": arg}).encode(),
                                     headers={"Content-Type": "application/json"})
        urllib.request.urlopen(req, timeout=5).read()
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            s = self.state()
            if s.get("cmd_ack_seq") != seq and s.get("cmd_ack_command") == name:
                return s.get("cmd_ack_accepted") == 1, s.get("cmd_ack_reason", "")
            time.sleep(0.1)
        return False, "no answer from the station"


def turret(state: dict):
    return float(state.get("q_yaw_rad") or 0.0), float(state.get("q_pitch_rad") or 0.0)


def measure(station: Station, pairs: int, args) -> dict:
    """Both cameras' target centres in the wide frame, averaged over `pairs` snapshot pairs, and how
    far the turret moved while they were taken."""
    state = station.state()
    q = [turret(state)]
    views = ((state.get("inference") or {}).get("main_camera") or {}).get("views") or {}
    scale = float((views.get("detail") or {}).get("scale") or 0)
    if not scale > 1:
        raise SystemExit("the station publishes no detail view scale: is the detail camera running?")
    rows = {"wide": [], "detail": []}
    first = {}
    for _ in range(pairs):
        for cam in ("detail", "wide"):
            rgb, _meta = station.snapshot(cam)
            try:
                t = find_target(rgb, at=getattr(args, f"{cam}_at"), window=getattr(args, f"{cam}_window"),
                                exclude_bottom=args.exclude_bottom, **COLOUR[cam])
            except ValueError as e:
                raise SystemExit(f"{cam} picture: {e}")
            if cam not in first:
                first[cam] = dict(t, picture=(rgb.shape[1], rgb.shape[0]))
                if getattr(args, "save", None):
                    Path(args.save).mkdir(parents=True, exist_ok=True)
                    save_crop(Path(args.save) / f"target_{cam}.png", rgb, t)
            rows[cam].append(wide_frame(cam, t["x"], t["y"], rgb.shape[1], rgb.shape[0], scale))
        q.append(turret(station.state()))
    intr = state.get("camera_intrinsics") or {}
    q = np.array(q)
    out = {"scale": scale, "intrinsics": intr, "first": first,
           "motion_mrad": float(1e3 * max(np.ptp(q[:, 0]), np.ptp(q[:, 1])))}
    for cam, r in rows.items():
        a = np.array(r)
        out[cam] = {"u": float(a[:, 0].mean()), "v": float(a[:, 1].mean()),
                    "spread_mrad": mrad(float(a[:, 0].std()), float(a[:, 1].std()), intr) if intr.get("fx") else None}
    return out


def crosshair_offsets(m: dict, state: dict) -> dict:
    """How far the target is from each camera's crosshair, in mrad right and down."""
    a = state.get("alignment") or {}
    out = {}
    for cam in ("wide", "detail"):
        c = a.get(cam) or {}
        if c.get("valid") and m.get("intrinsics", {}).get("fx"):
            out[cam] = mrad(m[cam]["u"] - c["x_norm"], m[cam]["v"] - c["y_norm"], m["intrinsics"])
    return out


def report(m: dict, state: dict) -> bool:
    """Prints the measurement; True when both crosshairs are on the target (DONE_MRAD)."""
    for cam in ("detail", "wide"):
        f, r = m["first"][cam], m[cam]
        spread = r["spread_mrad"]
        print(f"{cam:6}: target {f['size_px'][0]}x{f['size_px'][1]} px in the {f['picture'][0]}x{f['picture'][1]} picture "
              f"(circle fit rms {f['fit_rms_px']:.2f} px); wide frame u {r['u']:.6f} v {r['v']:.6f}"
              + (f", spread {spread[0]:.3f} / {spread[1]:.3f} mrad" if spread else ""))
    print(f"turret: moved {m['motion_mrad']:.2f} mrad while measuring")
    offsets = crosshair_offsets(m, state)
    if not offsets:
        print("no crosshairs to compare: the bore is off, or not valid, on this record")
        return False
    done = True
    for cam, (right, down) in offsets.items():
        on = max(abs(right), abs(down)) <= DONE_MRAD[cam]
        done &= on
        print(f"{cam:6}: the target is {right:+.2f} mrad right and {down:+.2f} mrad down of the crosshair "
              f"({'on it' if on else 'NOT on it'}: within {DONE_MRAD[cam]} mrad counts)")
    return done


def measurement_refusal(m: dict) -> str:
    """Why a measurement is not good enough to apply, or ""."""
    for cam in ("detail", "wide"):
        spread = m[cam]["spread_mrad"] or (0.0, 0.0)
        if max(spread) > MAX_SPREAD_MRAD:
            return f"the {cam} centres spread {max(spread):.2f} mrad (more than {MAX_SPREAD_MRAD}): look at the crops"
        if m["first"][cam]["fit_rms_px"] > MAX_FIT_RMS_PX:
            return (f"the {cam} circle fit is off by {m['first'][cam]['fit_rms_px']:.2f} px rms (more than {MAX_FIT_RMS_PX}): "
                    "the outline is cut (--exclude-bottom) or mixed with the background (a smaller window)")
    if m["motion_mrad"] > MAX_MOTION_MRAD:
        return f"the turret moved {m['motion_mrad']:.2f} mrad while measuring: wait until it settles"
    return ""


def bore_now(state: dict) -> str:
    """The record in force and its zero as it stands: what --apply replaces, to put back by hand."""
    rec = state.get("calibration_record") or {}
    b = next((r.get("bore") or {} for r in state.get("calibration_records") or [] if r.get("id") == rec.get("id")), {})
    parts = []
    for cam in ("wide", "detail"):
        c = b.get(cam) or {}
        parts.append(f"{cam} trims R {c.get('trim_right_mrad')} U {c.get('trim_up_mrad')} mrad, offsets "
                     f"{c.get('right_mm')}/{c.get('up_mm')}/{c.get('forward_mm')} mm, zeroed {c.get('zeroed_m') or 'no'}")
    return f"record #{rec.get('id')} {rec.get('name')}, range {b.get('range_m')} m: " + "; ".join(parts)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("action", choices=("measure", "check", "send"))
    ap.add_argument("words", nargs="*", help="send: the command's name and its argument")
    ap.add_argument("--address", default="192.168.2.103")
    ap.add_argument("--pairs", type=int, default=None, help="snapshot pairs to average (measure 20, check 6)")
    ap.add_argument("--distance", type=float, help="the target's distance from the payload, metres (measure)")
    ap.add_argument("--apply", action="store_true", help="send zero_bore for both cameras, then check")
    ap.add_argument("--wide-at", type=float, nargs=2, metavar=("X", "Y"), help="where the target is in the wide picture")
    ap.add_argument("--detail-at", type=float, nargs=2, metavar=("X", "Y"), help="where it is in the detail picture")
    ap.add_argument("--wide-window", type=int, default=WINDOW_PX["wide"])
    ap.add_argument("--detail-window", type=int, default=WINDOW_PX["detail"])
    ap.add_argument("--exclude-bottom", type=float, default=0.0,
                    help="leave this fraction of the outline's height out of the fit, from below (an occluded edge)")
    ap.add_argument("--save", metavar="DIR", help="write target_wide.png and target_detail.png: what was fitted")
    a = ap.parse_args(argv)
    station = Station(a.address)
    if a.action == "send":
        if not a.words:
            ap.error("send needs a command name, e.g. send set_bore_mode on")
        ok, reason = station.command(a.words[0], " ".join(a.words[1:]))
        print(f"{'accepted' if ok else 'REFUSED'}: {reason}")
        return 0 if ok else 1
    state = station.state()
    pairs = a.pairs or (20 if a.action == "measure" else 6)
    m = measure(station, pairs, a)
    done = report(m, state)
    if a.action == "check":
        return 0 if done else 1
    if not a.distance:
        print("give --distance (metres) for the zero_bore commands")
        return 0
    commands = [("zero_bore", f"detail {a.distance:g} {m['detail']['u']:.6f} {m['detail']['v']:.6f}"),
                ("zero_bore", f"wide {a.distance:g} {m['wide']['u']:.6f} {m['wide']['v']:.6f}")]
    for name, arg in commands:
        print(f"{name} {arg}")
    if not a.apply:
        return 0
    now = station.state()
    why = apply_refusal(now) or measurement_refusal(m)
    if why:
        print(f"not applied: {why}")
        return 1
    print("replacing: " + bore_now(now))
    for k, (name, arg) in enumerate(commands):
        ok, reason = station.command(name, arg)
        print(f"{'accepted' if ok else 'REFUSED'}: {reason}")
        if not ok:
            if reason == "no answer from the station":
                print("it may still have been applied: run `check`, and look at the record's bore on the page")
            if k:
                print("HALF APPLIED: the detail camera is zeroed and the wide one is not; fix the cause and run again")
            return 1
    time.sleep(0.5)
    check = measure(station, 6, a)
    print("check, 6 fresh pairs:")
    return 0 if report(check, station.state()) else 1


if __name__ == "__main__":
    sys.exit(main())
