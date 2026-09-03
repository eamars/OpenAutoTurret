#!/usr/bin/env python3
"""Composite the HUD's symbology from controld's real payload onto a real camera frame.

Why an image and not an assertion. The HUD's correctness claim is spatial: a box the controller
believes is at some normalised position must land where the scene shows the thing, and the reticle
must sit on the optical axis. Comparing numbers against numbers checks arithmetic; it cannot show
that the whole chain - sensor, calibration, wire, payload, geometry - agrees with what the camera
saw. So this script takes one JPEG straight off /api/video, takes the track list straight out of
controld's snapshot, draws both with the page's own mapping, and writes a PNG to look at.

The centring number it prints is the operator's acceptance metric: the gap between the anchor and
the principal point, expressed in fractions of the target box height, with the bar at 1/3.

    python3 tools/probe_hud_composite.py            # just draw what controld currently reports
    python3 tools/probe_hud_composite.py --track    # centre a synthetic world target first, then draw

The second form moves the axis. It reuses tools/probe_track_loop.py, including its world-frame
target and its soft-limit guard, rather than inventing a second one.
"""
from __future__ import annotations

import argparse
import io
import json
import os
import socket
import sys
import time
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from probe_track_loop import (BASE, CX, CY, CX as PCX, CY as PCY, FW, FH, BOX_H_NORM,
                              Publisher, axis_direction, base_to_los, command, state,
                              wait_ack, world_to_norm)

try:
    from PIL import Image, ImageDraw
except ImportError:
    print("PIL is required for the composite (it is the point of the probe)")
    raise SystemExit(2)

OUT = "/tmp/hud_composite.png"


def start_video() -> bool:
    """Ask webd for the preview, exactly as the HUD page does. /api/video answers 409 until the
    stream is started, so a probe that skips this cannot tell a dead camera from an idle one."""
    req = urllib.request.Request(BASE + "/api/video/start", data=b"{}",
                                 headers={"Content-Type": "application/json"}, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=8) as r:
            j = json.loads(r.read())
    except urllib.error.HTTPError as e:
        print("/api/video/start refused: HTTP %d %s" % (e.code, e.read()[:160].decode("utf-8", "replace")))
        return False
    print("preview: running=%s %sx%s fps=%s camera=%s orientation=%s"
          % (j.get("running"), j.get("width"), j.get("height"), j.get("fps"),
             j.get("camera"), j.get("orientation")))
    return bool(j.get("running"))


def grab_frame(timeout: float = 6.0):
    """One decoded JPEG from the MJPEG stream. The stream is continuous, so read until a
    complete part: a partial JPEG would composite half a picture and look like a geometry bug."""
    req = urllib.request.Request(BASE + "/api/video")
    with urllib.request.urlopen(req, timeout=timeout) as r:
        buf = b""
        deadline = time.time() + timeout
        while time.time() < deadline:
            buf += r.read(65536)
            start = buf.find(b"\xff\xd8")
            end = buf.find(b"\xff\xd9", start + 2) if start >= 0 else -1
            if start >= 0 and end > start:
                return Image.open(io.BytesIO(buf[start:end + 2])).convert("RGB")
    raise SystemExit("no complete JPEG arrived from /api/video")


def reticle(dd: ImageDraw.ImageDraw, cx: float, cy: float,
            colour=(149, 245, 139)) -> None:
    """The §7 optical-axis symbology, drawn as one unit so it can also be rendered on its own for
    the openness check. Four separated corner brackets, a vertical above and below, short bars
    left and right, and nothing in the middle."""
    r, gap = 30, 9
    for sx, sy in ((-1, -1), (1, -1), (-1, 1), (1, 1)):
        dd.line([cx + sx * r, cy + sy * gap, cx + sx * r, cy + sy * r], fill=colour, width=2)
        dd.line([cx + sx * gap, cy + sy * r, cx + sx * r, cy + sy * r], fill=colour, width=2)
    dd.line([cx, cy - r - 14, cx, cy - gap], fill=colour, width=1)
    dd.line([cx, cy + gap, cx, cy + r + 14], fill=colour, width=1)
    dd.line([cx - r - 14, cy, cx - gap - 6, cy], fill=colour, width=1)
    dd.line([cx + gap + 6, cy, cx + r + 14, cy], fill=colour, width=1)


def draw(img: Image.Image, tracks, intr, age_ms: float = -1.0) -> str:
    # Keep the untouched scene: the openness check below must not be confounded by
    # annotations drawn on top of the axis.
    scene = img.copy()
    d = ImageDraw.Draw(img)
    w, h = img.size
    cx = (intr.get("cx", w / 2.0) if intr else w / 2.0)
    cy = (intr.get("cy", h / 2.0) if intr else h / 2.0)
    GREEN = (149, 245, 139)
    DIM = (149, 245, 139)
    AMBER = (242, 179, 41)

    report = []
    for tr in tracks:
        ax, ay = tr.get("anchor_x"), tr.get("anchor_y")
        if not isinstance(ax, (int, float)) or not isinstance(ay, (int, float)):
            continue
        bb = tr.get("bbox")
        selected = bool(tr.get("selected"))
        col = GREEN if selected else DIM
        if isinstance(bb, list) and len(bb) == 4:
            x0, y0, x1, y1 = bb[0] * w, bb[1] * h, bb[2] * w, bb[3] * h
            if selected:
                d.rectangle([x0 - 2, y0 - 2, x1 + 2, y1 + 2], outline=(50, 105, 45), width=3)
            d.rectangle([x0, y0, x1, y1], outline=col, width=2 if selected else 1)
            label = "%s %s" % (str(tr.get("label") or ("#" + str(tr.get("track_id")))).upper(),
                               ("%d%%" % round(tr["confidence"] * 100))
                               if isinstance(tr.get("confidence"), (int, float)) else "")
            d.text((x0, max(0, y0 - 14)), label, fill=col)
            box_h_frac = (bb[3] - bb[1])
        else:
            box_h_frac = BOX_H_NORM
        px, py = ax * w, ay * h
        d.line([px - 8, py, px + 8, py], fill=col, width=1)
        d.line([px, py - 8, px, py + 8], fill=GREEN, width=1)
        gap = ((ax * w - cx) ** 2 + (ay * h - cy) ** 2) ** 0.5
        in_box = gap / (box_h_frac * h) if box_h_frac > 0 else float("nan")
        report.append((selected, gap, in_box, box_h_frac * h))

    # §7 reticle: four corner brackets, verticals above and below, short bars left and right,
    # open centre. Drawn at the DAEMON's principal point, never at the anchor of a target.
    reticle(d, cx, cy)

    # The openness check has to be made on a reticle with nothing else on it. When a target is
    # centred its own anchor cross sits exactly on the axis, and a naive centre-pixel read then
    # reports "filled" for a symbology that is open - a check that cannot tell the drawing from a
    # coincidence is not a check. So redraw the reticle alone on the untouched scene and look
    # there.
    chk = scene.copy()
    reticle(ImageDraw.Draw(chk), cx, cy)
    centre = chk.getpixel((int(round(cx)), int(round(cy))))
    open_centre = not (centre[1] > 180 and centre[0] < 200 and centre[2] < 200)
    open_note = "reticle-alone centre pixel %s -> open centre (no filled dot): %s" % (
        centre, "YES" if open_centre else "NO")

    if not report:
        return ("no tracks in controld's payload; reticle drawn at (%.1f, %.1f) from the loaded "
                "intrinsics; %s" % (cx, cy, open_note))
    sel = [x for x in report if x[0]] or report
    lines = []
    for _, gap, in_box, bh in sel:
        lines.append("anchor-to-reticle gap %.1f px = %.3f of box height (%.0f px); acceptance bar 0.333 -> %s"
                     % (gap, in_box, bh, "INSIDE" if in_box <= 1 / 3.0 else "OUTSIDE"))
    lines.append(open_note)
    if isinstance(age_ms, (int, float)) and age_ms > 500.0:
        lines.append("PAYLOAD IS STALE (track list age %.0f ms): the anchor above is the last one "
                     "received, NOT a live acceptance measurement. Run with --track to publish and "
                     "measure while the target is being observed." % age_ms)
    return "\n".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--track", action="store_true", help="centre a synthetic world target first (moves the axis)")
    ap.add_argument("--offset-deg", type=float, default=12.0)
    args = ap.parse_args()

    s = state()
    intr = {}
    h = json.load(urllib.request.urlopen(BASE + "/api/health", timeout=4))
    if not start_video():
        print("no preview to composite onto; the HUD would be showing the same thing")
        return 1
    if args.track:
        if not h.get("controld_connected") or not s.get("at_ready"):
            print("not connected/ready; refusing to move")
            return 1
        pub = Publisher(False)
        az0, el0 = base_to_los(axis_direction(s["q_yaw_rad"], s["q_pitch_rad"]))
        az_t, el_t = az0 + __import__("math").radians(args.offset_deg), el0
        end = time.monotonic() + 1.5
        while time.monotonic() < end:
            pub.publish(az_t, el_t)
            time.sleep(0.033)
        seq0 = s.get("cmd_ack_seq", 0)
        command("select_target", "1")
        wait_ack(seq0)
        s = state()
        if not s.get("selected_uuid_valid"):
            print("selection refused:", s.get("cmd_ack_reason"))
            pub.close()
            return 1
        seq0 = s.get("cmd_ack_seq", 0)
        command("set_mode", "AUTO_TRACK")
        wait_ack(seq0)
        # Sample WHILE the target is still being observed. Reading the payload once after the
        # publisher stops measures a stale anchor and a pose that has kept moving, which is how
        # this probe first reported 201.6 px against S1's 0.3 px for the same geometry: not a
        # controller disagreement, a probe that sampled at the wrong moment. Same discipline as
        # probe_track_loop: keep the stream alive, take the distribution over the final window.
        print("tracking; sampling the anchor while the target is still being published...")
        samples = []
        warm_end = time.monotonic() + 5.0
        end = time.monotonic() + 12.0
        while time.monotonic() < end:
            pub.publish(az_t, el_t)
            now = time.monotonic()
            if now > warm_end:
                ss = state()
                for tr in (ss.get("tracks") or []):
                    if not tr.get("selected"):
                        continue
                    ax, ay, bb = tr.get("anchor_x"), tr.get("anchor_y"), tr.get("bbox")
                    if not all(isinstance(x, (int, float)) for x in (ax, ay)) or not isinstance(bb, list):
                        continue
                    gap = ((ax * FW - PCX) ** 2 + (ay * FH - PCY) ** 2) ** 0.5
                    bh = (bb[3] - bb[1]) * FH
                    if bh > 0:
                        samples.append((gap, gap / bh, ss.get("track_state")))
            time.sleep(0.033)
        pub.close()
        if samples:
            fr = sorted(x[1] for x in samples)
            gaps = sorted(x[0] for x in samples)
            print("anchor-to-reticle over the final %.0fs (n=%d): p50 %.1f px / p95 %.1f px; "
                  "in box heights p50 %.3f / p95 %.3f (bar 0.333); states %s"
                  % (end - warm_end, len(samples), gaps[len(gaps) // 2],
                     gaps[min(len(gaps) - 1, int(0.95 * len(gaps)))],
                     fr[len(fr) // 2], fr[min(len(fr) - 1, int(0.95 * len(fr)))],
                     sorted({str(x[2]) for x in samples})))

    s = state()
    intr = s.get("camera_intrinsics") or {}
    img = grab_frame()
    note = draw(img, s.get("tracks") or [], intr, s.get("track_list_age_ms", -1))
    img.save(OUT)
    print("frame %dx%d, saved %s" % (img.size[0], img.size[1], OUT))
    print("intrinsics used for the reticle: %s" % (intr or "NONE - would have assumed centre"))
    print(note)
    if args.track:
        command("clear_target")
        time.sleep(0.2)
        command("set_mode", "MANUAL")
        print("returned to MANUAL")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
