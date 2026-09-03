#!/usr/bin/env python3
"""Synthetic target publisher — a FAKE visiond, for exercising the control path.

What this is for: the acquire/lose half of the tracking lifecycle needs a target
on demand — acquire, coast, confidence decay (§35), brake, reacquire after a
dropout — on a schedule, reproducibly, without a person standing in front of the
lens and without taking /dev/video1 away from webd while the operator is watching
the feed.

What this is NOT for, and the correction matters: it was written so the roaming
path could be reached, on the assumption that SEARCH requires a target first.
That assumption was wrong, and it was the bug — a station boots with no target
and there is no guarantee one ever appears, so cold-start roaming had to become
reachable with nothing in view (§36, tracking_state_machine.hpp). **Do not use
this tool to test roaming**: feeding a target in first would pass a test of a
feature that does not work for the only case that matters. Roaming is verified by
starting tracking with this tool NOT running.

What is real and what is not: this publishes the SAME 58-byte TargetMeasurement
visiond publishes (§6.1), so everything downstream of the socket — vision ingest,
LOS solve, reference manager, search planner, trajectory generator, safety
envelope, supervisor — is the production code path against a live controld.
The part that is NOT real is the scene. There is no person. Nothing about
detection quality, detector latency, or classifier confidence may be concluded
from anything this tool sends; it exists to test the CONTROL path on demand.

Modes:
  sweep   a target crossing the frame horizontally — the tracker must move yaw
  hold    a stationary target
  none    valid=False frames (a detector that sees nothing)

  --dry-run  print the frames instead of sending them (no controld required)

Examples:
  python3 -m tools.fake_vision sweep --seconds 12
  python3 -m tools.fake_vision hold --anchor-u 640 --anchor-v 360 --seconds 4
  python3 -m tools.fake_vision none --seconds 3
"""
from __future__ import annotations

import argparse
import math
import os
import socket
import sys
import time

# Imported lazily-ish: the tool must still print --help if the repo layout moved.
from vision.protocol import TargetMeasurement  # noqa: E402


def sweep_measurement(seq: int, t: float, *, width: int, height: int,
                      amplitude: float, period_s: float,
                      class_id: int, confidence: float,
                      track_id: int) -> TargetMeasurement:
    """A target oscillating horizontally about the frame centre.

    `amplitude` is a fraction of the frame WIDTH (not pixels): a target that
    sweeps 0.5 of the frame looks the same on any sensor, which keeps a run
    describable after the resolution changes. The bbox is 0.08 x 0.16 of the
    frame centred on the anchor, as normalised fractions (§6.2), and the anchor
    is the bbox centre in pixels — the same convention visiond uses (§10.1), so
    a tracker that disagrees with one of them fails here too.
    """
    phase = 2.0 * math.pi * (t / period_s)
    u_norm = 0.5 + amplitude * math.sin(phase)
    v_norm = 0.5
    half_w, half_h = 0.04, 0.08
    return TargetMeasurement(
        frame_sequence=seq,
        sensor_timestamp_ns=time.monotonic_ns(),
        valid=True,
        class_id=class_id,
        confidence=confidence,
        bbox_x_min_norm=max(0.0, u_norm - half_w),
        bbox_y_min_norm=max(0.0, v_norm - half_h),
        bbox_x_max_norm=min(1.0, u_norm + half_w),
        bbox_y_max_norm=min(1.0, v_norm + half_h),
        anchor_u_px=u_norm * width,
        anchor_v_px=v_norm * height,
        visual_track_id=track_id,
    )


def hold_measurement(seq: int, *, anchor_u: float, anchor_v: float,
                     class_id: int, confidence: float,
                     track_id: int) -> TargetMeasurement:
    half_w, half_h = 50.0, 100.0      # pixels, mirroring sweep's bbox fractions
    return TargetMeasurement(
        frame_sequence=seq, sensor_timestamp_ns=time.monotonic_ns(), valid=True,
        class_id=class_id, confidence=confidence,
        bbox_x_min_norm=max(0.0, (anchor_u - half_w) / 1280.0),
        bbox_y_min_norm=max(0.0, (anchor_v - half_h) / 720.0),
        bbox_x_max_norm=min(1.0, (anchor_u + half_w) / 1280.0),
        bbox_y_max_norm=min(1.0, (anchor_v + half_h) / 720.0),
        anchor_u_px=anchor_u, anchor_v_px=anchor_v,
        visual_track_id=track_id,
    )


def invalid_measurement(seq: int) -> TargetMeasurement:
    """A detector that sees nothing: same 58 bytes, valid=False.

    The daemon must treat this as "no target" WITHOUT counting a dropped
    datagram — a bad-size frame and an empty scene are different facts (§6.2).
    """
    return TargetMeasurement(frame_sequence=seq,
                             sensor_timestamp_ns=time.monotonic_ns(),
                             valid=False)


def connect_vision(path: str) -> socket.socket:
    """Connect to controld's vision ingest socket as a publisher.

    SOCK_SEQPACKET, not SOCK_STREAM: the ingest side is a SEQPACKET listener and
    preserves message boundaries, so a 58-byte write is one measurement. Getting
    this wrong on a stream socket would produce exactly the kind of framing bug
    the ingest counts as rx_error_frames — and it would look like a CAN fault.
    """
    if not os.path.exists(path):
        raise SystemExit(
            f"vision socket {path} does not exist.\n"
            "  controld creates it at boot ('vision ingest listening: UDS ...').\n"
            "  Check the daemon is running and that OTA_VISION_SOCKET matches the"
            " daemon's configured path.")
    s = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    s.settimeout(4.0)
    s.connect(path)
    return s


def _sleep_until(deadline: float) -> None:
    d = deadline - time.monotonic()
    if d > 0:
        time.sleep(d)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("mode", choices=("sweep", "hold", "none"))
    ap.add_argument("--socket", default=os.environ.get("OTA_VISION_SOCKET",
                                                       "/tmp/ota_vision.sock"))
    ap.add_argument("--hz", type=float, default=30.0,
                    help="publish rate (visiond runs ~30 Hz; §6.2 freshness "
                         "thresholds are tuned for that)")
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--amplitude", type=float, default=0.25,
                    help="sweep: fraction of frame width either side of centre")
    ap.add_argument("--period-s", type=float, default=8.0,
                    help="sweep: full oscillation period")
    ap.add_argument("--anchor-u", type=float, default=640.0, help="hold: u (px)")
    ap.add_argument("--anchor-v", type=float, default=360.0, help="hold: v (px)")
    ap.add_argument("--confidence", type=float, default=0.9)
    ap.add_argument("--class-id", type=int, default=1, help="1 = person (COCO)")
    ap.add_argument("--track-id", type=int, default=1)
    ap.add_argument("--dry-run", action="store_true",
                    help="print the frames instead of sending them")
    a = ap.parse_args(argv)

    t0 = time.monotonic()
    period = 1.0 / a.hz
    seq = 0
    sent = 0
    us: list[float] = []
    sock = None if a.dry_run else connect_vision(a.socket)
    if not a.dry_run:
        print(f"publishing FAKE targets ({a.mode}) to {a.socket} at {a.hz:g} Hz "
              f"for {a.seconds:g} s — no person exists in this scene")
    try:
        while True:
            t = time.monotonic() - t0
            if t >= a.seconds:
                break
            if a.mode == "sweep":
                m = sweep_measurement(seq, t, width=a.width, height=a.height,
                                      amplitude=a.amplitude,
                                      period_s=a.period_s, class_id=a.class_id,
                                      confidence=a.confidence,
                                      track_id=a.track_id)
            elif a.mode == "hold":
                m = hold_measurement(seq, anchor_u=a.anchor_u,
                                     anchor_v=a.anchor_v, class_id=a.class_id,
                                     confidence=a.confidence,
                                     track_id=a.track_id)
            else:
                m = invalid_measurement(seq)
            blob = m.encode()
            if a.dry_run:
                if seq % max(1, int(a.hz)) == 0:
                    print(f"  seq={m.frame_sequence:6d} valid={m.valid} "
                          f"u={m.anchor_u_px:7.1f} v={m.anchor_v_px:6.1f} "
                          f"conf={m.confidence:.2f} ({len(blob)} bytes)")
            else:
                assert sock is not None
                sock.send(blob)
            if m.valid:
                us.append(m.anchor_u_px)
            sent += 1
            seq += 1
            _sleep_until(t0 + seq * period)      # absolute schedule: no drift
    except (BrokenPipeError, ConnectionResetError):
        print(f"controld closed the socket after {sent} frames — the daemon is "
              f"gone or the ingest was torn down", file=sys.stderr)
        return 1
    finally:
        if sock is not None:
            # Closing deliberately: the FSM's coast/lost timers only start when
            # measurements STOP arriving, so an unclosed publisher would keep the
            # target "fresh" forever and the roaming path would never open.
            sock.close()
    span = f"  u {min(us):.0f}..{max(us):.0f} px" if us else "  (no valid frames)"
    print(f"sent {sent} frames in {time.monotonic()-t0:.2f} s "
          f"({sent / max(1e-9, time.monotonic()-t0):.1f} Hz){span}")
    print("feed stopped — the tracker will now coast, brake, and hand off to "
          "search if search is enabled")
    return 0


if __name__ == "__main__":
    sys.exit(main())
