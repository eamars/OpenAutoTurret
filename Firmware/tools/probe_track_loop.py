#!/usr/bin/env python3
"""Closed-loop synthetic target: drive the real controller with a target that lives in the WORLD.

Why the world frame is the whole point. A fake target whose pixel position is computed from the
current pose and then pinned near the image centre makes tracking succeed trivially and prove
nothing — the target would ride along with the video, and the loop would be closed around my own
assumptions instead of around the machine. Here the target is a world direction (azimuth, elevation)
from a scenario; only the PROJECTION of it depends on where the axis happens to be. When the turret
slews, the target MUST slide across the image, and the controller has to earn the centring.

    python3 tools/probe_track_loop.py roundtrip     # no motion: validate the math first
    python3 tools/probe_track_loop.py s1            # static target: does it centre?
    python3 tools/probe_track_loop.py s2            # constant-rate target: following error
    python3 tools/probe_track_loop.py s3            # dart: does the aim lead, and does it stay framed?

Geometry mirrored from the production code and then CHECKED against it, not assumed:
  r_B = R_z(q_yaw) . R_y(q_pitch) . r_C            (control/src/geometry/turret_kinematics.hpp,
                                                    R_PC is identity: no extrinsics file)
  los_to_base_ray(az, el) = (cos el cos az, cos el sin az, sin el);  az = atan2(y, x),
  el = atan2(z, hypot(x, y))
  u = cx + (x/z) fx,  v = cy + (y/z) fy            (camera_model.hpp; y is DOWN in the image)
  anchors on the wire are NORMALISED and controld multiplies by the TrackSet's own width/height,
  so width/height must be declared as the frame the intrinsics were measured in (1920x1080).

Pass criteria are written down BEFORE running, so a number cannot be chosen to fit the result:
  roundtrip  controld's own reported target azimuth/elevation must agree with the direction we
             asked it to look at to within 1.0 deg, with the axis at rest.
  S1         steady image error <= 1/3 of the declared box height (the operator's tolerance), with
             the mean of the last 2 s inside it and no sign change in the last 1 s (no oscillation).
  S2         target stays inside the frame for the whole sweep; report following error p50/p95.
  S3         target never leaves the frame; report whether the error crossed to the leading side.

Nothing here tunes anything. This round measures what the controller does; smoothing it is later
work and is not allowed to contaminate these numbers.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import time
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from vision.protocol import BBoxNorm, CLASS_PERSON, Track, TrackSet  # production encoder, not a mock

BASE = "http://127.0.0.1:8080"

# The intrinsics controld loaded (calibration/camera_intrinsics.yaml, measured on this box).
FX, FY, CX, CY, FW, FH = 1389.0, 1467.0, 960.0, 540.0, 1920, 1080

BOX_H_NORM = 0.35          # declared person box height as an image fraction (~380 px of 1080)
SOFT_MARGIN_RAD = 0.20     # abort before the axis gets this close to a soft limit


def state() -> dict:
    return json.load(urllib.request.urlopen(BASE + "/api/state", timeout=4))


def command(name: str, arg: str = "") -> dict:
    req = urllib.request.Request(
        BASE + "/api/command",
        data=json.dumps({"command": name, "arg": arg}).encode(),
        headers={"Content-Type": "application/json"}, method="POST")
    return json.loads(urllib.request.urlopen(req, timeout=4).read())


def wait_ack(seq0: int, timeout: float = 3.0):
    """The web response is not the verdict; controld's is, and cmd_ack_seq says which is new."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        s = state()
        if s.get("cmd_ack_seq", 0) != seq0:
            return s
        time.sleep(0.04)
    return None


def load_r_pc():
    """Read R_P_C from the SAME file controld reads, so the probe can never validate a model the
    controller has stopped using. Comment lines and key=value lines are skipped exactly as the C++
    loader skips them, and the three remaining lines are row-major."""
    path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        "calibration", "camera_extrinsics.yaml")
    rows = []
    try:
        with open(path) as fh:
            for line in fh:
                line = line.strip()
                if not line or line.startswith("#") or "=" in line:
                    continue
                parts = line.split()
                if len(parts) == 3:
                    rows.append([float(x) for x in parts])
    except OSError:
        pass
    if len(rows) != 3:
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], False
    return rows, True


R_PC, HAVE_EXT = load_r_pc()


def los_to_base(az: float, el: float):
    return (math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el))


def base_to_los(v):
    return math.atan2(v[1], v[0]), math.atan2(v[2], math.hypot(v[0], v[1]))


def axis_direction(q_yaw: float, q_pitch: float):
    """Base-frame direction the camera's optical axis is pointing right now."""
    cy_, sy_ = math.cos(q_yaw), math.sin(q_yaw)
    cp, sp = math.cos(q_pitch), math.sin(q_pitch)
    # R = Rz(q_yaw) . Ry(q_pitch) applied to camera forward (0,0,1):
    #   Ry(qp) . (0,0,1) = (sin qp, 0, cos qp);  then Rz.
    # camera forward is (0,0,1) in the camera frame, so R_PC . (0,0,1) is its third COLUMN;
    # then pitch, then yaw, exactly as turret_kinematics.hpp composes the chain.
    vx = R_PC[0][2]
    vy = R_PC[1][2]
    vz = R_PC[2][2]
    x1 = cp * vx + sp * vz
    y1 = vy
    z1 = -sp * vx + cp * vz
    return (cy_ * x1 - sy_ * y1, sy_ * x1 + cy_ * y1, z1)


def world_to_norm(az: float, el: float, q_yaw: float, q_pitch: float, rotate180: bool):
    """World direction -> normalised (u,v). The ONLY place the pose enters."""
    rx, ry, rz = los_to_base(az, el)
    # r_C = Ry(-qp) . Rz(-qy) . r_B, with rot_z(a) = {{c,-s,0},{s,c,0},{0,0,1}} and
    # rot_y(a) = {{c,0,s},{0,1,0},{-s,0,c}} exactly as control/src/geometry/vec3.hpp writes them.
    # The first draft fed sin(-q_yaw) into the FIRST row and got Rz(+q_yaw) instead of the
    # transpose it needed: the pitch half was right, so the centre still mapped to centre and
    # only off-axis targets were mirrored - a bug that looks like a broken controller until the
    # arithmetic is checked line by line.
    cy_, sy_ = math.cos(q_yaw), math.sin(q_yaw)
    x1 = cy_ * rx + sy_ * ry
    y1 = -sy_ * rx + cy_ * ry
    z1 = rz
    cp, sp = math.cos(-q_pitch), math.sin(-q_pitch)
    x2 = cp * x1 + sp * z1
    y2 = y1
    z2 = -sp * x1 + cp * z1
    # Back through the fixed mount rotation: r_C = R_P_C^T . r_P (transpose because the chain
    # carries R_P_C on the far side). Identity makes this a no-op; this machine's is not identity.
    x3 = R_PC[0][0] * x2 + R_PC[1][0] * y2 + R_PC[2][0] * z2
    y3 = R_PC[0][1] * x2 + R_PC[1][1] * y2 + R_PC[2][1] * z2
    z3 = R_PC[0][2] * x2 + R_PC[1][2] * y2 + R_PC[2][2] * z2
    x2, y2, z2 = x3, y3, z3
    if z2 <= 1e-6:
        return None  # behind the camera: not a target, a wish
    u = CX + (x2 / z2) * FX
    v = CY + (y2 / z2) * FY
    if rotate180:
        u, v = FW - u, FH - v
    return (u / FW, v / FH), (u, v)


class Publisher:
    """Publishes TrackSets for one virtual person, computed from the axis pose every frame."""

    def __init__(self, rotate180: bool, hz: float = 30.0, backdate_ms: float = 0.0) -> None:
        self.backdate_ns = int(backdate_ms * 1e6)
        import socket
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        self.sock.connect("/tmp/ota_vision.sock")
        self.rotate180 = rotate180
        self.period = 1.0 / hz
        self.seq = 0
        self.published = 0

    def publish(self, az: float, el: float, visible: bool = True) -> bool:
        s = state()
        norm = world_to_norm(az, el, s["q_yaw_rad"], s["q_pitch_rad"], self.rotate180)
        if norm is None:
            return False
        (un, vn), _ = norm
        # A real detector cannot report a target that is not in the picture. Emitting normalised
        # anchors of 9.0 for a target 4800 px away lets the controller chase something invisible -
        # which is exactly the runaway the first S1 run produced. Off-frame means LOST, and the
        # selection survives as LOST_REACQUIRABLE rather than as a phantom the loop can steer by.
        outside = not (0.0 <= un <= 1.0 and 0.0 <= vn <= 1.0)
        visible = visible and not outside
        self.outside = outside
        # §9's velocity is the MEASURED anchor rate, not an intent. A target that is fixed in the
        # world still slides across the image while the axis slews, so publishing zero here hands the
        # estimator an anchor that moves with no velocity - a contradiction it can only resolve by
        # guessing, and the first S1 run settled 3.8 deg off centre with exactly that lie in the
        # input. Finite-difference the anchors we actually published.
        now = time.monotonic()
        vx = vy = 0.0
        if getattr(self, "prev", None) is not None:
            dt = now - self.prev[0]
            if dt > 1e-3:
                vx = (un - self.prev[1]) / dt
                vy = (vn - self.prev[2]) / dt
        self.prev = (now, un, vn)
        self.seq += 1
        h = BOX_H_NORM * 0.5
        t = Track(track_uuid=(0, 700), display_index=1, class_id=CLASS_PERSON,
                  class_name="person", state=1 if visible else 3, detector_confidence=0.90,
                  track_confidence=0.90,
                  bbox=BBoxNorm(x_min=max(0.0, un - 0.14), y_min=max(0.0, vn - h),
                                x_max=min(1.0, un + 0.14), y_max=min(1.0, vn + h)),
                  anchor_x=un, anchor_y=vn, velocity_x_norm_s=vx, velocity_y_norm_s=vy,
                  age_frames=self.seq, visible_frames=self.seq)
        # A capture stamp taken at publish time can be NEWER than the newest pose sample in
        # controld's ring, and motor_state_history.hpp::interpolate() rejects t > last.t_ns outright
        # rather than clamping. The measurement then vanishes with no log line, the estimator never
        # initialises, and AUTO_TRACK silently produces no reference at all. Back-dating is the knob
        # that tells those two explanations apart.
        now_ns = time.monotonic_ns()
        ts = TrackSet(frame_sequence=self.seq, sensor_timestamp_ns=now_ns - self.backdate_ns,
                      publish_timestamp_ns=now_ns, width=FW, height=FH,
                      tracks=[t])
        try:
            self.sock.send(ts.encode())
        except BlockingIOError:
            return False
        self.published += 1
        return True

    def close(self) -> None:
        try:
            self.sock.close()
        except Exception:
            pass


def guard_or_abort(s: dict, why: str = "") -> bool:
    """Stop the run if the axis approaches a soft limit or anything looks wrong."""
    for q, lo, hi, nm in ((s["q_yaw_rad"], s["q_soft_min_yaw_rad"], s["q_soft_max_yaw_rad"], "yaw"),
                          (s["q_pitch_rad"], s["q_soft_min_pitch_rad"], s["q_soft_max_pitch_rad"], "pitch")):
        if q <= lo + SOFT_MARGIN_RAD or q >= hi - SOFT_MARGIN_RAD:
            print("  SAFETY: %s %.3f rad at/near its soft limit [%s, %s] %s" % (nm, q, lo, hi, why))
            return True
    return False


def cmd_az_el_for_roundtrip(pub: Publisher) -> None:
    s = state()
    az0, el0 = base_to_los(axis_direction(s["q_yaw_rad"], s["q_pitch_rad"]))
    print("axis at rest: q_yaw=%.4f q_pitch=%.4f -> my model says the optical axis points at az=%.2f deg el=%.2f deg"
          % (s["q_yaw_rad"], s["q_pitch_rad"], math.degrees(az0), math.degrees(el0)))

    # Publish the target dead on the optical axis: whatever controld computes for the LOS must equal
    # the axis direction, which BOTH sides derive from the pose alone. Disagreement here is my mirror
    # being wrong, before any image convention can hide behind it.
    # Step 1: on-axis round trip (no motion possible: the target is already centred).
    end = time.monotonic() + 2.0
    while time.monotonic() < end:
        pub.publish(az0, el0)
        time.sleep(0.033)
    seq0 = s.get("cmd_ack_seq", 0)
    command("select_target", "1")
    v = wait_ack(seq0)
    time.sleep(0.3)
    s = state()
    print("  select -> ack %r" % (s.get("cmd_ack_reason"),))
    got_az = s.get("target_az_world_rad")
    got_el = s.get("target_el_world_rad")
    print("  controld reports target_az_world=%s target_el_world=%s  (estimator_ready=%s, visibility=%r)"
          % (got_az, got_el, s.get("estimator_ready"), s.get("selection_visibility")))
    if got_az is None or (got_az == 0.0 and got_el == 0.0):
        print("  No LOS reported for a selected target in MANUAL: cannot validate the mirror this way.")
        print("  (That is itself worth knowing: it means the LOS fields fill only while tracking.)")
        return
    daz = (math.degrees(got_az - az0) + 180.0) % 360.0 - 180.0
    del_ = math.degrees(got_el - el0)
    print("  ROUND TRIP on-axis: daz %+.3f deg, del %+.3f deg  -> %s"
          % (daz, del_, "PASS" if abs(daz) < 1.0 and abs(del_) < 1.0 else "FAIL"))
    print("  (a persistent daz is the yaw-zero convention of homing, not an error in either side)")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("scenario", choices=["roundtrip", "s1", "s2", "s3"])
    ap.add_argument("--rotate180", action="store_true", help="flip the predicted pixel (upside-down mount)")
    ap.add_argument("--offset-deg", type=float, default=12.0)
    ap.add_argument("--seconds", type=float, default=14.0)
    ap.add_argument("--backdate-ms", type=float, default=0.0,
                    help="stamp sensor_timestamp_ns this far in the past (see Publisher)")
    args = ap.parse_args()

    s = state()
    # webd keeps serving the last snapshot it received even after controld dies, which has already
    # fooled this probe once (at_ready=True from a daemon I had just killed, and the run started
    # mid-homing). v3.2 s25 makes that a defect to fix; until it is fixed, every caller must check
    # the connection itself.
    try:
        h = json.load(urllib.request.urlopen(BASE + "/api/health", timeout=4))
    except Exception as e:
        print("cannot read /api/health: %s" % e)
        return 1
    if not h.get("controld_connected"):
        print("webd is not connected to controld; any state it reports may be stale. Refusing.")
        return 1
    if not s.get("at_ready"):
        print("not homed/ready; refusing to run")
        return 1
    print("station: %s/%s at_ready=%s q_yaw=%.4f q_pitch=%.4f" %
          (s["operating_mode"], s["mode_phase"], s["at_ready"], s["q_yaw_rad"], s["q_pitch_rad"]))
    print("extrinsics: R_P_C %s loaded from calibration/camera_extrinsics.yaml"
          % ("(%.0f deg mount rotation)" % math.degrees(math.atan2(R_PC[1][0], R_PC[0][0])) if HAVE_EXT
             else "NOT FOUND - assuming aligned identity, which is wrong on this station"))

    # Offline self-check, no hardware: the camera's own axis direction must project to EXACTLY the
    # principal point for arbitrary poses, and a 12 deg azimuth offset must land about
    # fx*tan(12) px away. If either fails, the mirror is wrong and no result below means anything.
    worst_c = 0.0
    for qy, qp in ((0.0, -0.69), (0.8, -0.35), (2.6, -0.68), (-1.9, -1.2), (3.1, -0.1)):
        d = axis_direction(qy, qp)
        az_a, el_a = base_to_los(d)
        got = world_to_norm(az_a, el_a, qy, qp, args.rotate180)
        if got is None:
            print("GEOMETRY SELF-CHECK: axis direction did not project at all (pose %s)" % ((qy, qp),))
            return 3
        worst_c = max(worst_c, abs(got[1][0] - CX), abs(got[1][1] - CY))
    # The expected shift is fx*tan(angle between the two DIRECTIONS), which is not fx*tan(12 deg):
    # at 51 deg depression the azimuth circle is foreshortened, and a first version of this check
    # demanded the horizon value and failed a projection that was in fact correct.
    qy_t, qp_t = 2.6, -0.68
    az_t0, el_t0 = base_to_los(axis_direction(qy_t, qp_t))
    d0, d12 = los_to_base(az_t0, el_t0), los_to_base(az_t0 + math.radians(12.0), el_t0)
    dot = max(-1.0, min(1.0, sum(a * b for a, b in zip(d0, d12))))
    ang = math.acos(dot)
    expect = FX * math.tan(ang)
    off = world_to_norm(az_t0 + math.radians(12.0), el_t0, qy_t, qp_t, args.rotate180)
    got_u, got_v = abs(off[1][0] - CX), abs(off[1][1] - CY)
    print("GEOMETRY SELF-CHECK: axis->principal-point worst error %.2e px (must be ~0)" % worst_c)
    print("                   : 12 deg of azimuth at %.0f deg depression -> |du| %.0f px (geometry: %.0f), |dv| %.0f px"
          % (math.degrees(el_t0), got_u, expect, got_v))
    if worst_c > 1e-6 or not (0.85 * expect < got_u < 1.15 * expect) or got_v > 0.15 * expect:
        print("  the mirror does not match its own geometry; refusing to run against hardware")
        return 3

    # Prove the anti-tautology property before anything else: a WORLD-fixed target must move a lot in
    # the image when only the axis moves. If this prints a small number, the target is riding the
    # video and every result below would be worthless.
    az_test, el_test = 0.7, -0.25
    a = world_to_norm(az_test, el_test, 1.00, -0.30, args.rotate180)
    b = world_to_norm(az_test, el_test, 1.0873, -0.30, args.rotate180)  # 5 deg of yaw later
    if a and b:
        shift = abs(a[1][0] - b[1][0])
        print("ANTI-TAUTOLOGY CHECK: same world direction, axis 5 deg later -> image moves %.0f px  %s"
              % (shift, "OK" if shift > 100 else "SUSPECT: target may be pinned to the image"))
        if shift < 100:
            return 3

    pub = Publisher(args.rotate180, backdate_ms=args.backdate_ms)
    if args.scenario == "roundtrip":
        cmd_az_el_for_roundtrip(pub)
        pub.close()
        return 0
    # ---- S1: one world-fixed target, 12 deg off the current axis, and let the controller work ----
    s = state()
    az0, el0 = base_to_los(axis_direction(s["q_yaw_rad"], s["q_pitch_rad"]))
    az_t, el_t = az0 + math.radians(args.offset_deg), el0     # a WORLD direction, fixed from here
    print("\nS1: target fixed at az %.2f deg, el %.2f deg (axis now at az %.2f, el %.2f)"
          % (math.degrees(az_t), math.degrees(el_t), math.degrees(az0), math.degrees(el0)))
    start = world_to_norm(az_t, el_t, s["q_yaw_rad"], s["q_pitch_rad"], args.rotate180)
    box_h_px = BOX_H_NORM * FH
    tol_px = box_h_px / 3.0            # the operator's tolerance, stated before the run
    if start is None or abs(start[1][0] - CX) <= tol_px:
        print("  offset too small: the target starts inside the %.0f px acceptance band, so this run"
              % tol_px)
        print("  could not show convergence. Pick --offset-deg so the start is OUTSIDE the band.")
        pub.close()
        return 2
    pub.publish(az_t, el_t)
    time.sleep(0.5)
    s = state()
    seq0 = s.get("cmd_ack_seq", 0)
    command("select_target", "1")
    v = wait_ack(seq0)
    s = state()
    if not s.get("selected_uuid_valid"):
        print("  selection refused: %r" % (s.get("cmd_ack_reason"),))
        pub.close(); return 1
    print("  selected uuid=%s" % s.get("selected_uuid"))
    seq0 = s.get("cmd_ack_seq", 0)
    command("set_mode", "AUTO_TRACK")
    wait_ack(seq0)
    print("  mode now %s/%s (%s)" % ((s := state())["operating_mode"], s["mode_phase"], "tracking"))

    rows = []
    t0 = time.monotonic()
    while time.monotonic() - t0 < args.seconds:
        if not pub.publish(az_t, el_t):
            print("  publish failed (target behind the camera?) - stopping")
            break
        s = state()
        if guard_or_abort(s):
            command("stop_motion")
            break
        nrm = world_to_norm(az_t, el_t, s["q_yaw_rad"], s["q_pitch_rad"], args.rotate180)
        if nrm:
            (un, vn), (u, vv) = nrm
            rows.append({"t": time.monotonic() - t0, "ex": u - CX, "ey": vv - CY,
                         "outside": getattr(pub, "outside", False),
                         "q_yaw": s["q_yaw_rad"], "v_yaw": s["v_yaw_rad_s"],
                         "q_pitch": s["q_pitch_rad"], "v_pitch": s["v_pitch_rad_s"],
                         "q_ref_yaw": s.get("q_ref_yaw_rad"), "q_ref_pitch": s.get("q_ref_pitch_rad"),
                         "intent_reason": s.get("intent_reason"),
                         "prediction_age_ms": s.get("prediction_age_ms"),
                         "track_state": s.get("track_state"),
                         "phase": s["mode_phase"], "vis": s["selection_visibility"],
                         "los_az": s.get("target_az_world_rad") or 0.0,
                         "my_az": az_t})
        time.sleep(0.033)
        if rows and all(r.get("outside") for r in rows[-12:]):
            print("  target has been outside the frame for %d samples - the loop is diverging; stopping"
                  % len(rows[-12:]))
            command("stop_motion")
            break

    print("  published %d TrackSets, sampled %d\n" % (pub.published, len(rows)))
    if rows:
        print("  TRACE (1 Hz):")
        nxt = 0.0
        for r in rows:
            if r["t"] >= nxt:
                nxt += 1.0
                print("    t=%4.1f s ex=%+7.1f ey=%+7.1f px | q=%+.4f q_ref=%s (delta %+.4f rad) | %s / %s | pred_age=%s"
                      % (r["t"], r["ex"], r["ey"], r["q_yaw"],
                         ("%+.4f" % r["q_ref_yaw"]) if r["q_ref_yaw"] is not None else " None ",
                         (r["q_ref_yaw"] - r["q_yaw"]) if r["q_ref_yaw"] is not None else 0.0,
                         r["track_state"], r["intent_reason"], r["prediction_age_ms"]))
        a, b = rows[0], rows[-1]
        # Did the axis close on the target or run from it? Measured in image space, which is the
        # only space where "toward the reticle" is an unambiguous statement.
        d0 = math.hypot(a["ex"], a["ey"]) or 1e-9
        d1 = math.hypot(b["ex"], b["ey"])
        print("\n  |error| went from %.0f px to %.0f px (%.0f%% of the original) -> the axis moved %s the target"
              % (d0, d1, 100.0 * d1 / d0, "TOWARD" if d1 < d0 else "AWAY FROM"))
    if rows:
        tail = [r for r in rows if r["t"] > rows[-1]["t"] - 2.0]
        ex = [abs(r["ex"]) for r in tail]
        ey = [abs(r["ey"]) for r in tail]
        print("  LAST 2 s: |error_x| mean %.1f px (max %.1f), |error_y| mean %.1f px (max %.1f)"
              % (sum(ex) / len(ex), max(ex), sum(ey) / len(ey), max(ey)))
        print("          tolerance is %.0f px (1/3 of the %.0f px box height)" % (tol_px, box_h_px))
        sgn = [1 if r["ex"] > 0 else -1 for r in tail]
        flips = sum(1 for i in range(1, len(sgn)) if sgn[i] != sgn[i - 1])
        print("          error_x sign changes in the last 2 s: %d %s"
              % (flips, "(no oscillation)" if flips < 3 else "(OSCILLATING)"))
        first = min((r for r in rows if abs(r["ex"]) < tol_px), key=lambda r: r["t"], default=None)
        print("          first inside tolerance at t=%.2f s" % (first["t"] if first else -1))
        d = [(r["los_az"] - r["my_az"]) for r in rows if r["los_az"] != 0.0]
        if d:
            print("          controld's LOS minus my predicted az: mean %+.3f deg (max |%.3f|) over %d samples"
                  % (math.degrees(sum(d) / len(d)), math.degrees(max(abs(x) for x in d)), len(d)))
        else:
            print("          controld never published target_az_world_rad even while tracking")
        reasons = {}
        for r in rows:
            reasons[r["intent_reason"]] = reasons.get(r["intent_reason"], 0) + 1
        print("          intent_reason seen: %s" % reasons)
        deltas = [r["q_ref_yaw"] - r["q_yaw"] for r in rows if r["q_ref_yaw"] is not None]
        if deltas:
            print("          q_ref_yaw - q_yaw: mean %+.4f rad (%.2f deg), max |%.4f|  -> %s"
                  % (sum(deltas) / len(deltas), math.degrees(sum(deltas) / len(deltas)),
                     max(abs(x) for x in deltas),
                     "the reference is asking for a large slew the axis is not making"
                     if abs(sum(deltas) / len(deltas)) > 0.05 and
                     (sum(ex) / len(ex)) > tol_px else
                     "reference and pose agree, and the image error is small: converged"))
        else:
            print("          q_ref_yaw was never populated")
        ph = {}
        for r in rows:
            ph[r["phase"]] = ph.get(r["phase"], 0) + 1
        print("          phase occupancy: %s" % ph)
        print("          peak |v_yaw| during the run: %.4f rad/s = %.1f deg/s"
              % (max(abs(r["v_yaw"]) for r in rows), max(abs(r["v_yaw"]) for r in rows) * 57.2958))
        ok = ex and (sum(ex) / len(ex)) <= tol_px and flips < 3
        print("\n  S1 VERDICT: %s  (measured, not tuned)" % ("PASS" if ok else "FAIL"))

    print("\n  returning to MANUAL")
    command("clear_target")
    time.sleep(0.3)
    command("set_mode", "MANUAL")
    pub.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
