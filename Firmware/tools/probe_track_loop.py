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
import urllib.error
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
    try:
        return json.loads(urllib.request.urlopen(req, timeout=4).read())
    except urllib.error.HTTPError as e:
        # The web gate answers refusals as 4xx with the reason in the body. Letting this raise
        # hides the only explanation the tool is going to get - this is the s52 response gap
        # showing up in the tooling instead of the protocol.
        try:
            body = json.loads(e.read().decode("utf-8", "replace"))
        except Exception:
            body = {"error": e.reason, "body": ""}
        body.setdefault("http_status", e.code)
        return body


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


_near_hist = {}


def guard_or_abort(s: dict, why: str = "") -> bool:
    """Stop the run when an axis is being DRIVEN THROUGH a soft limit.

    Two versions of this got it wrong, and both wrongs are worth recording. The first aborted on
    proximity: this station's homed pitch rests about 11 deg from its lower soft limit, so every
    scenario was refused before it began. The second added the velocity sign, and a holding axis
    ripples by +-0.2 rad/s, so it aborted anyway on feedback noise ("driven into its soft limit",
    when 3 s of sampling showed 0.02 deg of drift).

    What is actually dangerous is an axis near a limit that keeps MOVING FURTHER OUT, measured where
    measurement is reliable - on position, over a window. Sitting near a limit is this station's
    working life; crossing it is the hazard.
    """
    for key, q, lo, hi, nm in (
            ("yaw", s["q_yaw_rad"], s["q_soft_min_yaw_rad"], s["q_soft_max_yaw_rad"], "yaw"),
            ("pitch", s["q_pitch_rad"], s["q_soft_min_pitch_rad"], s["q_soft_max_pitch_rad"], "pitch")):
        if not (isinstance(q, float) and isinstance(lo, float) and isinstance(hi, float)):
            continue
        near_lo, near_hi = q <= lo + SOFT_MARGIN_RAD, q >= hi - SOFT_MARGIN_RAD
        now = time.monotonic()
        hist = [x for x in _near_hist.get(key, []) if now - x[0] <= 1.0]
        hist.append((now, q))
        _near_hist[key] = hist
        if not (near_lo or near_hi):
            continue
        # How far has it travelled in the outward direction inside the window?
        outward = (min(x[1] for x in hist) - q) if near_lo else (q - max(x[1] for x in hist))
        room = min(q - lo, hi - q)
        if outward > max(0.01, room):        # 0.6 deg, or all the room that is left
            print("  SAFETY: %s has moved %.3f rad OUTWARD while %.3f rad from its soft limit "
                  "[%s, %s]%s - stopping" % (nm, outward, room, lo, hi, (" " + why) if why else ""))
            return True
        if outward > 0.002:
            print("  note: %s creeping outward near its soft limit (%.3f rad in 1 s, %.3f left)"
                  % (nm, outward, room))
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


def ensure_yaw_centred(s):
    """Walk yaw back toward the middle of its travel before a scenario that needs room to follow.

    A previous run can leave the axis parked against its soft limit; the guard then stops a new
    scenario at t=0 and the criteria end up scoring an empty run. That happened, and the scenario
    reported PASS. Doing this in one place keeps S2 and S3 from each inventing their own version.
    """
    lo, hi = s.get("q_soft_min_yaw_rad"), s.get("q_soft_max_yaw_rad")
    if not (isinstance(lo, float) and isinstance(hi, float) and hi > lo):
        return s
    mid = 0.5 * (lo + hi)
    if abs(s["q_yaw_rad"] - mid) <= 0.2 * (hi - lo):
        return s
    if s.get("operating_mode") != "MANUAL":
        command("set_mode", "MANUAL")
        time.sleep(0.5)
    print("  walking yaw toward mid-travel before starting...")
    for _ in range(80):
        s = state()
        qy = s["q_yaw_rad"]
        if abs(qy - mid) <= 0.15 * (hi - lo):
            break
        step = max(-5.0, min(5.0, math.degrees(mid - qy)))
        command("manual_step", "yaw%+.2f" % step)
        time.sleep(1.0)
    print("  yaw now %.3f rad (travel %.3f..%.3f)" % (s["q_yaw_rad"], lo, hi))
    return wait_settled(s)


def wait_settled(s, timeout_s: float = 30.0, tol_rad: float = 5.0e-4, window_s: float = 0.5):
    """Wait until the pose is genuinely not moving, and hand back that state.

    Every scenario derives its world target from the CURRENT pose, so a pose that is travelling makes
    the target a fiction. The first version of this tested velocity against a threshold, and it never
    closed - not because the axes were moving, but because this station's velocity feedback ripples
    by +-0.2 rad/s on an axis that is demonstrably holding (measured: 0.022 deg of yaw drift over
    3 s). An instantaneous velocity sample on a holding axis is noise, not motion. So stability is
    judged on POSITION, which is the thing the geometry actually depends on: the tolerance below is
    0.03 deg, which is under one pixel of image travel.
    """
    deadline = time.monotonic() + timeout_s
    seen = []
    announced = False
    while time.monotonic() < deadline:
        s = state()
        qy, qp = s.get("q_yaw_rad"), s.get("q_pitch_rad")
        if not (isinstance(qy, float) and isinstance(qp, float)):
            return s
        now = time.monotonic()
        seen.append((now, qy, qp))
        seen = [x for x in seen if now - x[0] <= window_s]
        if len(seen) >= 4:
            span_y = max(x[1] for x in seen) - min(x[1] for x in seen)
            span_p = max(x[2] for x in seen) - min(x[2] for x in seen)
            if span_y < tol_rad and span_p < tol_rad:
                return s
        if not announced and now > deadline - timeout_s + 3.0:
            print("  waiting for the pose to settle (yaw span %.4f, pitch span %.4f rad over %.1fs)"
                  % (span_y, span_p, window_s))
            announced = True
        time.sleep(0.05)
    print("  pose still moving after %.0f s - proceeding anyway, and the result is suspect" % timeout_s)
    return s


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("scenario", choices=["roundtrip", "s1", "s2", "s3"])
    ap.add_argument("--rotate180", action="store_true", help="flip the predicted pixel (upside-down mount)")
    ap.add_argument("--offset-deg", type=float, default=12.0)
    ap.add_argument("--sweep-sign", dest="sweep_sign", default="", help=""
                    "S2: force +1 or -1 instead of choosing from the remaining travel")
    ap.add_argument("--dart-deg", dest="dart_deg", type=float, default=25.0,
                   help="S3: azimuth step of the dart (deg)")
    ap.add_argument("--dart-ramp-s", dest="dart_ramp_s", type=float, default=0.40,
                   help="S3: time the dart is completed over (s)")
    ap.add_argument("--hold-s", dest="hold_s", type=float, default=3.0,
                   help="S3: how long the target holds still after the dart (s)")
    ap.add_argument("--max-recovery-s", dest="max_recovery_s", type=float, default=1.50,
                   help="S3: how long the axis may take to get back inside tolerance (s)")
    ap.add_argument("--rate-deg-s", dest="rate_deg_s", type=float, default=8.0,
                   help="S2: constant world-azimuth rate of the moving target (deg/s)")
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

    # ---- S2: a target that MOVES at a constant rate, and does not stop ------------------------
    #
    # S1 only proves the loop can reach a still target. Requirement (b) is different: following has
    # to be smooth and must not let the target leave the frame before the axis catches up. Those are
    # measurable claims, and the criteria are fixed here, in the file, before the run - a threshold
    # chosen after seeing the data is not a threshold.
    if args.scenario == "s2":
        rate = args.rate_deg_s
        # If a previous run left the axis against its travel limit, no amount of scenario design
        # will produce a following measurement: the guard stops it at t=0. Walk it back to the
        # middle of the travel first, in MANUAL, one small step at a time.
        s = state()
        lo, hi = s.get("q_soft_min_yaw_rad"), s.get("q_soft_max_yaw_rad")
        s = ensure_yaw_centred(s)
        print("\nS2 criteria, fixed before running:")
        print("  C1 containment   : the target never leaves the frame while the axis follows")
        print("  C2 following     : aim-to-reticle <= 1/3 box height (%.0f px) for >=95%% of samples"
              % (BOX_H_NORM * FH / 3.0))
        print("                     after the first 1.0 s of steady motion")
        print("  C3 no divergence : no consecutive out-of-frame run, no soft-limit guard trip")
        print("  motion           : +%.1f deg/s in world azimuth for %.1f s" % (rate, args.seconds))

        s = state()
        az0, el0 = base_to_los(axis_direction(s["q_yaw_rad"], s["q_pitch_rad"]))
        box_h_px = BOX_H_NORM * FH
        tol_px = box_h_px / 3.0

        # Sweep TOWARD the middle of the travel, not away from it. The first runs of this scenario
        # ended at the yaw soft limit after 1.8 s and then scored a PASS off one leftover sample:
        # the axis was pinned, the target kept going, and the criteria never looked at whether
        # anything was being tracked. The station's yaw travel is wide but it is not infinite, and
        # where it starts depends on what ran before it.
        lo, hi = (s.get("q_soft_min_yaw_rad"), s.get("q_soft_max_yaw_rad"))
        sign = 1.0 if rate >= 0 else -1.0
        if args.sweep_sign:
            sign = float(args.sweep_sign)
        if isinstance(lo, float) and isinstance(hi, float) and hi > lo:
            qy = s["q_yaw_rad"]
            room_up, room_dn = hi - qy, qy - lo
            if (sign > 0 and room_up < 0.9 * abs(math.radians(rate)) * args.seconds / 1.0) or \
               (sign > 0 and room_up < room_dn):
                sign = -1.0
            elif sign < 0 and room_dn < room_up:
                sign = 1.0
            # Cap the run so the sweep cannot reach the limit even with the sign chosen right.
            room = room_up if sign > 0 else room_dn
            max_seconds = (0.75 * room) / max(abs(math.radians(rate)), 1e-6)
            if args.seconds > max_seconds:
                print("  shortening to %.1f s: %.2f rad of yaw travel left in that direction"
                      % (max_seconds, room))
                args.seconds = max(2.0, max_seconds)
        rate = sign * abs(rate)
        print("  sweep direction: %+.1f deg/s (chosen from the remaining yaw travel)" % rate)

        # Start ON the axis: S2 is about following, not about the initial slew.
        az_t, el_t = az0, el0
        if not pub.publish(az_t, el_t):
            print("  could not publish the starting target")
            pub.close(); return 1
        time.sleep(0.5)
        s = state()
        seq0 = s.get("cmd_ack_seq", 0)
        command("select_target", "1")
        wait_ack(seq0)
        s = state()
        if not s.get("selected_uuid_valid"):
            print("  selection refused: %r" % (s.get("cmd_ack_reason"),))
            pub.close(); return 1
        seq0 = s.get("cmd_ack_seq", 0)
        command("set_mode", "AUTO_TRACK")
        wait_ack(seq0)
        print("  selected uuid=%s, mode %s/%s" % (s.get("selected_uuid"),
                                                  (s := state())["operating_mode"], s["mode_phase"]))

        rows = []
        t0 = time.monotonic()
        while time.monotonic() - t0 < args.seconds:
            t = time.monotonic() - t0
            az_t = az0 + math.radians(rate) * t          # a WORLD sweep, not an image-space drag
            if not pub.publish(az_t, el_t):
                print("  publish failed at t=%.2f s (target behind the camera?)" % t)
                break
            s = state()
            if guard_or_abort(s):
                command("stop_motion")
                print("  soft-limit guard tripped at t=%.2f s" % t)
                break
            nrm = world_to_norm(az_t, el_t, s["q_yaw_rad"], s["q_pitch_rad"], args.rotate180)
            if nrm:
                (un, vn), (u, vv) = nrm
                au = s.get("target_aim_x_norm")
                av = s.get("target_aim_y_norm")
                aim_err = (math.hypot(au * FW - CX, av * FH - CY)
                           if s.get("target_aim_valid") and isinstance(au, float) else None)
                rows.append({"t": t, "ex": u - CX, "ey": vv - CY, "aim_err": aim_err,
                             "in_frame": (0.0 <= u <= FW) and (0.0 <= vv <= FH),
                             "outside": getattr(pub, "outside", False),
                             "v_yaw": s["v_yaw_rad_s"], "track_state": s.get("track_state"),
                             "az_err_deg": math.degrees(az_t - (s.get("target_az_world_rad") or az_t))})
            time.sleep(0.033)

        pub.close()
        print("  published %d TrackSets, sampled %d" % (pub.published, len(rows)))
        if not rows:
            print("  no samples - cannot judge anything")
            return 1
        steady = [r for r in rows if r["t"] >= 1.0]
        errs = sorted(r["aim_err"] for r in steady if r["aim_err"] is not None)
        out_of_frame = [r["t"] for r in rows if not r["in_frame"] or r["outside"]]
        tracking_frac = (sum(1 for r in steady if str(r["track_state"]) == "tracking") /
                         float(len(steady))) if steady else 0.0
        # A verdict needs data, and it needs the controller to have been doing the thing under
        # test. An earlier version of this printed PASS on a single leftover aim-point sample
        # while the tracker sat in ready_hold and the target estimate was 34 deg away - which is
        # every bit the sin this whole file exists to avoid, committed by the tool itself.
        n_aim = len(errs)
        meaningful = n_aim >= 30 and tracking_frac >= 0.8
        print("    samples: %d steady, %d with a live aim point, tracking in %.0f%% of them;"
              % (len(steady), n_aim, 100.0 * tracking_frac))
        print("              %d published as out-of-frame/LOST"
              % sum(1 for r in rows if r["outside"]))
        if not meaningful:
            print("\n  S2 VERDICT: INVALID, not a failure - the run did not exercise tracking.")
            print("          Need >=30 live aim samples and >=80% of steady samples in state")
            print("          'tracking'. Got n=%d, tracking=%.0f%%. Diagnose the acquisition"
                  % (n_aim, 100.0 * tracking_frac))
            print("          before reading anything into the error numbers.")
            command("clear_target"); time.sleep(0.2)
            command("set_mode", "MANUAL")
            return 3

        def pct(vals, q):
            return vals[min(len(vals) - 1, int(q * len(vals)))] if vals else float("nan")

        c1 = not out_of_frame
        c2 = bool(errs) and pct(errs, 0.95) <= tol_px
        print("\n  S2 result (using the aim point published by controld, %s):"
              % ("head aim" if any(r["aim_err"] is not None for r in steady) else "anchor fallback"))
        print("    following error p50 %.1f px / p95 %.1f px  (%.3f / %.3f of box height)"
              % (pct(errs, .5), pct(errs, .95), pct(errs, .5) / box_h_px, pct(errs, .95) / box_h_px))
        print("    world azimuth error p50 %.2f deg / max %.2f deg"
              % (sorted(abs(r["az_err_deg"]) for r in steady)[len(steady) // 2] if steady else float("nan"),
                 max((abs(r["az_err_deg"]) for r in steady), default=float("nan"))))
        print("    commanded yaw rate p50 %.2f deg/s (target rate %.2f deg/s)"
              % (sorted(abs(r["v_yaw"]) for r in rows)[len(rows) // 2] * 180 / math.pi, rate))
        print("    states seen: %s" % sorted({str(r["track_state"]) for r in rows}))
        print("    C1 containment  : %s%s" % ("PASS" if c1 else "FAIL",
              "" if c1 else "  left the frame at t=%s" % ["%.2f" % x for x in out_of_frame[:6]]))
        print("    C2 following    : %s (p95 %.1f px vs bar %.1f px, n=%d)"
              % ("PASS" if c2 else "FAIL", pct(errs, .95), tol_px, n_aim))
        print("    C0 was tracking : %s (needed >=80%% of steady samples)"
              % ("PASS" if tracking_frac >= 0.8 else "FAIL"))
        print("    C3 no divergence: %s" % ("PASS" if not any(
            all(r["outside"] for r in rows[i:i + 12]) for i in range(max(0, len(rows) - 11))) else "FAIL"))
        command("clear_target"); time.sleep(0.2)
        command("set_mode", "MANUAL")
        return 0 if (c1 and c2 and tracking_frac >= 0.8) else 1

    # ---- S3: a dart, and whether the aim gets ahead of it --------------------------------------
    #
    # Requirement (b) is not "follows a slow target". It is that the aim LEADS sharp movement by
    # projection, so the target does not leave the frame before the axis catches up. A constant-rate
    # sweep cannot speak to that: at 8 deg/s the axis simply keeps up, and the predictor never has to
    # earn its place. So this scenario moves the target fast and asks three separate questions - did
    # it stay in frame, did the axis come back inside tolerance, and was the reference actually AHEAD
    # of where the target really was while the dart was happening.
    if args.scenario == "s3":
        D = args.dart_deg
        ramp = args.dart_ramp_s
        px_per_deg = 24.2                     # measured by tools/probe_theodolite.py: 24.22 px/deg
        print("\nS3 criteria, fixed before running:")
        print("  C1 containment   : target never leaves the frame through the dart")
        print("  C2 recovery      : aim-to-reticle back within 1/3 box height (%.0f px) and held,"
              % (BOX_H_NORM * FH / 3.0))
        print("                     no more than %.2f s after the dart starts" % args.max_recovery_s)
        print("  C3 lead          : during the dart the REFERENCE (q_ref) must sit ahead of the")
        print("                     target's true position, in the direction of travel")
        print("  C4 no ringing    : aim error changes sign <= 2 times after arrival")
        print("  motion           : %.1f deg of azimuth in %.2f s (%.0f deg/s), then hold %.1f s"
              % (D, ramp, D / ramp, args.hold_s))
        if D * px_per_deg > 850.0:
            print("  dart too large: %.0f px of travel would leave the frame whatever the controller"
                  " does; pick a smaller --dart-deg" % (D * px_per_deg))
            pub.close()
            return 2

        s = ensure_yaw_centred(state())
        s = wait_settled(s)
        az0, el0 = base_to_los(axis_direction(s["q_yaw_rad"], s["q_pitch_rad"]))
        box_h_px = BOX_H_NORM * FH
        tol_px = box_h_px / 3.0
        if not pub.publish(az0, el0):
            pub.close()
            return 1
        time.sleep(0.5)
        s = state()
        seq0 = s.get("cmd_ack_seq", 0)
        command("select_target", "1")
        wait_ack(seq0)
        s = state()
        if not s.get("selected_uuid_valid"):
            print("  selection refused: %r" % (s.get("cmd_ack_reason"),))
            pub.close()
            return 1
        seq0 = s.get("cmd_ack_seq", 0)
        command("set_mode", "AUTO_TRACK")
        wait_ack(seq0)

        rows = []
        t0 = time.monotonic()
        total = 1.5 + ramp + args.hold_s
        while time.monotonic() - t0 < total:
            t = time.monotonic() - t0
            if t < 1.5:
                phase, az_t = "settle", az0
            elif t < 1.5 + ramp:
                phase, az_t = "dart", az0 + math.radians(D) * ((t - 1.5) / ramp)
            else:
                phase, az_t = "hold", az0 + math.radians(D)
            if not pub.publish(az_t, el0):
                print("  publish failed at t=%.3f s (%s)" % (t, phase))
                break
            s = state()
            if guard_or_abort(s):
                command("stop_motion")
                print("  soft-limit guard tripped at t=%.2f s" % t)
                break
            nrm = world_to_norm(az_t, el0, s["q_yaw_rad"], s["q_pitch_rad"], args.rotate180)
            if nrm:
                (un, vn), (u, vv) = nrm
                au, av = s.get("target_aim_x_norm"), s.get("target_aim_y_norm")
                aim_err = (math.hypot(au * FW - CX, av * FH - CY)
                           if s.get("target_aim_valid") and isinstance(au, float) else None)
                lead = None
                qry, qrp = s.get("q_ref_yaw_rad"), s.get("q_ref_pitch_rad")
                if isinstance(qry, float) and isinstance(qrp, float):
                    az_ref, _el = base_to_los(axis_direction(qry, qrp))
                    # Signed by the direction of travel: positive means the reference is ahead.
                    lead = math.degrees(az_ref - az_t)
                rows.append({"t": t, "phase": phase, "aim_err": aim_err, "lead": lead,
                             "in_frame": (0.0 <= u <= FW) and (0.0 <= vv <= FH),
                             "outside": getattr(pub, "outside", False),
                             "track_state": s.get("track_state"), "v_yaw": s["v_yaw_rad_s"],
                             "ex": u - CX})
            time.sleep(0.033)

        pub.close()
        print("  published %d TrackSets, sampled %d" % (pub.published, len(rows)))
        dart_start = 1.5
        hold_rows = [r for r in rows if r["phase"] == "hold"]
        dart_rows = [r for r in rows if r["phase"] == "dart"]
        errs = sorted(r["aim_err"] for r in hold_rows if r["aim_err"] is not None)
        tracking_frac = (sum(1 for r in hold_rows if str(r["track_state"]) == "tracking")
                         / float(len(hold_rows))) if hold_rows else 0.0
        n_aim = len(errs)
        if n_aim < 30 or tracking_frac < 0.8:
            print("\n  S3 VERDICT: INVALID - not enough tracking data after the dart (n=%d live aim"
                  " samples, tracking in %.0f%% of hold samples)." % (n_aim, 100.0 * tracking_frac))
            print("          The dart happened; the measurement did not. Diagnose acquisition first.")
            command("clear_target")
            time.sleep(0.2)
            command("set_mode", "MANUAL")
            return 3

        def pct(vals, q):
            return vals[min(len(vals) - 1, int(q * len(vals)))] if vals else float("nan")

        c1 = not any((not r["in_frame"]) or r["outside"] for r in rows)
        back = next((r["t"] - dart_start for r in rows
                     if r["phase"] == "hold" and r["aim_err"] is not None and r["aim_err"] <= tol_px),
                    None)
        c2 = back is not None and back <= args.max_recovery_s
        leads = sorted(r["lead"] for r in dart_rows if r["lead"] is not None)
        lead_p50 = pct(leads, .5)
        c3 = bool(leads) and lead_p50 > 0.0
        signs = [1 if r["ex"] > 0 else -1 for r in hold_rows if abs(r["ex"]) > 1.0]
        flips = sum(1 for a, b in zip(signs, signs[1:]) if a != b)
        c4 = flips <= 2
        print("\n  S3 result:")
        print("    reference lead during the dart (positive = ahead of the target in travel dir):")
        print("      n=%d  p50 %+.3f deg  min %+.3f  max %+.3f  ahead in %.0f%% of samples"
              % (len(leads), lead_p50, leads[0] if leads else float("nan"),
                 leads[-1] if leads else float("nan"),
                 (100.0 * sum(1 for x in leads if x > 0) / len(leads)) if leads else 0.0))
        print("    hold-window aim error: p50 %.1f px / p95 %.1f px (%.3f / %.3f of box height)"
              % (pct(errs, .5), pct(errs, .95), pct(errs, .5) / box_h_px, pct(errs, .95) / box_h_px))
        print("    back inside tolerance at t=%s s after the dart; peak |yaw rate| %.1f deg/s"
              % ("%.2f" % back if back is not None else "NEVER",
                 max((abs(r["v_yaw"]) for r in rows), default=0.0) * 180 / math.pi))
        print("    aim-error sign changes after arrival: %d" % flips)
        print("    C1 containment : %s" % ("PASS" if c1 else "FAIL - the target left the frame"))
        print("    C2 recovery    : %s%s" % ("PASS" if c2 else "FAIL",
              "" if back is None else "  (t=%.2f s, bar %.2f s)" % (back, args.max_recovery_s)))
        print("    C3 leads       : %s (p50 lead %+.3f deg)"
              % ("PASS" if c3 else ("FAIL" if leads else "UNMEASURABLE - no q_ref published"),
                 lead_p50))
        print("    C4 no ringing  : %s (%d sign changes)" % ("PASS" if c4 else "FAIL", flips))
        command("clear_target")
        time.sleep(0.2)
        command("set_mode", "MANUAL")
        return 0 if (c1 and c2 and c3 and c4) else 1

    # ---- S1: one world-fixed target, 12 deg off the current axis, and let the controller work ----
    s = wait_settled(state())   # a world target only means something if the pose it came from is still
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
