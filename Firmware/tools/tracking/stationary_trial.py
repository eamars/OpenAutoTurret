"""ADR-003 3b on the station: acquire and hold a stationary object through production's own chain.

Runs on the station beside the live stack and drives it only through the operator's web commands
(set_mode, manual_step, select_target), so selection, modes, the reference chain, Level 1 and the
ADR-002.2 servos are production's own. Per trial: MANUAL; step the turret off the object with
manual_step (5 deg steps); find the object again; select it; AUTO_TRACK; hold; back to MANUAL.
State is recorded at RATE_HZ with the station's monotonic receive time, beside controld's per-tick
tracking trace (OTA_TRACKING_TRACE), which carries the same clock.

    stationary_trial.py OUT.jsonl CLASS TRIALS [--hold S] [--rate HZ] [--block NAME]

TRIALS is a comma list of offsets such as yaw+10,yaw-10,pitch+5. The object is the CLASS track
nearest the image centre at the start; after each offset it is found again by uuid, or failing
that by class nearest to where the offset should have put it. A pitch aim within PITCH_MARGIN of a
soft limit refuses the trial (the pitch axis has end stops; owner ruling 2026-10-02).
"""
import argparse
import json
import math
import os
import time
import urllib.request

PITCH_MARGIN = math.radians(5.0)
SETTLE_SPEED = math.radians(0.3)      # rad/s: "stopped" for step completion
STEP_DEG = 5.0                        # manual_step's largest sanctioned size

run_dir = f"/tmp/ota-stack-{os.getuid()}"
port = open(os.path.join(run_dir, "web.port"), encoding="utf-8").read().strip()
BASE = f"http://127.0.0.1:{port}"


def state():
    with urllib.request.urlopen(BASE + "/api/state", timeout=1.0) as r:
        return json.loads(r.read())


def command(name, arg=""):
    body = json.dumps({"command": name, "arg": arg}).encode()
    req = urllib.request.Request(BASE + "/api/command", data=body, headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=2.0) as r:
        return json.loads(r.read())


class Recorder:
    def __init__(self, path, rate, block):
        self.f = open(path, "a", encoding="utf-8")
        self.period = 1.0 / rate
        self.block = block

    def mark(self, **fields):
        self.f.write(json.dumps({"rx_ns": time.monotonic_ns(), "block": self.block, "mark": fields}) + "\n")
        self.f.flush()

    def record(self, seconds, trial):
        """Record state for `seconds`; returns the last state."""
        end = time.monotonic() + seconds
        nxt = time.monotonic()
        s = None
        while time.monotonic() < end:
            nxt += self.period
            try:
                s = state()
                self.f.write(json.dumps({"rx_ns": time.monotonic_ns(), "block": self.block, "trial": trial, "state": s}) + "\n")
            except Exception as error:  # noqa: BLE001 - a gap is evidence too
                self.f.write(json.dumps({"rx_ns": time.monotonic_ns(), "block": self.block, "trial": trial, "error": str(error)}) + "\n")
            self.f.flush()
            time.sleep(max(0.0, nxt - time.monotonic()))
        return s


def wait_for(pred, timeout, what):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        s = state()
        if pred(s):
            return s
        time.sleep(0.1)
    raise RuntimeError(f"timed out waiting for {what}")


def settled(s):
    v = [s.get("v_yaw_rad_s"), s.get("v_pitch_rad_s")]
    return all(x is not None and abs(x) < SETTLE_SPEED for x in v)


def wait_settled(timeout=15.0):
    """Stopped for 0.5 s in a row."""
    end = time.monotonic() + timeout
    calm_since = None
    while time.monotonic() < end:
        s = state()
        if s.get("fault"):
            raise RuntimeError(f"station fault: {s['fault']}")
        if settled(s):
            calm_since = calm_since or time.monotonic()
            if time.monotonic() - calm_since >= 0.5:
                return s
        else:
            calm_since = None
        time.sleep(0.05)
    raise RuntimeError("axes did not settle")


def to_manual():
    command("set_mode", "MANUAL")
    wait_for(lambda s: s.get("operating_mode") == "MANUAL", 5.0, "MANUAL")
    return wait_settled()


def step_off(axis, degrees):
    """Offset the turret by `degrees` on `axis` in manual_step increments."""
    n = int(round(abs(degrees) / STEP_DEG))
    sign = "+" if degrees > 0 else "-"
    for _ in range(n):
        ack = command("manual_step", f"{axis}{sign}{STEP_DEG:g}")
        if not ack.get("ok", True) and ack.get("verdict") == "rejected":
            raise RuntimeError(f"manual_step refused: {ack}")
        time.sleep(0.2)
        wait_settled()


def tracks(s, cls):
    return [t for t in s.get("tracks", []) if t.get("class_name") == cls and t.get("selectable")]


def find(s, cls, uuid, expect_x, expect_y):
    candidates = tracks(s, cls)
    for t in candidates:
        if t["uuid"] == uuid:
            return t
    if not candidates:
        return None
    return min(candidates, key=lambda t: (t["anchor_x"] - expect_x) ** 2 + (t["anchor_y"] - expect_y) ** 2)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("out")
    ap.add_argument("cls")
    ap.add_argument("trials")
    ap.add_argument("--hold", type=float, default=30.0)
    ap.add_argument("--rate", type=float, default=10.0)
    ap.add_argument("--block", default="A")
    args = ap.parse_args()
    rec = Recorder(args.out, args.rate, args.block)

    s = to_manual()
    hfov = math.radians(s.get("effective_hfov_deg") or 69.3)
    vfov = math.radians(s.get("effective_vfov_deg") or 40.4)
    seed = tracks(s, args.cls)
    if not seed:
        raise SystemExit(f"no selectable {args.cls} in view")
    target = min(seed, key=lambda t: (t["anchor_x"] - .5) ** 2 + (t["anchor_y"] - .5) ** 2)
    uuid = target["uuid"]
    rec.mark(event="target", uuid=uuid, cls=args.cls, anchor=[target["anchor_x"], target["anchor_y"]],
             q=[s["q_yaw_rad"], s["q_pitch_rad"]])

    for k, spec in enumerate(args.trials.split(",")):
        axis = "".join(c for c in spec if c.isalpha())
        offset = float(spec[len(axis):])
        trial = f"{args.block}{k:02d}-{spec}"
        s = to_manual()
        here = find(s, args.cls, uuid, .5, .5)
        if here is None:
            rec.mark(event="skip", trial=trial, reason="target not in view before the offset")
            continue
        uuid = here["uuid"]
        # Where the offset should put it in the image (small-angle; the anchor is normalised).
        dx = -math.radians(offset) / hfov if axis == "yaw" else 0.0     # yaw+ aims left: target moves right
        dy = math.radians(offset) / vfov if axis == "pitch" else 0.0
        expect = (here["anchor_x"] - dx, here["anchor_y"] + dy)
        rec.mark(event="offset", trial=trial, axis=axis, deg=offset, before=[s["q_yaw_rad"], s["q_pitch_rad"]])
        step_off(axis, offset)
        time.sleep(1.0)  # let detection and the track list catch up at rest
        s = state()
        t = find(s, args.cls, uuid, *expect)
        if t is None:
            rec.mark(event="skip", trial=trial, reason="target lost after the offset")
            continue
        # The pitch aim this would ask for, from the image offset: refuse near a soft limit.
        pitch_goal = s["q_pitch_rad"] - (t["anchor_y"] - .5) * vfov
        lo, hi = s.get("q_soft_min_pitch_rad"), s.get("q_soft_max_pitch_rad")
        if lo is None or hi is None or not (lo + PITCH_MARGIN <= pitch_goal <= hi - PITCH_MARGIN):
            rec.mark(event="skip", trial=trial, reason="pitch aim near a soft limit", pitch_goal=pitch_goal, limits=[lo, hi])
            continue
        uuid = t["uuid"]
        rec.mark(event="select", trial=trial, uuid=uuid, display_index=t["display_index"],
                 anchor=[t["anchor_x"], t["anchor_y"]], q=[s["q_yaw_rad"], s["q_pitch_rad"]])
        command("select_target", str(t["display_index"]))
        command("set_mode", "AUTO_TRACK")
        rec.mark(event="track", trial=trial)
        last = rec.record(args.hold, trial)
        rec.mark(event="end", trial=trial, fault=(last or {}).get("fault"))
        if last and last.get("fault"):
            raise SystemExit(f"station fault during {trial}: {last['fault']}")
    to_manual()
    rec.mark(event="done")


if __name__ == "__main__":
    main()
