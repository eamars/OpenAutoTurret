#!/usr/bin/env python3
"""Find the one-shot actuator's REST (A) and ACTIVE (B) pulse widths, interactively, on the station.

The actuator is a hobby servo on GPIO15 (physical pin 10, owner 2026-10-30): it rests at A, and on
ACTIVATE moves to B, waits the dwell, and returns to A. This is the quick tool that finds A, B and the
dwell and saves them to the station's state directory; controld will read the same file once it owns
the servo, and then this tool must not run beside it.

    ssh -t eamars@rpi-turret python3 ~/actuator_calibrate.py            # a real terminal: keys below
    ssh -t eamars@rpi-turret python3 ~/actuator_calibrate.py --start 1500

Keys:
  a / d     pulse -step / +step
  1 2 3     step size 50 / 10 / 2 us (starts at 10)
  r         mark REST (A) at the current pulse
  b         mark ACTIVE (B) at the current pulse
  w         type the dwell, in ms
  g         go to REST
  t         test: REST -> ACTIVE -> dwell -> REST, with the marks so far
  s         save and quit
  q         quit without saving

With the servo on its mechanism, approach an end stop in small steps and back off as soon as the horn
stops moving or the motor buzzes: a stalled servo draws its stall current and strips its gears. The
tool never commands outside 500..2500 us.

Hardware PWM needs the owner's overlay (`sudo dtoverlay pwm pin=15 func=4`, lost at reboot); the
overlay muxes the pin to the wrong function on the Pi 5, so this tool sets it to PWM itself with
`pinctrl` (no sudo). It ends as it found the channel: on exit it returns to REST and, if the channel
was off when it started, switches it off again. Standard library only.
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import termios
import time
import tty
from datetime import datetime
from pathlib import Path

PWM0_ADDRESS = "1f00098000"  # RP1 PWM0, the block GPIO12/13/14/15/18/19 reach
# GPIO -> (pinctrl function that is PWM0, PWM0 channel), from `pinctrl funcs` on the station.
PWM0_PINS = {12: ("a0", 0), 13: ("a0", 1), 14: ("a0", 2), 15: ("a0", 3), 18: ("a3", 2), 19: ("a3", 3)}
PERIOD_NS = 20_000_000  # 50 Hz
HARD_MIN_US, HARD_MAX_US = 500, 2500
DEFAULT_STATE = Path.home() / "workspace/OpenAutoTurret/run/state"


class Channel:
    """One RP1 PWM0 channel through sysfs. period is written before duty_cycle, always."""

    def __init__(self, gpio: int):
        if gpio not in PWM0_PINS:
            sys.exit(f"GPIO{gpio} has no RP1 PWM0 channel (PWM0 pins: {sorted(PWM0_PINS)})")
        self.gpio = gpio
        func, self.index = PWM0_PINS[gpio]
        chips = [c for c in Path("/sys/class/pwm").glob("pwmchip*") if PWM0_ADDRESS in str(c.resolve())]
        if not chips:
            sys.exit(f"no PWM chip at {PWM0_ADDRESS}: the overlay is not loaded. The owner runs\n"
                     f"    sudo dtoverlay pwm pin={gpio} func=4")
        self.chip = chips[0]
        self.dir = self.chip / f"pwm{self.index}"
        self.exported_here = not self.dir.exists()
        if self.exported_here:
            (self.chip / "export").write_text(str(self.index))
        for _ in range(30):  # udev makes the files writable a moment after the export
            if os.access(self.dir / "period", os.W_OK):
                break
            time.sleep(0.1)
        else:
            sys.exit(f"{self.dir}/period is not writable (is {os.environ.get('USER', 'this user')} in the gpio group?)")
        self.was_enabled = self._read("enable") == "1"
        subprocess.run(["pinctrl", "set", str(gpio), func], check=True)
        mux = subprocess.run(["pinctrl", "get", str(gpio)], capture_output=True, text=True).stdout.strip()
        if f"PWM0_CHAN{self.index}" not in mux:
            sys.exit(f"GPIO{gpio} is not on PWM0 after pinctrl: {mux}")
        print(f"{self.dir}  GPIO{gpio}: {mux}")

    def _read(self, name: str) -> str:
        return (self.dir / name).read_text().strip()

    def set_us(self, us: float) -> None:
        us = max(HARD_MIN_US, min(HARD_MAX_US, us))
        if self._read("period") != str(PERIOD_NS):
            (self.dir / "period").write_text(str(PERIOD_NS))
        (self.dir / "duty_cycle").write_text(str(int(round(us * 1000))))
        if self._read("enable") != "1":
            (self.dir / "enable").write_text("1")

    def close(self) -> None:
        if not self.was_enabled:
            (self.dir / "enable").write_text("0")
        if self.exported_here:
            (self.chip / "unexport").write_text(str(self.index))


def getkey() -> str:
    """One key, unbuffered, from a terminal; from a pipe, the next character (for scripted checks)."""
    if not sys.stdin.isatty():
        return sys.stdin.read(1) or "q"
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def ask_number(prompt: str, default: float) -> float:
    raw = input(f"\n{prompt} [{default:g}]: ").strip()
    try:
        return float(raw) if raw else default
    except ValueError:
        print("not a number; kept", default)
        return default


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--gpio", type=int, default=15)
    ap.add_argument("--state", type=Path, default=DEFAULT_STATE, help="the station's state directory")
    ap.add_argument("--start", type=float, help="first pulse, us (default: saved REST, else 1500)")
    args = ap.parse_args()

    path = args.state / "actuator.json"
    saved = json.loads(path.read_text()) if path.exists() else {}
    if saved:
        print(f"saved {path}: {saved}")
    rest, active = saved.get("rest_us"), saved.get("active_us")
    dwell_ms = float(saved.get("dwell_ms", 300))
    us = float(args.start if args.start is not None else rest if rest is not None else 1500)
    step = 10

    ch = Channel(args.gpio)
    print(__doc__.split("Keys:")[1].split("With the servo")[0])
    print(f"first pulse {us:.0f} us: the servo moves there from wherever it is")
    ch.set_us(us)
    ok = False
    try:
        while True:
            print(f"\rpulse {us:6.0f} us  step {step:2d}  REST {rest}  ACTIVE {active}  dwell {dwell_ms:g} ms   ",
                  end="", flush=True)
            k = getkey()
            if k in ("a", "d"):
                us = max(HARD_MIN_US, min(HARD_MAX_US, us + (step if k == "d" else -step)))
                ch.set_us(us)
            elif k in ("1", "2", "3"):
                step = {"1": 50, "2": 10, "3": 2}[k]
            elif k == "r":
                rest = round(us)
            elif k == "b":
                active = round(us)
            elif k == "w":
                dwell_ms = max(0.0, ask_number("dwell at ACTIVE, ms", dwell_ms))
            elif k == "g" and rest is not None:
                us = float(rest)
                ch.set_us(us)
            elif k == "t":
                if rest is None or active is None:
                    print("\nmark REST (r) and ACTIVE (b) first")
                    continue
                ch.set_us(active)
                time.sleep(dwell_ms / 1000.0)
                ch.set_us(rest)
                us = float(rest)
            elif k == "s":
                if rest is None or active is None:
                    print("\nmark REST (r) and ACTIVE (b) first")
                    continue
                record = {"schema": 1, "gpio": args.gpio, "period_us": PERIOD_NS // 1000,
                          "rest_us": rest, "active_us": active, "dwell_ms": dwell_ms,
                          "saved": datetime.now().astimezone().isoformat(timespec="seconds")}
                args.state.mkdir(parents=True, exist_ok=True)
                tmp = path.with_suffix(".json.tmp")
                tmp.write_text(json.dumps(record, indent=2) + "\n")
                tmp.replace(path)
                print(f"\nsaved {path}: {record}")
                ok = True
                break
            elif k == "q":
                break
    except KeyboardInterrupt:
        pass
    finally:
        print()
        if rest is not None:
            ch.set_us(rest)
            time.sleep(0.5)
        ch.close()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
