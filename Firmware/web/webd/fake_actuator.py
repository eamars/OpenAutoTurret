"""A stand-in for controld's one-shot actuator (ADR-007), for the page's tests and the synthetic web.

It answers the actuator's commands the way `control/src/actuator/actuator.cpp` and
`ControlLoop::execute_actuator_command` do (the same refusals, the same states, the same telemetry
object), with the shot's time kept by a thread instead of a servo. No hardware: nothing here opens a
device. What it cannot imitate is the servo itself; the station check is the real one.
"""
from __future__ import annotations

import threading
import time
from typing import Callable, Optional, Tuple

MIN_US, MAX_US, MAX_DWELL_MS = 500, 2500, 5000.0


def _values_refusal(rest: int, active: int, dwell: float) -> str:
    for us in (rest, active):
        if us < MIN_US or us > MAX_US:
            return f"pulse {us} us is outside {MIN_US}..{MAX_US} us"
    if not (0 <= dwell <= MAX_DWELL_MS):
        return "dwell must be 0..5000 ms"
    return ""


class FakeActuator:
    """controld's actuator, minus the PWM. `on_change` is called with the new telemetry object."""

    def __init__(self, rest_us: Optional[int] = 1650, active_us: Optional[int] = 1450,
                 dwell_ms: float = 300.0, available: bool = True, reason: str = "",
                 settle_ms: int = 500, on_change: Optional[Callable[[dict], None]] = None) -> None:
        self._lock = threading.Lock()
        self._settle = settle_ms / 1000.0
        self._on_change = on_change
        self.configured = rest_us is not None and active_us is not None
        self.rest, self.active, self.dwell = rest_us or 0, active_us or 0, dwell_ms
        self.available = available
        self.reason = reason if not available else ("" if self.configured else
                                                     "no actuator.json yet: MENU > CALIBRATION > ACTUATOR finds A and B")
        self.state = "off" if available and self.configured else ("unconfigured" if available else "unavailable")
        self.pulse: Optional[int] = None
        self.shots = 0
        self.saved = -1
        self._gen = 0                 # cancels a pending timer when the state moves on
        self._back_to_cal = False
        self._shot_rest = self.rest

    # -- what the station would publish ---------------------------------------------------------
    def status(self) -> dict:
        with self._lock:
            return self._status_locked()

    def _status_locked(self) -> dict:
        powered = self.state in ("rest", "returning", "active", "calibrating", "stopping")
        return {"available": self.state != "unavailable", "configured": self.configured, "state": self.state,
                "reason": self.reason, "gpio": 15, "powered": powered,
                "pulse_us": self.pulse if powered else None,
                "rest_us": self.rest if self.configured else None,
                "active_us": self.active if self.configured else None,
                "dwell_ms": self.dwell if self.configured else None,
                "hold_at_rest": False, "shots": self.shots, "persistent": True, "saved": self.saved}

    def _changed_locked(self) -> None:
        if self._on_change:
            self._on_change(self._status_locked())

    def _after(self, seconds: float, fn: Callable[[], None]) -> None:
        self._gen += 1
        gen = self._gen

        def run() -> None:
            time.sleep(seconds)
            with self._lock:
                if gen == self._gen:
                    fn()
                    self._changed_locked()
        threading.Thread(target=run, daemon=True).start()

    def _settle_to(self, us: int) -> None:
        self.state, self.pulse = "returning", us

        def done() -> None:
            if self._back_to_cal:
                self._back_to_cal = False
                self.state = "calibrating"
            else:
                self.state, self.pulse = "off", None
        self._after(self._settle, done)

    def _shot(self, active: int, rest: int, dwell: float, back_to_cal: bool) -> None:
        self.state, self.pulse, self._back_to_cal, self._shot_rest = "active", active, back_to_cal, rest
        self._after(dwell / 1000.0, lambda: self._settle_to(rest))

    # -- commands, as controld answers them: (accepted, reason or what was done) ----------------
    def handle(self, command: str, arg: str, faulted: bool = False) -> Tuple[bool, str]:
        with self._lock:
            ok, why = self._handle_locked(command, arg, faulted)
            self._changed_locked()
            return ok, why

    def _handle_locked(self, command: str, arg: str, faulted: bool) -> Tuple[bool, str]:
        words = arg.split()
        if command == "stop_motion":
            if self.state == "active":
                self._settle_to(self._shot_rest)  # a test shot's own rest, as controld does
            return True, ""
        closing = command == "actuator_calibration" and words[:1] == ["stop"]
        if faulted and not closing:
            return False, "faulted: no actuation beside a fault; HOME recovers first"
        if command == "actuate":
            if self.state == "unavailable":
                return False, "actuator unavailable: " + self.reason
            if self.state == "unconfigured":
                return False, "actuator not calibrated: " + self.reason
            if self.state == "calibrating":
                return False, "the actuator's calibration is open"
            if self.state == "active":
                return False, "shot in progress"
            if self.state == "returning":
                return False, "the servo is still returning to rest"
            self.shots += 1
            self._shot(self.active, self.rest, self.dwell, False)
            return True, "fired"
        if command == "actuator_calibration":
            if closing:
                if self.state not in ("calibrating",) and not self._back_to_cal:
                    return True, "actuator calibration closed; back to rest"
                self._back_to_cal = False
                if self.configured:
                    self._settle_to(self.rest)
                else:
                    self.state, self.pulse = "unconfigured", None
                    self._gen += 1
                return True, "actuator calibration closed; back to rest"
            if self.state == "unavailable":
                return False, "actuator unavailable: " + self.reason
            if self.state == "calibrating":
                return True, "actuator calibration open"
            if self.state in ("active", "returning"):
                return False, "wait: the servo is moving"
            start = int(words[1]) if len(words) > 1 else (self.rest if self.configured else 1500)
            if not MIN_US <= start <= MAX_US:
                return False, _values_refusal(start, start, 0)
            self.state, self.pulse = "calibrating", start
            self._gen += 1
            return True, "actuator calibration open"
        if command == "actuator_pulse":
            if self.state != "calibrating":
                return False, "the actuator's calibration is not open"
            us = int(words[0])
            if not MIN_US <= us <= MAX_US:
                return False, _values_refusal(us, us, 0)
            self.pulse = us
            return True, f"pulse {us} us"
        if command in ("actuator_test", "set_actuator"):
            if self.state != "calibrating":
                return False, ("test in progress" if command == "actuator_test" and self._back_to_cal
                               else "the actuator's calibration is not open")
            rest, active, dwell = int(words[0]), int(words[1]), float(words[2])
            why = _values_refusal(rest, active, dwell)
            if why:
                return False, why
            if command == "actuator_test":
                self._shot(active, rest, dwell, True)
                return True, "test shot"
            self.rest, self.active, self.dwell, self.configured = rest, active, dwell, True
            self.reason, self.saved = "", 1
            return True, f"saved: rest {rest} us, active {active} us"
        return False, "not an actuator command"


ACTUATOR_COMMANDS = ("actuate", "actuator_calibration", "actuator_pulse", "actuator_test", "set_actuator")
