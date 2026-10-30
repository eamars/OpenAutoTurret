"""ADR-007 D8: the one-shot actuator on the page -- the ACTUATE button and MENU › CALIBRATION's
ACTUATOR section. What each control sends, when it is greyed and why, and that the button keeps its
place whether or not the D-pad shows (owner, 2026-10-30).

The builders are pure functions in the page's own JS, executed under node from the bytes the browser
receives. The fake actuator the synthetic web runs on is checked against controld's refusals too, so a
page tested on it is tested against the station's answers.
"""
from __future__ import annotations

import json
import os
import re
import shutil
import subprocess
import tempfile
import time
import unittest

from ..fake_actuator import FakeActuator
from ..hud import HUD_CSS, HUD_GEOMETRY_JS, HUD_HTML, HUD_JS

ACT = {"available": True, "configured": True, "state": "off", "reason": "", "gpio": 15, "powered": False,
       "pulse_us": None, "rest_us": 1650, "active_us": 1450, "dwell_ms": 300, "hold_at_rest": False,
       "shots": 0, "persistent": True, "saved": -1}


def station(**actuator) -> dict:
    return {"phase": "idle", "operating_mode": "MANUAL", "actuator": dict(ACT, **actuator)}


@unittest.skipUnless(shutil.which("node"), "node not installed; the builders cannot be executed")
class ActuatorBuilders(unittest.TestCase):
    maxDiff = None

    def setUp(self) -> None:
        self._geo = tempfile.NamedTemporaryFile("w", suffix=".js", delete=False)
        self._geo.write(HUD_GEOMETRY_JS + "\nmodule.exports = { hudActuateButton, hudCalibrationActuator, "
                        "hudActuatorSession };\n")
        self._geo.close()

    def tearDown(self) -> None:
        os.unlink(self._geo.name)

    def _node(self, expr: str):
        with tempfile.NamedTemporaryFile("w", suffix=".js", delete=False) as fh:
            fh.write("const T = require(%r);\nconsole.log(JSON.stringify(%s));\n" % (self._geo.name, expr))
            main = fh.name
        try:
            r = subprocess.run(["node", main], capture_output=True, text=True, timeout=30)
        finally:
            os.unlink(main)
        self.assertEqual(r.returncode, 0, r.stderr)
        return json.loads(r.stdout)

    def button(self, t: dict):
        return self._node("T.hudActuateButton(%s)" % json.dumps(t))

    def section(self, t: dict, ui: dict | None = None):
        return self._node("T.hudCalibrationActuator(%s, %s)" % (json.dumps(t), json.dumps(ui or {})))

    # -- the button ----------------------------------------------------------------------------

    def test_no_actuator_no_button(self) -> None:
        self.assertIsNone(self.button({"phase": "idle"}))
        self.assertIsNone(self.section({"phase": "idle"}))

    def test_it_fires_in_shutdown_and_when_homed(self) -> None:
        # Owner, 2026-10-30: in any state but a fault.
        for t in (station(), dict(station(), phase="hold", operating_mode="AUTO_ROAM")):
            b = self.button(t)
            self.assertEqual((b["command"], b["arg"], b["note"]), ("actuate", "", ""), t)

    def test_it_is_greyed_with_the_reason_whenever_controld_would_refuse(self) -> None:
        cases = [
            (station(available=False, state="unavailable", reason="no PWM chip"), "UNAVAILABLE"),
            (station(configured=False, state="unconfigured"), "NOT CALIBRATED"),
            (dict(station(), phase="fault"), "FAULT"),
            (dict(station(), phase="recovering"), "RECOVERING"),
            (dict(station(), calibrating=True), "CALIBRATING"),
            (station(state="calibrating", powered=True, pulse_us=1600), "IN CALIBRATION"),
            (station(state="active", powered=True, pulse_us=1450), "FIRING"),
            (station(state="returning", powered=True, pulse_us=1650), "RETURNING"),
            (station(state="stopping"), "STOPPING"),
        ]
        for t, why in cases:
            b = self.button(t)
            self.assertIsNone(b["command"], why)
            self.assertEqual(b["note"], why)
        self.assertEqual(self.button(cases[0][0])["title"], "no PWM chip", "controld's reason is the tooltip")
        self.assertTrue(self.button(dict((why, t) for t, why in cases)["FIRING"])["firing"])

    # -- the section, closed ---------------------------------------------------------------------

    def test_adjust_opens_controlds_session(self) -> None:
        s = self.section(station())
        self.assertFalse(s["open"])
        self.assertEqual(s["saved"], "A 1650 · B 1450 µs · 300 ms")
        self.assertEqual((s["adjust"]["command"], s["adjust"]["arg"]), ("actuator_calibration", "start"))
        self.assertIn("A (1650 µs)", s["adjust"]["note"])
        # Never calibrated: it can still be calibrated, starting at the servo's centre.
        s = self.section(station(configured=False, state="unconfigured", reason="no actuator.json yet"))
        self.assertEqual(s["adjust"]["command"], "actuator_calibration")
        self.assertIn("1500 µs", s["adjust"]["note"])
        self.assertEqual(s["warn"], "no actuator.json yet")

    def test_adjust_is_refused_where_controld_refuses_it(self) -> None:
        for t, why in [(station(available=False, state="unavailable", reason="no PWM chip"), "no PWM chip"),
                       (dict(station(), phase="fault"), "faulted"),
                       (dict(station(), calibrating=True), "calibration run"),
                       (station(state="active", powered=True, pulse_us=1450), "shot is running")]:
            a = self.section(t)["adjust"]
            self.assertIsNone(a["command"])
            self.assertIn(why, a["note"])

    # -- the section, open ---------------------------------------------------------------------

    def test_the_pulse_steps_from_what_the_page_last_sent_within_the_bounds(self) -> None:
        open_ = station(state="calibrating", powered=True, pulse_us=1650)
        s = self.section(open_, {"open": True, "step": 50, "pulse": 1700})
        self.assertEqual((s["pulse"]["down"], s["pulse"]["up"]), ("1650", "1750"),
                         "a second press before the station's next frame must not resend the same value")
        s = self.section(open_, {"open": True, "step": 50, "pulse": 2480})
        self.assertEqual(s["pulse"]["up"], "2500")
        s = self.section(open_, {"open": True, "step": 10, "pulse": 2500})
        self.assertIsNone(s["pulse"]["up"])
        s = self.section(open_, {"open": True, "step": 7})
        self.assertEqual([c["label"] for c in s["steps"] if c["on"]], ["10"], "an unknown step is the default")
        self.assertEqual(s["pulse"]["down"], "1640", "the station's pulse until the page has sent one")

    def test_test_and_save_send_the_marks_and_save_only_a_change(self) -> None:
        open_ = station(state="calibrating", powered=True, pulse_us=1650)
        s = self.section(open_, {"open": True})
        self.assertEqual((s["rest"]["text"], s["active"]["text"]), ("1650 µs", "1450 µs"), "marks start at the saved")
        self.assertEqual((s["test"]["command"], s["test"]["arg"]), ("actuator_test", "1650 1450 300"))
        self.assertIsNone(s["save"]["command"], "nothing new to save")
        s = self.section(open_, {"open": True, "rest": 1670, "dwell": 350})
        self.assertEqual((s["save"]["command"], s["save"]["arg"]), ("set_actuator", "1670 1450 350"))
        self.assertIn("unsaved", s["done"]["note"])
        self.assertEqual((s["done"]["command"], s["done"]["arg"]), ("actuator_calibration", "stop"))

    def test_a_first_calibration_needs_both_marks(self) -> None:
        open_ = station(configured=False, state="calibrating", powered=True, pulse_us=1500,
                        rest_us=None, active_us=None, dwell_ms=None)
        s = self.section(open_, {"open": True})
        self.assertEqual((s["rest"]["text"], s["active"]["text"]), ("—", "—"))
        self.assertIsNone(s["test"]["command"])
        self.assertIsNone(s["save"]["command"])
        s = self.section(open_, {"open": True, "rest": 1600, "active": 1400})
        self.assertEqual(s["save"]["arg"], "1600 1400 300", "the tool's default dwell")

    def test_during_a_test_shot_nothing_else_is_sent(self) -> None:
        shot = station(state="active", powered=True, pulse_us=1450)
        s = self.section(shot, {"open": True, "pulse": 1650})
        self.assertTrue(s["open"] and s["testing"])
        self.assertIsNone(s["pulse"]["up"])
        self.assertIsNone(s["test"]["command"])
        self.assertIsNone(s["save"]["command"])
        self.assertIsNone(s["rest"]["ui"])
        self.assertEqual(s["done"]["command"], "actuator_calibration", "DONE stays: it ends the test too")
        # Another page's ordinary shot is not this page's session.
        self.assertFalse(self.section(shot, {"open": False})["open"])
        self.assertTrue(self._node("T.hudActuatorSession(%s, false)" % json.dumps(
            station(state="calibrating", powered=True, pulse_us=1600))), "a session opened elsewhere shows")


class ActuatorPageStructure(unittest.TestCase):
    def test_the_button_has_its_own_place_outside_the_d_pad(self) -> None:
        pad = re.search(r'<div id="manual-pad".*?</div>', HUD_HTML, re.S).group(0)
        self.assertNotIn("actuate", pad, "inside the pad it would vanish with it")
        self.assertIn('<div id="actuate-slot" hidden>', HUD_HTML)
        slot = re.search(r"#actuate-slot \{([^}]*)\}", HUD_CSS).group(1)
        padcss = re.search(r"#manual-pad \{([^}]*)\}", HUD_CSS).group(1)
        self.assertIn("left:24px", slot)
        self.assertIn("left:24px", padcss)
        self.assertIn("top:calc(50% - 95px)", padcss, "the slot's offset below is computed from this")
        self.assertIn("top:calc(50% + 85px)", slot)
        self.assertIn("width:176px", slot)

    def test_leaving_calibration_closes_the_actuators_session(self) -> None:
        leave = re.search(r"function leaveCalibration\(\) \{.*?\n\}", HUD_JS, re.S).group(0)
        self.assertIn('actuatorRequest("actuator_calibration", "stop")', leave)
        # Second review, 2026-10-30: a session this page did not open (a reload, another tab), or one
        # mid-test, must close too: the station's own word, with a test shot counted as the session.
        self.assertIn("hudActuatorSession(lastTelemetry, true)", leave)

    def test_a_page_that_goes_away_closes_its_session(self) -> None:
        hide = re.search(r'window\.addEventListener\("pagehide".*?\n\}\);', HUD_JS, re.S).group(0)
        self.assertIn("keepalive: true", hide)
        self.assertIn('"actuator_calibration", arg: "stop"', hide)

    def test_after_a_test_the_pulse_is_the_stations_again(self) -> None:
        # Second review: the page kept the B it last sent, and "+" stepped from there.
        sync = re.search(r"function syncCalAct\(t\) \{.*?\n\}", HUD_JS, re.S).group(0)
        self.assertIn('if (a.state !== "calibrating") calAct.pulse = null;', sync)

    def test_actuator_commands_go_in_order_and_a_tap_under_the_button_is_not_a_zero(self) -> None:
        self.assertIn("if (actuatorCmd) actuatorRequest(cmd", HUD_JS)
        self.assertIn("#manual-pad, #actuate-slot, #dock", HUD_JS)

    def test_its_refusals_reach_the_calibration_panel(self) -> None:
        for c in ("actuator_calibration", "actuator_pulse", "actuator_test", "set_actuator"):
            self.assertIn('"%s"' % c, re.search(r"const HUD_CAL_COMMANDS = \[.*?\];", HUD_GEOMETRY_JS, re.S).group(0))


class FakeActuatorAnswersLikeControld(unittest.TestCase):
    """The synthetic web's actuator: the same refusals as control/src/actuator/actuator.cpp."""

    def test_a_shot_refuses_a_second_and_ends_off(self) -> None:
        a = FakeActuator(dwell_ms=50, settle_ms=50)
        self.assertEqual(a.handle("actuate", ""), (True, "fired"))
        self.assertEqual(a.handle("actuate", ""), (False, "shot in progress"))
        for _ in range(100):
            if a.status()["state"] == "off":
                break
            time.sleep(0.01)
        self.assertEqual(a.status()["state"], "off")
        self.assertEqual(a.status()["shots"], 1)

    def test_refusals(self) -> None:
        self.assertIn("not calibrated", FakeActuator(rest_us=None, active_us=None).handle("actuate", "")[1])
        self.assertIn("unavailable", FakeActuator(available=False, reason="x").handle("actuate", "")[1])
        self.assertIn("fault", FakeActuator().handle("actuate", "", faulted=True)[1])
        self.assertTrue(FakeActuator().handle("actuator_calibration", "stop", faulted=True)[0])
        a = FakeActuator()
        self.assertEqual(a.handle("actuator_pulse", "1600")[1], "the actuator's calibration is not open")
        a.handle("actuator_calibration", "start")
        self.assertEqual(a.handle("actuator_pulse", "400")[1], "pulse 400 us is outside 500..2500 us")
        self.assertEqual(a.handle("actuate", "")[1], "the actuator's calibration is open")
        self.assertFalse(a.handle("actuator_pulse", "abc")[0], "a malformed argument is refused, not raised")

    def test_a_first_calibration_closed_mid_test_returns_to_the_tests_rest(self) -> None:
        a = FakeActuator(rest_us=None, active_us=None, settle_ms=50)
        a.handle("actuator_calibration", "start 1600")
        a.handle("actuator_test", "1600 1400 3000")
        a.handle("actuator_calibration", "stop")
        self.assertEqual((a.status()["state"], a.status()["pulse_us"]), ("returning", 1600))
        for _ in range(100):
            if a.status()["state"] == "unconfigured":
                break
            time.sleep(0.01)
        self.assertEqual(a.status()["state"], "unconfigured")


if __name__ == "__main__":
    unittest.main()
