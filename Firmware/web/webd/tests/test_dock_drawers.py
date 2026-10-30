"""§13 dock and §14 drawers: what the buttons send, and what the page does with the answer.

The dock is the first part of this HUD that *acts*, so the tests are about actions rather than pixels.
Every command name and argument spelling was read out of the daemon's handlers before it appears here -
`select_target` takes a display index as a number, `manual_jog_start` takes yaw+/yaw-/pitch+/pitch-,
`manual_step` takes yaw+1, `set_mode` takes MANUAL/AUTO_TRACK/AUTO_ROAM, STOP MOTION is `hold` - because
a drawer that sends a plausible-looking wrong argument produces a refusal the operator has to decode,
and this project has already met a select_target argument mismatch once.

The command rows are built as data by a pure function, so "which commands will this drawer send" is
answerable under node with no browser and no socket.
"""

from __future__ import annotations

import json
import os
import pathlib
import re
import shutil
import subprocess
import tempfile
import time
import unittest

from fastapi.testclient import TestClient

from ..app import create_app
from ..controld_client import ControldClient
from ..config import WebConfig
from ..fake_controld import FakeControld
from ..hud import HUD_CSS, HUD_GEOMETRY_JS, HUD_HTML, HUD_JS
from . import fake_camera

TRACKS = [
    {"track_id": 3, "display_index": 3, "label": "person", "confidence": 0.82, "selected": True},
    {"track_id": 1, "display_index": 1, "label": "vehicle", "confidence": 0.44, "selected": False},
    {"track_id": 0, "display_index": 0, "label": "ghost", "confidence": 0.90, "selected": False},
]


@unittest.skipUnless(shutil.which("node"), "node not installed; the builders cannot be executed")
class DockAndDrawerBehaviour(unittest.TestCase):
    maxDiff = None

    def setUp(self) -> None:
        self._geo = tempfile.NamedTemporaryFile("w", suffix=".js", delete=False)
        self._geo.write(HUD_GEOMETRY_JS +
                        "\nmodule.exports = { hudDockSpecs, hudDrawerActions, hudDiagRows, hudTargetTypeChips,"
                        " otaJogForArrow, otaAxisArrow, hudRowAwaitsConfirm, hudConfirmKey,"
                        " hudCalibrationEntry, hudCalibrationPanel, hudCalibrationBore, hudBoreZeroName,"
                        " hudCalibrationRun, hudRerunKind, hudCalibrationLastRun, hudCalibrationZero, hudSettled,"
                        " hudUnproject, hudProject, hudCalibrationExit, hudCalibrationHoldStands,"
                        " hudCalibrationSessionStep, hudResumeMode };\n")
        self._geo.close()

    def tearDown(self) -> None:
        os.unlink(self._geo.name)

    def _node(self, script: str):
        with tempfile.NamedTemporaryFile("w", suffix=".js", delete=False) as fh:
            fh.write("const T = require(%r);\n%s" % (self._geo.name, script))
            main = fh.name
        try:
            r = subprocess.run(["node", main], capture_output=True, text=True, timeout=30)
        finally:
            os.unlink(main)
        self.assertEqual(r.returncode, 0, r.stderr)
        out = r.stdout.strip()
        try:
            return json.loads(out)
        except json.JSONDecodeError:
            return out

    def _chips(self, telemetry: dict):
        return self._node("console.log(JSON.stringify(T.hudTargetTypeChips(%s)));" % json.dumps(telemetry))

    def test_follow_chips_send_the_type_and_the_one_in_force_sends_nothing(self):
        # Owner, 2026-10-20: one type at a time, chosen at the head of the TARGETS drawer.
        spec = self._chips({"target_type": "HEAD", "target_type_switching": False})
        self.assertEqual(spec["label"], "FOLLOW")
        self.assertEqual([c["type"] for c in spec["chips"]], ["HUMAN", "HEAD", "CAT", "DOG"])
        by = {c["type"]: c for c in spec["chips"]}
        self.assertTrue(by["HEAD"]["current"])
        self.assertIsNone(by["HEAD"]["command"])
        self.assertEqual((by["CAT"]["command"], by["CAT"]["arg"]), ("set_target_type", "CAT"))
        self.assertEqual(spec["note"], "")

    def test_a_switch_in_progress_is_on_screen(self):
        spec = self._chips({"target_type": "CAT", "target_type_switching": True})
        self.assertTrue({c["type"]: c for c in spec["chips"]}["CAT"]["pending"])
        self.assertEqual(spec["note"], "SWITCHING TO CAT \u00b7 HOLDING")
        overdue = self._chips({"target_type": "CAT", "target_type_switching": True, "target_type_overdue": True})
        self.assertEqual(overdue["note"], "CAT NOT APPLIED \u00b7 HOLDING")

    def test_no_chips_from_a_controller_that_reports_no_type(self):
        self.assertIsNone(self._chips({"operating_mode": "MANUAL"}))

    def test_the_drawer_repaints_when_the_follow_chips_change(self):
        # Station 2026-10-20 23:04: CAT kept blinking after the switch ended, because the drawer's
        # repaint key held nothing the FOLLOW row draws. The key must include what the drawer shows.
        paint = HUD_JS[HUD_JS.index("function paint(t)"):HUD_JS.index("function updateStaleness")]
        for field in ("target_type", "target_type_switching", "target_type_overdue"):
            self.assertIn("value.%s" % field, paint, field)

    def test_the_switch_bar_slides_while_waiting_fills_while_settling_and_is_amber_when_overdue(self):
        # Owner, 2026-10-20: a minimal progress bar instead of a blinking chip.
        self.assertIsNone(self._chips({"target_type": "HEAD", "target_type_switching": False})["progress"])
        wait = self._chips({"target_type": "CAT", "target_type_switching": True, "target_type_progress": 0})
        self.assertEqual(wait["progress"]["mode"], "wait", "a swap's length is not knowable: no fake fill")
        fill = self._chips({"target_type": "CAT", "target_type_switching": True, "target_type_progress": 0.5})
        self.assertEqual(fill["progress"], {"mode": "fill", "fraction": 0.5})
        late = self._chips({"target_type": "CAT", "target_type_switching": True, "target_type_overdue": True,
                            "target_type_progress": 0})
        self.assertEqual(late["progress"]["mode"], "overdue")

    def _rows(self, drawer: str, telemetry: dict):
        return self._node("console.log(JSON.stringify(T.hudDrawerActions(%s, %s)));"
                          % (json.dumps(drawer), json.dumps(telemetry)))

    def test_menu_after_a_failed_homing_offers_home_and_does_not_claim_the_motors_are_off(self) -> None:
        # Owner, 2026-10-25: a homing that failed without a hazard holds unhomed, pitch held.
        rows = {r["label"]: r for r in self._rows("MENU", {
            "phase": "idle", "homing_attempt": 3, "homing_failure": "repeatability exceeded"})}
        self.assertEqual((rows["HOME"]["kind"], rows["HOME"]["command"]), ("act", "start_homing"))
        self.assertIn("Homing failed", rows["HOME"]["note"])
        for k in ("PARK", "SHUTDOWN"):
            self.assertEqual(rows[k]["kind"], "gated")
            self.assertNotIn("OFF", rows[k]["note"], k)
        boot = {r["label"]: r for r in self._rows("MENU", {"phase": "idle"})}
        self.assertIn("MOTORS ARE OFF", boot["SHUTDOWN"]["note"], "a boot is Shutdown, motors off")

    # -- the dock itself ---------------------------------------------------------------

    def test_the_dock_is_targets_mode_menu(self) -> None:
        # §13 listed five; DIAG left on 2026-10-03 (owner): its rows are in the stats overlay, which
        # MENU > SETTINGS turns on. MANUAL left on 2026-10-24 (owner): the D-pad is the manual control.
        spec = self._node("console.log(JSON.stringify(T.hudDockSpecs({})));")
        self.assertEqual([b["key"] for b in spec], ["TARGETS", "MODE", "MENU"])
        self.assertTrue(all(not b["active"] for b in spec))

    def test_exactly_one_button_shows_the_open_drawer(self) -> None:
        spec = self._node("console.log(JSON.stringify(T.hudDockSpecs({open:\"MODE\"})));")
        self.assertEqual([b["key"] for b in spec if b["active"]], ["MODE"])
        # MENU > CALIBRATION is MENU's own panel: MENU stays lit while it is open.
        spec = self._node("console.log(JSON.stringify(T.hudDockSpecs({open:\"CALIBRATION\"})));")
        self.assertEqual([b["key"] for b in spec if b["active"]], ["MENU"])

    def test_the_page_keeps_a_single_open_drawer_variable(self) -> None:
        # §13.2 "only one drawer should be open at a time". Asserted structurally: one variable and one
        # setter, rather than five booleans each handler must remember to clear. Five booleans is how a
        # UI ends up with two drawers stacked after someone adds a sixth button.
        self.assertIn("let drawerOpen = null;", HUD_JS)
        self.assertIn("function setDrawer(key)", HUD_JS)
        self.assertNotIn("let drawerTargetsOpen", HUD_JS)
        self.assertIn('drawerOpen = (drawerOpen === key) ? null : key', HUD_JS)

    # -- TARGETS -----------------------------------------------------------------------

    def test_target_rows_send_the_display_index_the_daemon_asks_for(self) -> None:
        rows = self._rows("TARGETS", {"tracks": TRACKS, "operating_mode": "MANUAL"})
        by_label = {r["label"]: r for r in rows}
        person = [r for r in rows if r["label"].startswith("#3")][0]
        self.assertEqual(person["command"], None, "the already-selected target must not send a command")
        self.assertEqual(person["kind"], "current")
        vehicle = [r for r in rows if r["label"].startswith("#1")][0]
        self.assertEqual(vehicle["command"], "select_target")
        self.assertEqual(vehicle["arg"], "1", "controld parses the argument as a number; a label string "
                                             "is the mismatch this project has already been bitten by")
        self.assertFalse([r for r in rows if "ghost" in r["label"].lower()],
                         "display_index 0 is refused by the daemon, so the row must not exist")
        self.assertIn("CLEAR SELECTION", [r["label"] for r in rows])

    def test_no_targets_says_so_instead_of_showing_an_empty_box(self) -> None:
        rows = self._rows("TARGETS", {"tracks": [], "operating_mode": "MANUAL"})
        self.assertEqual([r["label"] for r in rows], ["NO TARGETS"])
        self.assertIsNone(rows[0]["command"])

    # -- MODE --------------------------------------------------------------------------

    def test_mode_rows_use_the_names_the_parser_accepts(self) -> None:
        # operating_mode_from_name returns false rather than guessing, so a wrong spelling is a refusal
        # and not a fallback; the exact wire spellings belong in a test.
        rows = self._rows("MODE", {"operating_mode": "MANUAL", "watch_point_usable": True})
        modes = [r for r in rows if r["command"] in ("set_mode", None) and r["arg"]]
        self.assertEqual([r["arg"] for r in modes], ["MANUAL", "AUTO_TRACK", "AUTO_ROAM", "SURVEILLANCE"])
        self.assertEqual([r["label"] for r in modes], ["MANUAL", "AUTO TRACK", "AUTO ROAM", "SURVEILLANCE"])
        self.assertIsNone(rows[0]["command"], "the mode already in force must not be re-issued")
        self.assertEqual(rows[1]["command"], "set_mode")
        self.assertEqual(rows[3]["command"], "set_mode")

    def test_surveillance_is_greyed_with_the_reason_until_a_watch_point_is_usable(self) -> None:
        # controld refuses SURVEILLANCE with no usable watch point (owner, 2026-10-05); a live-looking
        # row whose only outcome is a refusal is the failure mode this drawer exists to avoid.
        none = self._rows("MODE", {"operating_mode": "AUTO_ROAM", "phase": "hold"})
        surv = [r for r in none if r["label"] == "SURVEILLANCE"][0]
        self.assertIsNone(surv["command"])
        self.assertEqual(surv["kind"], "gated")
        self.assertIn("SET A WATCH POINT", surv["note"])
        unusable = self._rows("MODE", {"operating_mode": "AUTO_ROAM", "phase": "hold",
                                       "watch_point_set": True, "watch_point_usable": False})
        self.assertIn("NOT USABLE", [r for r in unusable if r["label"] == "SURVEILLANCE"][0]["note"])
        ready = self._rows("MODE", {"operating_mode": "AUTO_ROAM", "phase": "hold",
                                    "watch_point_set": True, "watch_point_usable": True})
        self.assertEqual([r for r in ready if r["label"] == "SURVEILLANCE"][0]["command"], "set_mode")

    def test_no_row_asks_to_be_confirmed_until_it_has_been_pressed_once(self) -> None:
        # 2026-10-05: with nothing pending the old key was null, which is also every inert row's
        # command, so the active mode read "CONFIRM ... PRESS AGAIN".
        rows = self._rows("MODE", {"operating_mode": "SURVEILLANCE", "phase": "hold",
                                   "watch_point_set": True, "watch_point_usable": True}) + \
            self._rows("MENU", {"phase": "hold"})
        got = self._node("const rows = %s; console.log(JSON.stringify(["
                         "rows.map(r => T.hudRowAwaitsConfirm(r, null)),"
                         "rows.map(r => T.hudRowAwaitsConfirm(r, T.hudConfirmKey('request_shutdown', '')))]));"
                         % json.dumps(rows))
        self.assertEqual(got[0], [False] * len(rows), "nothing pressed, nothing waiting")
        self.assertEqual([r["label"] for r, w in zip(rows, got[1]) if w], ["SHUTDOWN"],
                         "only the row that was pressed, and only one that asks")
        active = [r for r in rows if r["label"] == "SURVEILLANCE"][0]
        self.assertEqual((active["kind"], active["note"]), ("current", "ACTIVE"))

    def test_the_quick_mode_buttons_are_gone(self) -> None:
        # Owner, 2026-10-05: modes are switched in the MODE drawer only.
        for marker in ('id="mode-controls"', 'id="manual-mode"', 'id="auto-mode"'):
            self.assertNotIn(marker, HUD_HTML)
        self.assertNotIn('$("manual-mode")', HUD_JS)
        self.assertNotIn('$("auto-mode")', HUD_JS)

    def test_set_watch_point_saves_here_in_one_press(self) -> None:
        first = [r for r in self._rows("MODE", {"operating_mode": "MANUAL", "phase": "hold"})
                 if r["label"] == "SET WATCH POINT"][0]
        self.assertEqual((first["command"], first["arg"], first["kind"]), ("set_watch_point", "", "act"))
        again = [r for r in self._rows("MODE", {"operating_mode": "MANUAL", "phase": "hold",
                                                "watch_point_set": True})
                 if r["label"] == "SET WATCH POINT"][0]
        self.assertEqual((again["command"], again["kind"]), ("set_watch_point", "act"),
                         "owner, 2026-10-05: only SHUTDOWN asks twice")
        unhomed = [r for r in self._rows("MODE", {"operating_mode": "MANUAL", "phase": "idle"})
                   if r["label"] == "SET WATCH POINT"][0]
        self.assertIsNone(unhomed["command"])
        self.assertEqual(unhomed["kind"], "gated")

    # -- MANUAL ------------------------------------------------------------------------

    def test_jog_arguments_match_the_daemon(self) -> None:
        self.assertEqual(set(re.findall(r'data-direction="([^"]+)"', HUD_HTML)),
                         {"left", "right", "up", "down", "up-left", "up-right", "down-left",
                          "down-right"})
        # Eight directions (owner, 2026-10-03): a diagonal is its two arrows in one jog, so the
        # page still sends directions only and controld applies every cap and margin per axis.
        self.assertEqual(self._node("console.log(JSON.stringify(['up-left','up-right','down-left',"
                                     "'down-right'].map(T.otaJogForArrow)));"),
                         ["pitch-|yaw+", "pitch-|yaw-", "pitch+|yaw+", "pitch+|yaw-"])
        # Station probes established yaw+ = camera left, pitch+ = camera down.
        # The button and every other spatial indicator use one conversion.
        self.assertEqual(self._node("console.log(JSON.stringify(["
                                     "'left','right','up','down'].map(T.otaJogForArrow)));"),
                         ["yaw+", "yaw-", "pitch-", "pitch+"])
        self.assertIn('b.dataset.jog = otaJogForArrow(b.dataset.direction)', HUD_JS)
        self.assertEqual(self._node("console.log(JSON.stringify(["
                                     "T.otaAxisArrow('yaw',1),T.otaAxisArrow('yaw',-1),"
                                     "T.otaAxisArrow('pitch',1),T.otaAxisArrow('pitch',-1)]));"),
                         ["←", "→", "↓", "↑"])

    def test_the_manual_drawer_is_gone_and_the_pad_is_the_manual_control(self) -> None:
        # Owner, 2026-10-24: "I don't want the MANUAL anymore as we have the DPAD already".
        self.assertEqual(self._rows("MANUAL", {"operating_mode": "MANUAL"}), [])
        self.assertIn('id="manual-pad"', HUD_HTML)

    def test_the_pad_paces_by_the_main_camera_and_its_centre_is_the_pace(self) -> None:
        # Owner, 2026-10-03: coarse on wide, fine on detail (controld's "view" profile); the centre
        # HOLD that looked like a button became the pace toggle, and releasing an arrow is the stop.
        self.assertIn('"view"', HUD_JS)
        self.assertNotIn('":normal"', HUD_JS)
        pad = re.search(r'<div id="manual-pad".*?</div>', HUD_HTML, re.S).group()
        self.assertNotIn("HOLD", pad)
        self.assertIn('id="pad-pace"', pad)
        self.assertEqual(len(re.findall(r"<button", pad)), 9, "eight directions and the pace")
        self.assertNotIn("data-jog", re.search(r'<button id="pad-pace".*?</button>', pad).group(),
                         "the pace button must never start a jog")

    # -- MENU > CALIBRATION (ADR-005) -------------------------------------------------------

    CAL_RECORDS = [
        {"id": 1, "name": "AS SHIPPED", "unusable": "", "parts": {
            "yaw": {"origin": "shipped", "date": "2026-10-02", "run": "yaw-20261002-123425", "from": 0},
            "pitch": {"origin": "shipped", "date": "2026-10-02", "run": "", "from": 0},
            "accuracy": {"origin": "shipped", "date": "2026-10-02", "run": "", "from": 0},
            "extrinsic": {"origin": "shipped", "date": "", "run": "", "from": 0},
            "bore": {"origin": "shipped", "date": "", "run": "", "from": 0}},
         "bore": {"mode": "off", "range_m": 10, "wide": {"trim_right_mrad": 0, "trim_up_mrad": 0}}},
        {"id": 2, "name": "FRICTION 10-20", "unusable": "", "parts": {
            "yaw": {"origin": "measured", "date": "2026-10-20", "run": "", "from": 0},
            "pitch": {"origin": "inherited", "date": "", "run": "", "from": 1},
            "accuracy": {"origin": "inherited", "date": "", "run": "", "from": 1},
            "extrinsic": {"origin": "inherited", "date": "", "run": "", "from": 1},
            "bore": {"origin": "inherited", "date": "", "run": "", "from": 1}},
         "bore": {"mode": "off", "range_m": None, "wide": {"trim_right_mrad": 1.5, "trim_up_mrad": -0.3}}},
        {"id": 4, "name": "BENCH", "unusable": "yaw_servo.json does not parse", "parts": {}, "bore": {}},
    ]

    def _panel(self, **t):
        t.setdefault("calibration_records", self.CAL_RECORDS)
        return self._node("console.log(JSON.stringify(T.hudCalibrationPanel(%s)));" % json.dumps(t))

    def test_the_menu_row_names_the_record_in_force_and_is_absent_without_records(self) -> None:
        entry = self._node("console.log(JSON.stringify(T.hudCalibrationEntry("
                           "{calibration_record:{id:2,name:'FRICTION 10-20'}})));")
        self.assertEqual(entry, {"label": "CALIBRATION ›", "note": "#2 FRICTION 10-20"})
        switching = self._node("console.log(JSON.stringify(T.hudCalibrationEntry("
                               "{calibration_record:{id:2,name:'X'},calibration_switching:true})));")
        self.assertEqual(switching["note"], "#2 X · SWITCHING")
        self.assertIsNone(self._node("console.log(JSON.stringify(T.hudCalibrationEntry({calibration_record:null})));"))

    def test_the_chips_switch_records_and_the_one_in_force_sends_nothing(self) -> None:
        p = self._panel(calibration_record={"id": 2, "name": "FRICTION 10-20"}, phase="hold")
        self.assertEqual([(c["label"], c["command"], c["arg"]) for c in p["chips"]],
                         [("#1 AS SHIPPED", "set_calibration_record", "1"),
                          ("#2 FRICTION 10-20", None, "2"),
                          ("#4 BENCH", "set_calibration_record", "4")])
        self.assertEqual([c["bad"] for c in p["chips"]], [False, False, True])
        self.assertEqual([x["text"] for x in p["parts"]],
                         ["MEASURED 10-20", "FROM #1", "FROM #1", "FROM #1", "FROM #1"])

    def test_the_shipped_backup_is_locked_and_never_offered_for_removal(self) -> None:
        # Owner, 2026-10-24: records leave the way they came, but #1 AS SHIPPED is the failsafe backup.
        p = self._panel(calibration_record={"id": 1, "name": "AS SHIPPED"}, phase="hold")
        self.assertTrue(p["inForce"]["locked"])
        self.assertEqual([x["arg"] for x in p["removable"]], ["2", "4"], "never #1, never the record in force")
        self.assertTrue(all(x["kind"] == "danger" and x["command"] == "delete_calibration_record" for x in p["removable"]))
        measure = {m["label"]: m for m in p["measure"]}
        self.assertIn("MEASURE A COPY…", measure, "#1 is never measured into; a copy is")
        self.assertNotIn("RE-RUN", measure)
        p2 = self._panel(calibration_record={"id": 2, "name": "FRICTION 10-20"}, phase="hold")
        self.assertFalse(p2["inForce"]["locked"])
        self.assertEqual([x["arg"] for x in p2["removable"]], ["4"])

    def _panel_opts(self, opts, **t):
        t.setdefault("calibration_records", self.CAL_RECORDS)
        return self._node("console.log(JSON.stringify(T.hudCalibrationPanel(%s, %s)));" % (json.dumps(t), json.dumps(opts)))

    def test_runs_take_two_presses_and_are_offered_homed_and_still(self) -> None:
        # ADR-005 D6/D7: VERIFY on every record, RE-RUN with its options; two presses each.
        p = self._panel(calibration_record={"id": 2, "name": "F"}, phase="hold", calibration_unverified=True)
        measure = {m["label"]: m for m in p["measure"]}
        verify = measure["VERIFY"]
        self.assertEqual((verify["kind"], verify["command"], verify["arg"]), ("danger", "verify_calibration", ""))
        self.assertTrue(verify["nudge"], "a switch since the last VERIFY: a nudge, never a run")
        self.assertIn("NOT VERIFIED SINCE SWITCH", verify["note"])
        rerun = measure["RE-RUN"]
        self.assertEqual((rerun["kind"], rerun["command"], rerun["arg"]), ("danger", "run_calibration", "RERUN"))
        self.assertIn("~20 MIN", rerun["note"])
        self.assertEqual([o["label"] for o in rerun["options"]], ["+ACCURACY", "+CROSSTALK"])
        both = {m["label"]: m for m in self._panel_opts({"accuracy": True, "crosstalk": True},
                                                        calibration_record={"id": 2, "name": "F"}, phase="hold")["measure"]}
        self.assertEqual(both["RE-RUN"]["arg"], "RERUN+ACCURACY+CROSSTALK")
        self.assertIn("~42 MIN", both["RE-RUN"]["note"])
        self.assertEqual(p["stop"], {"label": "STOP MOTION", "command": "hold", "arg": "", "kind": "stop"})
        for t, why in (({"phase": "idle"}, "HOME FIRST"), ({"phase": "hold", "rest_park": "parked"}, "PARKED"),
                       ({"phase": "hold", "calibration_switching": True}, "WAIT"), ({"phase": "hold", "calibrating": True}, "WAIT")):
            m = {x["label"]: x for x in self._panel(calibration_record={"id": 2, "name": "F"}, **t)["measure"]}
            self.assertEqual((m["VERIFY"]["kind"], m["VERIFY"]["command"]), ("gated", None), t)
            self.assertIn(why, m["VERIFY"]["note"], t)

    def test_the_run_view_says_where_the_run_is_and_offers_stop(self) -> None:
        run = {"id": 4, "kind": "RERUN+ACCURACY", "active": True, "state": "session", "elapsed_s": 750.4, "sessions": 3,
               "session": {"label": "yaw-20261024-1530-02-crosstalk", "axis": "yaw", "elapsed_s": 42, "expected_s": 84},
               "last_session": "yaw-...-01-survey: COMPLETE", "note": "yaw | inertia 0.0412, loop delay 1.6 ms", "verdict": ""}
        r = self._node("console.log(JSON.stringify(T.hudCalibrationRun(%s)));" % json.dumps(
            {"calibrating": True, "calibration_run": run, "calibration_record": {"id": 2, "name": "F"}}))
        self.assertEqual(r["line2"], "RE-RUN + ACCURACY · YAW UPDATE")
        self.assertEqual([(x["label"], x["state"]) for x in r["steps"]],
                         [("PITCH", "done"), ("YAW UPDATE", "now"), ("ACCURACY", "next")])
        self.assertEqual((r["session"]["axis"], r["session"]["text"], r["session"]["fraction"]), ("YAW", "0:42 / ~1:24", 0.5))
        self.assertEqual(r["note"], "inertia 0.0412, loop delay 1.6 ms")
        self.assertEqual(r["elapsed"], "12:30 OF ~40 MIN")
        self.assertEqual((r["stop"]["command"], r["stop"]["kind"]), ("stop_calibration", "stop"), "one press")
        between = self._node("console.log(JSON.stringify(T.hudCalibrationRun(%s)));" % json.dumps(
            {"calibrating": True, "calibration_run": dict(run, session=None, state="fitting")}))
        self.assertEqual((between["session"]["label"], between["session"]["text"]), ("FITTING AND DESIGNING", "TURRET HELD"))
        self.assertIsNone(self._node("console.log(JSON.stringify(T.hudCalibrationRun({calibrating:false})));"))

    def test_the_panel_says_how_the_record_s_own_last_run_ended(self) -> None:
        last = lambda rec, run=None: self._node("console.log(JSON.stringify(T.hudCalibrationLastRun(%s, %s)));" % (
            json.dumps({"calibration_run": run}), json.dumps(rec)))
        rec = {"id": 2, "last_run": {"kind": "VERIFY", "verdict": "REUSE_EXACT", "run": "0002-verify-x", "date": "2026-10-24"}}
        self.assertEqual(last(rec), {"text": "VERIFY · REUSE_EXACT", "tone": "good", "elapsed": "10-24"})
        just = last(rec, {"kind": "VERIFY", "active": False, "state": "done", "verdict": "REUSE_EXACT", "elapsed_s": 61})
        self.assertEqual(just["elapsed"], "1:01", "the run that just ended says how long it took")
        drift = dict(rec, last_run=dict(rec["last_run"], verdict="UPDATE_PARAMETERS"))
        self.assertEqual(last(drift)["tone"], "warn")
        self.assertIsNone(last({"id": 2, "last_run": None}), "a record with no run says nothing")

    def test_a_run_dims_the_dock_hides_the_pad_and_takes_the_drawer(self) -> None:
        self.assertIn('!t.calibration_switching && !t.calibrating', HUD_JS)
        self.assertIn('|| !!(t && t.calibrating)', HUD_JS, "the D-pad is hidden during a run")
        self.assertIn('dock.classList.toggle("held", running)', HUD_JS)
        self.assertIn('drawerOpen !== "RUN") setDrawer', HUD_JS, "the dock cannot close the run view")
        self.assertIn("#dock.held", HUD_CSS)

    def test_a_switch_shows_its_bar_and_says_why_when_it_is_not_applied(self) -> None:
        wait = self._panel(calibration_record={"id": 2, "name": "F"}, calibration_switching=True,
                           calibration_switch_from=1, calibration_progress=0)
        self.assertEqual(wait["bar"]["mode"], "wait")
        self.assertIn("#1 → #2", wait["notes"][0]["text"])
        self.assertIsNotNone(wait["chips"][1]["command"], "choosing again retries")
        fill = self._panel(calibration_record={"id": 2, "name": "F"}, calibration_switching=True, calibration_progress=0.5)
        self.assertEqual(fill["bar"], {"mode": "fill", "fraction": 0.5})
        over = self._panel(calibration_record={"id": 2, "name": "F"}, calibration_switching=True,
                           calibration_overdue=True, calibration_blocked="simulated refusal")
        self.assertEqual(over["bar"]["mode"], "overdue")
        self.assertTrue(over["notes"][0]["warn"])
        self.assertIn("simulated refusal", over["notes"][0]["text"])
        blocked = self._panel(calibration_record={"id": 2, "name": "F"}, calibration_blocked="record #2: x")
        self.assertIn("HOME is refused", blocked["notes"][0]["text"])

    # ADR-005 D9 (owner, 2026-10-24): bore mode per record; RANGE (ADR-006: the working distance) and the MRAD trims apply with it on.
    INTR = {"valid": True, "fx": 1389, "fy": 1467}
    ON = {"mode": "manual_depth", "range_m": 25,
          "wide": {"trim_right_mrad": 1.5, "trim_up_mrad": -0.3, "right_mm": 75, "up_mm": 75}}

    def _bore(self, pace=0.1, bore=None, **t):
        recs = json.loads(json.dumps(self.CAL_RECORDS))
        if bore is not None:
            recs[1]["bore"] = bore
        t.setdefault("calibration_records", recs)
        t.setdefault("calibration_record", {"id": 2, "name": "FRICTION 10-20"})
        t.setdefault("camera_intrinsics", self.INTR)
        return self._node("console.log(JSON.stringify(T.hudCalibrationBore(%s, %s)));" % (json.dumps(t), json.dumps(pace)))

    def test_the_shipped_backup_has_no_bore_controls(self) -> None:
        b = self._bore(calibration_record={"id": 1, "name": "AS SHIPPED"}, phase="hold", operating_mode="MANUAL")
        self.assertTrue(b["locked"])
        self.assertIn("NEW", b["note"])

    def test_off_offers_only_on_and_greys_the_zero_and_the_trims(self) -> None:
        b = self._bore(phase="hold", operating_mode="MANUAL")
        self.assertFalse(b["on"])
        self.assertEqual([(m["label"], m["on"], m["command"]) for m in b["mode"]],
                         [("OFF", True, None), ("ON", False, "set_bore_mode")])
        self.assertEqual(b["note"], "the tracker aims the camera centre")
        self.assertEqual((b["range"]["down"], b["range"]["up"]), (None, None))
        self.assertTrue(all(x["command"] is None for x in b["wide"]["buttons"]))
        self.assertIn("turn the bore on", b["wide"]["note"])

    def test_on_in_a_manual_hold_trims_at_the_chosen_pace_and_steps_the_zero_ladder(self) -> None:
        b = self._bore(pace=1, bore=self.ON, phase="hold", operating_mode="MANUAL")
        self.assertEqual(b["mode"][0]["command"], "set_bore_mode", "OFF is offered")
        self.assertEqual([(x["command"], x["arg"]) for x in b["wide"]["buttons"]],
                         [("trim_bore", "wide right -1"), ("trim_bore", "wide right 1"),
                          ("trim_bore", "wide up -1"), ("trim_bore", "wide up 1")])
        self.assertEqual(b["wide"]["text"], "R +1.5 · U −0.3 MRAD (+2.1 / −0.4 px)")
        self.assertEqual([p["on"] for p in b["paces"]], [False, True, False])
        self.assertIn("1 mrad (1.4 px on the wide camera)", b["paceNote"])
        self.assertEqual((b["range"]["name"], b["range"]["down"], b["range"]["up"]), ("25 m", "10", "50"))
        self.assertIn("4.24 mrad parallax from the 106 mm offset", b["range"]["note"])
        self.assertIsNone(b["hold"])
        fine = self._bore(bore=self.ON, phase="idle")
        self.assertEqual(fine["wide"]["buttons"][1]["arg"], "wide right 0.1", "the MIL click; Shutdown is still too")
        # ADR-006 D3: the detail camera has the same row, its own trims and zero.
        self.assertEqual([x["arg"] for x in b["detail"]["buttons"]],
                         ["detail right -1", "detail right 1", "detail up -1", "detail up 1"])
        self.assertEqual(b["wide"]["zeroed"], "NOT ZEROED")
        zeroed = self._bore(bore=dict(self.ON, detail={"trim_right_mrad": -10.47, "trim_up_mrad": -22.69, "right_mm": 145,
                                                        "up_mm": 40, "zeroed_m": 37.5, "zeroed_on": "2026-10-24"}), phase="idle")
        self.assertEqual(zeroed["detail"]["zeroed"], "ZEROED 10-24 AT 37.5 m")
        self.assertEqual(zeroed["detail"]["text"], "R −10.5 · U −22.7 MRAD", "no wide pixels on the detail row")
        self.assertEqual(zeroed["detail"]["installed"], "145 R · 40 U MM")

    def test_the_ladder_ends_at_infinity_and_a_hand_edited_distance_steps_to_its_neighbours(self) -> None:
        inf = self._bore(bore=dict(self.ON, range_m=None), phase="idle")
        self.assertEqual((inf["range"]["name"], inf["range"]["down"], inf["range"]["up"]), ("∞", "1500", None))
        self.assertIn("no parallax", inf["range"]["note"])
        odd = self._bore(bore=dict(self.ON, range_m=30), phase="idle")
        self.assertEqual((odd["range"]["down"], odd["range"]["up"]), ("25", "50"))
        self.assertEqual(self._node("console.log(JSON.stringify([T.hudBoreZeroName(1e6), T.hudBoreZeroName(10), "
                                    "T.hudBoreZeroName(0.5)]));"), ["∞", "10 m", "0.5 m"])

    # ADR-006 D6: the ZERO procedure.
    def _zero(self, z=None, settle=None, bore=None, **t):
        recs = json.loads(json.dumps(self.CAL_RECORDS))
        recs[1]["bore"] = bore if bore is not None else dict(self.ON, wide=dict(self.ON["wide"], forward_mm=0),
                                                             detail={"right_mm": 145, "up_mm": 40, "forward_mm": 0})
        t.setdefault("calibration_records", recs)
        t.setdefault("calibration_record", {"id": 2, "name": "FRICTION 10-20"})
        return self._node("console.log(JSON.stringify(T.hudCalibrationZero(%s, %s, %s)));" % (
            json.dumps(t), json.dumps(z or {"distance": None, "armed": None}), json.dumps(settle)))

    def test_zero_walks_through_hold_distance_and_settle_before_the_tap(self) -> None:
        roam = self._zero(phase="hold", operating_mode="AUTO_ROAM")
        self.assertIsNone(roam["tap"]["ui"])
        self.assertIn("MANUAL hold", roam["tap"]["note"])
        nod = self._zero(phase="hold", operating_mode="MANUAL", settle={"ok": True, "spread": 0.1})
        self.assertEqual(nod["tap"]["note"], "type the target's distance first")
        self.assertEqual(nod["distance"]["text"], "NOT SET")
        moving = self._zero({"distance": 37.5, "armed": None}, {"ok": False, "spread": 1.84}, phase="hold", operating_mode="MANUAL")
        self.assertEqual(moving["settle"]["text"], "SETTLING · 1.8 MRAD")
        self.assertIsNone(moving["tap"]["ui"])
        ready = self._zero({"distance": 37.5, "armed": None}, {"ok": True, "spread": 0.1}, phase="hold", operating_mode="MANUAL")
        self.assertEqual((ready["tap"]["ui"], ready["tap"]["label"]), ("zero:arm", "TAP THE TARGET · WIDE"))
        self.assertEqual(ready["arg"], "wide 37.5")
        self.assertEqual(ready["distance"]["text"], "37.5 m")
        self.assertEqual([o["text"] for o in ready["offsets"]], ["75 R · 75 U · 0 F MM", "145 R · 40 U · 0 F MM"])
        armed = self._zero({"distance": 37.5, "armed": "wide"}, {"ok": True, "spread": 0.1}, phase="hold", operating_mode="MANUAL")
        self.assertEqual((armed["tap"]["ui"], armed["tap"]["label"]), ("zero:cancel", "CANCEL THE TAP"))
        # The detail camera on the main display: its tap.
        det = self._zero({"distance": 37.5, "armed": None}, {"ok": True, "spread": 0.1}, phase="hold", operating_mode="MANUAL",
                         inference={"main_camera": {"role": "detail", "views": {"detail": {"scale": 5.9}},
                                                    "available": ["wide", "detail"]}})
        self.assertEqual((det["tap"]["label"], det["arg"]), ("TAP THE TARGET · DETAIL", "detail 37.5"))
        self.assertIsNone(self._zero(calibration_record={"id": 1, "name": "AS SHIPPED"}), "#1: the BORE rows say why")
        off = self._zero(bore={"mode": "off"}, phase="hold", operating_mode="MANUAL")
        self.assertEqual(off["tap"]["note"], "turn the bore on first")
        self.assertIsNone(off["offsets"][0]["ui"])

    def test_the_settle_check_wants_0_6_s_within_0_3_mrad(self) -> None:
        still = [{"t": i * 100, "yaw": 1.0 + 1e-5 * (i % 2), "pitch": -0.1} for i in range(8)]
        drift = [{"t": i * 100, "yaw": 1.0 + 2e-4 * i, "pitch": -0.1} for i in range(8)]
        r = self._node("console.log(JSON.stringify([T.hudSettled(%s, 700), T.hudSettled(%s, 700), T.hudSettled(%s, 250)]));"
                       % (json.dumps(still), json.dumps(drift), json.dumps(still[:3])))
        self.assertTrue(r[0]["ok"])
        self.assertFalse(r[1]["ok"])
        self.assertAlmostEqual(r[1]["spread"], 1.2, places=6)
        self.assertFalse(r[2]["ok"], "too short a history is not yet settled")

    def test_a_tap_unprojects_into_the_wide_frame_on_either_picture(self) -> None:
        # hudUnproject undoes hudProject, with the detail picture's scale too; a letterbox bar is not a tap.
        r = self._node(
            "const out = [];"
            "for (const k of [1, 5.9]) { const lay = Object.assign(T.hudLayout ? {} : {}, {ok: true, s: 0.5, ox: 0, oy: 60,"
            " w: 960, h: 540, k: k});"
            "  for (const [u, v] of [[0.5, 0.5], [0.48, 0.53], [0.52, 0.47]]) {"
            "    const p = T.hudProject(u, v, lay), q = T.hudUnproject(p.x, p.y, lay); out.push([q.ok, q.u - u, q.v - v]); } }"
            "const lay = {ok: true, s: 0.5, ox: 0, oy: 60, w: 960, h: 540, k: 1};"
            "out.push(T.hudUnproject(400, 30, lay).ok);"
            "console.log(JSON.stringify(out));")
        for ok, du, dv in r[:-1]:
            self.assertTrue(ok)
            self.assertAlmostEqual(du, 0, places=12)
            self.assertAlmostEqual(dv, 0, places=12)
        self.assertFalse(r[-1], "the bar above the picture")

    def test_a_moving_turret_offers_the_hold_and_nothing_else(self) -> None:
        roam = self._bore(bore=self.ON, phase="hold", operating_mode="AUTO_ROAM")
        self.assertEqual((roam["hold"]["command"], roam["hold"]["arg"]), ("set_mode", "MANUAL"))
        self.assertFalse(roam["still"])
        self.assertIn("trim in MANUAL", roam["note"])
        self.assertEqual(roam["wide"]["note"], "", "said once, on the BORE row")
        self.assertTrue(all(m["command"] is None for m in roam["mode"]))
        self.assertTrue(all(x["command"] is None for x in roam["wide"]["buttons"]))
        for t in ({"phase": "hold", "operating_mode": "MANUAL", "calibration_switching": True},
                  {"phase": "parked", "rest_park": "parked"}, {"phase": "homing"}):
            b = self._bore(bore=self.ON, **t)
            self.assertIsNone(b["hold"], t)
            self.assertFalse(b["live"], t)

    # The calibration lifecycle closes (owner, 2026-10-25): leaving MENU > CALIBRATION puts back what
    # calibration changed -- the main display, the mode its HOLD HERE interrupted -- and nothing else.

    def test_the_hold_says_which_mode_comes_back(self) -> None:
        roam = self._bore(bore=self.ON, phase="hold", operating_mode="AUTO_ROAM")
        self.assertEqual(roam["hold"]["ui"], "hold")
        self.assertEqual(roam["hold"]["note"], "MANUAL until you leave calibration, then AUTO ROAM again")
        self.assertIsNone(roam["held"])
        # controld's own rule for runs and swap holds: a track gives back its return mode, not the track.
        track = self._bore(bore=self.ON, phase="hold", operating_mode="AUTO_TRACK", auto_return_mode="SURVEILLANCE")
        self.assertEqual(track["hold"]["note"], "MANUAL until you leave calibration, then SURVEILLANCE again")
        held = self._node("console.log(JSON.stringify(T.hudCalibrationBore(%s, 0.1, 'SURVEILLANCE')));" % json.dumps(
            {"calibration_records": self.CAL_RECORDS, "calibration_record": {"id": 2, "name": "F"},
             "phase": "hold", "operating_mode": "MANUAL"}))
        self.assertEqual(held["held"], "HELD FOR CALIBRATION · SURVEILLANCE AGAIN WHEN YOU LEAVE · STOP MOTION KEEPS IT HELD")

    def test_the_resumed_mode_follows_controlds_rule(self) -> None:
        mode = lambda **t: self._node("console.log(JSON.stringify(T.hudResumeMode(%s)));" % json.dumps(t))
        self.assertEqual(mode(operating_mode="AUTO_ROAM"), "AUTO_ROAM")
        self.assertEqual(mode(operating_mode="SURVEILLANCE"), "SURVEILLANCE")
        self.assertEqual(mode(operating_mode="AUTO_TRACK", auto_return_mode="SURVEILLANCE"), "SURVEILLANCE")
        self.assertEqual(mode(operating_mode="AUTO_TRACK"), "AUTO_ROAM")

    def _exit(self, session, fresh=True, **t):
        return self._node("console.log(JSON.stringify(T.hudCalibrationExit(%s, %s, %s)));" % (
            json.dumps(session), json.dumps(t), json.dumps(fresh)))

    @staticmethod
    def _main(role):
        return {"inference": {"main_camera": {"role": role, "available": ["wide", "detail"]}}}

    def test_leaving_calibration_puts_back_the_camera_and_the_interrupted_mode(self) -> None:
        manual = dict(phase="hold", operating_mode="MANUAL", **self._main("detail"))
        ours = {"main": "wide", "resume": "AUTO_ROAM", "swapped": True}
        self.assertEqual(self._exit(ours, **manual), {"main": "wide", "mode": "AUTO_ROAM"})
        self.assertEqual(self._exit(None, **manual), {"main": None, "mode": None}, "never entered: nothing to undo")
        self.assertEqual(self._exit({"main": "wide", "resume": None, "swapped": False}, **manual), {"main": None, "mode": None},
                         "a swap made on another page, or before calibration, stays")
        # By name, whatever the last frame says: a frame that lags the swap cannot send it the wrong way.
        self.assertEqual(self._exit(ours, **dict(manual, **self._main("wide")))["main"], "wide")
        self.assertEqual(self._exit(dict(ours, resume="MANUAL"), **manual)["mode"], None)
        self.assertEqual(self._exit(ours, fresh=False, **manual), {"main": "wide", "mode": None},
                         "stale telemetry: nothing resumes, the turret stays held")
        # The hold is not calibration's any more, or not a still hold: nothing is resumed.
        for t in ({"phase": "hold", "operating_mode": "AUTO_TRACK"}, {"phase": "idle"}, {"phase": "fault"},
                  dict(manual, calibrating=True), dict(manual, calibration_switching=True), dict(manual, rest_park="parked")):
            self.assertIsNone(self._exit(ours, **t)["mode"], t)
        alone = dict(manual, inference={"main_camera": {"role": "wide", "available": ["wide"]}})
        self.assertIsNone(self._exit({"main": "detail", "resume": None, "swapped": True}, **alone)["main"], "no detail stream")

    def _step(self, session, prev, t):
        return self._node("console.log(JSON.stringify(T.hudCalibrationSessionStep(%s, %s, %s)));" % (
            json.dumps(session), json.dumps(prev), json.dumps(t)))

    def test_a_stop_from_any_page_cancels_the_resume(self) -> None:
        # Review 2026-10-25: STOP MOTION in calibration's own MANUAL hold changes nothing the hold check
        # watches, and a STOP during an accepted run's install ends the run `done`: the ack is the word.
        manual = {"phase": "hold", "operating_mode": "MANUAL", "cmd_ack_seq": 7, "cmd_ack_command": "set_mode",
                  "cmd_ack_accepted": 1}
        held = {"main": "wide", "resume": "AUTO_ROAM", "taken": True, "swapped": False}
        for command in ("hold", "stop_calibration"):
            after = self._step(held, manual, dict(manual, cmd_ack_seq=8, cmd_ack_command=command))
            self.assertIsNone(after["resume"], command)
        refused = self._step(held, manual, dict(manual, cmd_ack_seq=8, cmd_ack_command="hold", cmd_ack_accepted=0))
        self.assertEqual(refused["resume"], "AUTO_ROAM", "a refused STOP stopped nothing")
        same = self._step(held, dict(manual, cmd_ack_command="hold"), dict(manual, cmd_ack_command="hold"))
        self.assertEqual(same["resume"], "AUTO_ROAM", "an old ack seen again is not a new STOP")

    def test_the_hold_counts_once_it_stands_and_ends_when_the_station_leaves_it(self) -> None:
        roam = {"phase": "hold", "operating_mode": "AUTO_ROAM"}
        manual = {"phase": "hold", "operating_mode": "MANUAL"}
        asked = {"main": "wide", "resume": "AUTO_ROAM", "taken": False, "swapped": False}
        waiting = self._step(asked, roam, roam)
        self.assertEqual((waiting["resume"], waiting["taken"]), ("AUTO_ROAM", False), "HOLD HERE not yet in force")
        standing = self._step(waiting, roam, manual)
        self.assertEqual((standing["resume"], standing["taken"]), ("AUTO_ROAM", True))
        self.assertEqual(self._step(standing, manual, dict(manual, calibrating=True))["resume"], "AUTO_ROAM", "a run keeps it")
        self.assertIsNone(self._step(standing, manual, {"phase": "hold", "operating_mode": "AUTO_TRACK"})["resume"],
                          "a mode chosen elsewhere ends it")
        self.assertIsNone(self._step(standing, manual, {"phase": "fault"})["resume"])
        run = lambda state: {"id": 3, "active": False, "state": state}
        self.assertIsNone(self._step(standing, dict(manual, calibrating=True), dict(manual, calibration_run=run("stopped")))["resume"])

    def test_calibrations_hold_ends_when_the_station_leaves_it_or_a_run_is_stopped(self) -> None:
        stands = lambda prev, t: self._node("console.log(JSON.stringify(T.hudCalibrationHoldStands(%s, %s)));" % (
            json.dumps(prev), json.dumps(t)))
        manual = {"phase": "hold", "operating_mode": "MANUAL"}
        self.assertTrue(stands(None, manual))
        self.assertTrue(stands(manual, dict(manual, calibrating=True)), "a run keeps it")
        self.assertTrue(stands(manual, {"phase": "hold", "calibration_switching": True}), "a swap hold keeps it")
        self.assertFalse(stands(manual, {"phase": "hold", "operating_mode": "AUTO_TRACK"}))
        self.assertFalse(stands(manual, {"phase": "fault"}))
        run = lambda state: {"id": 3, "active": False, "state": state}
        self.assertFalse(stands(dict(manual, calibrating=True), dict(manual, calibration_run=run("stopped"))),
                         "STOP is the operator's word: leaving must not start the turret again")

    def test_the_page_wires_the_lifecycle_to_every_way_out(self) -> None:
        # The wiring, which needs a browser to run: leaving is setDrawer's (‹ MENU and every dock button
        # go through it); a run's end comes back into calibration; every frame steps the session; a STOP
        # pressed here cancels at once; the PIP's swap is noted; the exit swaps back by name.
        for line in ('const leaving = drawerOpen === "CALIBRATION" && key !== "CALIBRATION";',
                     "if (leaving) leaveCalibration();",
                     'drawerOpen = "CALIBRATION"; enterCalibration();',
                     "if (calSession) calSession = hudCalibrationSessionStep(calSession, lastTelemetry, t);",
                     "if (drawerOpen && (drawerChanged || settleFlipped || heldMoved || actClosed) && !calZero.armed) renderDrawer();",
                     'if (calSession && (cmd === "hold" || cmd === "stop_calibration")) calSession.resume = null;',
                     "window.otaSwapPip = function () { noteCalibrationSwap(); return window.otaSetMain(panes.pip.role); };",
                     "if (exit.main && window.otaSetMain) window.otaSetMain(exit.main);",
                     'hudReticle(t, stale, view.main, drawerOpen === "CALIBRATION")',
                     "if (!s.settle.ok) { say(",
                     'classList.contains("stale")) { say("NO LIVE DATA FROM THE STATION',
                     "(t.calibration_record || {}).id !== calZero.record"):
            self.assertIn(line, HUD_JS + HUD_HTML)

    def test_the_pad_dims_and_stops_jogging_while_a_swap_holds_the_turret(self) -> None:
        self.assertIn("!t.target_type_switching && !t.calibration_switching", HUD_JS)
        self.assertIn('pad.classList.toggle("held"', HUD_JS)
        self.assertIn("#manual-pad.held", HUD_CSS)

    # -- MENU / DIAG -------------------------------------------------------------------

    # Owner ruling 2026-10-03: HOME (recover whatever latched, then home), PARK (yaw 0, pitch onto its
    # rest stop, hold), SHUTDOWN (park, then motors off). A row offers its command only where controld
    # will carry it out; elsewhere it is greyed with the reason, and sends nothing.

    def _menu(self, **t) -> dict:
        return {r["label"]: r for r in self._rows("MENU", t)}

    def test_menu_is_home_park_shutdown(self) -> None:
        rows = self._rows("MENU", {"phase": "hold"})
        self.assertEqual([r["label"] for r in rows], ["HOME", "PARK", "SHUTDOWN"])
        self.assertEqual([r["command"] for r in rows], ["start_homing", "request_park", "request_shutdown"])

    def test_home_is_the_way_out_of_a_fault_and_of_motors_off(self) -> None:
        for phase in ("fault", "idle", "hold", "parked"):
            home = self._menu(phase=phase)["HOME"]
            self.assertEqual((home["kind"], home["command"]), ("act", "start_homing"), phase)
        self.assertIn("Recover", self._menu(phase="fault")["HOME"]["note"])

    def test_home_is_visibly_busy_while_homing_or_recovering(self) -> None:
        for phase, note in (("homing", "HOMING"), ("recovering", "RECOVERING")):
            home = self._menu(phase=phase)["HOME"]
            self.assertEqual((home["kind"], home["command"]), ("gated", None))
            self.assertIn(note, home["note"])

    def test_park_and_shutdown_need_a_homed_turret(self) -> None:
        for phase in ("fault", "idle", "homing", "recovering"):
            menu = self._menu(phase=phase)
            for label in ("PARK", "SHUTDOWN"):
                self.assertEqual((menu[label]["kind"], menu[label]["command"]), ("gated", None), (phase, label))
        self.assertIn("MOTORS ARE OFF", self._menu(phase="idle")["SHUTDOWN"]["note"])

    def test_parked_on_the_stop_is_current_and_shutdown_is_offered(self) -> None:
        menu = self._menu(phase="parked", rest_park="parked", rest_park_on_stop=True)
        self.assertEqual((menu["PARK"]["kind"], menu["PARK"]["command"]), ("current", None))
        self.assertEqual(menu["SHUTDOWN"]["command"], "request_shutdown")
        short = self._menu(phase="parked", rest_park="parked", rest_park_on_stop=False)
        self.assertEqual(short["PARK"]["command"], "request_park", "short of the stop, PARK touches again")

    def test_a_park_under_way_says_so_and_releasing_offers_nothing(self) -> None:
        moving = self._menu(phase="hold", rest_park="moving")
        self.assertEqual(moving["PARK"]["kind"], "gated")
        self.assertIn("PARKING", moving["PARK"]["note"])
        self.assertEqual(moving["SHUTDOWN"]["command"], "request_shutdown", "a park can become a shutdown")
        releasing = self._menu(phase="parking", rest_park="releasing")
        self.assertEqual([r["command"] for r in releasing.values()], [None, None, None])

    def test_only_shutdown_needs_two_presses(self) -> None:
        # Owner, 2026-10-05: SHUTDOWN switches both motors off and asks twice; HOME, PARK and every
        # mode are one press.
        rows = self._rows("MENU", {"phase": "hold"}) + \
            self._rows("MODE", {"operating_mode": "MANUAL", "phase": "hold", "watch_point_set": True,
                                "watch_point_usable": True})
        danger = [r for r in rows if r["kind"] == "danger"]
        self.assertEqual({r["command"] for r in danger}, {"request_shutdown"})
        self.assertIn("two-press", HUD_JS.lower().replace("two press", "two-press"),
                      "the handler's confirm path must exist, not just the data")

    def test_diag_is_read_only_and_dashes_what_is_missing(self) -> None:
        self.assertEqual(self._rows("DIAG", {"tracks": TRACKS}), [])
        rows = self._node("console.log(JSON.stringify(T.hudDiagRows({})));")
        values = [kv[1] for kv in rows]
        self.assertGreaterEqual(sum(1 for v in values if "--" in v), 6,
                                "an empty snapshot must not be presented as measured zeros")
        self.assertEqual(values[-1], "ABSENT", "§20's IMU absence belongs where an operator reads it")


class DrawerPresentation(unittest.TestCase):
    maxDiff = None

    def test_markup_order_puts_the_dock_and_drawer_above_every_symbology_layer(self) -> None:
        # §18: dock 30, drawer 40, above the z=20 symbology and below dialogs. This page has no SVG
        # z-index, so document order is the stacking order; the divs are also last in the container.
        self.assertLess(HUD_HTML.index('id="g-tapes"'), HUD_HTML.index('id="dock"'))
        self.assertLess(HUD_HTML.index('id="dock"'), HUD_HTML.index('id="drawer"'))
        self.assertLess(HUD_CSS.index("#dock"), HUD_CSS.index("#drawer"))
        self.assertIn("z-index:30", HUD_CSS)
        self.assertIn("z-index:40", HUD_CSS)

    def test_the_drawer_overlays_the_video_instead_of_resizing_it(self) -> None:
        # §13.2's requirement, and the reason the drawer is absolutely positioned rather than a flex
        # sibling: a drawer that reflows the viewport shrinks the picture at exactly the moment the
        # operator most needs to see it.
        self.assertIn("#drawer { position:absolute;", HUD_CSS)
        self.assertIn("#drawer[hidden] { display:none; }", HUD_CSS)

    def test_the_dock_style_follows_the_rules_it_is_given(self) -> None:
        # §13.1: dark translucent background, thin low-opacity outline, green line icon, small white
        # label, no bright solid fill.
        block = HUD_CSS[HUD_CSS.index(".dockbtn {"):HUD_CSS.index(".dockbtn:hover")]
        self.assertIn("rgba(3,6,5,.62)", block)
        self.assertIn("rgba(230,245,230,.22)", block, "thin low-opacity outline")
        self.assertNotIn("#95f58b", block, "a solid green fill would be the bright fill §13.1 forbids")
        self.assertIn("fill=\"none\"", HUD_JS[HUD_JS.index("function dockIcon"):HUD_JS.index("function renderDock")],
                       "§13.1 asks for line icons, not filled ones")

    def test_gated_rows_are_disabled_so_a_click_cannot_lie(self) -> None:
        at = HUD_JS.index("const inert = a.command === null")
        self.assertIn('(inert ? " disabled" : "")', HUD_JS[at:at + 700])
        self.assertIn("if (!b || b.disabled) return;", HUD_JS,
                      "the click handler must respect the disabled row, not rely on the browser alone")

    def test_the_drawer_does_not_narrate_commands(self) -> None:
        # Owner, 2026-10-05: "less is more". The drawer's ack line echoed raw command names, and the
        # DPAD's keepalive (every 75 ms) rewrote it and rebuilt the drawer thirteen times a second. What a
        # command did shows where it lands; the daemon's verdict stays in STATS FOR NERDS.
        self.assertNotIn("dack", HUD_JS)
        self.assertNotIn("dack", HUD_CSS)
        at = HUD_JS.index("async function sendCommand")
        body = HUD_JS[at:HUD_JS.index("// The pad's DOM stays in place", at)]
        self.assertNotIn("renderDrawer", body, "a jog keepalive must not rebuild the drawer")
        self.assertNotIn('"  ACCEPTED"', body, "the page never claims a verdict; the daemon publishes it")
        self.assertNotIn("cmd_ack_seq", HUD_JS[HUD_JS.index("function paint(t)"):HUD_JS.index("function updateStaleness")],
                         "the drawer refreshes on what it shows, not on every acknowledged command")
        self.assertIn('"LAST COMMAND"', HUD_JS, "the daemon's own verdict is still on the stats overlay")
        self.assertIn("cmd_ack_accepted === 1", HUD_JS)



class CommandPathIsReal(unittest.TestCase):
    """The endpoint the drawer posts to, with the body the drawer posts."""

    def setUp(self) -> None:
        self.tc = fake_camera.install(frame_delay=0.0)
        self._tmp = tempfile.TemporaryDirectory(prefix="ota_webd_dock_")
        self.sock = os.path.join(self._tmp.name, "controld.sock")
        self.fake = FakeControld(self.sock, telemetry_hz=20.0)
        self.fake.start()
        self.client = ControldClient(self.sock, reconnect_interval=0.05)
        self.app = create_app(self.client, WebConfig(host="127.0.0.1", port=0, socket_path=self.sock))
        self.http = TestClient(self.app)
        self.http.__enter__()

    def tearDown(self) -> None:
        try:
            self.http.__exit__(None, None, None)
        finally:
            self.client.stop()
        self.fake.stop()
        fake_camera.restore(self.tc)
        self._tmp.cleanup()

    def test_the_body_shape_matches_what_the_page_sends(self) -> None:
        # hud.py posts {"command": ..., "arg": ...}; a drift here means every button silently 422s while
        # the page still looks idle.
        self.assertIn('JSON.stringify({ command: cmd, arg: arg || "" })', HUD_JS)
        r = self.http.post("/api/command", json={"command": "set_mode", "arg": "AUTO_TRACK"})
        self.assertEqual(r.status_code, 200, r.text)
        body = r.json()
        self.assertIn("ok", body)
        self.assertIn("error", body, "the page reads `error` to explain a refusal")

    def test_the_page_is_served_with_the_dock_in_place(self) -> None:
        html = self.http.get("/").text
        for needle in ('id="dock"', 'id="drawer"', "hudDockSpecs", "hudDrawerActions", "setDrawer"):
            self.assertIn(needle, html, "%s never reached the served page" % needle)



class CommandVerdictIsNotAmbiguous(unittest.TestCase):
    """`ok` answers "did the gate take it". Nothing on the wire said so, and the HUD read it as "the
    station did it". The two answers are now named, at every hop, because a field that one hop drops is
    a field the next reader cannot see.
    """

    maxDiff = None

    # controld's emitter is C++; the case TheCommandResponseSaysWhichQuestionItAnswered in
    # control/tests/test_web_server.cpp covers that side. Everything below is the chain the HUD reads.

    def test_a_response_from_the_daemon_arrives_with_its_verdict(self) -> None:
        from ..protocol import parse_message

        # Spelled as controld's C++ emitter writes it, because that is the line webd has to read. There
        # is no Python response encoder to round-trip through - responses are produced by the daemon - so
        # the only honest version of this test quotes the wire.
        submitted = '{"type":"response","command":"select_target","ok":true,"verdict":"submitted"}'
        kind, msg = parse_message(submitted)
        self.assertEqual(kind, "response")
        self.assertEqual(msg.verdict, "submitted",
                         "the parser must declare the field or it vanishes and the ambiguity is back")

        rejected = ('{"type":"response","command":"select_target","ok":false,'
                    '"verdict":"rejected","error":"needs the number shown on screen"}')
        _, refused = parse_message(rejected)
        self.assertEqual(refused.verdict, "rejected")
        self.assertIn("number", refused.error)

        # A daemon that predates the field must still parse, with the verdict unknown rather than False,
        # so that "we did not hear" cannot be read as "it was refused".
        _, old = parse_message('{"type":"response","command":"hold","ok":true}')
        self.assertIsNone(old.verdict)

    def test_an_unknown_verdict_stays_unknown(self) -> None:
        # The client's own timeout answer. Asserting a refusal for a command that may well have executed
        # would be the same error pointed the other way.
        src = pathlib.Path(__file__).resolve().parents[1].joinpath("controld_client.py").read_text()
        at = src.index("no response within timeout")
        self.assertNotIn("verdict=", src[at - 200:at + 200],
                         "the timeout path must leave the verdict None, not invent one")

    def test_the_endpoint_passes_the_verdict_through(self) -> None:
        import json as _json
        import os
        import tempfile
        from fastapi.testclient import TestClient
        from ..app import create_app
        from ..controld_client import ControldClient
        from ..config import WebConfig
        from ..fake_controld import FakeControld
        from . import fake_camera

        tc = fake_camera.install(frame_delay=0.0)
        tmp = tempfile.TemporaryDirectory(prefix="ota_webd_verdict_")
        sock = os.path.join(tmp.name, "controld.sock")
        fake = FakeControld(sock, telemetry_hz=20.0)
        fake.start()
        client = ControldClient(sock, reconnect_interval=0.05)
        app = create_app(client, WebConfig(host="127.0.0.1", port=0, socket_path=sock))
        try:
            http = TestClient(app)
            http.__enter__()
            r = http.post("/api/command", json={"command": "hold", "arg": ""})
            self.assertEqual(r.status_code, 200, r.text)
            self.assertEqual(r.json().get("verdict"), "submitted",
                             "webd must not be the hop that loses the distinction")
            http.__exit__(None, None, None)
        finally:
            client.stop()
            fake.stop()
            fake_camera.restore(tc)
            tmp.cleanup()


if __name__ == "__main__":
    unittest.main()
