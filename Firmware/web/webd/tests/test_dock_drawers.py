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
                        "\nmodule.exports = { hudDockSpecs, hudDrawerActions, hudDiagRows };\n")
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

    def _rows(self, drawer: str, telemetry: dict):
        return self._node("console.log(JSON.stringify(T.hudDrawerActions(%s, %s)));"
                          % (json.dumps(drawer), json.dumps(telemetry)))

    # -- the dock itself ---------------------------------------------------------------

    def test_the_five_buttons_are_the_five_the_revision_names(self) -> None:
        spec = self._node("console.log(JSON.stringify(T.hudDockSpecs({})));")
        self.assertEqual([b["key"] for b in spec], ["TARGETS", "MODE", "MANUAL", "DIAG", "MENU"],
                         "§13 lists these five in this order")
        self.assertTrue(all(not b["active"] for b in spec))

    def test_exactly_one_button_shows_the_open_drawer(self) -> None:
        spec = self._node("console.log(JSON.stringify(T.hudDockSpecs({open:\"MANUAL\"})));")
        self.assertEqual([b["key"] for b in spec if b["active"]], ["MANUAL"])

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
        rows = self._rows("MODE", {"operating_mode": "MANUAL"})
        self.assertEqual([r["arg"] for r in rows], ["MANUAL", "AUTO_TRACK", "AUTO_ROAM"])
        self.assertEqual([r["label"] for r in rows], ["MANUAL", "AUTO TRACK", "AUTO ROAM"])
        self.assertIsNone(rows[0]["command"], "the mode already in force must not be re-issued")
        self.assertEqual(rows[1]["command"], "set_mode")

    # -- MANUAL ------------------------------------------------------------------------

    def test_jog_and_step_arguments_match_the_daemon(self) -> None:
        rows = self._rows("MANUAL", {"operating_mode": "MANUAL"})
        self.assertEqual([r["arg"] for r in rows if r["command"] == "manual_jog_start"],
                         ["yaw-", "yaw+", "pitch+", "pitch-"],
                         "parse_jog_arg accepts exactly these four directions")
        for r in [r for r in rows if r["command"] == "manual_step"]:
            self.assertRegex(r["arg"], r"^(yaw|pitch)[+-][0-9.]+$")

    def test_stop_motion_is_always_offered_and_is_hold(self) -> None:
        for mode in ("MANUAL", "AUTO_TRACK", "AUTO_ROAM"):
            rows = self._rows("MANUAL", {"operating_mode": mode})
            stop = [r for r in rows if r["label"] == "STOP MOTION"]
            self.assertEqual(len(stop), 1, "a stop that vanishes in some modes is not a stop")
            self.assertEqual(stop[0]["command"], "hold")
            self.assertEqual(stop[0]["kind"], "stop", "§14 reserves red for stop and fault")

    def test_jogs_are_shown_gated_rather_than_hidden_outside_manual(self) -> None:
        rows = self._rows("MANUAL", {"operating_mode": "AUTO_TRACK"})
        jogs = [r for r in rows if r["command"] == "manual_jog_start"]
        self.assertTrue(all(r["kind"] == "gated" for r in jogs))
        self.assertTrue(all("MANUAL" in r["note"] for r in jogs),
                        "the reason belongs on the row; a control that silently greys out teaches "
                        "guessing")

    # -- MENU / DIAG -------------------------------------------------------------------

    def test_supervisory_actions_need_two_presses(self) -> None:
        rows = self._rows("MENU", {})
        danger = [r for r in rows if r["kind"] == "danger"]
        self.assertEqual({r["command"] for r in danger}, {"request_park", "request_shutdown"})
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

    def test_only_the_daemons_published_ack_may_say_accepted(self) -> None:
        # Found by posting a command the station could not honour: controld logged
        # "select_target 9999: REFUSED (no vision data has reached controld yet)" while /api/command
        # answered ok:true. A page that renders the socket reply as ACCEPTED would tell an operator the
        # turret had acquired a target that does not exist. So the reply is reported as SENT, and
        # ACCEPTED/REFUSED come only from cmd_ack_accepted on the next snapshot.
        at = HUD_JS.index("async function sendCommand")
        body = HUD_JS[at:HUD_JS.index("dock.addEventListener", at)]
        self.assertIn('"  SENT"', body, "the socket reply must not be called a verdict")
        # The literal, not the word: the comment beside it explains why it may not be claimed, and a test
        # that cannot tell prose from code will pass or fail on the colour of the comment.
        self.assertNotIn('"  ACCEPTED"', body, "acceptance may not be claimed from the response")
        self.assertIn('"  ACCEPTED"', HUD_JS[HUD_JS.index("function resolveAckFromTelemetry"):
                                              HUD_JS.index("async function sendCommand")],
                      "the resolver is the only place allowed to say it")
        self.assertLess(HUD_JS.index("function resolveAckFromTelemetry"), HUD_JS.index('"  SENT"'),
                        "the ack resolver belongs beside the sender that feeds it")
        self.assertIn("cmd_ack_accepted", HUD_JS)
        self.assertIn("cmd_ack_seq", HUD_JS, "matched on sequence, or a stale ack would answer a new command")
        self.assertIn("NO ACK FROM CONTROLD", HUD_JS,
                      "a command that never gets an ack must not be left looking accepted")


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

    def test_the_page_tells_submitted_and_rejected_apart(self) -> None:
        at = HUD_JS.index("async function sendCommand")
        body = HUD_JS[at:HUD_JS.index("dock.addEventListener", at)]
        self.assertIn('j.verdict === "rejected"', body,
                      "a gate rejection is a decision and must show as one, at once")
        self.assertIn('"  SUBMITTED"', body, "a receipt for queueing must not be dressed as success")
        self.assertIn('"  SENT"', body, "an older daemon with no verdict gets honest wording, not a guess")
        self.assertNotIn('"  ACCEPTED"', body, "acceptance still belongs to cmd_ack_accepted alone")

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
