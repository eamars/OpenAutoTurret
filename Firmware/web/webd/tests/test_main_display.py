"""The main display is a station state, and both panes heal by one rule (owner, 2026-10-02).

1. "The PIP camera requires refresh every time it starts (if the old session exists). The main
   camera feed doesn't." After a redeploy the new webd has started neither stream; the main pane
   asked for its stream again, the PIP only re-requested one nobody had started, so it stayed frozen
   until a reload. Both panes now run hudPaneStep: start a stopped stream, re-point a restarted one.
2. "Only the one on the main display should be fed into the AI HAT." The swap therefore asks the
   station, every page follows the published answer, and every overlay is drawn in the wide frame
   mapped onto whichever picture is on the main display.
"""
from __future__ import annotations

import json
import os
import shutil
import socket
import subprocess
import tempfile
import threading
import time
import unittest

from ..hud import HUD_GEOMETRY_JS, HUD_HTML
from ..selection_client import request_main_camera
from ..video import VideoSource


@unittest.skipUnless(shutil.which("node"), "node not installed; the page's rules cannot be executed")
class PageRulesExecuted(unittest.TestCase):
    def _node(self, expr: str):
        with tempfile.TemporaryDirectory() as box:
            geo = os.path.join(box, "geo.js")
            with open(geo, "w", encoding="utf-8") as fh:
                fh.write(HUD_GEOMETRY_JS + "\nmodule.exports = { hudPaneStep, hudMainView, "
                         "hudProject, hudViewFov, hudAcceptMainView };\n")
            main = os.path.join(box, "main.js")
            with open(main, "w", encoding="utf-8") as fh:
                fh.write("const T = require(%s);\nconsole.log(JSON.stringify(%s));\n"
                         % (json.dumps(geo), expr))
            r = subprocess.run(["node", main], capture_output=True, text=True, timeout=30)
        self.assertEqual(r.returncode, 0, r.stderr)
        return json.loads(r.stdout)

    def test_a_pane_after_a_redeploy_starts_its_stream_whichever_pane_it_is(self):
        # The new webd: the source this pane showed is not running in this process.
        self.assertEqual(self._node('T.hudPaneStep({epoch: "old:1", lastAttemptMs: 0}, '
                                    '{running: false}, 10000)'), "start")

    def test_a_stream_restarted_under_the_pane_is_re_pointed_even_without_an_error_event(self):
        # Another page (or the other pane's poll) already started it: running, but not the stream
        # this <img> is connected to.
        self.assertEqual(self._node('T.hudPaneStep({epoch: "old:1"}, {running: true, '
                                    'epoch: "new:1"}, 10000)'), "point")
        self.assertEqual(self._node('T.hudPaneStep({epoch: "new:1"}, {running: true, '
                                    'epoch: "new:1"}, 10000)'), "none")

    def test_a_refusing_source_is_asked_at_a_spaced_rate_not_in_a_loop(self):
        self.assertEqual(self._node('T.hudPaneStep({lastAttemptMs: 9500}, {running: false}, 10000)'),
                         "wait")
        self.assertEqual(self._node('T.hudPaneStep({lastAttemptMs: 8000}, {running: false}, 10000)'),
                         "start")

    def test_the_main_display_comes_from_the_station_and_defaults_to_wide(self):
        self.assertEqual(self._node("T.hudMainView({})")["main"], "wide")
        fresh = ('{inference: {present: true, fresh: true, main_camera: {role: "detail", '
                 'generation: 3, available: ["wide", "detail"], views: {detail: {scale: 5.9}}}}}')
        view = self._node("T.hudMainView(%s)" % fresh)
        self.assertEqual((view["main"], view["pip"], view["k"], view["generation"]),
                         ("detail", "wide", 5.9, 3))
        stale = fresh.replace("fresh: true", "fresh: false")
        self.assertEqual(self._node("T.hudMainView(%s)" % stale)["main"], "wide",
                         "a stale report must not keep the picture zoomed in")

    def test_a_report_older_than_the_answered_swap_is_ignored_within_one_visiond(self):
        self.assertIsNone(self._node('T.hudAcceptMainView({boot: "a", generation: 4}, '
                                     '{boot: "a", generation: 3})'))
        self.assertEqual(self._node('T.hudAcceptMainView({boot: "a", generation: 4}, '
                                    '{boot: "a", generation: 5})'), {"boot": "a", "generation": 5})

    def test_a_restarted_visiond_is_followed_although_its_generation_starts_again(self):
        # A page that saw generation 4 must not ignore the restarted daemon's generation 0 (wide on
        # the main display) as "older" and keep showing detail.
        self.assertEqual(self._node('T.hudAcceptMainView({boot: "a", generation: 4}, '
                                    '{boot: "b", generation: 0})'), {"boot": "b", "generation": 0})

    def test_overlays_land_on_the_detail_picture_where_visiond_put_them(self):
        lay = "{ok: true, ox: 0, oy: 0, w: 1000, h: 500, k: 5.9}"
        # A point the detail camera saw at 0.8 of its width was published at 0.5 + 0.3/5.9.
        p = self._node("T.hudProject(0.5 + 0.3 / 5.9, 0.5, %s)" % lay)
        self.assertAlmostEqual(p["x"], 800.0, places=6)
        centre = self._node("T.hudProject(0.5, 0.5, %s)" % lay)
        self.assertAlmostEqual(centre["x"], 500.0)
        self.assertAlmostEqual(self._node("T.hudProject(0.25, 0.5, {ok: true, ox: 0, oy: 0, "
                                          "w: 1000, h: 500})")["x"], 250.0)

    def test_the_tapes_window_is_the_detail_cameras_field_of_view(self):
        self.assertAlmostEqual(self._node("T.hudViewFov(69.3, 5.9)"), 13.4, places=1)
        self.assertEqual(self._node("T.hudViewFov(69.3, 1)"), 69.3)


class PageStructure(unittest.TestCase):
    def test_both_panes_are_polled_and_started_by_the_same_functions(self):
        self.assertIn("for (const pane of [panes.main, panes.pip])", HUD_HTML)
        self.assertIn("paneErrored(panes.main)", HUD_HTML)
        self.assertIn("paneErrored(panes.pip)", HUD_HTML)
        self.assertNotIn("otaPipTick", HUD_HTML, "the PIP's separate, weaker path is gone")

    def test_the_swap_asks_the_station_instead_of_swapping_the_page(self):
        self.assertIn('fetch("/api/camera/main"', HUD_HTML)


class SwapRelay(unittest.TestCase):
    def test_the_request_reaches_visiond_and_its_answer_comes_back(self):
        with tempfile.TemporaryDirectory() as box:
            path = os.path.join(box, "sel.sock")
            server = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
            server.bind(path)
            server.listen(1)
            seen = {}

            def serve():
                conn, _ = server.accept()
                with conn:
                    seen.update(json.loads(conn.recv(4096)))
                    conn.sendall(json.dumps({"accepted": True,
                                             "main_camera": {"role": "detail"}}).encode())

            thread = threading.Thread(target=serve)
            thread.start()
            reply = request_main_camera(path, "detail")
            thread.join(2)
            server.close()
        self.assertEqual(seen, {"type": "set_main_camera", "role": "detail"})
        self.assertTrue(reply["accepted"])

    def test_an_unknown_role_never_leaves_webd(self):
        self.assertFalse(request_main_camera("/nonexistent", "zoom")["accepted"])


class TapReaderDeath(unittest.TestCase):
    def test_a_source_whose_tap_vanished_stops_claiming_to_run(self):
        with tempfile.TemporaryDirectory() as box:
            tap = os.path.join(box, "preview.jpg")
            with open(tap, "wb") as fh:
                fh.write(b"\xff\xd8jpeg")
            source = VideoSource()
            state = source.start(640, 360, 10.0, 80, tap_path_override=tap, stream_role="detail")
            self.assertTrue(state.running)
            self.assertEqual(source.starts, 1)
            os.unlink(tap)
            deadline = time.monotonic() + 20
            while source.is_running() and time.monotonic() < deadline:
                time.sleep(0.1)
            self.assertFalse(source.is_running(),
                             "a dead reader that still says running is never restarted by a page")
            source.stop()


if __name__ == "__main__":
    unittest.main()
