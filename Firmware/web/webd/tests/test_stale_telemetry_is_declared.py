"""§25: "stale telemetry stops visual interpolation and indicates stale/disconnected state".

Two halves, both measured rather than asserted from a reading of the source.

Server half: webd must know how old its own data is and must say so, because `controld_connected`
reports a SOCKET and nothing else. The station defect this closes is the one where the daemon stops
publishing while holding the connection open: every number on the page freezes, and the health
endpoint that is supposed to warn about it reports "connected". Age is therefore taken from the
arrival stamp and computed when the snapshot is READ, so property 4 below checks it climbing.

Page half: the staleness rule is one pure function, executed here under node from the page's own
source text. The previous page kept three comparisons inline in its render path, which is not
testable, and which cannot be checked against the five ways the picture can lie.
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import tempfile
import time
import unittest

from fastapi.testclient import TestClient

from ..app import create_app
from ..controld_client import ControldClient
from ..fake_controld import FakeControld
from ..hud import HUD_GEOMETRY_JS, HUD_JS
from ..protocol import TELEMETRY_STALE_AFTER_S
from . import fake_camera


class ServerDeclaresStaleness(unittest.TestCase):
    def setUp(self) -> None:
        self.tc = fake_camera.install(frame_delay=0.0)
        self._tmp = tempfile.TemporaryDirectory(prefix="ota_webd_stale_")
        self.sock = os.path.join(self._tmp.name, "controld.sock")
        self.fake = FakeControld(self.sock, telemetry_hz=20.0)
        self.fake.start()
        self.client = ControldClient(self.sock, reconnect_interval=0.05)
        self.config = _web_config(self.sock)
        self.app = create_app(self.client, self.config)
        self.http = TestClient(self.app)
        self.http.__enter__()
        deadline = time.time() + 5.0
        while time.time() < deadline and self.client.latest_telemetry() is None:
            time.sleep(0.05)

    def tearDown(self) -> None:
        try:
            self.http.__exit__(None, None, None)
        finally:
            self.client.stop()
        if not getattr(self, "_silenced", False):
            self.fake.stop()
        fake_camera.restore(self.tc)
        self._tmp.cleanup()

    def _go_silent(self) -> None:
        """Stop the fake daemon publishing, leaving webd holding its last frame.

        This is the state under test and it is not a contrivance: it is controld wedged, or the
        vision link stalled upstream of it. It has to be arranged by stopping the publisher rather
        than by writing a timestamp, because a live publisher simply overwrites whatever stamp the
        test injects - which is how the first draft of three of these tests failed, and it is a
        better lesson than the tests themselves.
        """
        if not getattr(self, "_silenced", False):
            self.fake.stop()
            self._silenced = True
            time.sleep(0.20)  # let an in-flight frame land, so the cache holds the last real one

    def _state(self) -> dict:
        r = self.http.get("/api/state")
        self.assertEqual(r.status_code, 200, r.text)
        return r.json()

    def test_fresh_data_is_not_called_stale(self) -> None:
        body = self._state()
        self.assertFalse(
            body["telemetry_stale"],
            "a page whose data is 50 ms old must not be shouting STALE; a banner that cries at "
            "normal jitter is a banner the operator learns to ignore",
        )
        self.assertIsInstance(body["telemetry_age_ms"], int)
        self.assertLess(body["telemetry_age_ms"], 500)

    def test_a_cached_frame_ages_even_though_nobody_receives_one(self) -> None:
        """The property the whole fix rests on: age is measured at read time.

        Stamping the age on arrival would produce a snapshot that reports itself fresh forever,
        because the last thing to touch it declared it recent.
        """
        self._go_silent()
        self.client._latest_mono = time.monotonic() - 3.0  # "last frame landed 3 s ago"
        first = self._state()
        self.assertTrue(first["telemetry_stale"])
        self.assertTrue(
            2400 <= first["telemetry_age_ms"] <= 4200,
            f"age should report the 3 s since the injected stamp, got {first['telemetry_age_ms']}",
        )
        time.sleep(0.30)
        second = self._state()
        self.assertGreaterEqual(
            second["telemetry_age_ms"] - first["telemetry_age_ms"],
            150,
            "age did not move while nobody received a frame - it is being stamped on arrival, "
            "which is exactly how a page ends up showing dead numbers as current",
        )

    def test_health_reports_age_beside_connection(self) -> None:
        """"connected but silent" is the diagnosis, so the two must be reported together."""
        self._go_silent()
        self.client._latest_mono = time.monotonic() - 2.0
        h = self.http.get("/api/health").json()
        self.assertIn("telemetry_age_ms", h)
        self.assertGreaterEqual(h["telemetry_age_ms"], 1500)

    def test_no_telemetry_yet_is_not_reported_as_zero_seconds_old(self) -> None:
        # A daemon that never answered must not read as "0 ms old", which is the same lie with the
        # sign changed. The field is absent/None, not zero.
        self._go_silent()
        self.client._latest = None
        self.client._latest_mono = None
        self.assertEqual(self.http.get("/api/state").status_code, 503)
        h = self.http.get("/api/health").json()
        self.assertIsNone(h["telemetry_age_ms"])


def _web_config(sock: str):
    from ..config import WebConfig

    return WebConfig(host="127.0.0.1", port=0, socket_path=sock)


# --- the page's rule, executed from its own source ------------------------------------------

_STALE_CASES = [
    # (name, arguments handed to hudStale, expected verdict)
    ("all fresh",
     dict(transportOk=True, telemetryStale=False, telemetryAgeMs=20, msgAgeMs=30,
          trackListAgeMs=40, staleAfterMs=500, quietAfterMs=1500, trackAfterMs=500), False),
    ("transport down",
     dict(transportOk=False, telemetryStale=False, telemetryAgeMs=None, msgAgeMs=0,
          trackListAgeMs=None, staleAfterMs=500, quietAfterMs=1500, trackAfterMs=500), True),
    ("server says stale",
     dict(transportOk=True, telemetryStale=True, telemetryAgeMs=900, msgAgeMs=0,
          trackListAgeMs=0, staleAfterMs=500, quietAfterMs=1500, trackAfterMs=500), True),
    # The station defect: socket open, health says connected, data stopped arriving.
    ("hung daemon on a live socket",
     dict(transportOk=True, telemetryStale=False, telemetryAgeMs=640, msgAgeMs=640,
          trackListAgeMs=0, staleAfterMs=500, quietAfterMs=1500, trackAfterMs=500), True),
    # A link that merely goes quiet, with no close event and a healthy server.
    ("silent link to this page only",
     dict(transportOk=True, telemetryStale=False, telemetryAgeMs=None, msgAgeMs=2000,
          trackListAgeMs=0, staleAfterMs=500, quietAfterMs=1500, trackAfterMs=500), True),
    # Different emergency: the attitude is live, the target list is not.
    ("target list died while attitude stayed live",
     dict(transportOk=True, telemetryStale=False, telemetryAgeMs=20, msgAgeMs=20,
          trackListAgeMs=900, staleAfterMs=500, quietAfterMs=1500, trackAfterMs=500), True),
    # Nothing known yet must read as "no evidence of staleness", not as "stale": at boot the page
    # has no ages to judge, and flagging that would flash a false alarm on every reload.
    ("nothing known yet",
     dict(transportOk=True, telemetryStale=False, telemetryAgeMs=None, msgAgeMs=None,
          trackListAgeMs=None, staleAfterMs=500, quietAfterMs=1500, trackAfterMs=500), False),
]


@unittest.skipUnless(shutil.which("node"), "node not installed; the page's rule cannot be executed")
class StalenessRuleExecutedFromPageSource(unittest.TestCase):
    """Runs the page's own hudStale under node. Not a re-implementation in Python."""

    def _run(self, script: str) -> str:
        with tempfile.NamedTemporaryFile("w", suffix=".js", delete=False) as fh:
            fh.write(HUD_GEOMETRY_JS + "\nmodule.exports = { hudStale };\n")
            geo = fh.name
        try:
            with tempfile.NamedTemporaryFile("w", suffix=".js", delete=False) as fh:
                fh.write("const { hudStale } = require(%r);\n%s" % (geo, script))
                main = fh.name
            r = subprocess.run(["node", main], capture_output=True, text=True, timeout=20)
        finally:
            os.unlink(geo)
            os.unlink(main)
        self.assertEqual(r.returncode, 0, r.stderr)
        return r.stdout.strip()

    def test_the_rule_agrees_with_the_page_thresholds(self) -> None:
        lines = ["var fails = 0;"]
        for name, args, want in _STALE_CASES:
            lines.append(
                "if (hudStale(%s) !== %s) { console.log('WRONG: %s'); fails++; }"
                % (json.dumps(args), "true" if want else "false", name)
            )
        lines.append("console.log(fails ? 'FAIL ' + fails : 'ok');")
        self.assertEqual(self._run("\n".join(lines)), "ok",
                         "a case above disagrees with the page's own rule")

    def test_thresholds_are_the_same_number_on_both_sides(self) -> None:
        # A page that calls data fresh at 600 ms while the server calls it stale at 500 ms is two
        # truths on one screen. The server's figure is imported, the page's is parsed out of source.
        want_ms = int(TELEMETRY_STALE_AFTER_S * 1000)
        self.assertIn("const STALE_AFTER_MS = %d;" % want_ms, HUD_JS)

    def test_the_watchdog_that_notices_silence_exists(self) -> None:
        # Without this the page only learns it is stale when something arrives to tell it.
        self.assertIn("QUIET_AFTER_MS", HUD_JS)
        self.assertIn("msgAgeMs", HUD_JS)
        self.assertIn("updateStaleness", HUD_JS)


if __name__ == "__main__":
    unittest.main()
