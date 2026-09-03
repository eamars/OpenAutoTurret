"""The dashboard's JavaScript must PARSE. That is the whole file's purpose.

Why this exists: the dashboard is a single inline ``<script>`` in an HTML string,
so nothing in the Python or C++ test suites ever looked at it. One bad line — a
CAN-state map written as ``{-1: [...]}``, which is not a valid property name —
made the entire script throw at parse time. Every statement died with it,
``connect()`` included, while the HTML around it still rendered. The operator
sees a page that loads and never connects: indistinguishable from "the backend
is down", and the daemon really was fine.

So: hand the extracted script to ``node --check`` if node exists on this machine
(it does on the station). Where node genuinely isn't installed the test skips
loudly rather than pretending to have verified anything, and one cheap textual
guard against the exact shape that caused this stays on either path.
"""
from __future__ import annotations

import re
import shutil
import subprocess
import tempfile
import unittest
import os

from ..dashboard import DASHBOARD_HTML


def _script() -> str:
    m = re.search(r"<script>(.*)</script>", DASHBOARD_HTML, re.S)
    assert m, "dashboard has no <script> block"
    return m.group(1)


class DashboardJSTest(unittest.TestCase):

    def test_no_object_literal_starts_a_key_with_a_sign(self):
        # The bug that took the dashboard down. Valid: {a: 1}, {[-1]: 1}.
        # Invalid: {-1: 1} — a leading sign is an expression, not a name.
        #
        # This is a cheap guard for machines without node, so it is allowed to be
        # crude — but crude in the SAFE direction: occurrences inside a comment
        # are skipped (this very file's explanation of the bug would otherwise
        # match it, which is exactly how a whole-text search burned the flag check
        # in tools/install_station.py). test_dashboard_js_parses is the real gate.
        js = _script()
        for m in re.finditer(r"\{\s*[-+]\s*[0-9]", js):
            line_before = js[js.rfind("\n", 0, m.start()) + 1:m.start()]
            if "//" in line_before:
                continue
            self.fail("object literal key starts with a sign (JavaScript "
                      f"syntax error): ...{js[max(0, m.start()-30):m.start()+30]}...")

    def test_dashboard_js_parses(self):
        node = shutil.which("node") or shutil.which("nodejs")
        if not node:
            self.skipTest("no node on this machine: the dashboard's JavaScript "
                          "is NOT being syntax-checked here")
        with tempfile.NamedTemporaryFile("w", suffix=".js", delete=False) as fh:
            fh.write(_script())
            path = fh.name
        try:
            proc = subprocess.run([node, "--check", path],
                                  capture_output=True, text=True, timeout=30)
        finally:
            os.unlink(path)
        self.assertEqual(
            proc.returncode, 0,
            "dashboard JavaScript does not parse — the page would load and then "
            f"never connect:\n{proc.stdout}\n{proc.stderr}")

    def test_connect_is_still_wired_up(self):
        # A parse error is not the only way connect() stops happening.
        js = _script()
        self.assertIn("function connect()", js)
        self.assertIn("connect();", js)
        self.assertIn('"/ws"', js)
        # ...and it must not depend solely on onclose, which a hidden tab freezes.
        self.assertIn("setInterval", js)
        self.assertIn("visibilitychange", js)


if __name__ == "__main__":
    unittest.main()


def test_dashboard_ids_are_unique():
    """The JS guard above proves the script PARSES. It cannot prove the handlers
    ever get attached: getElementById returns the FIRST match, so a duplicated id
    silently wires one panel and leaves the other's buttons inert — the page looks
    healthy, every button in it does nothing, and no test notices. This one does.
    (Caught in the v3 mode row, which reused the developer panel's id="controls".)
    """
    from collections import Counter

    ids = re.findall(r'id="([^"]+)"', DASHBOARD_HTML)
    dupes = sorted(i for i, n in Counter(ids).items() if n > 1)
    assert not dupes, f"duplicate element ids would leave controls unwired: {dupes}"


def test_every_button_is_wired():
    """Every button on the dashboard must name a command controld accepts.

    The vocabulary is PARSED from controld's validator
    (control/src/web/command_validation.hpp) rather than duplicated here, because a
    hardcoded list drifts and a drifting list passes: it would keep certifying the
    buttons as real long after the daemon stopped honouring them. That is precisely
    how `enable_search` came to answer ok:true for a request nobody acted on, and
    how `select_target` still does.
    """
    import re as _re
    from pathlib import Path

    header = (Path(__file__).resolve().parents[3]
              / "control" / "src" / "web" / "command_validation.hpp")
    text = header.read_text(encoding="utf-8")
    accepted = set(_re.findall(r'command\s*==\s*"([a-z_]+)"', text))
    assert accepted, f"no commands parsed from {header} — the validator changed shape"

    used = set(re.findall(r'data-cmd="([^"]+)"', DASHBOARD_HTML))
    assert used, "no buttons found — the dashboard changed shape"
    unknown = sorted(used - accepted)
    assert not unknown, (
        "dashboard offers buttons controld does not accept: "
        f"{unknown}; validator knows {sorted(accepted)}"
    )


def test_mode_buttons_exist_and_spelling_is_exact():
    """§45: the mode selector is the operator's primary control. The argument
    spelling must match what controld parses character-for-character — a lower-case
    or renamed variant is refused at runtime, and a refused button looks identical
    to a working one until someone presses it."""
    for mode in ("MANUAL", "AUTO_TRACK", "AUTO_ROAM"):
        assert f'data-cmd="set_mode" data-mode="{mode}"' in DASHBOARD_HTML, (
            f"no button selects {mode} (§45)"
        )
    assert 'data-cmd="stop_motion"' in DASHBOARD_HTML, (
        "§27: STOP MOTION has to be reachable in one press, from any mode"
    )
