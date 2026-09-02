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
