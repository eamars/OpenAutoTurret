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


def test_mode_row_degrades_when_controld_is_older():
    """During any rollout the dashboard is newer than the daemon for a while. The
    old controld has no `set_mode` and no operating_mode field, so the mode buttons
    would be pressable, get refused, and leave the operator reasoning about a
    rejection the page could have predicted. One telemetry field of difference is
    enough to know — so the page must use it."""
    assert 'const v3 = !!t.operating_mode;' in DASHBOARD_HTML
    assert "btn.disabled = !v3;" in DASHBOARD_HTML
    assert 'id="mode-unsupported"' in DASHBOARD_HTML


def _vocabulary():
    import re as _re
    from pathlib import Path

    header = (Path(__file__).resolve().parents[3]
              / "control" / "src" / "web" / "command_validation.hpp")
    return set(_re.findall(r'command\s*==\s*"([a-z_]+)"',
                           header.read_text(encoding="utf-8")))


def _mode_row():
    html = DASHBOARD_HTML
    row = html[html.index('id="mode-controls"'):]
    return row[: row.index("</section>")]


def test_selection_controls_are_in_the_operators_row():
    """§45/§12: choosing a target is an operator action, so it sits with the mode
    controls — and there is now exactly one place on the page that sends it.

    The legacy version lived in the developer section as a bare number box with a
    0..15 range nobody documented, in a part of the page an operator has no reason to
    open. Two inputs for one command is also how the wrong number gets sent with no way
    to tell from the page which box was read.
    """
    row = _mode_row()
    assert 'data-cmd="select_target"' in row
    assert 'data-cmd="clear_target"' in row
    assert 'id="sel-index"' in row
    assert 'id="selection"' in DASHBOARD_HTML
    assert 'id="selection-note"' in DASHBOARD_HTML
    assert 'id="sel-target"' not in DASHBOARD_HTML, (
        "a second select-target input came back; it would be dead, and a dead control "
        "that looks live is the bug this project already paid for once")
    assert "clear_target" in _vocabulary(), (
        "the button names a command controld does not accept")


def test_stop_motion_is_not_reachable_by_the_version_skew_disable():
    """§27: STOP MOTION is accepted in every state, including fault.

    The v3-absent degradation disables the mode row's buttons so an operator cannot
    press something an old controld will ignore. Doing that to STOP MOTION would take
    the one control that must always be available and make it depend on the version of
    a daemon — the exact inversion of what it is for.
    """
    js = _script()
    assert ":not(.danger)" in js, "the disable selector must exclude the danger control"
    row = _mode_row()
    assert '<button class="danger" data-cmd="stop_motion">' in row, (
        "STOP MOTION must keep the class the selector excludes")


def test_ambiguous_reacquisition_tells_the_operator_to_reselect():
    """§21: when controld refuses to guess, the page must not stay calm.

    "remain LOST_HOLD and ask the operator to reselect" is only useful if the asking
    survives the trip to the screen. Otherwise the turret sits still and the dashboard
    looks like it is working, which is the failure an operator notices only after
    walking over to the station.
    """
    js = _script()
    assert "selection_ambiguous" in js
    assert "AMBIGUOUS" in js
    assert "selection_visibility" in js
    assert "LOST_REACQUIRABLE" in js


def test_manual_jog_row_renews_faster_than_the_lease_it_holds():
    """§38, checked across the two languages that have to agree.

    The dead-man lease only works if the browser asks to renew it comfortably inside the
    deadline controld is holding. Either number on its own is a constant; the *ratio* is
    the safety property, and both sides are parsed from source rather than repeated here
    — a list copied into a test certifies nothing once the code moves.
    """
    from pathlib import Path

    root = Path(__file__).resolve().parents[3]
    ctrl = (root / "control" / "src" / "mode" / "manual_controller.hpp").read_text(
        encoding="utf-8"
    )
    lease = re.search(r"int64_t lease_ms = (\d+)", ctrl)
    assert lease, "the jog lease timeout moved shape; update this guard"
    lease_ms = int(lease.group(1))

    keepalive = re.search(r"JOG_KEEPALIVE_MS = (\d+)", DASHBOARD_HTML)
    assert keepalive, "the dashboard no longer names its keepalive interval"
    keep_ms = int(keepalive.group(1))

    assert keep_ms > 0 and lease_ms > 0
    # Three renewals inside the lease: two lost requests are survivable, and a third
    # lapses the jog rather than extending it indefinitely on a bad link.
    assert keep_ms * 3 <= lease_ms, (
        f"browser renews every {keep_ms} ms against a {lease_ms} ms lease: the turret "
        "would stop under a held button on any ordinary network hiccup"
    )
    # And not so fast that the page becomes a request generator on the control path.
    assert keep_ms >= 50, f"{keep_ms} ms keepalive floods the command path"

    for cmd in ("manual_jog_start", "manual_jog_keepalive", "manual_jog_stop"):
        assert cmd in DASHBOARD_HTML, f"{cmd} is not wired from the page"
    # Release must be bound to *every* way a held pointer silently disappears. A missing
    # one is not visible until someone's tab dies while they hold yaw+.
    for event in ("pointerdown", "pointerup", "pointercancel", "blur",
                  "visibilitychange", "pagehide"):
        assert event in DASHBOARD_HTML, f"{event} is not handled by the jog control"


def test_step_choices_offered_are_the_ones_controld_allows():
    """§41: 0.5 / 1 / 5 degrees. The page must not offer a step controld would refuse.

    The refusal itself is correct behaviour, but a button that is *designed* to be
    refused is the dead-button problem wearing a different hat: the operator pressed it,
    and something on the screen has to be wrong for that to be a normal outcome.
    """
    from pathlib import Path

    root = Path(__file__).resolve().parents[3]
    # The list lives where the refusal lives: control_loop.cpp checks an incoming step
    # against it, so that is the file that actually decides what is allowed.
    ctrl = (root / "control" / "src" / "control" / "control_loop.cpp").read_text(
        encoding="utf-8"
    )
    allowed = re.search(r"const double allowed\[[0-9]+\] = \{([^}]*)\}", ctrl)
    assert allowed, "the sanctioned step sizes moved shape; update this guard"
    sizes = sorted(float(x) for x in allowed.group(1).split(","))

    # The selected= attribute sits before the closing bracket on one of these, so the
    # pattern must stop at the quote: a stricter pattern silently returns two of the
    # three choices, which would make the guard below compare the wrong sets.
    offered = sorted(float(x) for x in re.findall(
        r'<option value="([0-9.]+)"', DASHBOARD_HTML.split('id="step-size"')[1][:400]))
    assert offered == sizes, f"page offers {offered}, controld allows {sizes}"

    # Directions: four jog buttons, both axes, both signs.
    jogs = set(re.findall(r'data-jog="([^"]+)"', DASHBOARD_HTML))
    assert jogs == {"yaw+", "yaw-", "pitch+", "pitch-"}, jogs


def test_candidate_list_is_built_from_telemetry_not_from_clicks():
    """§11: the operator chooses from what the machine sees, not from what was last pressed.

    Three things are checked because each has been the failure: the list must read the
    published `tracks` array (a page that invents its own list is a second source of
    truth), it must treat an aged list as unusable (controld deliberately does not rewrite
    track states between frames — §58 puts association in visiond — so freshness is the
    page's job), and the labels must reach the DOM as text, because a label is assembled
    from a class name that arrived over a socket from another process.
    """
    src = DASHBOARD_HTML
    assert "t.tracks" in src, "the candidate list is not read from the telemetry array"
    assert "track_list_age_ms" in src, "the list's freshness is not consulted"
    assert "select_target" in src, "candidate buttons do not name a target"

    # No markup interpolation of anything that came off the wire.
    inject = re.findall(r"innerHTML\s*=\s*[`\"][^`\"]*\$\{", src)
    assert not inject, f"template interpolation into innerHTML: {inject[:3]}"

    # The grey-out must be a threshold with a reason, not a bare truthiness test.
    assert re.search(r"track_list_age_ms[^\n]*\n?[^\n]*>\s*\d+", src, re.M) or \
        re.search(r"age\s*>\s*\d+", src), "no staleness threshold on the candidate list"


def test_webd_declares_the_fields_the_dashboard_reads():
    """The daemon re-serialises telemetry through its own dataclass, so any field the
    page reads that the dataclass does not declare simply does not arrive — with no error
    anywhere. `selected_track_id` once looked like a vision bug for exactly this reason.
    """
    from pathlib import Path

    proto = (Path(__file__).resolve().parents[1] / "protocol.py").read_text(
        encoding="utf-8"
    )
    for field_name in ("tracks", "track_count", "track_list_age_ms", "selected_display_index",
                       "selection_visibility", "manual_lease_active", "roam_sweep_direction",
                       "confidence_band"):
        assert re.search(rf"^\s*{field_name}\s*:", proto, re.M), (
            f"webd does not declare {field_name}: the dashboard would read undefined "
            "and the dashboard's own fallback text would hide it"
        )
