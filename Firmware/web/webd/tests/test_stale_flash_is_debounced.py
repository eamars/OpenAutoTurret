"""The §25 overlay must not fire on the browser's own hiccups.

Measured on this station on 2026-09-06, with an operator watching the page and reporting that it
flashed "telemetry lost" intermittently: controld's telemetry aged at most **66 ms** over 19,335
samples, and the page's WebSocket stayed regular at **p50 66 ms / max 80 ms even while another
client drained 1.25 MiB/s of MJPEG video**. Nothing upstream was stale. Two paths in the page could
still paint the overlay anyway:

  * `ws.onclose` sets `transportOk = false` and reconnects after **1000 ms**, and the predicate
    treated a closed socket as staleness *instantly* — so every dropped socket announced a station
    fault for a second and then took it back.
  * a main thread stalled decoding 1080p MJPEG stops running `onmessage` for as long as it likes;
    one overdue 250 ms tick was enough to declare the link quiet.

Both were fixed on the caller side (`updateStaleness`), leaving the pure `hudStale` predicate and its
existing tests alone. That means the fix lives in code no existing test executes, so this file runs
the real function in node against a scripted clock. A test that could not have failed before the fix
is not evidence: every expectation below is chosen because the previous code gives the opposite
answer, and the notes say which.
"""

import json
import shutil
import subprocess
import sys
import textwrap
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from webd.hud import HUD_JS  # noqa: E402

NODE = shutil.which("node")

# A stub world small enough to be honest about: only the surface updateStaleness touches.
HARNESS = textwrap.dedent("""
    // The page wires itself up at load - docks, drawers, buttons all call addEventListener during
    // module evaluation - so the stub has to be a world, not one element. Only "viewport" is real,
    // because that is the element whose verdict we are actually measuring.
    const mk = () => ({
      classList: { toggle: () => {}, add: () => {}, remove: () => {}, contains: () => false },
      addEventListener: () => {}, appendChild: () => {}, removeChild: () => {}, setAttribute: () => {},
      insertAdjacentHTML: () => {}, focus: () => {}, querySelector: () => mk(), querySelectorAll: () => [],
      style: {}, dataset: {}, value: "", textContent: "", innerHTML: "",
      getBoundingClientRect: () => ({ width: 1600, height: 900, left: 0, top: 0 }),
    });
    const viewport = mk();
    const toggles = [];
    viewport.classList.toggle = (name, on) => { toggles.push([name, !!on]); };
    globalThis.document = {
      getElementById: (id) => (id === "viewport" ? viewport : mk()),
      addEventListener: () => {}, createElement: () => mk(), createElementNS: () => mk(),
      body: mk(), documentElement: mk(),
    };
    globalThis.window = globalThis;
    globalThis.addEventListener = () => {};
    globalThis.removeEventListener = () => {};
    globalThis.requestAnimationFrame = () => 0;
    globalThis.navigator = { userAgent: "node" };
    globalThis.location = { protocol: "http:", host: "127.0.0.1:8080" };
    globalThis.matchMedia = () => ({ matches: false, addEventListener: () => {}, addListener: () => {} });
    globalThis.WebSocket = function () { this.close = () => {}; };
    globalThis.setInterval = () => 0;
    globalThis.setTimeout = () => 0;

    let clock = 100000;
    Date.now = () => clock;

    const SRC = __SRC__;
    // Indirect eval => global scope, so the appended hook can reach the module's own `let` bindings
    // without any of them being made public for the sake of testing.
    (0, eval)(SRC + `
      globalThis.__drive = (t, ageMs, transport) => {
        lastTelemetry = t; lastTelemetryAt = Date.now() - ageMs; transportOk = transport;
        return updateStaleness(t);
      };
      globalThis.__reset = () => { linkOverdue = 0; };
    `);

    const HEALTHY = { telemetry_stale: false, track_list_age_ms: 15 };
    const JANK = { telemetry_stale: false, track_list_age_ms: 15 };
    const out = {};

    // 1. Healthy: fresh message, server says fine. Must be false before and after the fix.
    out.healthy = __drive(HEALTHY, 40, true);

    // 2. The reported flash, part one: the socket closed, reconnect is in flight, and the last
    //    message is only 200 ms old. The old code returned TRUE here. It must not.
    out.closed_but_fresh = __drive(HEALTHY, 200, false);

    // 3. The reported flash, part two: a stalled main thread, 1600 ms since any message ran.
    //    One overdue tick must not declare it; the second one must.
    __reset();
    out.jank_tick1 = __drive(JANK, 1600, true);
    out.jank_tick2 = __drive(JANK, 1900, true);

    // 4. Server truth is never debounced: controld itself says stale -> first tick.
    __reset();
    out.server_stale_at_once = __drive({ telemetry_stale: true, track_list_age_ms: 15 }, 40, true);

    // 5. A link that really is gone still gets declared: quiet for seconds, two ticks.
    __reset();
    out.dead_link_tick1 = __drive(HEALTHY, 4000, true);
    out.dead_link_tick2 = __drive(HEALTHY, 4250, true);

    // 6. Recovery: a fresh message after a declared loss clears it on the next tick.
    out.recovered = __drive(HEALTHY, 30, true);

    console.log(JSON.stringify(out));
""").replace("__SRC__", json.dumps(HUD_JS))


@pytest.mark.skipif(NODE is None, reason="node not installed")
def test_page_clock_hiccups_do_not_assert_stale_but_station_truth_still_does():
    proc = subprocess.run([NODE, "-e", HARNESS], capture_output=True, text=True, timeout=60)
    assert proc.returncode == 0, f"node harness failed: {proc.stderr[-800:]}"
    got = json.loads(proc.stdout.strip().splitlines()[-1])

    assert got["healthy"] is False, "a healthy station must never be declared stale"

    # The instant-on-close false positive the operator saw.
    assert got["closed_but_fresh"] is False, (
        "a WebSocket that closed one second before its scheduled reconnect, with a message 200 ms "
        "old, is the page's condition - not a station that stopped believing what it was doing")

    # Jank needs two consecutive overdue ticks; a resumed main thread drains its queue and cannot
    # produce two, a dead link always does.
    assert got["jank_tick1"] is False, "one overdue tick is a stalled main thread, not a lost link"
    assert got["jank_tick2"] is True, "the second tick is the grace running out - that must declare"

    # What must never be softened.
    assert got["server_stale_at_once"] is True, (
        "§25 staleness reported by controld acts on the first tick, whatever the page thinks")
    assert got["dead_link_tick2"] is True, (
        "debouncing must delay a real loss by one 250 ms tick, not hide it")
    assert got["recovered"] is False, "the verdict must clear as soon as messages are fresh again"
