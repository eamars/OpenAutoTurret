"""Execute production menu update logic in Node; no browser or video access."""
import re
import subprocess

from web.webd.hud import HUD_GEOMETRY_JS, HUD_JS


def main():
    paint = re.search(r"function paint\(t\) \{.*?\n\}", HUD_JS, re.S).group()
    script = HUD_GEOMETRY_JS + """
let lastTelemetry = null, lastTelemetryAt = 0, transportOk = false;
let drawerOpen = 'MENU', rows = [], updates = 0;
function render() {}
function resolveAckFromTelemetry() {}
function renderDrawer() { rows = hudDrawerActions(drawerOpen, lastTelemetry); ++updates; }
""" + paint + """
// Owner ruling 2026-10-03: HOME is the way out of everything but its own run; SHUTDOWN needs a homed turret.
for (const phase of ['hold', 'parking', 'fault', 'recovering', 'idle', 'parked', 'homing']) {
  paint({phase, operating_mode: 'MANUAL', tracks: []});
  const home = rows.find(r => r.label === 'HOME'), off = rows.find(r => r.label === 'SHUTDOWN');
  const homeExpected = !['recovering', 'homing'].includes(phase);
  const offExpected = ['hold', 'parked', 'parking'].includes(phase);
  const homeEnabled = home.command === 'start_homing', offEnabled = off.command === 'request_shutdown';
  console.log(JSON.stringify({phase, homeEnabled, homeExpected, offEnabled, offExpected, updates}));
  if (homeEnabled !== homeExpected || offEnabled !== offExpected) process.exitCode = 1;
}
paint({phase: 'parked', operating_mode: 'MANUAL', tracks: []});
const before = updates;
for (let i = 0; i < 50; ++i)
  paint({phase: 'parked', operating_mode: 'MANUAL', tracks: [{uuid: String(i)}]});
console.log(JSON.stringify({trackChurnMenuReplacements: updates - before}));
if (updates !== before) process.exitCode = 1;
"""
    result = subprocess.run(["node", "-e", script], check=False)
    raise SystemExit(result.returncode)


if __name__ == "__main__":
    main()
