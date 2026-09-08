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
for (const phase of ['hold', 'parking', 'fault', 'recovering', 'idle', 'parked']) {
  paint({phase, operating_mode: 'MANUAL', tracks: []});
  const recovery = rows.find(r => r.command === 'recover_motors');
  const expected = ['fault', 'idle', 'parked'].includes(phase);
  const enabled = recovery.kind !== 'gated';
  console.log(JSON.stringify({phase, enabled, expected, updates}));
  if (enabled !== expected) process.exitCode = 1;
}
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
