#!/usr/bin/env bash
# Drive-mode item 2: measure the position lead on hardware, and ALWAYS leave the station as it was found.
#
# Why a script and not hand commands: the trial has to (a) change a control parameter, (b) move the axis, and
# (c) put the parameter back. Hand-run, the middle step is where an interrupted session leaves an unsigned control
# change running. Everything below runs inside `trap ... EXIT`, so a Ctrl-C, a crash or a failed measurement still
# restores the config and restarts controld against the original file.
#
# pkill -x controld is deliberate: `-x` matches the process NAME only, so it cannot match this script's own command
# line (the mistake that SIGTERM'd my own shell twice this session, see PROGRESS round ~10).
set -uo pipefail

FIRMWARE="$(cd "$(dirname "$0")/.." && pwd)"
CFG="$FIRMWARE/config/turret.yaml"
BAK="/tmp/turret.yaml.lead-trial-backup"
LEAD="${1:-0.1}"                      # seconds of lead to trial
S="${2:-http://127.0.0.1:8080}"

state() { curl -s -m 5 "$S/api/state" | python3 -c "import json,sys;d=json.load(sys.stdin);print(d.get('operating_mode'),d.get('mode_phase'),d.get('safety_action'),d.get('at_ready'))" 2>/dev/null; }

restore() {
  echo "=== RESTORING (this runs on every exit path)"
  [ -f "$BAK" ] && cp "$BAK" "$CFG" && echo "config restored from $BAK"
  pkill -x controld 2>/dev/null; sleep 3
  ( cd "$FIRMWARE" && OTA_BLACKBOX_DIR=/var/lib/ota/blackbox setsid nohup ./build/control/controld config/turret.yaml \
      > /tmp/controld_lead-trial-restore.log 2>&1 < /dev/null & )
  sleep 25; echo "post-restore state: $(state)"
}
trap restore EXIT

echo "=== BEFORE"; cp "$CFG" "$BAK"; echo "backed up to $BAK"; echo "state: $(state)"

python3 - "$CFG" "$LEAD" <<'PY'
import pathlib, re, sys
cfg, lead = pathlib.Path(sys.argv[1]), float(sys.argv[2])
s = cfg.read_text()
pat = r"\n( *)#? *position_lead_s:.*"
if not re.search(pat, s):
    print("ERROR: no position_lead_s line to trial; refusing to continue", file=sys.stderr); sys.exit(2)
s = re.sub(pat, f"\n\\1position_lead_s: {lead}", s, count=1)
cfg.write_text(s); print(f"trial value written: position_lead_s: {lead}")
PY
[ $? -ne 0 ] && { echo "aborting before any motion"; exit 2; }

pkill -x controld 2>/dev/null; sleep 3
( cd "$FIRMWARE" && OTA_BLACKBOX_DIR=/var/lib/ota/blackbox setsid nohup ./build/control/controld config/turret.yaml \
    > /tmp/controld_lead-trial.log 2>&1 < /dev/null & )

# A controld restart does NOT come back ready: the second run of this script sat for 150 s on `MANUAL HOLD ALLOW
# False` and correctly refused to move, because homing state is not restored from the drive - it is established by
# homing. So the trial asks for it explicitly (control_loop.cpp:2973 "start_homing") and gives it room. Homing is real
# motion to the limits by design; if that is not wanted, do not run this script.
echo "=== waiting for supervisory Ready, homing if needed (up to 300 s)"
ready=""; homed=0
for _ in $(seq 1 100); do
  sleep 3; s4=$(state); echo "  $s4"
  case "$s4" in *"ALLOW True"*) ready=1; break;; esac
  if [ "$homed" -eq 0 ]; then
    echo "  not ready - requesting homing (verdict below)"
    curl -s -m 10 -X POST -H "Content-Type: application/json" -d '{"command":"start_homing","arg":""}' "$S/api/command" | head -c 160; echo
    homed=1
  fi
done
if [ -z "$ready" ]; then echo "NOT READY - no motion will be commanded; restoring"; exit 3; fi

python3 - "$S" "$LEAD" <<'PY'
import json, statistics, sys, time, urllib.request
S, lead = sys.argv[1], float(sys.argv[2]); D = 57.29577951308232
def st():
    with urllib.request.urlopen(S + "/api/state", timeout=4) as f: return json.load(f)
def cmd(c, a=""):
    r = urllib.request.Request(S + "/api/command", data=json.dumps({"command": c, "arg": a}).encode(),
                              headers={"Content-Type": "application/json"}, method="POST")
    with urllib.request.urlopen(r, timeout=8) as f: return json.load(f)
rows = []; bad = None
# The first run of this script produced 1501 samples of a STATION THAT NEVER MOVED: measured and reference were both
# pinned at 148.27 deg, and following error came out a triumphant 0.000. The reason is that nothing checked whether
# AUTO_ROAM had actually been entered - a mode request that is refused still returns a JSON body, so the loop just
# politely sampled a stationary axis and reported a perfect score. Verifying the mode is not optional here; without it
# the script can report success on a turret that did nothing, which is worse than reporting nothing.
v = cmd("set_mode", "AUTO_ROAM")
print("set_mode(AUTO_ROAM) verdict:", v)
entered = None
for _ in range(30):
    time.sleep(0.5)
    d = st()
    if d.get("operating_mode") == "AUTO_ROAM":
        entered = d.get("mode_phase"); break
if entered is None:
    print("!! AUTO_ROAM was never entered - REFUSING to measure. Current state:", st())
    sys.exit(4)
print(f"AUTO_ROAM entered (phase {entered}); sweeping before sampling so the reference is genuinely moving")
for _ in range(20):
    time.sleep(0.5)
    if st().get("mode_phase") == "SWEEP": break
t0 = time.time() + 50
try:
    while time.time() < t0:
        d = st(); act = d.get("safety_action")
        rows.append({"t": time.time(), "meas": d.get("q_yaw_rad"), "ref": d.get("q_ref_yaw_rad"), "act": act})
        if act not in (None, "ALLOW"): bad = act; break
        time.sleep(0.03)
finally:
    json.dump({"lead": lead, "rows": rows}, open("/tmp/r104_lead_after.json", "w"))
    if bad: print(f"!! safety action {bad} — stopping"); cmd("stop_motion")
    cmd("set_mode", "MANUAL"); time.sleep(1.2); cmd("set_mode", "AUTO_TRACK")
def p(v, q): return sorted(v)[min(len(v) - 1, int(q * len(v)))] if v else 0.0
sw = [x for x in rows if all(isinstance(x[k], (int, float)) for k in ("meas", "ref"))]
e = [abs(x["ref"] - x["meas"]) * D for x in sw]
meas = [x["meas"] * D for x in sw]; ref = [x["ref"] * D for x in sw]
print(f"\nAFTER  lead={lead}s  samples={len(e)}  safety_abort={bad}")
if e:
    print(f"following error deg: p50 {p(e,.5):.3f} p95 {p(e,.95):.3f} max {max(e):.3f}   (BEFORE: p50 3.629 p95 4.274 max 4.617)")
    print(f"measured extremes {min(meas):.2f}..{max(meas):.2f}  reference extremes {min(ref):.2f}..{max(ref):.2f}")
    # Overshoot, stated as two signed facts rather than one clever number: how far past the reference's own
    # high/low extreme the encoder actually went. Positive on either line means the axis went further than asked.
    print(f"past reference max by {max(meas)-max(ref):+.2f} deg | past reference min by {min(ref)-min(meas):+.2f} deg")
    print("(a lead trades following error for exactly these two numbers; if either goes positive by more than the")
    print(" lead in degrees, the envelope is being asked harder than the smoothness is worth)")
PY
