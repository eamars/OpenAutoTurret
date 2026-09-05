#!/usr/bin/env bash
#
# run_application.sh — bring up the WHOLE OpenAutoTurret application and enter
# roam/tracking automatically. This is the single entrypoint for the core feature.
#
# What "bring up everything" means, in one place:
#   * the CAN/motor link  (controld owns it; turret.yaml points at the USB-CAN adapter)
#   * controld            (the control loop: homing, envelope, modes, the motor)
#   * perception.visiond  (IMX500 target detection; publishes the selected target as a
#                          58-byte TargetMeasurement to controld's socket each frame)
#   * webd                (the live web UI)
# and then it drives the turret into the autonomous mode sequence:
#   home → AUTO_ROAM (sweep/search) → [app's own track_on_acquire_ms hand-off] → AUTO_TRACK
#
# The launcher does NOT hand-assemble or bypass safety. It asks for the mode; the control loop's
# envelope, hard limits, safety authority and the CAN supervisor decide what may actually move.
# When a target appears in view, perception.visiond's publisher hands it to controld and
# AUTO_TRACK follows it; on loss the app returns to AUTO_ROAM on its own.
#
# Usage:  sudo ./run_application.sh [--no-web] [--dev] [--profile NAME] [--frames N]
#   --no-web            skip webd (for headless / debugging)
#   --profile NAME      the perception model profile (default person_detect_available)
#   --frames N          run perception for N frames then stop (default: live, run until killed)
#
# Safety: it refuses to home/roam unless controld reports at_ready, and it never issues a raw
# motor command — only mode requests via controld's own API.

set -uo pipefail

APP=/home/eamars/workspace/OpenAutoTurret/Firmware
PY=/usr/bin/python3
CONFIG="$APP/config/turret.yaml"
CONTROLD="$APP/build/control/controld"
CONTROL_SOCK=/run/ota/controld-web.sock
VISION_SOCK=/tmp/ota_vision.sock
WEB_PORT=8080
API="http://127.0.0.1:$WEB_PORT"

PLAN_SCHEMA_VERSION=1
PROFILE=person_detect_available
FRAMES="0"              # 0 = live
START_WEB=1
MOTION_ALLOWED=1        # gate: 0 = bring everything up but nothing may move
while [ $# -gt 0 ]; do
  case "$1" in
    --no-web) START_WEB=0; shift ;;
    --profile) PROFILE="$2"; shift 2 ;;
    --frames) FRAMES="$2"; shift 2 ;;
    --hold-motion) MOTION_ALLOWED=0; shift ;;
    --dev) set -x; shift ;;
    *) echo "unknown option $1"; exit 2 ;;
  esac
done

log() { echo "[run_application] $*"; }
die() { echo "[run_application] FATAL: $*" >&2; exit 1; }

# --- preflight ----------------------------------------------------------------
[ -x "$CONTROLD" ] || die "controld not built at $CONTROLD"
[ -f "$CONFIG" ] || die "config not found at $CONFIG"
mkdir -p /run/ota logs

if [ "$MOTION_ALLOWED" -eq 1 ]; then
  log "MOTION ALLOWED: the turret may home and roam. Ctrl-C now to abort."
  sleep 1
fi

cmd() { curl -s -m 5 -H "Content-Type: application/json" \
  -d "{\"command\":\"$1\",\"arg\":\"$2\"}" "$API/api/command" 2>/dev/null; }
field() { curl -s -m 3 "$API/api/state" 2>/dev/null | python3 -c \
  "import json,sys; d=json.load(sys.stdin); print(d.get('$1',''))" 2>/dev/null; }

# --- stop stale peers ----------------------------------------------------------
# A leftover visiond/pipewire holding the sensor would make the live run open a busy camera.
# This launcher owns the stack, so it clears its own predecessors and lets the app come up clean.
log "stopping any stale in-tree vision daemon and its sensor holders"
pkill -KILL -f "perception.visiond" 2>/dev/null || true
pkill -KILL -f "vision.visiond" 2>/dev/null || true
# pipewire/wireplumber hold the camera as a user session; restart them so they release it.
if [ "$MOTION_ALLOWED" -eq 1 ]; then
  sudo -u eamars env XDG_RUNTIME_DIR=/run/user/1000 \
    DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus \
    systemctl --user stop pipewire.socket pipewire wireplumber 2>/dev/null || true
fi
sleep 2

# --- start controld (the motor owner) ------------------------------------------
if ! pgrep -f "$CONTROLD" >/dev/null; then
  log "starting controld"
  OTA_WEB_SOCKET="$CONTROL_SOCK" OTA_WEB_HZ=15 setsid nohup \
    "$CONTROLD" "$CONFIG" > logs/controld.log 2>&1 </dev/null &
  sleep 2
else
  log "controld already running"
fi

# --- start web UI --------------------------------------------------------------
if [ "$START_WEB" -eq 1 ] && ! pgrep -f "web.webd.app" >/dev/null; then
  log "starting web UI"
  OTA_WEB_HOST=0.0.0.0 OTA_WEB_PORT="$WEB_PORT" OTA_WEB_SOCKET="$CONTROL_SOCK" OTA_WEB_HZ=15 \
    OTA_VIDEO_ORIENTATION=rotate_180 setsid nohup \
    "$PY" -m web.webd.app > logs/webd.log 2>&1 </dev/null &
fi

# --- wait for controld + web readiness -----------------------------------------
log "waiting for controld + web to come up"
for i in $(seq 1 30); do
  [ "$(field operating_mode)" != "" ] && break
  sleep 1
done
[ "$(field operating_mode)" != "" ] || die "controld/ web did not come up (see logs/controld.log)"
log "controld ready: mode=$(field operating_mode) phase=$(field mode_phase) at_ready=$(field at_ready)"

# --- start perception vision (live tracking publisher) --------------------------
log "starting perception vision (profile=$PROFILE, publish-socket=$VISION_SOCK)"
FRAMES_ARG=""
[ "$FRAMES" != "0" ] && FRAMES_ARG="--max-frames $FRAMES"
set +e
OTA_VISION_FRAME_TAP="${OTA_VISION_FRAME_TAP:-/tmp/ota_vision_frame.jpg}" \
  setsid nohup "$PY" -m perception.visiond \
    --config configs/perception_v1.json --profile "$PROFILE" \
    --publish-socket "$VISION_SOCK" ${FRAMES_ARG:-} --quiet \
    > logs/vision.log 2>&1 </dev/null &
VISION_PID=$!
set -e
sleep 3
if ! kill -0 "$VISION_PID" 2>/dev/null; then
  die "perception vision failed to start (see logs/vision.log)"
fi
log "perception vision started (pid $VISION_PID)"

# --- if motion is gated off, we're done (dry-run) --------------------------------
if [ "$MOTION_ALLOWED" -eq 0 ]; then
  log "MOTION HELD: stack is up. The turret will not move. Home/roam skipped."
  log "  watch: $API/api/state  |  vision log: logs/vision.log"
  exit 0
fi

# --- home (only if not already homed) -------------------------------------------
HOMED=$(field homed 2>/dev/null || field home_done 2>/dev/null)
if [ "$HOMED" != "1" ] && [ "$(field mode_phase)" != "HOLD" ]; then
  log "requesting start_homing"
  cmd start_homing "" >/dev/null
  for i in $(seq 1 120); do
    PH=$(field mode_phase)
    [ "$PH" = "HOLD" ] || [ "$PH" = "ROAM" ] && break
    sleep 2
  done
  log "homing: mode_phase=$(field mode_phase)"
else
  log "already homed/positioned; skipping home"
fi

# --- AUTO_ROAM (the app auto-hand-offs to AUTO_TRACK on a confirmed target) -------
log "requesting AUTO_ROAM"
cmd set_mode AUTO_ROAM >/dev/null
for i in $(seq 1 30); do
  [ "$(field operating_mode)" = "AUTO_ROAM" ] && break
  sleep 1
done
log "mode now: $(field operating_mode) / phase=$(field mode_phase) at_ready=$(field at_ready)"

log "--------------------------------------------------------------"
log "  APPLICATION IS UP. Core feature:"
log "    roam:  operating_mode=$(field operating_mode) phase=$(field mode_phase)"
log "    track: the app hand-offs to AUTO_TRACK on a confirmed target;"
log "           the motor moves via $CONFIG's USB-CAN adapter."
log "  Watch it live:  $API/api/state"
log "                  $( [ "$START_WEB" -eq 1 ] && echo "http://127.0.0.1:$WEB_PORT" || echo '(web disabled)')"
log "  Vision log: logs/vision.log   Control log: logs/controld.log"
log "--------------------------------------------------------------"
log "Ctrl-C stops nothing (daemons run on). Stop with systemctl or pkill."
