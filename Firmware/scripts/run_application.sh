#!/usr/bin/env bash
# One supervised stack. Default: perception only, with no motor connection.
# --sim adds the real controller/web processes with simulated motors.
# --hardware starts the real controller, which homes during boot; modes stay operator-driven.
set -euo pipefail
APP="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PY="${OTA_PYTHON:-$APP/../.venv/bin/python}"
RUN="${OTA_RUN_DIR:-/tmp/ota-stack-$UID}"
PROFILE=person_detect_available
FRAMES=0
MODE=perception
START_WEB=1
PRODUCTION=0
while [ $# -gt 0 ]; do
  case "$1" in
    --hold-motion) MODE=perception; shift ;;
    --sim) MODE=sim; shift ;;
    --hardware) MODE=hardware; shift ;;
    --no-web) START_WEB=0; shift ;;
    --production) PRODUCTION=1; shift ;;
    --profile) PROFILE="${2:?--profile requires a name}"; shift 2 ;;
    --frames) FRAMES="${2:?--frames requires a count}"; shift 2 ;;
    --dev) set -x; shift ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
done
[[ "$FRAMES" =~ ^[0-9]+$ ]] || { echo '--frames must be non-negative' >&2; exit 2; }
[ -x "$PY" ] || { echo "Project Python missing: $PY; set OTA_PYTHON" >&2; exit 2; }
umask 077
mkdir -p "$RUN"
exec 9>"$RUN/launcher.lock"
flock -n 9 || { echo "A stack already owns $RUN" >&2; exit 1; }
cd "$APP"
export OTA_VISION_FRAME_TAP="$RUN/preview.jpg"
export OTA_SELECTION_SOCKET="$RUN/selection.sock"
export OTA_VISION_SOCKET="$RUN/vision.sock"
export OTA_WEB_SOCKET="$RUN/control-web.sock"
export OTA_WEB_PORT="${OTA_WEB_PORT:-8080}"
export OTA_WEB_HOST="${OTA_WEB_HOST:-0.0.0.0}"
vision_args=(--config perception/configs/perception_v1.json --profile "$PROFILE"
             --max-frames "$FRAMES" --publish-dir "$RUN/perception"
             --selection-socket "$OTA_SELECTION_SOCKET")
if [ "$PRODUCTION" -eq 1 ]; then vision_args+=(--production); fi
# Validate before any controller can home or enable motors.
"$PY" -c 'import sys; from perception.visiond import build_parser,load_config; load_config(build_parser().parse_args(sys.argv[1:]))' "${vision_args[@]}"
if [ "$MODE" = perception ]; then
  echo "Perception only: no controller connection. Diagnostics: $RUN/perception"
  exec "$PY" -m perception.visiond "${vision_args[@]}"
fi
CONTROLD="$APP/build/control/controld"
[ -x "$CONTROLD" ] || { echo "Build controld first: $CONTROLD" >&2; exit 2; }
children=()
cleanup() {
  trap - EXIT INT TERM
  echo 'Stopping this stack; controller performs its own park/disable sequence.'
  for pid in "${children[@]}"; do kill -INT "$pid" 2>/dev/null || true; done
  # Never SIGKILL a hardware controller while it is parking.
  for pid in "${children[@]}"; do wait "$pid" || true; done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
controller_args=("$APP/config/turret.yaml")
if [ "$MODE" = sim ]; then
  controller_args+=(--sim)
  echo 'SIMULATED motors; camera and web are real.'
else
  echo 'HARDWARE controller: boot homing can move both axes. No mode is selected automatically.'
fi
"$CONTROLD" "${controller_args[@]}" >"$RUN/controller.log" 2>&1 &
children+=("$!")
if [ "$START_WEB" -eq 1 ]; then
  "$PY" -m web.webd.app >"$RUN/web.log" 2>&1 &
  children+=("$!")
  vision_args+=(--controller-state-url "http://127.0.0.1:$OTA_WEB_PORT/api/state")
fi
"$PY" -m perception.visiond "${vision_args[@]}" --publish-socket "$OTA_VISION_SOCKET" >"$RUN/vision.log" 2>&1 &
children+=("$!")
echo "Stack logs: $RUN; web port: $OTA_WEB_PORT. Ctrl-C stops this stack."
# A failed child or finite capture ends its own stack; unrelated processes are untouched.
wait -n "${children[@]}"
