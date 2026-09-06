#!/usr/bin/env bash
# One supervised stack. No arguments starts the real automatic station.
# --sim adds the real controller/web processes with simulated motors.
# start (or no action) runs in the foreground; stop and status work from another shell.
set -euo pipefail
APP="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PY="${OTA_PYTHON:-$APP/../run/station-venv/bin/python}"
if [ ! -x "$PY" ] && [ -z "${OTA_PYTHON:-}" ]; then PY="$APP/../.venv/bin/python"; fi
RUN="${OTA_RUN_DIR:-/tmp/ota-stack-$UID}"
ACTION=start
if [[ "${1:-}" = start || "${1:-}" = stop || "${1:-}" = status ]]; then ACTION="$1"; shift; fi
owned_launcher() {
  [ -r "$RUN/launcher.pid" ] || return 1
  read -r launcher_pid launcher_start < "$RUN/launcher.pid"
  [[ "$launcher_pid" =~ ^[0-9]+$ && "$launcher_start" =~ ^[0-9]+$ ]] || return 1
  [ -r "/proc/$launcher_pid/stat" ] || return 1
  [ "$(awk '{print $22}' "/proc/$launcher_pid/stat")" = "$launcher_start" ]
}
if [ "$ACTION" = status ]; then
  if owned_launcher; then echo "Running (launcher $launcher_pid); web http://$(hostname):${OTA_WEB_PORT:-8080}/";
  else echo 'Stopped'; exit 1; fi
  exit 0
fi
if [ "$ACTION" = stop ]; then
  if ! owned_launcher; then echo 'Already stopped'; exit 0; fi
  kill -TERM "$launcher_pid"
  for ((attempt=0; attempt<120; attempt++)); do
    if ! owned_launcher; then echo 'Stopped'; exit 0; fi
    sleep 1
  done
  echo "Controller shutdown is still in progress; inspect $RUN/controller.log" >&2
  exit 1
fi
PROFILE=person_detect_available
FRAMES=0
MODE=hardware
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
printf '%s %s\n' "$$" "$(awk '{print $22}' /proc/$$/stat)" > "$RUN/launcher.pid"
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
  for pid in "${children[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
  # The first child is the controller. Wait for its park/disable without a
  # kill deadline. Camera drivers can remain blocked after a sensor stall;
  # bound cleanup of the non-motor children after the controller has exited.
  if [ "${#children[@]}" -gt 0 ]; then wait "${children[0]}" || true; fi
  for pid in "${children[@]:1}"; do
    for ((attempt=0; attempt<50; attempt++)); do
      kill -0 "$pid" 2>/dev/null || break
      sleep 0.1
    done
    if kill -0 "$pid" 2>/dev/null; then kill -KILL "$pid" 2>/dev/null || true; fi
    wait "$pid" || true
  done
  rm -f -- "$RUN/launcher.pid"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
controller_args=("${OTA_CONTROL_CONFIG:-$APP/config/turret.yaml}")
if [ "$MODE" = sim ]; then
  controller_args+=(--sim)
  echo 'SIMULATED motors; camera and web are real.'
else
  echo 'HARDWARE station: homing establishes calibration, then automatic roam/track begins. Web Manual overrides autonomy.'
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
