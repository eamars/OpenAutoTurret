#!/usr/bin/env bash
# One supervised stack. No arguments starts the real automatic station.
# --sim adds the real controller/web processes with simulated motors.
# start (or no action) detaches; run stays in the foreground.
set -euo pipefail
APP="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PY="${OTA_PYTHON:-$APP/../run/station-venv/bin/python}"
if [ ! -x "$PY" ] && [ -z "${OTA_PYTHON:-}" ]; then PY="$APP/../.venv/bin/python"; fi
RUN="${OTA_RUN_DIR:-/tmp/ota-stack-$UID}"
ACTION=start
case "${1:-}" in
  start|run|stop|status|check|deploy) ACTION="$1"; shift ;;
  -h|--help)
    echo 'Usage: run_application.sh [deploy|check|start|run|status|stop] [options]'
    echo 'No action: background start, using config/turret.yaml (AUTO_ROAM).'
    echo 'deploy: build, test and preflight this checkout; does not start motors.'
    echo 'deploy --probe-build: build only controld and preflight; defer regression tests.'
    echo 'run: foreground supervision; stop: controlled park/disable and full stack cleanup.'
    echo 'Options: --sim (real camera), --hold-motion (camera only), --profile NAME,'
    echo '         --no-web, --frames N, --production, --dev. See docs/STATION_OPERATIONS.md.'
    exit 0 ;;
esac
options=("$@")
if [[ "$ACTION" = stop || "$ACTION" = status ]] && [ "$#" -ne 0 ]; then
  echo "$ACTION takes no options; use the same account and OTA_RUN_DIR as start." >&2
  exit 2
fi
owned_launcher() {
  [ -r "$RUN/launcher.pid" ] || return 1
  read -r launcher_pid launcher_start < "$RUN/launcher.pid"
  [[ "$launcher_pid" =~ ^[0-9]+$ && "$launcher_start" =~ ^[0-9]+$ ]] || return 1
  [ -r "/proc/$launcher_pid/stat" ] || return 1
  [ "$(awk '{print $22}' "/proc/$launcher_pid/stat")" = "$launcher_start" ]
}
stopped_status() {
  if [ -r "$RUN/shutdown.result" ]; then
    cat "$RUN/shutdown.result"
  else
    echo 'Stopped (last park outcome unavailable)'
  fi
}
if [ "$ACTION" = status ]; then
  if owned_launcher; then
    echo "Running (launcher $launcher_pid); checkout: $(readlink "/proc/$launcher_pid/cwd")"
    [ ! -r "$RUN/stack.info" ] || cat "$RUN/stack.info"
    echo "Logs: $RUN/{launcher,controller,vision,web}.log"
    # Process ownership is not motor readiness. Query the active stack's port,
    # not an environment override belonging to this status caller.
    if [ -r "$RUN/web.port" ]; then
      read -r port < "$RUN/web.port"
      if [[ "$port" =~ ^[0-9]+$ ]]; then
        curl --max-time 2 -fsS "http://127.0.0.1:$port/api/state" || echo 'Web telemetry unavailable (starting/stopping or failed).'
        echo
      fi
    fi
  else stopped_status; exit 1; fi
  exit 0
fi
if [ "$ACTION" = stop ]; then
  if ! owned_launcher; then echo 'Already stopped'; stopped_status; exit 0; fi
  kill -TERM "$launcher_pid"
  for ((attempt=0; attempt<120; attempt++)); do
    if ! owned_launcher; then stopped_status; exit 0; fi
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
PROBE_BUILD=0
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
    --probe-build) PROBE_BUILD=1; shift ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
done
if [ "$PROBE_BUILD" = 1 ] && [ "$ACTION" != deploy ]; then
  echo '--probe-build is only valid for deploy' >&2; exit 2
fi
[[ "$FRAMES" =~ ^[0-9]+$ ]] || { echo '--frames must be non-negative' >&2; exit 2; }
[ -x "$PY" ] || { echo "Project Python missing: $PY; set OTA_PYTHON" >&2; exit 2; }
umask 077
mkdir -p "$RUN"
cd "$APP"
if [ "$ACTION" = deploy ]; then
  # Build in place only while this checkout is inactive. Other release trees
  # may be built without replacing the executable/config of a running stack.
  if owned_launcher && [ "$(readlink "/proc/$launcher_pid/cwd")" = "$APP" ]; then
    echo 'Stop this checkout before deployment, or build a separate release directory.' >&2
    exit 1
  fi
  cmake -S "$APP" -B "$APP/build" -DCMAKE_BUILD_TYPE=Release
  if [ "$PROBE_BUILD" = 1 ]; then
    cmake --build "$APP/build" --target controld -j"${OTA_BUILD_JOBS:-2}"
    echo 'Probe build: regression tests deferred until runtime viability is established.'
  else
    cmake --build "$APP/build" -j"${OTA_BUILD_JOBS:-2}"
    ctest --test-dir "$APP/build" --output-on-failure
  fi
  ACTION=check
fi
if [ "$ACTION" = start ]; then
  # Serialize concurrent SSH starts; the child closes this descriptor.
  exec 11>"$RUN/start.lock"
  flock 11
  if owned_launcher; then
    if [ "$(readlink "/proc/$launcher_pid/cwd")" != "$APP" ]; then
      echo 'Another checkout owns the station; stop it before starting this release.' >&2
      exit 1
    fi
    echo "Already running (launcher $launcher_pid). Use status for readiness."
    exit 0
  fi
  nohup setsid bash "$APP/scripts/run_application.sh" run "${options[@]}" \
    >"$RUN/launcher.log" 2>&1 < /dev/null 11>&- &
  child=$!
  for ((attempt=0; attempt<200; attempt++)); do
    if owned_launcher && [ -r "$RUN/started" ] &&
        [ "$(cat "$RUN/started")" = "$launcher_pid $launcher_start" ]; then
      echo "Started (launcher $launcher_pid). Homing may take about six minutes."
      echo "Web: http://$(hostname):${OTA_WEB_PORT:-8080}/; logs: $RUN"
      exit 0
    fi
    if ! kill -0 "$child" 2>/dev/null; then
      wait "$child" || true
      cat "$RUN/launcher.log" >&2
      exit 1
    fi
    sleep 0.1
  done
  echo "Startup still pending; inspect $RUN/launcher.log and run status. No process was killed." >&2
  exit 1
fi
# Check is read-only: no camera open, CAN connection, or motor enable.
"$PY" "$APP/tools/station_preflight.py" "${OTA_CONTROL_CONFIG:-$APP/config/turret.yaml}" "$MODE" "$PROFILE" "$PRODUCTION"
if [ "$ACTION" = check ]; then
  echo 'Preflight passed. deploy/check do not start the station.'
  exit 0
fi
exec 9>"$RUN/launcher.lock"
flock -n 9 || { echo "A stack already owns $RUN" >&2; exit 1; }
children=()
controller_pid=''
cleanup() {
  trap - EXIT INT TERM
  echo 'Stopping this stack; controller performs its own park/disable sequence.'
  for pid in "${children[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
  # Never force-kill the motor controller. Its own deadlines supervise park.
  if [ -n "$controller_pid" ]; then wait "$controller_pid" || true; fi
  # Keep the terminal controller outcome after ownership metadata is removed.
  # A clean process exit alone does not prove that the motors reached park.
  if [ -n "$controller_pid" ]; then
    if grep -q 'PARKED (motors de-energized' "$RUN/controller.log"; then
      echo 'Stopped: PARKED (both axes de-energized)' > "$RUN/shutdown.result"
    else
      { echo 'Stopped: PARK FAILED or park not confirmed';
        tail -n 8 "$RUN/controller.log"; } > "$RUN/shutdown.result"
    fi
    cat "$RUN/shutdown.result"
  fi
  for pid in "${children[@]}"; do
    [ "$pid" != "$controller_pid" ] || continue
    for ((attempt=0; attempt<50; attempt++)); do
      kill -0 "$pid" 2>/dev/null || break
      sleep 0.1
    done
    if kill -0 "$pid" 2>/dev/null; then kill -KILL "$pid" 2>/dev/null || true; fi
    wait "$pid" || true
  done
  rm -f -- "$RUN/launcher.pid" "$RUN/started" "$RUN/stack.info" "$RUN/web.port"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
rm -f -- "$RUN/shutdown.result"
printf '%s %s\n' "$$" "$(awk '{print $22}' /proc/$$/stat)" > "$RUN/launcher.pid"
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
CONTROLD="$APP/build/control/controld"
controller_args=("${OTA_CONTROL_CONFIG:-$APP/config/turret.yaml}")
if [ "$MODE" = sim ]; then
  controller_args+=(--sim)
  echo 'SIMULATED motors; camera and web are real.'
elif [ "$MODE" = hardware ]; then
  echo 'HARDWARE station: homing establishes calibration, then automatic roam/track begins. Web Manual overrides autonomy.'
fi
if [ "$MODE" != perception ]; then
if [ "$MODE" = hardware ] && pgrep -x controld >/dev/null; then
  echo 'A controller already runs outside this launcher. Resolve its owner; do not start a second CAN owner.' >&2
  exit 1
fi
"$CONTROLD" "${controller_args[@]}" >"$RUN/controller.log" 2>&1 &
controller_pid=$!
children+=("$controller_pid")
else
  echo 'Perception only: no controller connection.'
  START_WEB=0
fi
if [ "$START_WEB" -eq 1 ]; then
  "$PY" -m web.webd.app >"$RUN/web.log" 2>&1 &
  children+=("$!")
  printf '%s\n' "$OTA_WEB_PORT" > "$RUN/web.port"
  vision_args+=(--controller-state-url "http://127.0.0.1:$OTA_WEB_PORT/api/state")
fi
if [ "$MODE" != perception ]; then vision_args+=(--publish-socket "$OTA_VISION_SOCKET"); fi
"$PY" -m perception.visiond "${vision_args[@]}" >"$RUN/vision.log" 2>&1 &
children+=("$!")

printf 'Mode: %s\nConfig: %s\nPython: %s\nChildren: %s\n' "$MODE" "${controller_args[0]}" "$PY" "${children[*]}" > "$RUN/stack.info"
cp "$RUN/launcher.pid" "$RUN/started"
echo "Stack logs: $RUN; web port: $OTA_WEB_PORT. Ctrl-C stops this stack."
# A failed child or finite capture ends its own stack; unrelated processes are untouched.
wait -n "${children[@]}"
