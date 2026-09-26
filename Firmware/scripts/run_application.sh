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
    echo '         --commission-hardware [--yaw-voltage N --pulse-ms N --observe-ms N],'
    echo '         --apply-pitch-limit (commissioning only; volatile <=5 A, no pitch enable),'
    echo '         --yaw-speed-deg-s N (commissioning PI loop; integer +/-5, <=1500 raw),'
    echo '         --probe-imu [--imu-seconds N] (IMU capture only, 1..120 seconds),'
    echo '         --with-imu (commissioning only; capture IMU alongside bounded motor probe),'
    echo '         --pitch-step-mdeg N (commissioning with IMU; +/-500 max, 5 A, 0.5 deg/s),'
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
YAW_VOLTAGE=0
YAW_SPEED_DEG_S=0
PULSE_MS=100
OBSERVE_MS=2000
APPLY_PITCH_LIMIT=0
IMU_SECONDS=10
WITH_IMU=0
PITCH_STEP_MDEG=0
while [ $# -gt 0 ]; do
  case "$1" in
    --hold-motion) MODE=perception; shift ;;
    --sim) MODE=sim; shift ;;
    --hardware) MODE=hardware; shift ;;
    --commission-hardware) MODE=commission; START_WEB=0; shift ;;
    --probe-imu) MODE=imu; START_WEB=0; shift ;;
    --with-imu) WITH_IMU=1; shift ;;
    --pitch-step-mdeg) PITCH_STEP_MDEG="${2:?--pitch-step-mdeg requires a value}"; shift 2 ;;
    --imu-seconds) IMU_SECONDS="${2:?--imu-seconds requires a value}"; shift 2 ;;
    --apply-pitch-limit) APPLY_PITCH_LIMIT=1; shift ;;
    --yaw-voltage) YAW_VOLTAGE="${2:?--yaw-voltage requires a signed value}"; shift 2 ;;
    --yaw-speed-deg-s) YAW_SPEED_DEG_S="${2:?--yaw-speed-deg-s requires a signed value}"; shift 2 ;;
    --pulse-ms) PULSE_MS="${2:?--pulse-ms requires a value}"; shift 2 ;;
    --observe-ms) OBSERVE_MS="${2:?--observe-ms requires a value}"; shift 2 ;;
    --no-web) START_WEB=0; shift ;;
    --production) PRODUCTION=1; shift ;;
    --profile) PROFILE="${2:?--profile requires a name}"; shift 2 ;;
    --frames) FRAMES="${2:?--frames requires a count}"; shift 2 ;;
    --dev) set -x; shift ;;
    --probe-build) PROBE_BUILD=1; shift ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
done
if [ "$PITCH_STEP_MDEG" != 0 ]; then
  if [ "$MODE" != commission ] || [ "$WITH_IMU" != 1 ] || [ "$YAW_VOLTAGE" != 0 ] || [ "$YAW_SPEED_DEG_S" != 0 ] || [ "$APPLY_PITCH_LIMIT" != 0 ]; then
    echo 'Pitch steps require commissioning with IMU, without yaw motion or separate limit setup' >&2; exit 2
  fi
  if ! [[ "$PITCH_STEP_MDEG" =~ ^-?[0-9]+$ ]] || ((PITCH_STEP_MDEG < -500 || PITCH_STEP_MDEG > 500)); then
    echo 'Pitch step outside +/-500 millidegrees' >&2; exit 2
  fi
fi
if [ "$WITH_IMU" = 1 ] && [ "$MODE" != commission ]; then
  echo '--with-imu requires --commission-hardware' >&2; exit 2
fi
if ! [[ "$IMU_SECONDS" =~ ^[0-9]+$ ]] || ((IMU_SECONDS < 1 || IMU_SECONDS > 120)); then
  echo '--imu-seconds must be 1..120' >&2; exit 2
fi
if [ "$MODE" != commission ] && { [ "$APPLY_PITCH_LIMIT" != 0 ] || [ "$YAW_SPEED_DEG_S" != 0 ] || [ "$YAW_VOLTAGE" != 0 ] || [ "$PULSE_MS" != 100 ] || [ "$OBSERVE_MS" != 2000 ]; }; then
  echo 'Voltage/pulse options require --commission-hardware' >&2; exit 2
fi
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
    if [ "$MODE" = imu ]; then
    cmake --build "$APP/build" --target imu-bno085 -j"${OTA_BUILD_JOBS:-2}"
    elif [ "$MODE" = commission ]; then
    cmake --build "$APP/build" --target probe-mixed-hardware probe-pitch-motion imu-bno085 -j"${OTA_BUILD_JOBS:-2}"
    else
    cmake --build "$APP/build" --target controld probe-mixed-hardware -j"${OTA_BUILD_JOBS:-2}"
    fi
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
      echo "Started (launcher $launcher_pid). Inspect status for mode and readiness."
      if [ "$START_WEB" = 1 ] && [ "$MODE" != perception ]; then echo "Web: http://$(hostname):${OTA_WEB_PORT:-8080}/; logs: $RUN"; fi
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
if [ "$MODE" = hardware ] || [ "$MODE" = commission ] || [ "$MODE" = imu ]; then
  exec 8>"/tmp/ota-motion-$(id -u).lock"
  flock -n 8 || { echo 'Another launcher owns station motion, including across runtime directories.' >&2; exit 1; }
fi
children=()
controller_pid=''
cleanup() {
  trap - EXIT INT TERM
  if [ "$MODE" = imu ]; then
    echo 'Ending IMU acquisition; no motor process was started.'
    echo 'Stopped: IMU capture ended; motors were not commanded' > "$RUN/shutdown.result"
  elif [ "$MODE" = perception ]; then
    echo 'Ending perception capture; no motor process was started.'
    echo 'Stopped: perception capture ended; motors were not commanded' > "$RUN/shutdown.result"
  elif [ "$MODE" = commission ]; then
    echo 'Ending commissioning; an active yaw probe requests zero voltage.'
  else
    echo 'Stopping this stack; controller performs its own park/disable sequence.'
  fi
  for pid in "${children[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
  # Never force-kill the motor controller. Its own deadlines supervise park.
  if [ -n "$controller_pid" ]; then wait "$controller_pid" || true; fi
  # Keep the terminal controller outcome after ownership metadata is removed.
  # A clean process exit alone does not prove that the motors reached park.
  if [ -n "$controller_pid" ]; then
    if [ "$MODE" = commission ]; then
      { echo 'Stopped: commissioning probe ended; not a park/disable certification';
        tail -n 3 "$RUN/controller.log"; } > "$RUN/shutdown.result"
    elif grep -q 'PARKED (motors de-energized' "$RUN/controller.log"; then
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
if [ "$MODE" = imu ]; then
  if pgrep -x controld >/dev/null || pgrep -x imu_main >/dev/null; then
    echo 'Existing controller or legacy IMU consumer; refusing capture.' >&2; exit 1
  fi
  "$APP/build/imu-bno085" "$IMU_SECONDS" >"$RUN/imu.ndjson" 2>"$RUN/imu.log" &
  imu_pid=$!; children+=("$imu_pid")
  printf 'Mode: IMU capture\nTrace: %s\n' "$RUN/imu.ndjson" > "$RUN/stack.info"
  cp "$RUN/launcher.pid" "$RUN/started"
  wait "$imu_pid"
  exit $?
fi
if [ "$MODE" = commission ]; then
  if pgrep -x controld >/dev/null; then
    echo 'A controller already runs outside this launcher; refusing hardware probe.' >&2; exit 1
  fi
  PROBE="$APP/build/probe-mixed-hardware"
  imu_pid=''
  if [ "$WITH_IMU" = 1 ]; then
    if pgrep -x imu_main >/dev/null; then echo 'Legacy IMU consumer still running' >&2; exit 1; fi
    "$APP/build/imu-bno085" 120 >"$RUN/imu.ndjson" 2>"$RUN/imu.log" &
    imu_pid=$!; children+=("$imu_pid")
    # Observe a stationary host tare before starting the independent motor probe.
    for ((attempt=0; attempt<50; attempt++)); do
      kill -0 "$imu_pid" 2>/dev/null || { echo 'IMU startup failed' >&2; exit 1; }
      if grep -q '"kind":"tare"' "$RUN/imu.ndjson"; then break; fi
      sleep 0.1
    done
    "$PY" - "$RUN/imu.ndjson" <<'PY'
import json, sys, time
rows = []
for line in open(sys.argv[1]):
    try: rows.append(json.loads(line))
    except json.JSONDecodeError: pass  # writer may be halfway through its last row
tares = [r for r in rows if r.get('kind') == 'tare']
samples = [r for r in rows if r.get('sensor') == 'game_rv']
if not tares or not samples or tares[-1]['generation'] != samples[-1]['generation'] or not 0 <= time.monotonic_ns() - samples[-1]['rx_ns'] < 100_000_000:
    raise SystemExit('Fresh, tared IMU required before commissioning motion')
PY
  fi
  probe_options=()
  if [ "$APPLY_PITCH_LIMIT" = 1 ]; then probe_options+=(--apply-pitch-limit); fi
  if [ "$PITCH_STEP_MDEG" != 0 ]; then
  "$APP/build/probe-pitch-motion" "$PITCH_STEP_MDEG" "$RUN/pitch-probe.csv" >"$RUN/controller.log" 2>&1 &
  else
  "$PROBE" --config "${OTA_HARDWARE_PROBE_CONFIG:-$APP/config/hardware_probe.yaml}" \
    --yaw-voltage "$YAW_VOLTAGE" --pulse-ms "$PULSE_MS" --observe-ms "$OBSERVE_MS" \
    --yaw-speed-deg-s "$YAW_SPEED_DEG_S" \
    --trace "$RUN/hardware-probe.csv" "${probe_options[@]}" >"$RUN/controller.log" 2>&1 &
  fi
  controller_pid=$!; children+=("$controller_pid")
  printf 'Mode: commissioning\nYaw voltage: %s\nTrace: %s\n' "$YAW_VOLTAGE" "$RUN/hardware-probe.csv" > "$RUN/stack.info"
  cp "$RUN/launcher.pid" "$RUN/started"
  if [ -n "$imu_pid" ]; then
    # An IMU process failure ends the probe through the same cleanup/zero path.
    wait -n "$controller_pid" "$imu_pid"
  else
    wait "$controller_pid"
  fi
  exit $?
fi
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
