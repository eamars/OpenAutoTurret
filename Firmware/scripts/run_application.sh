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
    echo 'No action: background start, using config/turret_mixed.yaml (AUTO_ROAM) on hardware.'
    echo 'deploy: build, test and preflight this checkout; does not start motors.'
    echo 'deploy --probe-build: build only controld and preflight; defer regression tests.'
    echo 'run: foreground supervision; stop: controlled park/disable and full stack cleanup.'
    echo 'Options: --sim (real camera), --hold-motion (camera only), --profile NAME,'
    echo '         --commission-hardware [--yaw-current-a A --yaw-voltage N --pulse-ms N --observe-ms N],'
    echo '         (yaw push unit follows axes.yaw.control_mode: --yaw-current-a on a current drive,'
    echo '          --yaw-voltage only on a voltage one; the probe refuses the wrong one)'
    echo '         --apply-pitch-limit (commissioning only; volatile <=5 A, no pitch enable),'
    echo '         --yaw-speed-deg-s N (commissioning PI loop; integer +/-5, <=1500 raw),'
    echo '         --yaw-sweep-deg N --yaw-sweep-ff-a A (drag sweep: drag the axis N deg under speed'
    echo '          regulation with A amperes of breakaway feedforward; current mode only),'
    echo '         --yaw-step-deg N (commissioning with IMU; continuous 15..45 deg out/return),'
    echo '         --probe-imu [--imu-seconds N] (IMU capture only, 1..120 seconds),'
    echo '         --with-imu (commissioning only; capture IMU alongside bounded motor probe),'
    echo '         --pitch-step-mdeg N (enabled +/-15 deg session; 5 A ceiling),'
    echo '         --mixed-backend-check (commissioning + IMU tare; observe-only),'
    echo '         --commission-mixed-controller (manual-mode mixed controller; no vision/web),'
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
  # A stop that cannot name its trigger is a forensics hole: show the recorded
  # cause first, then how the motors were brought down.
  [ ! -r "$RUN/shutdown.cause" ] || cat "$RUN/shutdown.cause"
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
  # An operator stop leaves a credential, so the launcher can later tell "someone
  # asked for this" apart from "a child died and cleanup followed". Without it a
  # clean-looking stop is unattributable, and /tmp logs get truncated on restart.
  printf 'who=operator pid=%s uid=%s utc=%s launcher=%s\n' \
    "$$" "$(id -u)" "$(date -u +%Y-%m-%dT%H:%M:%SZ)" "$launcher_pid" > "$RUN/stop.request"
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
YAW_CURRENT_A=0
YAW_SPEED_DEG_S=0
YAW_SWEEP_DEG=0
YAW_SWEEP_FF_A=0
YAW_STEP_DEG=0
PULSE_MS=100
OBSERVE_MS=2000
APPLY_PITCH_LIMIT=0
IMU_SECONDS=10
WITH_IMU=0
PITCH_STEP_MDEG=0
PITCH_PROBE=0
PITCH_TEST_GAINS=0
PITCH_RESTORE_GAINS=0
MIXED_BACKEND_CHECK=0
MIXED_CONTROLLER_COMMISSION=0
COMMISSION_REQUESTED=0
SIM_REQUESTED=0
while [ $# -gt 0 ]; do
  case "$1" in
    --hold-motion) MODE=perception; shift ;;
    --sim) MODE=sim; SIM_REQUESTED=1; shift ;;
    --hardware) MODE=hardware; shift ;;
    --commission-hardware) MODE=commission; COMMISSION_REQUESTED=1; START_WEB=0; shift ;;
    --probe-imu) MODE=imu; START_WEB=0; shift ;;
    --with-imu) WITH_IMU=1; shift ;;
    --pitch-step-mdeg) PITCH_PROBE=1; PITCH_STEP_MDEG="${2:?--pitch-step-mdeg requires a value}"; shift 2 ;;
    --pitch-test-gains) PITCH_TEST_GAINS=1; shift ;;
    --pitch-restore-gains) PITCH_RESTORE_GAINS=1; shift ;;
    --mixed-backend-check) MIXED_BACKEND_CHECK=1; shift ;;
    --commission-mixed-controller) MIXED_CONTROLLER_COMMISSION=1; MODE=mixed-controller-commission; START_WEB=0; shift ;;
    --imu-seconds) IMU_SECONDS="${2:?--imu-seconds requires a value}"; shift 2 ;;
    --apply-pitch-limit) APPLY_PITCH_LIMIT=1; shift ;;
    --yaw-voltage) YAW_VOLTAGE="${2:?--yaw-voltage requires a signed value}"; shift 2 ;;
    --yaw-current-a) YAW_CURRENT_A="${2:?--yaw-current-a requires a signed ampere value}"; shift 2 ;;
    --yaw-speed-deg-s) YAW_SPEED_DEG_S="${2:?--yaw-speed-deg-s requires a signed value}"; shift 2 ;;
    --yaw-sweep-deg) YAW_SWEEP_DEG="${2:?--yaw-sweep-deg requires a signed travel in degrees}"; shift 2 ;;
    --yaw-sweep-ff-a) YAW_SWEEP_FF_A="${2:?--yaw-sweep-ff-a requires an ampere magnitude}"; shift 2 ;;
    --yaw-step-deg) YAW_STEP_DEG="${2:?--yaw-step-deg requires a value}"; shift 2 ;;
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
# One derived fact: did anything ask the yaw axis to move? There are now two spellings of the push
# (--yaw-voltage in raw counts, --yaw-current-a in amperes), and every "no other motor probe" rule
# below needs to see both. A request this list misses is two processes driving one CAN bus.
YAW_PUSH=0
if [ "$YAW_VOLTAGE" != 0 ] || [ "$YAW_CURRENT_A" != 0 ] || [ "$YAW_SPEED_DEG_S" != 0 ] ||
   [ "$YAW_SWEEP_DEG" != 0 ] || [ "$YAW_SWEEP_FF_A" != 0 ]; then
  YAW_PUSH=1
fi
if [ "$PITCH_RESTORE_GAINS" = 1 ] && { [ "$PITCH_PROBE" != 1 ] || [ "$PITCH_STEP_MDEG" != 0 ] || [ "$PITCH_TEST_GAINS" != 0 ]; }; then
  echo '--pitch-restore-gains requires a zero-step diagnostic probe without test gains' >&2; exit 2
fi
if [ "$PITCH_TEST_GAINS" = 1 ] && [ "$PITCH_PROBE" != 1 ]; then
  echo '--pitch-test-gains requires a pitch probe' >&2; exit 2
fi
if [ "$PITCH_PROBE" = 1 ]; then
  if [ "$MODE" != commission ] || [ "$WITH_IMU" != 1 ] || [ "$YAW_PUSH" != 0 ] || [ "$YAW_STEP_DEG" != 0 ] || [ "$APPLY_PITCH_LIMIT" != 0 ]; then
    echo 'Pitch steps require commissioning with IMU, without yaw motion or separate limit setup' >&2; exit 2
  fi
  if ! [[ "$PITCH_STEP_MDEG" =~ ^-?[0-9]+$ ]] || ((PITCH_STEP_MDEG < -15000 || PITCH_STEP_MDEG > 15000)); then
    echo 'Pitch step outside +/-15000 millidegrees' >&2; exit 2
  fi
fi
if [ "$MIXED_BACKEND_CHECK" = 1 ] && {
  [ "$MODE" != commission ] || [ "$COMMISSION_REQUESTED" != 1 ] || [ "$WITH_IMU" != 1 ] || [ "$PITCH_PROBE" != 0 ] ||
  [ "$YAW_STEP_DEG" != 0 ] || [ "$YAW_PUSH" != 0 ] ||
  [ "$APPLY_PITCH_LIMIT" != 0 ] || [ "$PITCH_TEST_GAINS" != 0 ] || [ "$PITCH_RESTORE_GAINS" != 0 ];
}; then
  echo '--mixed-backend-check requires --commission-hardware --with-imu and no other motor probe' >&2; exit 2
fi
if [ "$MIXED_CONTROLLER_COMMISSION" = 1 ] && {
  [ "$MODE" != mixed-controller-commission ] || [ "$MIXED_BACKEND_CHECK" != 0 ] ||
  [ "$COMMISSION_REQUESTED" != 0 ] || [ "$SIM_REQUESTED" != 0 ] ||
  [ "$WITH_IMU" != 0 ] || [ "$PITCH_PROBE" != 0 ] || [ "$YAW_STEP_DEG" != 0 ] ||
  [ "$YAW_PUSH" != 0 ] || [ "$APPLY_PITCH_LIMIT" != 0 ];
}; then
  echo '--commission-mixed-controller cannot be combined with another motor probe or --with-imu' >&2; exit 2
fi
if [ "$YAW_STEP_DEG" != 0 ]; then
  if [ "$MODE" != commission ] || [ "$WITH_IMU" != 1 ] || [ "$PITCH_PROBE" != 0 ] || [ "$YAW_PUSH" != 0 ] || [ "$APPLY_PITCH_LIMIT" != 0 ] ||
     ! [[ "$YAW_STEP_DEG" =~ ^[0-9]+$ ]] || ((YAW_STEP_DEG < 15 || YAW_STEP_DEG > 45)); then
    echo 'Yaw step requires commissioning with IMU, 15..45 degrees, and no other motor probe' >&2; exit 2
  fi
fi
if [ "$WITH_IMU" = 1 ] && [ "$MODE" != commission ]; then
  echo '--with-imu requires --commission-hardware' >&2; exit 2
fi
if ! [[ "$IMU_SECONDS" =~ ^[0-9]+$ ]] || ((IMU_SECONDS < 1 || IMU_SECONDS > 120)); then
  echo '--imu-seconds must be 1..120' >&2; exit 2
fi
if [ "$MODE" != commission ] && { [ "$APPLY_PITCH_LIMIT" != 0 ] || [ "$YAW_PUSH" != 0 ] || [ "$YAW_STEP_DEG" != 0 ] || [ "$PULSE_MS" != 100 ] || [ "$OBSERVE_MS" != 2000 ]; }; then
  echo 'Commissioning push/pulse options require --commission-hardware' >&2; exit 2
fi
if [ "$PROBE_BUILD" = 1 ] && [ "$ACTION" != deploy ]; then
  echo '--probe-build is only valid for deploy' >&2; exit 2
fi
[[ "$FRAMES" =~ ^[0-9]+$ ]] || { echo '--frames must be non-negative' >&2; exit 2; }
[ -x "$PY" ] || { echo "Project Python missing: $PY; set OTA_PYTHON" >&2; exit 2; }
umask 077
mkdir -p "$RUN"
cd "$APP"
DEFAULT_CONTROL_CONFIG="$APP/config/turret.yaml"
if [ "$MODE" = hardware ]; then
  DEFAULT_CONTROL_CONFIG="$APP/config/turret_mixed.yaml"
fi
ACTIVE_CONTROL_CONFIG="${OTA_CONTROL_CONFIG:-$DEFAULT_CONTROL_CONFIG}"
if [ "$MIXED_CONTROLLER_COMMISSION" = 1 ]; then
  ACTIVE_CONTROL_CONFIG="$APP/config/turret_mixed.yaml"
fi
MIXED_CONFIG_ACTIVE="$($PY - "$ACTIVE_CONTROL_CONFIG" <<'PY'
import sys, yaml
config = yaml.safe_load(open(sys.argv[1]))
print("1" if config.get("hardware_profile") else "0")
PY
)"
if [ "$ACTION" = deploy ]; then
  # Build in place only while this checkout is inactive. Other release trees
  # may be built without replacing the executable/config of a running stack.
  if owned_launcher && [ "$(readlink "/proc/$launcher_pid/cwd")" = "$APP" ]; then
    echo 'Stop this checkout before deployment, or build a separate release directory.' >&2
    exit 1
  fi
  if [ "${OTA_PREBUILT:-0}" = 1 ] && [ -x "$APP/build-arm64/control/controld" ]; then
    # The binaries were cross-compiled by the deploying machine (Firmware/tools/cross_build.py)
    # and uploaded beside this source. Compiling them again here bought nothing: the station has
    # no knowledge of this code that the machine which built it lacks. Running the suite is the
    # part that needs the hardware, so that part stays here -- each test binary is executed
    # directly rather than through ctest, because a CTest cache would point back at the machine
    # that built it. The symlink keeps every later path in this script unchanged.
    ln -sfn "$APP/build-arm64" "$APP/build"
    # The tests resolve their config against this: the binary was compiled elsewhere, so its
    # compiled-in source path describes a machine this station has never been.
    export OTA_FIRMWARE_ROOT="$APP"
    tests_run=0
    tests_failed=0
    while IFS= read -r t; do
      case "$t" in */_deps/*) continue ;; esac
      tests_run=$((tests_run + 1))
      if ! "$t" >"$APP/build-arm64/last-test.log" 2>&1; then
        tests_failed=$((tests_failed + 1))
        echo "FAILED $t" >&2
        tail -n 15 "$APP/build-arm64/last-test.log" >&2
      fi
    done < <(find "$APP/build-arm64" -type f -name 'test_*' -perm -u+x)
    echo "Prebuilt suite on station: $tests_run binaries, $tests_failed failed"
    if [ "$tests_run" -lt 40 ] || [ "$tests_failed" -ne 0 ]; then
      # A count that small means the upload lost targets, which would otherwise read as a pass.
      echo "refusing to call that a green suite" >&2
      exit 1
    fi
  else
    cmake -S "$APP" -B "$APP/build" -DCMAKE_BUILD_TYPE=Release
  fi
  if [ "${OTA_PREBUILT:-0}" != 1 ] || [ ! -x "$APP/build-arm64/control/controld" ]; then
  if [ "$PROBE_BUILD" = 1 ]; then
    if [ "$MODE" = imu ]; then
    cmake --build "$APP/build" --target imu-bno085 -j"${OTA_BUILD_JOBS:-2}"
    elif [ "$MODE" = commission ]; then
    cmake --build "$APP/build" --target probe-mixed-hardware probe-mixed-backend probe-pitch-motion probe-yaw-motion imu-bno085 -j"${OTA_BUILD_JOBS:-2}"
    elif [ "$MIXED_CONTROLLER_COMMISSION" = 1 ]; then
    cmake --build "$APP/build" --target controld probe-mixed-backend imu-bno085 -j"${OTA_BUILD_JOBS:-2}"
    else
    cmake --build "$APP/build" --target controld probe-mixed-hardware probe-mixed-backend imu-bno085 -j"${OTA_BUILD_JOBS:-2}"
    fi
    echo 'Probe build: regression tests deferred until runtime viability is established.'
  else
    cmake --build "$APP/build" -j"${OTA_BUILD_JOBS:-2}"
    ctest --test-dir "$APP/build" --output-on-failure
  fi
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
"$PY" "$APP/tools/station_preflight.py" "$ACTIVE_CONTROL_CONFIG" "$MODE" "$PROFILE" "$PRODUCTION" "$MIXED_BACKEND_CHECK" "$MIXED_CONTROLLER_COMMISSION"
if [ "$ACTION" = check ]; then
  echo 'Preflight passed. deploy/check do not start the station.'
  exit 0
fi
exec 9>"$RUN/launcher.lock"
flock -n 9 || { echo "A stack already owns $RUN" >&2; exit 1; }
if [ "$MODE" = hardware ] || [ "$MODE" = commission ] || [ "$MODE" = imu ] || [ "$MODE" = mixed-controller-commission ]; then
  exec 8>"/tmp/ota-motion-$(id -u).lock"
  flock -n 8 || { echo 'Another launcher owns station motion, including across runtime directories.' >&2; exit 1; }
fi
# A restart must not erase the previous stack's evidence. Every run reuses the
# same $RUN, so the previous round is archived before anything is truncated.
rotate_stack_logs() {
  local keep="${1:-10}" stamp src old
  ls "$RUN"/*.log >/dev/null 2>&1 || return 0
  stamp="$(date -u +%Y%m%dT%H%M%SZ)"
  src="$RUN/logs-history/$stamp-launcher$$"
  mkdir -p "$src" || return 0
  for f in "$RUN"/*.log "$RUN"/shutdown.result "$RUN"/shutdown.cause "$RUN"/stack.info; do
    [ -f "$f" ] && mv -f "$f" "$src/" 2>/dev/null || true
  done
  # Trip traces are evidence of the round that faulted, so the directory moves
  # whole: a redeploy must not be able to truncate the thing it is there to explain.
  # controld recreates it on the next freeze.
  [ -d "$RUN/traces" ] && mv -f "$RUN/traces" "$src/traces" 2>/dev/null || true
  # $RUN lives in /tmp: keep the newest $keep rounds and no more.
  ( cd "$RUN/logs-history" 2>/dev/null || exit 0
    ls -1dt */ 2>/dev/null | tail -n +"$((keep + 1))" | while IFS= read -r old; do
      rm -rf -- "$old" || true
    done ) || true
}
rotate_stack_logs 10
children=()
declare -A child_name=()
controller_pid=''
imu_pid=''
cause_signal=''
first_child_status=''
exited_pid=''
exited_name=''
# wait -n reports a status but not the child it belongs to; record both while the
# siblings are still alive to point at, i.e. before cleanup signals anyone.
note_child_exit() {
  local pid gone=()
  for pid in "${children[@]}"; do
    kill -0 "$pid" 2>/dev/null || gone+=("$pid")
  done
  for pid in "${gone[@]}"; do
    if [ -n "${child_name[$pid]:-}" ]; then exited_pid="$pid"; exited_name="${child_name[$pid]}"; return 0; fi
  done
  if [ "${#gone[@]}" -gt 0 ]; then exited_pid="${gone[0]}"; exited_name=unknown; fi
}
describe_status() {
  local s="$1"
  if [[ "$s" =~ ^[0-9]+$ ]] && [ "$s" -gt 128 ]; then
    printf '%s(signal %s)' "$s" "$((s - 128))"
  else
    printf '%s' "$s"
  fi
}
stop_cause_line() {
  local reason="$1" operator=''
  printf 'cause=%s utc=%s launcher=%s uptime_s=%s' "$reason" \
    "$(date -u +%Y-%m-%dT%H:%M:%SZ)" "$$" "$SECONDS"
  if [ -r "$RUN/stop.request" ]; then
    read -r operator < "$RUN/stop.request" || true
    [ -n "$operator" ] && printf ' operator="%s"' "$operator"
  fi
  [ -z "$exited_pid" ] || printf ' exited_child=%s exited_pid=%s wait_status=%s' \
    "$exited_name" "$exited_pid" "$(describe_status "${first_child_status:-unset}")"
  [ -z "$cause_signal" ] || printf ' signal=%s' "$cause_signal"
  printf '\n'
}
cleanup() {
  trap - EXIT INT TERM
  local reason
  if [ -r "$RUN/stop.request" ]; then reason=operator_stop
  elif [ -n "$exited_pid" ]; then reason=child_exit
  elif [ -n "$cause_signal" ]; then reason=external_signal
  else reason=unattributed; fi
  stop_cause_line "$reason" > "$RUN/shutdown.cause"
  cat "$RUN/shutdown.cause"
  rm -f -- "$RUN/stop.request"
  if [ "$MODE" = imu ]; then
    echo 'Ending IMU acquisition; no motor process was started.'
    echo 'Stopped: IMU capture ended; motors were not commanded' > "$RUN/shutdown.result"
  elif [ "$MODE" = perception ]; then
    echo 'Ending perception capture; no motor process was started.'
    echo 'Stopped: perception capture ended; motors were not commanded' > "$RUN/shutdown.result"
  elif [ "$MODE" = commission ] && [ "$MIXED_BACKEND_CHECK" = 1 ]; then
    echo 'Ending mixed-backend no-motion probe; yaw zero and pitch STOP were requested.'
  elif [ "$MODE" = commission ]; then
    echo 'Ending commissioning session; pitch disables, yaw requests zero if its probe was active.'
  else
    echo 'Stopping this stack; controller performs its own park/disable sequence.'
  fi
  for pid in "${children[@]}"; do
    [ "$pid" != "$controller_pid" ] || continue
    [ -n "$imu_pid" ] && [ "$pid" = "$imu_pid" ] && continue
    kill -TERM "$pid" 2>/dev/null || true
  done
  # Ask the exact owned controller child to run its own controlled shutdown.
  # Waiting without signaling it leaves the launcher stuck in cleanup.
  if [ -n "$controller_pid" ]; then
    kill -TERM "$controller_pid" 2>/dev/null || true
    wait "$controller_pid" || true
  fi
  # Keep the single BNO085 owner alive through the controller's controlled
  # stop, then release I2C ownership.
  if [ -n "$imu_pid" ]; then
    kill -TERM "$imu_pid" 2>/dev/null || true
    wait "$imu_pid" || true
  fi
  # If a commissioning child exited abnormally, its in-process guard cannot
  # send again. The launcher still owns the station lock here and requests a
  # final zero/STOP on the selected bus after the child has exited.
  if [ "$MODE" = commission ] && [ "$YAW_STEP_DEG" != 0 ]; then
    "$PY" "$APP/tools/commissioning_fallback_stop.py" yaw || echo 'Yaw fallback zero request failed' >&2
  elif [ "$MODE" = commission ] && [ "$PITCH_PROBE" = 1 ] && [ "$PITCH_STEP_MDEG" != 0 ]; then
    "$PY" "$APP/tools/commissioning_fallback_stop.py" pitch || echo 'Pitch fallback STOP request failed' >&2
  fi
  # Keep the terminal controller outcome after ownership metadata is removed.
  # A clean process exit alone does not prove that the motors reached park.
  if [ -n "$controller_pid" ]; then
    if [ "$MODE" = commission ] && [ "$MIXED_BACKEND_CHECK" = 1 ]; then
      { echo 'Stopped: mixed-backend no-motion probe ended; inspect STOP feedback result below';
        tail -n 8 "$RUN/controller.log"; } > "$RUN/shutdown.result"
    elif [ "$MODE" = commission ]; then
      { echo 'Stopped: commissioning probe ended; not a park/disable certification';
        tail -n 3 "$RUN/controller.log"; } > "$RUN/shutdown.result"
    elif grep -Fq 'STOPPED (pitch disable confirmed; GM6020 yaw zero requested, disable state unavailable)' "$RUN/controller.log"; then
      echo 'Stopped: STOPPED (pitch disable confirmed; GM6020 yaw zero requested, disable state unavailable)' > "$RUN/shutdown.result"
    elif grep -Fq 'STOP FAILED:' "$RUN/controller.log"; then
      { echo 'Stopped: STOP FAILED'; tail -n 8 "$RUN/controller.log"; } > "$RUN/shutdown.result"
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
trap 'cause_signal=INT; exit 130' INT
trap 'cause_signal=TERM; exit 143' TERM
rm -f -- "$RUN/shutdown.result"
printf '%s %s\n' "$$" "$(awk '{print $22}' /proc/$$/stat)" > "$RUN/launcher.pid"
if [ "$MODE" = imu ]; then
  if pgrep -x controld >/dev/null || pgrep -x imu_main >/dev/null; then
    echo 'Existing controller or legacy IMU consumer; refusing capture.' >&2; exit 1
  fi
  "$APP/build/imu-bno085" "$IMU_SECONDS" >"$RUN/imu.ndjson" 2>"$RUN/imu.log" &
  imu_pid=$!; children+=("$imu_pid"); child_name[$imu_pid]=imu-bno085
  printf 'Mode: IMU capture\nTrace: %s\n' "$RUN/imu.ndjson" > "$RUN/stack.info"
  cp "$RUN/launcher.pid" "$RUN/started"
  first_child_status=0
  wait "$imu_pid" || first_child_status=$?
  note_child_exit
  exit "$first_child_status"
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
    imu_pid=$!; children+=("$imu_pid"); child_name[$imu_pid]=imu-bno085
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
  if [ "$PITCH_PROBE" = 1 ]; then
  pitch_options=()
  if [ "$PITCH_TEST_GAINS" = 1 ]; then pitch_options+=(tuned); fi
  if [ "$PITCH_RESTORE_GAINS" = 1 ]; then pitch_options+=(restore); fi
  "$APP/build/probe-pitch-motion" "$PITCH_STEP_MDEG" "$RUN/pitch-probe.csv" "${pitch_options[@]}" >"$RUN/controller.log" 2>&1 &
  elif [ "$YAW_STEP_DEG" != 0 ]; then
  "$APP/build/probe-yaw-motion" "$YAW_STEP_DEG" "$RUN/yaw-probe.csv" >"$RUN/controller.log" 2>&1 &
  elif [ "$MIXED_BACKEND_CHECK" = 1 ]; then
  "$APP/build/probe-mixed-backend" --config "${OTA_MIXED_HARDWARE_CONFIG:-$APP/config/mixed_hardware.yaml}" \
    --observe-seconds 10 >"$RUN/controller.log" 2>&1 &
  else
  "$PROBE" --config "${OTA_HARDWARE_PROBE_CONFIG:-$APP/config/hardware_probe.yaml}" \
    --yaw-voltage "$YAW_VOLTAGE" --yaw-current-a "$YAW_CURRENT_A" \
    --pulse-ms "$PULSE_MS" --observe-ms "$OBSERVE_MS" \
    --yaw-speed-deg-s "$YAW_SPEED_DEG_S" \
    --yaw-sweep-deg "$YAW_SWEEP_DEG" --yaw-sweep-ff-a "$YAW_SWEEP_FF_A" \
    --trace "$RUN/hardware-probe.csv" "${probe_options[@]}" >"$RUN/controller.log" 2>&1 &
  fi
  controller_pid=$!; children+=("$controller_pid"); child_name[$controller_pid]=probe-mixed-hardware
  if [ "$PITCH_PROBE" = 1 ]; then
    printf 'Mode: pitch commissioning\nStep millidegrees: %s\nTrace: %s\n' "$PITCH_STEP_MDEG" "$RUN/pitch-probe.csv" > "$RUN/stack.info"
  elif [ "$YAW_STEP_DEG" != 0 ]; then
    printf 'Mode: yaw commissioning\nStep degrees: %s\nTrace: %s\n' "$YAW_STEP_DEG" "$RUN/yaw-probe.csv" > "$RUN/stack.info"
  elif [ "$MIXED_BACKEND_CHECK" = 1 ]; then
    printf 'Mode: mixed backend observe-only check\nOutput: %s\n' "$RUN/controller.log" > "$RUN/stack.info"
  else
    printf 'Mode: commissioning\nYaw push: %s V / %s A (unit follows the probe profile control_mode)\nTrace: %s\n' "$YAW_VOLTAGE" "$YAW_CURRENT_A" "$RUN/hardware-probe.csv" > "$RUN/stack.info"
  fi
  cp "$RUN/launcher.pid" "$RUN/started"
  if [ -n "$imu_pid" ]; then
    # An IMU process failure ends the probe through the same cleanup/zero path.
    first_child_status=0; wait -n "$controller_pid" "$imu_pid" || first_child_status=$?
  else
    first_child_status=0; wait "$controller_pid" || first_child_status=$?
  fi
  note_child_exit
  exit "$first_child_status"
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
if [ "$MODE" != mixed-controller-commission ]; then
  "$PY" -c 'import sys; from perception.visiond import build_parser,load_config; load_config(build_parser().parse_args(sys.argv[1:]))' "${vision_args[@]}"
fi
CONTROLD="$APP/build/control/controld"
controller_args=("$ACTIVE_CONTROL_CONFIG")
if [ "$MODE" = sim ]; then
  controller_args+=(--sim)
  echo 'SIMULATED motors; camera and web are real.'
elif [ "$MODE" = hardware ]; then
  echo 'HARDWARE station: homing establishes calibration, then automatic roam/track begins. Web Manual overrides autonomy.'
elif [ "$MODE" = mixed-controller-commission ]; then
  export OTA_MIXED_COMMISSION_MANUAL=1
  echo 'MIXED CONTROLLER COMMISSION: manual startup only; no vision/web; supervise pitch homing and controlled stop.'
fi
if [ "$MODE" != perception ]; then
if { [ "$MODE" = hardware ] || [ "$MODE" = mixed-controller-commission ]; } && pgrep -x controld >/dev/null; then
  echo 'A controller already runs outside this launcher. Resolve its owner; do not start a second CAN owner.' >&2
  exit 1
fi
if { [ "$MODE" = hardware ] || [ "$MODE" = mixed-controller-commission ]; } && [ "$MIXED_CONFIG_ACTIVE" = 1 ]; then
  if pgrep -x imu_main >/dev/null; then
    echo 'Legacy IMU consumer still running; refusing BNO085 continuous capture.' >&2; exit 1
  fi
  export OTA_IMU_TRACE="$RUN/imu.ndjson"
  "$APP/build/imu-bno085" --continuous --retain-lines 4096 >"$RUN/imu.ndjson" 2>"$RUN/imu.log" &
  imu_pid=$!; children+=("$imu_pid"); child_name[$imu_pid]=imu-bno085
  # Require a fresh host tare and same-generation game rotation sample before
  # the controller starts. The IMU residual remains observe-only.
  imu_ready=0
  for ((attempt=0; attempt<50; attempt++)); do
    kill -0 "$imu_pid" 2>/dev/null || { echo 'Continuous BNO085 startup failed' >&2; exit 1; }
    if "$PY" - "$RUN/imu.ndjson" <<'PY'
import json, sys, time
try:
    rows = [json.loads(line) for line in open(sys.argv[1]) if line.strip()]
except (OSError, json.JSONDecodeError):
    raise SystemExit(1)
tares = [r for r in rows if r.get('kind') == 'tare']
samples = [r for r in rows if r.get('sensor') == 'game_rv']
if not tares or not samples:
    raise SystemExit(1)
tare, sample = tares[-1], samples[-1]
if (tare.get('generation') != sample.get('generation') or sample.get('rx_ns', 0) < tare.get('rx_ns', 0)
        or not 0 <= time.monotonic_ns() - sample.get('rx_ns', 0) < 100_000_000):
    raise SystemExit(1)
PY
    then imu_ready=1; break; fi
    sleep 0.1
  done
  if [ "$imu_ready" != 1 ]; then
    echo 'Fresh same-generation BNO085 host tare/sample unavailable; controller not started.' >&2
    exit 1
  fi
fi
"$CONTROLD" "${controller_args[@]}" >"$RUN/controller.log" 2>&1 &
controller_pid=$!
children+=("$controller_pid")
child_name[$controller_pid]=controld
else
  echo 'Perception only: no controller connection.'
  START_WEB=0
fi
if [ "$START_WEB" -eq 1 ]; then
  "$PY" -m web.webd.app >"$RUN/web.log" 2>&1 &
  web_pid=$!
  children+=("$web_pid")
  child_name[$web_pid]=webd
  printf '%s\n' "$OTA_WEB_PORT" > "$RUN/web.port"
  vision_args+=(--controller-state-url "http://127.0.0.1:$OTA_WEB_PORT/api/state")
fi
if [ "$MODE" != mixed-controller-commission ]; then
  if [ "$MODE" != perception ]; then
    vision_args+=(--publish-socket "$OTA_VISION_SOCKET")
  fi
  "$PY" -m perception.visiond "${vision_args[@]}" >"$RUN/vision.log" 2>&1 &
  vision_pid=$!
  children+=("$vision_pid")
  child_name[$vision_pid]=visiond
elif [ "$MODE" = mixed-controller-commission ]; then
  echo 'Mixed controller commissioning: vision and web are not started.'
fi

printf 'Mode: %s\nConfig: %s\nPython: %s\nChildren: %s\n' "$MODE" "${controller_args[0]}" "$PY" "${children[*]}" > "$RUN/stack.info"
cp "$RUN/launcher.pid" "$RUN/started"
echo "Stack logs: $RUN; web port: $OTA_WEB_PORT. Ctrl-C stops this stack."
# A failed child or finite capture ends its own stack; unrelated processes are untouched.
first_child_status=0
wait -n "${children[@]}" || first_child_status=$?
note_child_exit
