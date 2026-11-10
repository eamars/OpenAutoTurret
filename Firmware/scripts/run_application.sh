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
    echo '         --establish-homing MANIFEST (bounded pitch sensorless homing acquisition),'
    echo '         --control-yaw MANIFEST (ADR-002.2 supervised yaw shared-core 3a control),'
    echo '         --home (home at start; a hardware start is otherwise SHUTDOWN, motors off),'
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
# ADR-008 section 8: a launcher killed outright (SIGKILL, OOM) runs no cleanup, and its children --
# controld still holding both CAN buses and the motion lock -- outlive it. $RUN/children names them
# (name, pid, start time; rewritten at every start and restart), so a stop can still bring them down
# in the launcher's order: controld first, which parks, then the rest. This is for a stack started
# with `start`. Under the boot service systemd runs no ExecStop after its main process is killed;
# its KillMode=control-group sends the children TERM instead, on which controld parks as well.
stop_orphans() {
  [ -r "$RUN/children" ] || return 1
  local name pid start ctl='' others=() p attempt alive
  while read -r name pid start; do
    [[ "$pid" =~ ^[0-9]+$ && "$start" =~ ^[0-9]+$ ]] || continue
    [ -r "/proc/$pid/stat" ] && [ "$(awk '{print $22}' "/proc/$pid/stat" 2>/dev/null)" = "$start" ] || continue
    if [ "$name" = controld ]; then ctl="$pid"; else others+=("$pid"); fi
  done < "$RUN/children"
  [ -n "$ctl" ] || [ "${#others[@]}" -gt 0 ] || return 1
  echo "The launcher is gone but its children are not (controld ${ctl:-gone}, ${#others[@]} other); stopping them, controld first."
  printf 'cause=orphan_stop utc=%s controld=%s others=%s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)" \
    "${ctl:-none}" "${others[*]:-none}" > "$RUN/shutdown.cause" 2>/dev/null || true
  if [ -n "$ctl" ]; then
    kill -TERM "$ctl" 2>/dev/null || true
    for ((attempt=0; attempt<120; attempt++)); do kill -0 "$ctl" 2>/dev/null || break; sleep 1; done
  fi
  for p in "${others[@]}"; do kill -TERM "$p" 2>/dev/null || true; done
  for ((attempt=0; attempt<50; attempt++)); do
    alive=0; for p in "${others[@]}"; do kill -0 "$p" 2>/dev/null && alive=1; done
    [ "$alive" = 1 ] || break; sleep 0.1
  done
  for p in "${others[@]}"; do kill -KILL "$p" 2>/dev/null || true; done
  if [ -n "$ctl" ] && kill -0 "$ctl" 2>/dev/null; then
    echo "Stopped: controld did not exit within 120 s of TERM; it is still running ($ctl)" > "$RUN/shutdown.result"
  elif grep -Fq 'SHUT DOWN (pitch released on its rest stop' "$RUN/controller.log" 2>/dev/null; then
    echo 'Stopped: SHUT DOWN (pitch on its rest stop, yaw at 0, both motors off; orphaned stack)' > "$RUN/shutdown.result"
  else
    { echo 'Stopped: orphaned stack stopped; park not confirmed'; tail -n 8 "$RUN/controller.log" 2>/dev/null; } > "$RUN/shutdown.result"
  fi
  rm -f -- "$RUN/children" "$RUN/launcher.pid" "$RUN/started" "$RUN/web.port"
  return 0
}
if [ "$ACTION" = status ]; then
  if owned_launcher; then
    echo "Running (launcher $launcher_pid); checkout: $(readlink "/proc/$launcher_pid/cwd")"
    [ ! -r "$RUN/stack.info" ] || cat "$RUN/stack.info"
    echo "Logs: $RUN/{launcher,controller,vision,web}.log"
    # ADR-008 D6: what each process holds and how often it has been restarted, once a minute.
    [ ! -r "$RUN/resources.json" ] || { echo "Resources: $(cat "$RUN/resources.json")"; }
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
  if ! owned_launcher; then
    if stop_orphans; then stopped_status; exit 0; fi
    echo 'Already stopped'; stopped_status; exit 0
  fi
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
PROFILE=hailo_yolov8s_pose   # person boxes + COCO-17 keypoints; the aim anchor is the head (2026-10-02)
# A plain start follows the operator's target type (HUMAN/HEAD/CAT/DOG, owner 2026-10-20): visiond starts
# on this profile and swaps to the persisted type. An explicit --profile pins the model for a developer
# run: visiond acknowledges type requests without changing its model or classes.
PROFILE_PINNED=0
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
# The BNO085's RST is wired to GPIO4 (2026-10-05). imu-bno085 pulses it on every (re)connect, which
# frees a sensor holding SDA low; an I2C soft reset cannot. OTA_IMU_RESET_GPIO= (empty) disables it.
IMU_RESET_GPIO="${OTA_IMU_RESET_GPIO-/dev/gpiochip0:4}"
imu_reset_args=()
[ -n "$IMU_RESET_GPIO" ] && imu_reset_args=(--reset-gpio "$IMU_RESET_GPIO")
PITCH_STEP_MDEG=0
PITCH_PROBE=0
PITCH_TEST_GAINS=0
PITCH_RESTORE_GAINS=0
MIXED_BACKEND_CHECK=0
MIXED_CONTROLLER_COMMISSION=0
COMMISSION_REQUESTED=0
SIM_REQUESTED=0
SESSION_MANIFEST=''
START_STATE_HOME=0
while [ $# -gt 0 ]; do
  case "$1" in
    --hold-motion) MODE=perception; shift ;;
    --sim) MODE=sim; SIM_REQUESTED=1; shift ;;
    --hardware) MODE=hardware; shift ;;
    --commission-hardware) MODE=commission; COMMISSION_REQUESTED=1; START_WEB=0; shift ;;
    --establish-homing) SESSION_MANIFEST="${2:?--establish-homing requires a manifest}"; MODE=establish-homing; START_WEB=0; shift 2 ;;
    --control-yaw) SESSION_MANIFEST="${2:?--control-yaw requires a manifest}"; MODE=control-yaw; START_WEB=0; shift 2 ;;
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
    --home) START_STATE_HOME=1; shift ;;
    --production) PRODUCTION=1; shift ;;
    --profile) PROFILE="${2:?--profile requires a name}"; PROFILE_PINNED=1; shift 2 ;;
    --frames) FRAMES="${2:?--frames requires a count}"; shift 2 ;;
    --dev) set -x; shift ;;
    --probe-build) PROBE_BUILD=1; shift ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
done
if [ -n "$SESSION_MANIFEST" ]; then
  [ "${#options[@]}" -eq 2 ] && { [ "$MODE" = establish-homing ] || [ "$MODE" = control-yaw ]; } || {
    echo 'Acquisition cannot be combined with other startup options' >&2; exit 2;
  }
  [ "$ACTION" != deploy ] || {
    echo 'Acquisition uses workstation-built binaries; do not build it on the station.' >&2; exit 2;
  }
  SESSION_MANIFEST="$(realpath -- "$SESSION_MANIFEST")"
  options=("--$MODE" "$SESSION_MANIFEST")
fi
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
    # part that needs the hardware, so that part stays here. Rebase CTest's generated command and
    # working-directory paths from the build machine before invoking the registered suite.
    ln -sfn "$APP/build-arm64" "$APP/build"
    "$PY" "$APP/tools/relocate_ctest_paths.py" "$APP/build-arm64" "$APP"
    # The tests resolve their config against this: the binary was compiled elsewhere, so its
    # compiled-in source path describes a machine this station has never been.
    export OTA_FIRMWARE_ROOT="$APP"
    ctest_list="$APP/build-arm64/prebuilt-ctest-list.json"
    ctest --test-dir "$APP/build-arm64" --show-only=json-v1 >"$ctest_list"
    tests_run=$("$PY" -c 'import json,sys; print(len(json.load(open(sys.argv[1], encoding="utf-8"))["tests"]))' "$ctest_list")
    echo "Prebuilt CTest suite on station: $tests_run registered tests"
    if [ "$tests_run" -lt 40 ]; then
      # A count that small means the upload lost test registrations, which could read as a pass.
      echo "refusing to run an incomplete suite" >&2
      exit 1
    fi
    if ! ctest --test-dir "$APP/build-arm64" --output-on-failure >"$APP/build-arm64/last-test.log" 2>&1; then
      tail -n 80 "$APP/build-arm64/last-test.log" >&2
      echo "Prebuilt CTest suite failed" >&2
      exit 1
    fi
    cat "$APP/build-arm64/last-test.log"
    echo "Prebuilt CTest suite on station: $tests_run registered tests, 0 failed"
  else
    cmake -S "$APP" -B "$APP/build" -DCMAKE_BUILD_TYPE=Release
  fi
  if [ "${OTA_PREBUILT:-0}" != 1 ] || [ ! -x "$APP/build-arm64/control/controld" ]; then
  if [ "$PROBE_BUILD" = 1 ]; then
    if [ "$MODE" = imu ]; then
    cmake --build "$APP/build" --target imu-bno085 -j"${OTA_BUILD_JOBS:-$(nproc)}"
    elif [ "$MODE" = commission ]; then
    cmake --build "$APP/build" --target probe-mixed-hardware probe-mixed-backend probe-pitch-motion probe-yaw-motion imu-bno085 -j"${OTA_BUILD_JOBS:-$(nproc)}"
    elif [ "$MIXED_CONTROLLER_COMMISSION" = 1 ]; then
    cmake --build "$APP/build" --target controld probe-mixed-backend imu-bno085 -j"${OTA_BUILD_JOBS:-$(nproc)}"
    else
    cmake --build "$APP/build" --target controld probe-mixed-hardware probe-mixed-backend imu-bno085 -j"${OTA_BUILD_JOBS:-$(nproc)}"
    fi
    echo 'Probe build: regression tests deferred until runtime viability is established.'
  else
    cmake --build "$APP/build" -j"${OTA_BUILD_JOBS:-$(nproc)}"
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
  # Save the previous launcher log before shell redirection opens the new one.
  # The child rotates the other old files after acquiring station ownership.
  if [ -f "$RUN/launcher.log" ]; then
    previous_log_dir="$RUN/logs-history/$(date -u +%Y%m%dT%H%M%SZ)-previous-launcher$$"
    mkdir -p "$previous_log_dir"
    mv -- "$RUN/launcher.log" "$previous_log_dir/launcher.log"
  fi
  # Append (ADR-008 D5): the launcher's housekeeping truncates its logs in place, and a writer
  # without O_APPEND would carry on at its old offset into a hole.
  nohup setsid bash "$APP/scripts/run_application.sh" run "${options[@]}" \
    >>"$RUN/launcher.log" 2>&1 < /dev/null 11>&- &
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
if [ "$MODE" = establish-homing ] || [ "$MODE" = control-yaw ]; then
  "$PY" "$APP/tools/adr0022_capture_launch.py" "$SESSION_MANIFEST" "--$MODE" --preflight-only
else
  "$PY" "$APP/tools/station_preflight.py" "$ACTIVE_CONTROL_CONFIG" "$MODE" "$PROFILE" "$PRODUCTION" "$MIXED_BACKEND_CHECK" "$MIXED_CONTROLLER_COMMISSION"
fi
if [ "$ACTION" = check ]; then
  echo 'Preflight passed. deploy/check do not start the station.'
  exit 0
fi
exec 9>"$RUN/launcher.lock"
flock -n 9 || { echo "A stack already owns $RUN" >&2; exit 1; }
if [ "$MODE" = hardware ] || [ "$MODE" = commission ] || [ "$MODE" = imu ] || [ "$MODE" = mixed-controller-commission ] || [ "$MODE" = establish-homing ] || [ "$MODE" = control-yaw ]; then
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
  for f in "$RUN"/*.log "$RUN"/*.log.1 "$RUN"/shutdown.result "$RUN"/shutdown.cause "$RUN"/stack.info; do
    # start has already opened this run's launcher.log. Moving that inode makes
    # status/startup failure reporting point at a missing file for the whole run.
    if [ "$f" = "$RUN/launcher.log" ] && { [ "$f" -ef "/proc/$$/fd/1" ] || [ "$f" -ef "/proc/$$/fd/2" ]; }; then continue; fi
    # Baseline captures own immutable logs alongside their attempt record.
    if [[ "$f" = *.imu.log ]] && [ -f "${f%.imu.log}.attempt.json" ]; then continue; fi
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
  prune_history_bytes "$HISTORY_BUDGET_BYTES"
}
# ADR-008 D5: every byte the running stack writes has a budget. $RUN is a tmpfs, i.e. RAM: on
# 2026-11-10 controller.log alone had reached 833 MB in 10.5 days, with nothing to stop it. Each log
# is capped in place (copytruncate: the file is copied to <name>.1, then truncated; every writer
# opens it O_APPEND, so it carries on at the new end), logs-history has a byte budget as well as its
# count, and the whole run dir has a backstop. OTA_LOG_CAP_BYTES overrides every log cap (the
# ADR-008 S3 proof runs at 1 MiB).
MIB=1048576
HISTORY_BUDGET_BYTES="${OTA_HISTORY_BUDGET_BYTES:-$((256 * MIB))}"
RUN_BUDGET_BYTES="${OTA_RUN_BUDGET_BYTES:-$((768 * MIB))}"
log_cap() {
  if [ -n "${OTA_LOG_CAP_BYTES:-}" ]; then echo "$OTA_LOG_CAP_BYTES"; return; fi
  case "$1" in
    controller.log) echo $((64 * MIB)) ;;
    vision.log) echo $((16 * MIB)) ;;
    resources.ndjson) echo $((4 * MIB)) ;;
    *) echo $((8 * MIB)) ;;
  esac
}
cap_file() {
  local f="$1" cap size
  cap="$(log_cap "$(basename -- "$f")")"
  size="$(stat -c %s -- "$f" 2>/dev/null)" || return 0
  [ "$size" -gt "$cap" ] || return 0
  cp -f -- "$f" "$f.1" 2>/dev/null || return 0
  : > "$f" 2>/dev/null || return 0
  printf '[%s] launcher: rotated at %s bytes (cap %s); the previous part is %s.1\n' \
    "$(date '+%Y-%m-%d %H:%M:%S')" "$size" "$cap" "$(basename -- "$f")" >> "$f" 2>/dev/null || true
}
# Exactly one number: du exits non-zero when a file vanishes mid-scan, yet still prints its total.
dir_bytes() {
  local v
  v="$(du -sb -- "$1" 2>/dev/null | cut -f1 | head -n 1)" || true
  [[ "$v" =~ ^[0-9]+$ ]] || v=0
  echo "$v"
}
prune_history_bytes() {
  local budget="$1" oldest
  [ -d "$RUN/logs-history" ] || return 0
  while [ "$(dir_bytes "$RUN/logs-history")" -gt "$budget" ]; do
    oldest="$(cd "$RUN/logs-history" 2>/dev/null && ls -1dtr */ 2>/dev/null | head -n 1)" || oldest=''
    [ -n "$oldest" ] || break
    rm -rf -- "$RUN/logs-history/${oldest%/}" 2>/dev/null || break
  done
}
# Once per housekeeping tick (10 s): cap the logs, keep logs-history inside its budget, and if the
# run dir as a whole is still over its own, give up history first and then the .1 parts.
housekeeping() {
  local f
  for f in "$RUN"/*.log "$RUN"/resources.ndjson; do [ -f "$f" ] && cap_file "$f"; done
  prune_history_bytes "$HISTORY_BUDGET_BYTES"
  if [ "$(dir_bytes "$RUN")" -gt "$RUN_BUDGET_BYTES" ]; then
    echo "launcher: run dir over its $RUN_BUDGET_BYTES-byte budget; dropping log history, then .1 parts"
    prune_history_bytes 0
    [ "$(dir_bytes "$RUN")" -gt "$RUN_BUDGET_BYTES" ] && rm -f -- "$RUN"/*.log.1 "$RUN"/resources.ndjson.1
  fi
  return 0
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
# A child is registered once per process: a restart replaces its predecessor's pid, so neither
# children nor child_name grows with the number of restarts (ADR-008 I1, the launcher included).
register_child() {
  local name="$1" pid="$2" old kept=()
  for old in "${children[@]}"; do
    [ "${child_name[$old]:-}" = "$name" ] && { unset "child_name[$old]"; continue; }
    kept+=("$old")
  done
  children=("${kept[@]}" "$pid")
  child_name[$pid]="$name"
  write_children
}
# $RUN/children: name, pid and kernel start time of each child, for a stop that finds the launcher
# gone (stop_orphans). Written whole and renamed, so a reader never sees half of it.
children_written=0
write_children() {
  local pid
  children_written=1
  { for pid in "${children[@]}"; do
      printf '%s %s %s\n' "${child_name[$pid]:-unknown}" "$pid" \
        "$(awk '{print $22}' "/proc/$pid/stat" 2>/dev/null || echo 0)"
    done; } > "$RUN/children.tmp" 2>/dev/null && mv -f -- "$RUN/children.tmp" "$RUN/children" 2>/dev/null || true
}
# The services (ADR-008 D2): each started by one function, so a restart runs exactly the first
# start's command. Logs are opened O_APPEND (D5), and a restart appends to the same log.
launch_webd() {
  "${web_sched[@]}" "$PY" -m web.webd.app >>"$RUN/web.log" 2>&1 &
  web_pid=$!; register_child webd "$web_pid"
}
# ADR-005 D7: calibd, idle until the page starts a calibration run; controld runs the sessions and
# calibd fits and designs between them. Low priority: a calibd that dies moves nothing, and controld
# ends a run that hears nothing from it for 15 s.
launch_calibd() {
  OTA_SERVO_SIM="$APP/build/axis_control_core/servo-sim" nice -n 10 \
    "$PY" "$APP/tools/servo_commission/calibd.py" >>"$RUN/calibd.log" 2>&1 &
  calibd_pid=$!; register_child calibd "$calibd_pid"
}
launch_visiond() {
  "${vision_sched[@]}" "$PY" -m perception.visiond "${vision_args[@]}" >>"$RUN/vision.log" 2>&1 &
  vision_pid=$!; register_child visiond "$vision_pid"
}
# The continuous IMU capture. Its trace is truncated at each (re)start; it trims itself after that.
# In normal operation imu_pid and imu_shadow_pid name the same process (cleanup keeps the IMU alive
# through the controller's stop by imu_pid); a restart moves both.
launch_imu() {
  "${imu_sched[@]}" "$APP/build/imu-bno085" --continuous --retain-lines 4096 "${imu_reset_args[@]}" \
    >"$RUN/imu.ndjson" 2>>"$RUN/imu.log" &
  local old="${imu_shadow_pid:-}"
  imu_shadow_pid=$!; register_child imu-bno085 "$imu_shadow_pid"
  if [ -n "$old" ] && [ "${imu_pid:-}" = "$old" ]; then imu_pid="$imu_shadow_pid"; fi
}
launch_service() {
  case "$1" in
    webd) launch_webd ;; calibd) launch_calibd ;; visiond) launch_visiond ;; imu-bno085) launch_imu ;;
    *) return 1 ;;
  esac
}
# ADR-008 D2: supervision, with restarts, in normal operation only -- the station and its simulation,
# not a finite capture (--frames) and not a commissioning or perception-only run, whose children
# end their own stack as before.
SUPERVISE=0
if { [ "$MODE" = hardware ] || [ "$MODE" = sim ]; } && [ "$FRAMES" = 0 ]; then SUPERVISE=1; fi
declare -A svc_pid=() svc_started=() svc_restarts=() svc_delay=() svc_due=() svc_last_exit=() svc_evidence_at=()
# ADR-008 D5: crash evidence on the SD card, beside run/state, because the run dir is RAM and a
# reboot takes it. A bundle is the cause and the last MiB of each log that matters; the newest 20
# are kept, 200 MiB at most.
evidence_dir() {
  if [ -n "${OTA_EVIDENCE_DIR:-}" ]; then echo "$OTA_EVIDENCE_DIR"; return; fi
  local state; state="$(readlink -f -- "${OTA_STATE_DIR:-$RUN/state}" 2>/dev/null)" || state="$RUN/state"
  echo "$(dirname -- "$state")/evidence"
}
write_evidence() {
  local cause="$1" dir d f old
  shift
  dir="$(evidence_dir)"
  d="$dir/$(date -u +%Y%m%dT%H%M%SZ)-$cause"
  mkdir -p -- "$d" 2>/dev/null || return 0
  for f in "$RUN/shutdown.cause" "$RUN/shutdown.result" "$RUN/launcher.log" "$RUN/resources.json" "$@"; do
    [ -f "$f" ] && tail -c "$MIB" -- "$f" > "$d/$(basename -- "$f")" 2>/dev/null
  done
  # Under the boot service the launcher's own output is the user journal, not launcher.log.
  if [ -n "${INVOCATION_ID:-}" ] && command -v journalctl >/dev/null 2>&1; then
    # The station keeps no user journal; a user unit's lines are in the system journal (readable by
    # the adm group the operator is in).
    journalctl _SYSTEMD_USER_UNIT=ota-station.service -n 300 --no-pager > "$d/launcher.journal" 2>/dev/null || true
  fi
  printf 'cause=%s utc=%s launcher=%s uptime_s=%s release=%s\n' "$cause" \
    "$(date -u +%Y-%m-%dT%H:%M:%SZ)" "$$" "$SECONDS" "$APP" > "$d/what.txt" 2>/dev/null || true
  # The newest 10 of each cause, so a flapping service cannot push a controld crash's evidence out;
  # then 200 MiB in all, oldest first. A delete that fails ends the pruning, never loops on it.
  ( cd -- "$dir" 2>/dev/null || exit 0
    ls -1dt -- */ 2>/dev/null | awk -F/ '{c=$1; sub(/^[0-9TZ]+-/, "", c); if (++n[c] > 10) print $1}' |
      while IFS= read -r old; do rm -rf -- "$old" || break; done
    while [ "$(dir_bytes .)" -gt $((200 * MIB)) ]; do
      old="$(ls -1dtr -- */ 2>/dev/null | head -n 1)"; [ -n "$old" ] || break
      rm -rf -- "${old%/}" || break
    done ) || true
  echo "launcher: evidence saved in $d"
  return 0
}
# True when a bundle of this cause was written in the last 10 minutes: a stack that fails the same
# way at every start (a stray controld, a locked CAN bus) is restarted by systemd for months, and
# must not write megabytes to the SD card at each one.
evidence_recent() {
  local dir; dir="$(evidence_dir)"
  [ -d "$dir" ] && find "$dir" -maxdepth 1 -type d -name "*-$1" -mmin -10 2>/dev/null | grep -q .
}
# ADR-008 D6: what every process holds, once a minute, so a leak is seen long before it bites.
# resources.json is the latest (status prints it); resources.ndjson keeps the history, capped.
proc_line() {
  local name="$1" pid="$2" rss=0 thr=0 fds=0 up=0 restarts last
  if [ -n "$pid" ] && [ -r "/proc/$pid/status" ]; then
    rss="$(awk '/^VmRSS:/ {print $2}' "/proc/$pid/status" 2>/dev/null)"; rss="${rss:-0}"
    thr="$(awk '/^Threads:/ {print $2}' "/proc/$pid/status" 2>/dev/null)"; thr="${thr:-0}"
    fds="$(ls -1 "/proc/$pid/fd" 2>/dev/null | wc -l)"
  fi
  up=$((SECONDS - ${svc_started[$name]:-0}))
  restarts="${svc_restarts[$name]:-0}"; last="${svc_last_exit[$name]:-}"
  printf '"%s":{"pid":%s,"rss_kb":%s,"threads":%s,"fds":%s,"up_s":%s,"restarts":%s,"last_exit":"%s"}' \
    "$name" "${pid:-0}" "$rss" "$thr" "$fds" "$up" "$restarts" "$last"
}
write_resources() {
  local line name sep=',' tmp_avail sd_avail
  tmp_avail="$(df -B1 --output=avail -- "$RUN" 2>/dev/null | tail -n 1 | tr -d ' ')"
  sd_avail="$(df -B1 --output=avail -- "$APP" 2>/dev/null | tail -n 1 | tr -d ' ')"
  line="{\"utc\":\"$(date -u +%Y-%m-%dT%H:%M:%SZ)\",\"uptime_s\":$SECONDS,\"run_dir_bytes\":$(dir_bytes "$RUN"),"
  line+="\"tmpfs_avail_bytes\":${tmp_avail:-0},\"sd_avail_bytes\":${sd_avail:-0},\"processes\":{"
  line+="$(proc_line launcher "$$")$sep$(proc_line controld "${controller_pid:-}")"
  for name in webd calibd visiond imu-bno085; do
    [ -n "${svc_restarts[$name]+set}" ] || continue
    line+="$sep$(proc_line "$name" "${svc_pid[$name]:-}")"
  done
  line+="}}"
  printf '%s\n' "$line" > "$RUN/resources.json.tmp" 2>/dev/null && mv -f -- "$RUN/resources.json.tmp" "$RUN/resources.json" 2>/dev/null
  printf '%s\n' "$line" >> "$RUN/resources.ndjson" 2>/dev/null || true
}
# wait -n reports a status but not the child it belongs to; record both while the
# siblings are still alive to point at, i.e. before cleanup signals anyone.
note_child_exit() {
  local pid gone=()
  for pid in "${children[@]}"; do
    # An observe-only IMU that died earlier is not what ended the stack.
    [ "$pid" = "${imu_shadow_pid:-}" ] && continue
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
# A stop request counts only if it names this launcher (ADR-008 review, 2026-11-10).
stop_requested() {
  [ -r "$RUN/stop.request" ] && grep -q "launcher=$$\( \|\$\)" "$RUN/stop.request" 2>/dev/null
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
# ADR-007 D4: controld returns the actuator to rest and switches its PWM off when it stops. A controld
# that crashed cannot, and the hardware PWM keeps its last pulse -- B, if it died mid-shot -- so after
# the controller has exited the launcher does it: rest from the station's file, half a second of
# travel, off. Nothing to do when the channel is already off (the normal case). It runs inside cleanup,
# under errexit: nothing in it may fail the shell (no file, an unwritable channel), or the rest of the
# stack is never stopped.
actuator_release() {
  [ -n "${OTA_ACTUATOR_GPIO-}" ] || return 0
  local ch chip='' c dir rest='' how
  case "$OTA_ACTUATOR_GPIO" in 12) ch=0 ;; 13) ch=1 ;; 14) ch=2 ;; 15) ch=3 ;; *) return 0 ;; esac
  for c in /sys/class/pwm/pwmchip*; do
    case "$(readlink -f "$c" 2>/dev/null || true)" in *1f00098000*) chip=$c ;; esac
  done
  dir="${chip:-/nonexistent}/pwm$ch"
  [ "$(cat "$dir/enable" 2>/dev/null || true)" = 1 ] || return 0
  if [ -r "${OTA_STATE_DIR:-/nonexistent}/actuator.json" ]; then
    rest=$(sed -n 's/.*"rest_us": *\([0-9][0-9]*\).*/\1/p' "$OTA_STATE_DIR/actuator.json" 2>/dev/null | head -n 1) || rest=''
  fi
  how="left it where it was (no rest known)"
  if [ -n "$rest" ]; then
    if echo "${rest}000" > "$dir/duty_cycle" 2>/dev/null; then how="returned it to rest ($rest us)"; sleep 0.5
    else how="could not return it to rest ($rest us)"; fi
  fi
  if echo 0 > "$dir/enable" 2>/dev/null; then how="$how and switched it off"; else how="$how; switching it off FAILED"; fi
  echo "actuator: controld left the PWM on; the launcher $how"
}
cleanup() {
  # A second INT or TERM while the stack comes down is noted, not obeyed: with the default action
  # it killed the launcher in the middle of the controlled stop, before the controller was told to
  # park (GNU timeout sends TERM to the launcher and then to its group: two signals, ADR-008 dry
  # run, 2026-11-10). A trap that only records is reset in every child, so nothing inherits it.
  trap - EXIT
  trap 'cause_signal="${cause_signal:-INT}"' INT
  trap 'cause_signal="${cause_signal:-TERM}"' TERM
  # Nothing in here may end the shell before the children are signalled and the controller has
  # parked: a full tmpfs failing the write of shutdown.cause used to be enough, under errexit
  # (ADR-008 D5).
  set +e
  local reason
  if stop_requested; then reason=operator_stop
  elif [ -n "$exited_pid" ]; then reason=child_exit
  elif [ -n "$cause_signal" ]; then reason=external_signal
  else reason=unattributed; fi
  stop_cause_line "$reason" > "$RUN/shutdown.cause"
  cat "$RUN/shutdown.cause"
  rm -f -- "$RUN/stop.request"
  if [ "$MODE" = control-yaw ]; then
    echo 'Ending supervised yaw shared-core control; the C++ owner records current zero and pitch STOP feedback.'
  elif [ "$MODE" = establish-homing ]; then
    echo 'Ending bounded pitch sensorless homing; the C++ owner records zero/STOP feedback.'
  elif [ "$MODE" = imu ]; then
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
  # A trapped signal ends a wait early, with controld still parking (ADR-008 review, 2026-11-10):
  # wait again until the process is really gone.
  if [ -n "$controller_pid" ]; then
    kill -TERM "$controller_pid" 2>/dev/null || true
    wait "$controller_pid" || true
    while kill -0 "$controller_pid" 2>/dev/null; do wait "$controller_pid" || true; done
  fi
  actuator_release
  # Keep the single BNO085 owner alive through the controller's controlled
  # stop, then release I2C ownership.
  if [ -n "$imu_pid" ]; then
    kill -TERM "$imu_pid" 2>/dev/null || true
    wait "$imu_pid" || true
    while kill -0 "$imu_pid" 2>/dev/null; do wait "$imu_pid" || true; done
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
    if [ "$MODE" = control-yaw ]; then
      { echo 'Yaw shared-core control ended; inspect runtime parameter readback, raw trace and 3a measurements';
        tail -n 3 "$RUN/controller.log"; } > "$RUN/shutdown.result"
    elif [ "$MODE" = establish-homing ]; then
      { echo 'Sensorless homing ended; inspect immutable result; no retained calibration or stop qualification claim';
        tail -n 3 "$RUN/controller.log"; } > "$RUN/shutdown.result"
    elif [ "$MODE" = commission ] && [ "$MIXED_BACKEND_CHECK" = 1 ]; then
      { echo 'Stopped: mixed-backend no-motion probe ended; inspect STOP feedback result below';
        tail -n 8 "$RUN/controller.log"; } > "$RUN/shutdown.result"
    elif [ "$MODE" = commission ]; then
      { echo 'Stopped: commissioning probe ended; not a park/disable certification';
        tail -n 3 "$RUN/controller.log"; } > "$RUN/shutdown.result"
    elif grep -Fq 'SHUT DOWN (pitch released on its rest stop' "$RUN/controller.log"; then
      # The web's SHUTDOWN sequence, run by the stop itself (owner, 2026-10-03).
      echo 'Stopped: SHUT DOWN (pitch on its rest stop, yaw at 0, both motors off)' > "$RUN/shutdown.result"
    elif grep -Fq 'STOPPED (already shut down: both motors off)' "$RUN/controller.log"; then
      echo 'Stopped: already shut down (both motors off)' > "$RUN/shutdown.result"
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
  # Only the launcher that wrote the children record removes it: one that refused to start beside a
  # stray controld must leave the record that lets stop find it (ADR-008 review, 2026-11-10).
  [ "$children_written" != 1 ] || rm -f -- "$RUN/children"
  # ADR-008 D4: the exit code carries the cause to the boot service, whose Restart=on-failure
  # brings a crashed stack back in SHUTDOWN (owner, 2026-11-10). A stop someone asked for -- the
  # launcher's stop, a deploy, systemctl, a signal -- is a success and is not restarted; a
  # controller that exited, or a cause nobody can name, is a failure. Until 2026-11-10 every
  # path ended 0 here, and the station stayed down after a crash.
  if [ "$SUPERVISE" = 1 ]; then
    case "$reason" in
      operator_stop|external_signal) exit 0 ;;
      *) evidence_recent "$reason" || write_evidence "$reason" "$RUN"/*.log; exit 1 ;;
    esac
  fi
}
trap cleanup EXIT
trap 'cause_signal=INT; exit 130' INT
trap 'cause_signal=TERM; exit 143' TERM
rm -f -- "$RUN/shutdown.result"
# A request addressed to an earlier launcher (a stop that arrived while that one was already in its
# cleanup) must not make this one's crash look like an operator stop, and so not be restarted.
rm -f -- "$RUN/stop.request"
printf '%s %s\n' "$$" "$(awk '{print $22}' /proc/$$/stat)" > "$RUN/launcher.pid"
if [ "$MODE" = establish-homing ] || [ "$MODE" = control-yaw ]; then
  "$PY" "$APP/tools/adr0022_capture_launch.py" "$SESSION_MANIFEST" "--$MODE" >"$RUN/controller.log" 2>&1 &
  controller_pid=$!; children+=("$controller_pid"); child_name[$controller_pid]=commission-current-supervisor
  current_label='bounded pitch sensorless homing; no retained calibration write'
  [ "$MODE" != control-yaw ] || current_label='supervised yaw shared-core 3a control; pitch disabled, runtime parameter readback'
  printf 'Mode: %s\nManifest: %s\nMotion purpose, bounds, attendance and authorization recorded in manifest\n' "$current_label" "$SESSION_MANIFEST" > "$RUN/stack.info"
  cp "$RUN/launcher.pid" "$RUN/started"
  first_child_status=0; wait "$controller_pid" || first_child_status=$?
  note_child_exit
  if [ "$MODE" = control-yaw ]; then
    "$PY" "$APP/tools/adr0022_yaw_vibration.py" --manifest "$SESSION_MANIFEST" \
      >"$RUN/yaw-vibration-report.log" 2>&1 || echo 'Yaw IMU report could not be saved; raw capture and session outcome remain retained.' >&2
  fi
  exit "$first_child_status"
fi
if [ "$MODE" = imu ]; then
  if pgrep -x controld >/dev/null || pgrep -x imu_main >/dev/null; then
    echo 'Existing controller or legacy IMU consumer; refusing capture.' >&2; exit 1
  fi
  "$APP/build/imu-bno085" "$IMU_SECONDS" "${imu_reset_args[@]}" >"$RUN/imu.ndjson" 2>"$RUN/imu.log" &
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
    "$APP/build/imu-bno085" 120 "${imu_reset_args[@]}" >"$RUN/imu.ndjson" 2>"$RUN/imu.log" &
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
# (b): visiond 是唯一持有物理相机的人，所以由它发布"有哪些有名字的流"。
# 缺这个变量 visiond 就不发布（默认关闭），老部署不受影响。
export OTA_VISION_STREAM_MANIFEST="$RUN/video_streams.json"
# 推理后端自述（后端名/模型/输入尺寸/推理计数/model 耗时）。web 不碰相机，只读这份现读。
export OTA_INFERENCE_HEALTH="$RUN/inference_health.json"
# 第二颗传感器的**开关只有一个：配置文档里的 vision.secondary.model**（代码默认空 = 这台站没有第二颗）。
# 以前这里还写死了一个 imx477 默认值，于是"有没有第二路"存在两处真相 —— 朝向那次就是被这种重复
# 掩住的：文档里的值没被读到，环境变量把型号补上了，症状一个都没有。
# 环境变量保留成**一次启动的显式覆盖**：填型号就换一颗，填 `off` 就这一次启动不开这路
# （visiond 会点名拒绝并把广角照常服务），不设就照文档。开不起时只降级这条流，不带走广角。
export OTA_VISION_DETAIL_SENSOR="${OTA_VISION_DETAIL_SENSOR-}"
export OTA_SELECTION_SOCKET="$RUN/selection.sock"
export OTA_VISION_SOCKET="$RUN/vision.sock"
export OTA_WEB_SOCKET="$RUN/control-web.sock"
# SURVEILLANCE's watch point (owner, 2026-10-05) must outlive restarts, deploys and reboots, so the
# station keeps it beside the shared venv: <root>/run/state, which deploy_station.py links into
# every release (a checkout reaches the same directory directly). Anything but the real station
# keeps its own in the run dir, so a simulation never moves the station's point.
if [ "$MODE" = hardware ] && [ "$SIM_REQUESTED" != 1 ]; then
  OTA_STATE_DIR="${OTA_STATE_DIR:-$APP/../run/state}"
else
  OTA_STATE_DIR="${OTA_STATE_DIR:-$RUN/state}"
fi
export OTA_STATE_DIR
# ADR-007: the one-shot actuator's pin (owner, 2026-10-30: a hobby servo on GPIO15, physical pin 10).
# The owner's pwm overlay (config.txt) creates the RP1 PWM0 chip but leaves the pin on another function
# on the Pi 5, so the mux is set here on every start and checked; controld drives the channel. Any
# problem goes to controld, which publishes it and refuses ACTUATE: a degrade, never a stop.
# GPIO18/19 are refused: on this station they are SPI1, the pitch bus's chip select and MISO.
# OTA_ACTUATOR_GPIO= (empty) disables the actuator.
OTA_ACTUATOR_UNAVAILABLE=
if [ "$MODE" = hardware ] && [ "$SIM_REQUESTED" != 1 ]; then
  OTA_ACTUATOR_GPIO="${OTA_ACTUATOR_GPIO-15}"
else
  OTA_ACTUATOR_GPIO="${OTA_ACTUATOR_GPIO-}"
fi
if [ -n "$OTA_ACTUATOR_GPIO" ]; then
  case "$OTA_ACTUATOR_GPIO" in
    12|13|14|15)
      if ! pinctrl set "$OTA_ACTUATOR_GPIO" a0 2>/dev/null; then
        OTA_ACTUATOR_UNAVAILABLE="pinctrl could not mux GPIO$OTA_ACTUATOR_GPIO"
      elif ! pinctrl get "$OTA_ACTUATOR_GPIO" | grep -q PWM0_CHAN; then
        OTA_ACTUATOR_UNAVAILABLE="GPIO$OTA_ACTUATOR_GPIO is not on PWM0 ($(pinctrl get "$OTA_ACTUATOR_GPIO" 2>&1 || true))"
      elif pgrep -f '^[^ ]*python[0-9.]* [^ ]*actuator_calibrate\.py' >/dev/null 2>&1; then
        # The bench tool writes the same channel; two writers would fight over the servo.
        OTA_ACTUATOR_UNAVAILABLE="the bench tool actuator_calibrate.py is running; stop it and restart the stack"
      fi ;;
    *) OTA_ACTUATOR_UNAVAILABLE="GPIO$OTA_ACTUATOR_GPIO is not an actuator pin here (PWM0 on GPIO12-15 only)" ;;
  esac
  if [ -n "$OTA_ACTUATOR_UNAVAILABLE" ]; then
    echo "actuator: $OTA_ACTUATOR_UNAVAILABLE; ACTUATE will be refused" >&2
  else
    echo "actuator: GPIO$OTA_ACTUATOR_GPIO on PWM0"
  fi
fi
export OTA_ACTUATOR_GPIO OTA_ACTUATOR_UNAVAILABLE
# ADR-003 D17: controld's per-tick tracking record (one JSON line per Level-1 tick while tracking).
export OTA_TRACKING_TRACE="$RUN/tracking-trace.jsonl"
export OTA_PERCEPTION_TRACE="$RUN/perception-trace.jsonl"   # joins the tracking trace by sensor_ns
export OTA_WEB_PORT="${OTA_WEB_PORT:-8080}"
export OTA_WEB_HOST="${OTA_WEB_HOST:-0.0.0.0}"
# Scheduling (owner ruling 2026-10-03; docs/operations/os-setup.md). Real time belongs to threads,
# not processes: OTA_RT=1 lets controld put its motor threads (CAN RX + yaw servo, pitch servo,
# guards, the 200 Hz loop) on SCHED_FIFO and nice its web/IMU/log threads; nothing else is real
# time. Per process: controld gets CPU 3 to itself; perception, the IMU observer and the web UI
# share CPUs 0-2, the web UI niced lowest, so the UI is what waits when the station is busy.
# OTA_RT=0 keeps every controld thread SCHED_OTHER; OTA_CPU_PIN=0 leaves the CPUs unpinned.
# Two states (owner ruling 2026-10-03): Homed or Shutdown. A hardware start is Shutdown -- web,
# camera and controller up, both motors off, nothing moves until the web's HOME -- unless --home
# (or OTA_START_STATE=homed) asks for the station to home now; a deploy passes the state it found.
# --sim keeps homing at start: the simulation tools drive a homed plant.
START_STATE="${OTA_START_STATE:-}"
[ "$START_STATE_HOME" = 1 ] && START_STATE=homed
if [ -z "$START_STATE" ]; then
  if [ "$MODE" = hardware ]; then START_STATE=shutdown; else START_STATE=homed; fi
fi
case "$START_STATE" in
  homed|shutdown) ;;
  *) echo "OTA_START_STATE must be homed or shutdown, not '$START_STATE'" >&2; exit 2 ;;
esac
control_sched=(env "OTA_RT=${OTA_RT:-1}" "OTA_START_STATE=$START_STATE")
vision_sched=()
imu_sched=(nice -n 10)
web_sched=(nice -n 19)
if [ "${OTA_CPU_PIN:-1}" = 1 ] && command -v taskset >/dev/null && [ "$(nproc)" -ge 4 ]; then
  control_sched+=(taskset -c "${OTA_CONTROL_CPUS:-3}")
  vision_sched=(taskset -c "${OTA_APP_CPUS:-0-2}" "${vision_sched[@]}")
  imu_sched=(taskset -c "${OTA_APP_CPUS:-0-2}" "${imu_sched[@]}")
  web_sched=(taskset -c "${OTA_APP_CPUS:-0-2}" "${web_sched[@]}")
fi
vision_args=(--config perception/configs/perception_v1.json --profile "$PROFILE"
             --max-frames "$FRAMES" --publish-dir "$RUN/perception"
             --selection-socket "$OTA_SELECTION_SOCKET")
if [ "$PRODUCTION" -eq 1 ]; then vision_args+=(--production); fi
if [ "$PROFILE_PINNED" -eq 1 ]; then vision_args+=(--pin-profile); fi
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
  if [ "$START_STATE" = shutdown ]; then
    echo 'HARDWARE station, SHUTDOWN: web and camera up, both motors off. MENU > HOME homes it, then automatic roam/track begins.'
  else
    echo 'HARDWARE station: homing establishes calibration, then automatic roam/track begins. Web Manual overrides autonomy.'
  fi
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
  launch_imu; imu_pid="$imu_shadow_pid"
  # Commissioning waits on this IMU (its failure ends the session); only normal operation makes it
  # observe-only, below.
  [ "$MODE" = hardware ] || imu_shadow_pid=''
  # Commissioning requires a fresh host tare and same-generation game rotation sample before the
  # controller starts: there the IMU trace is evidence. Normal operation only waits for one, then
  # starts without it (below). The IMU residual remains observe-only either way.
  imu_ready=0
  for ((attempt=0; attempt<50; attempt++)); do
    if ! kill -0 "$imu_pid" 2>/dev/null; then
      [ "$MODE" = hardware ] && break
      echo 'Continuous BNO085 startup failed' >&2; exit 1
    fi
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
  if [ "$imu_ready" != 1 ] && [ "$MODE" != hardware ]; then
    echo 'Fresh same-generation BNO085 host tare/sample unavailable; controller not started.' >&2
    exit 1
  fi
  if [ "$MODE" = hardware ]; then
    # Station, 2026-10-03 16:35: the BNO085 held SDA low (the known I2C fault) and this gate kept the
    # whole station down after a deploy. In normal operation the IMU is an instrument, not a
    # precondition (owner ruling 2026-10-03; controld says the same): start without it, let it keep
    # reconnecting, and do not wait on it -- the same footing as the observe-only capture below.
    if [ "$imu_ready" != 1 ]; then
      if kill -0 "$imu_pid" 2>/dev/null; then
        echo 'BNO085: no fresh tare within 5 s; starting without it (observe-only; it keeps reconnecting).' >&2
      else
        echo 'BNO085 capture exited at startup; starting without it (the UI reports the IMU absent).' >&2
        unset OTA_IMU_TRACE
      fi
    fi
    imu_shadow_pid="$imu_pid"
  fi
elif { [ "$MODE" = hardware ] || [ "$MODE" = mixed-controller-commission ]; }; then
  # 观测档 IMU：给 UI 用的那路（§20 的 imu 块）。和上面那条的区别是**它不硬**——标定档要求
  # "没有新鲜 tare 就不启动控制器"，而日常启动里 IMU 说话说不通只该让 UI 少一块信息。
  # 用另一个变量名是有意的：上面那条把 imu_pid 交给了 wait -n，传感器一掉就把整栈收尾；
  # 观测档不能这么干，所以它只登记进 children（停止时清），不参与 wait -n。
  if [ "${OTA_IMU_ENABLE:-1}" = "1" ] && [ -x "$APP/build/imu-bno085" ]; then
    if pgrep -x imu_main >/dev/null; then
      echo 'Legacy IMU consumer still running; the UI will report the IMU as not configured.' >&2
    else
      export OTA_IMU_TRACE="$RUN/imu.ndjson"
      launch_imu
      imu_shadow_ready=0
      for ((attempt=0; attempt<30; attempt++)); do
        kill -0 "$imu_shadow_pid" 2>/dev/null || break
        if grep -q '"kind":"sample"' "$RUN/imu.ndjson" 2>/dev/null; then imu_shadow_ready=1; break; fi
        sleep 0.1
      done
      if [ "$imu_shadow_ready" = 1 ]; then
        echo 'BNO085 continuous capture running (observe-only; the station does not depend on it).'
      elif kill -0 "$imu_shadow_pid" 2>/dev/null; then
        # It keeps reconnecting (imu_bno085.c), so a sensor that is slow or briefly off the bus
        # joins later instead of being given up on at boot.
        echo 'BNO085 produced no samples within 3s; it keeps reconnecting and the station keeps running.' >&2
      else
        echo 'BNO085 capture exited at startup; the UI reports the IMU as absent and the station keeps running.' >&2
        unset OTA_IMU_TRACE
      fi
    fi
  fi
fi
# ADR-005: an empty calibration store gets record #1 from this release's configured files; a store
# with records is left alone (no comparison with the source, owner 2026-10-24). controld reads the
# record in force; if seeding fails it says so and runs on the configured files.
if ! "$PY" "$APP/tools/calibration_records.py" seed --state "$OTA_STATE_DIR" \
     --config "$ACTIVE_CONTROL_CONFIG" --root "$APP" >"$RUN/calibration-seed.log" 2>&1; then
  echo "calibration records: seeding failed (see $RUN/calibration-seed.log); controld uses the configured files" >&2
else
  cat "$RUN/calibration-seed.log"
fi
"${control_sched[@]}" "$CONTROLD" "${controller_args[@]}" >>"$RUN/controller.log" 2>&1 &
controller_pid=$!
children+=("$controller_pid")
child_name[$controller_pid]=controld
else
  echo 'Perception only: no controller connection.'
  START_WEB=0
fi
if [ "$START_WEB" -eq 1 ]; then
  launch_webd
  printf '%s\n' "$OTA_WEB_PORT" > "$RUN/web.port"
  launch_calibd
fi
if [ "$MODE" != mixed-controller-commission ]; then
  if [ "$MODE" != perception ]; then
    vision_args+=(--publish-socket "$OTA_VISION_SOCKET")
    # The operating mode comes from controld itself, not through the web UI: the UI runs last
    # (docs/operations/os-setup.md), and perception must not wait on it.
    vision_args+=(--controller-state-url "unix:$OTA_WEB_SOCKET")
  fi
  launch_visiond
elif [ "$MODE" = mixed-controller-commission ]; then
  echo 'Mixed controller commissioning: vision and web are not started.'
fi

printf 'Mode: %s\nConfig: %s\nPython: %s\nChildren: %s\n' "$MODE" "${controller_args[0]}" "$PY" "${children[*]}" > "$RUN/stack.info"
cp "$RUN/launcher.pid" "$RUN/started"
echo "Stack logs: $RUN; web port: $OTA_WEB_PORT. Ctrl-C stops this stack."
write_children
first_child_status=0
if [ "$SUPERVISE" != 1 ]; then
  # A failed child or finite capture ends its own stack; unrelated processes are untouched.
  # The observe-only IMU and calibd are not among them: they sit in children so cleanup stops them,
  # but they are not waited on. Station, 2026-10-03 14:27:32: the BNO085 lost its I2C link, its
  # process exited, and this wait took the whole stack down with it (owner ruling 2026-10-02: losing
  # a display-only sensor is not even a HOLD).
  waited=()
  for pid in "${children[@]}"; do
    [ "$pid" = "${imu_shadow_pid:-}" ] || [ "$pid" = "${calibd_pid:-}" ] || waited+=("$pid")
  done
  wait -n "${waited[@]}" || first_child_status=$?
  note_child_exit
  exit "$first_child_status"
fi

# ADR-008 D2: supervision. Only controld owns motor safety, so only its exit ends the stack. The
# services -- visiond, webd, calibd, the observe-only IMU -- fail alone and are started again, with a
# backoff of 1, 2, 4 ... s capped at 60 s, reset by 10 minutes up; never given up on. Station,
# 2026-11-10 13:13: visiond died on one bad label, this wait stopped the whole station, and nothing
# brought it back. controld meets each one going away as it already must: vision lost (its tracker
# loses the target, a HOLD at most), a page that reconnects, a calibration run that fails. The
# station stays homed through any of them.
# A one-second ticker in the same wait drives the restarts, the housekeeping (10 s) and the resource
# watch (60 s); wait -n -p (bash 5.1+) names whichever child ended.
for pid in "${children[@]}"; do
  name="${child_name[$pid]:-}"
  case "$name" in
    webd|calibd|visiond) ;;
    imu-bno085) [ -n "${OTA_IMU_TRACE:-}" ] && [ "$pid" = "${imu_shadow_pid:-}" ] || continue ;;
    *) continue ;;
  esac
  svc_pid[$name]="$pid"; svc_started[$name]="$SECONDS"; svc_restarts[$name]=0; svc_delay[$name]=0
done
service_log() {
  case "$1" in webd) echo "$RUN/web.log" ;; visiond) echo "$RUN/vision.log" ;;
    calibd) echo "$RUN/calibd.log" ;; imu-bno085) echo "$RUN/imu.log" ;; esac
}
service_exited() {
  local name="$1" status="$2" up delay log
  up=$((SECONDS - ${svc_started[$name]}))
  if [ "$up" -ge 600 ]; then delay=1
  else delay=$((2 * ${svc_delay[$name]})); [ "$delay" -ge 1 ] || delay=1; [ "$delay" -le 60 ] || delay=60; fi
  svc_delay[$name]="$delay"
  svc_restarts[$name]=$((${svc_restarts[$name]} + 1))
  svc_last_exit[$name]="$(describe_status "$status") at $(date -u +%Y-%m-%dT%H:%M:%SZ) after ${up}s"
  svc_pid[$name]=''
  svc_due[$name]=$((SECONDS + delay))
  log="$(service_log "$name")"
  echo "launcher: $name exited ($(describe_status "$status")) after ${up} s; restart ${svc_restarts[$name]} in ${delay} s"
  printf '[%s] launcher: %s exited (%s) after %s s; restart %s in %s s\n' "$(date '+%Y-%m-%d %H:%M:%S')" \
    "$name" "$(describe_status "$status")" "$up" "${svc_restarts[$name]}" "$delay" >> "$log" 2>/dev/null || true
  # One evidence bundle per service per 10 minutes: enough for the why, and a service crash-looping
  # at the 60 s cap does not write a megabyte a minute to the SD card.
  if [ $((SECONDS - ${svc_evidence_at[$name]:--600})) -ge 600 ]; then
    svc_evidence_at[$name]="$SECONDS"
    write_evidence "restart-$name" "$log"
  fi
}
restart_due_services() {
  local name
  for name in "${!svc_due[@]}"; do
    [ "$SECONDS" -ge "${svc_due[$name]}" ] || continue
    unset "svc_due[$name]"
    launch_service "$name" || continue
    svc_pid[$name]="${children[-1]}"      # register_child appends the new process last
    svc_started[$name]="$SECONDS"
    echo "launcher: $name started again (pid ${svc_pid[$name]}, restart ${svc_restarts[$name]})"
    printf '[%s] launcher: %s started again (pid %s, restart %s)\n' "$(date '+%Y-%m-%d %H:%M:%S')" \
      "$name" "${svc_pid[$name]}" "${svc_restarts[$name]}" >> "$(service_log "$name")" 2>/dev/null || true
  done
}
tick=0
while :; do
  sleep 1 &
  ticker=$!
  waitlist=("$ticker" "$controller_pid")
  for name in "${!svc_pid[@]}"; do [ -z "${svc_pid[$name]}" ] || waitlist+=("${svc_pid[$name]}"); done
  ended=''; status=0
  wait -n -p ended "${waitlist[@]}" || status=$?
  if [ "$ended" = "$ticker" ]; then
    tick=$((tick + 1))
    restart_due_services
    if [ $((tick % 10)) -eq 0 ]; then
      housekeeping
      # Belt and braces for wait -n: a child that died while this shell was busy elsewhere is a
      # zombie (state Z), which kill -0 cannot tell from a live one. Reap it here (ADR-008 review).
      for name in "${!svc_pid[@]}" controld; do
        if [ "$name" = controld ]; then pid="$controller_pid"; else pid="${svc_pid[$name]}"; fi
        [ -n "$pid" ] || continue
        pstate=''
        { read -r _ _ pstate _ < "/proc/$pid/stat"; } 2>/dev/null || pstate=gone
        [ "$pstate" = Z ] || [ "$pstate" = gone ] || continue
        status=0; wait "$pid" 2>/dev/null || status=$?
        if [ "$name" = controld ]; then
          first_child_status="$status"; exited_pid="$pid"; exited_name=controld; break 2
        fi
        service_exited "$name" "$status"
      done
    fi
    [ $((tick % 60)) -ne 0 ] || write_resources
    continue
  fi
  kill "$ticker" 2>/dev/null || true; wait "$ticker" 2>/dev/null || true
  # A signal ends the shell through its trap before control comes back here; a wait that returned
  # without naming a child is taken as controld's exit only if controld is in fact gone.
  if [ -z "$ended" ] && kill -0 "$controller_pid" 2>/dev/null; then continue; fi
  if [ -z "$ended" ] || [ "$ended" = "$controller_pid" ]; then
    first_child_status="$status"; exited_pid="${ended:-$controller_pid}"; exited_name=controld
    break
  fi
  for name in "${!svc_pid[@]}"; do
    if [ "${svc_pid[$name]}" = "$ended" ]; then service_exited "$name" "$status"; fi
  done
  write_resources
done
exit 1
