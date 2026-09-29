#!/usr/bin/env bash
# Read-only Stage 2 inventory. No device opens, CAN sends, service actions,
# checkout updates, compilation, package installs or station file writes.
set -u
export LC_ALL=C GIT_OPTIONAL_LOCKS=0
station_root=${1:-/home/eamars/workspace/OpenAutoTurret}
runtime=${2:-/tmp/ota-stack-$(id -u)}
# The operator supplies a hash of a launcher whose status branch they inspected.
# A missing/different hash is evidence missing, never permission to execute it.
trusted=${3:-}
errors=0
[[ "$station_root" = /* && "$runtime" = /* ]] || { printf 'Absolute paths required\n' >&2; exit 2; }
section() {
  printf '\n@@BEGIN %s\n' "$1"
  shift
  "$@" 2>&1
  rc=$?
  if [ "$rc" -ne 0 ]; then errors=$((errors+1)); fi
  printf '@@END rc=%s\n' "$rc"
}
read_if_present() {
  if [ -f "$1" ]; then head -c 65536 -- "$1"; else printf 'ABSENT %s\n' "$1"; fi
}
tail_if_present() {
  if [ -f "$1" ]; then tail -c 32768 -- "$1"; else printf 'ABSENT %s\n' "$1"; fi
}
metadata() {
  if [ -e "$1" ] || [ -L "$1" ]; then
    stat -c '%n | %F | %s bytes | %y | %A | %U:%G' -- "$1"
    if [ -L "$1" ]; then readlink -- "$1"; fi
  else printf 'ABSENT %s\n' "$1"; fi
}
root_listing() {
  for item in "$station_root" "$station_root/run" "$station_root/run/"* "$station_root/.venv"; do
    metadata "$item" || return
  done
}
release_listing() {
  if [ -d "$station_root/run/releases" ]; then
    find "$station_root/run/releases" -mindepth 1 -maxdepth 1 -type d -printf '%T@ %p\n' | sort -nr
  else printf 'ABSENT %s/run/releases\n' "$station_root"; fi
}
launcher_status() {
  local code
  bash "$launcher" status
  code=$?
  printf '\nSTATUS_EXIT_CODE=%s\n' "$code"
  # The inspected launcher returns 1 when its PID file has no live owner.
  # Preserve that observation; it is not proof of motor disable or settling.
  [ "$code" -eq 0 ] || [ "$code" -eq 1 ]
}
section utc date -u +%Y-%m-%dT%H:%M:%SZ
section account id
section kernel uname -a
section os read_if_present /etc/os-release
section uptime read_if_present /proc/uptime
section locks read_if_present /proc/locks
section memory read_if_present /proc/meminfo
section disk df -Pk "$station_root" /tmp
section processes ps -u "$(id -u)" -o pid,ppid,lstart,comm,args -ww
section head git --no-optional-locks -c core.fsmonitor=false -C "$station_root" rev-parse HEAD
section status git --no-optional-locks -c core.fsmonitor=false -C "$station_root" status --porcelain=v1 --untracked-files=normal
section dirty_diff git --no-optional-locks -c core.fsmonitor=false -C "$station_root" diff --no-ext-diff --no-textconv HEAD -- Firmware/config Firmware/scripts Firmware/control
section links ip -j -details -statistics link show
section can_service_active systemctl is-active ota-can-links.service
section can_service_enabled systemctl is-enabled ota-can-links.service
if command -v vcgencmd >/dev/null 2>&1; then
  section power vcgencmd get_throttled
  section cpu_temperature vcgencmd measure_temp
fi
for item in launcher.pid stack.info shutdown.cause shutdown.result web.port; do
  section "runtime/$item" read_if_present "$runtime/$item"
done
for item in launcher.log controller.log; do
  section "runtime/$item" tail_if_present "$runtime/$item"
done
section root_listing root_listing
for item in run/station-venv/bin/python .venv/bin/python Firmware/.venv/bin/python; do
  section "$item" metadata "$station_root/$item"
done
for item in /dev/i2c-1 /dev/i2c-0 /sys/class/net/can0/device /sys/class/net/can1/device /tmp/ota-motion-$(id -u).lock; do
  section "$item" metadata "$item"
done
section release_listing release_listing
# Inspect at most the three most recent release directories. No artifact executes.
count=0
while IFS= read -r release; do
  [ -d "$release" ] || continue
  count=$((count+1))
  [ "$count" -le 3 ] || break
  section "$release/REVISION" read_if_present "$release/REVISION"
  for item in Firmware/config/mixed_hardware.yaml Firmware/config/turret_mixed.yaml Firmware/scripts/run_application.sh; do
    section "$release/$item" read_if_present "$release/$item"
  done
  for item in Firmware/build-arm64/control/controld Firmware/build/control/controld Firmware/build-arm64/axis_control_core/commissiond run/station-venv/bin/python; do
    section "$release/$item" metadata "$release/$item"
    if [ -f "$release/$item" ]; then section "$release/$item/sha256" sha256sum -- "$release/$item"; fi
  done
done < <(release_listing | sed -n 's/^[0-9.]* //p')
section runtime_listing ls -lah "$runtime"
section retained_paths find "$station_root/Firmware" -maxdepth 3 -type f -name '*retained*'
# Execute only the status branch already inspected at this exact source hash.
launcher="$station_root/Firmware/scripts/run_application.sh"
if [[ "$trusted" =~ ^[0-9a-f]{64}$ ]] && [ -f "$launcher" ] && [ "$(sha256sum -- "$launcher" | cut -d ' ' -f 1)" = "$trusted" ]; then
  section launcher_status launcher_status
else
  printf '\n@@STATUS_NOT_EXECUTED source differs or is absent; use recorded processes\n'
fi
printf '\n@@INVENTORY_COMPLETE errors=%s\n' "$errors"
[ "$errors" -eq 0 ] || exit 2
