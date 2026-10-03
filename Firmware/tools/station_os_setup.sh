#!/usr/bin/env bash
# One-time OS setup of the station for controld's real-time motor threads.
# Procedure, reasons and verification: Firmware/docs/operations/os-setup.md.
#
#   bash Firmware/tools/station_os_setup.sh --check              # no sudo; reports, changes nothing
#   sudo bash Firmware/tools/station_os_setup.sh --apply [options]
#
# --apply always installs the real-time grant (limits.d). The rest are opt-in:
#   --autostart             start the station at boot, in SHUTDOWN (motors off until HOME): keeps the
#                           operator's user manager running without a login (linger) and gives it
#                           the same real-time grant. One time only: every deploy after this
#                           installs and refreshes the service itself, without sudo.
#   --isolate-cpu3          kernel command line: isolcpus=3 irqaffinity=0-2 (reboot needed)
#   --performance-governor  pin the CPU clock at its maximum from boot (only once the supply is
#                           stiff: it raises the current draw on a rail that already droops)
#   --headless              boot to multi-user.target (no desktop; nothing on the station uses it)
#   --user NAME             the operator account (default: $SUDO_USER, else eamars)
#
# Every step is idempotent and backs up what it edits. Nothing here restarts the stack.
set -euo pipefail

LIMITS=/etc/security/limits.d/90-ota-realtime.conf
CMDLINE=/boot/firmware/cmdline.txt
GOVERNOR_UNIT=/etc/systemd/system/ota-cpu-governor.service
# Highest SCHED_FIFO priority the operator may take. controld's motor threads use 44..48; the
# kernel's threaded IRQ handlers (the SPI CAN controllers that feed them) run at 50 and must stay
# above every user thread, so the grant stops at 49.
RTPRIO_MAX=49

mode=""; isolate=0; governor=0; headless=0; autostart=0; user="${SUDO_USER:-eamars}"
while [ $# -gt 0 ]; do
  case "$1" in
    --check) mode=check ;;
    --apply) mode=apply ;;
    --isolate-cpu3) isolate=1 ;;
    --performance-governor) governor=1 ;;
    --headless) headless=1 ;;
    --autostart) autostart=1 ;;
    --user) user="$2"; shift ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
  shift
done
[ -n "$mode" ] || { sed -n '2,15p' "$0"; exit 2; }

check() {
  echo "== station OS setup check (user $user)"
  local uid; uid=$(id -u "$user")
  if [ -r "$LIMITS" ]; then echo "limits:   $LIMITS present:"; sed 's/^/            /' "$LIMITS"
  else echo "limits:   MISSING ($LIMITS) -- controld's SCHED_FIFO and mlockall requests will be refused"; fi
  echo "session:  this shell has rtprio=$(ulimit -r) memlock=$(ulimit -l) (a new login picks up limits.d)"
  if grep -qs pam_limits /etc/pam.d/sshd /etc/pam.d/common-session; then
    echo "pam:      pam_limits active for logins"
  else
    echo "pam:      pam_limits NOT found in sshd/common-session -- limits.d would not apply"
  fi
  echo "governor: $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null || echo unknown)"
  echo "isolcpus: $(cat /sys/devices/system/cpu/isolated 2>/dev/null || true) (empty = none); cmdline:" \
       "$(grep -o 'isolcpus=[^ ]*\|irqaffinity=[^ ]*' /proc/cmdline | tr '\n' ' ')"
  echo "target:   $(systemctl get-default)"
  echo "throttle: $(vcgencmd get_throttled 2>/dev/null || echo unknown)"
  echo "rt cap:   sched_rt_runtime_us=$(cat /proc/sys/kernel/sched_rt_runtime_us) (950000 = the kernel keeps 5% for others)"
  echo "boot:     linger=$(loginctl show-user "$user" -p Linger --value 2>/dev/null || echo unknown)"        "user-manager grant=$([ -r "/etc/systemd/system/user@$uid.service.d/ota-realtime.conf" ] && echo present || echo MISSING)"
  local mgr; mgr=$(pgrep -u "$user" -x systemd | head -1 || true)
  [ -z "$mgr" ] || echo "          user manager now: $(grep -E 'realtime priority' "/proc/$mgr/limits" | tr -s ' ')"                        "(the grant applies from the next boot)"
  echo "          run/current -> $(readlink "$(getent passwd "$user" | cut -d: -f6)/workspace/OpenAutoTurret/run/current" 2>/dev/null || echo 'none yet (the next deploy makes it)')"
}

if [ "$mode" = check ]; then check; exit 0; fi
[ "$(id -u)" = 0 ] || { echo "--apply needs sudo" >&2; exit 1; }
id "$user" >/dev/null 2>&1 || { echo "no such user: $user" >&2; exit 1; }
stamp=$(date +%Y%m%d-%H%M%S)

# 1. The real-time grant: SCHED_FIFO up to $RTPRIO_MAX and unlimited mlock, for the operator only.
want="# OpenAutoTurret (Firmware/docs/operations/os-setup.md): controld's motor threads run SCHED_FIFO
# 44..48 and lock their memory. Capped below the kernel's IRQ threads (FIFO 50).
$user - rtprio $RTPRIO_MAX
$user - memlock unlimited"
if [ "$(cat "$LIMITS" 2>/dev/null)" != "$want" ]; then
  [ -e "$LIMITS" ] && cp -a "$LIMITS" "$LIMITS.bak-$stamp"
  printf '%s\n' "$want" > "$LIMITS"
  chmod 644 "$LIMITS"
  echo "installed $LIMITS (takes effect at the operator's next login; restart the stack from a new SSH session)"
else
  echo "limits already in place"
fi

# 2. Optional: keep CPU 3 for controld alone (the launcher pins controld there).
if [ "$isolate" = 1 ]; then
  line=$(head -n1 "$CMDLINE")
  new="$line"
  case " $new " in *" isolcpus="*) ;; *) new="$new isolcpus=3" ;; esac
  case " $new " in *" irqaffinity="*) ;; *) new="$new irqaffinity=0-2" ;; esac
  if [ "$new" != "$line" ]; then
    cp -a "$CMDLINE" "$CMDLINE.bak-$stamp"
    # cmdline.txt must stay one line; write it whole, then check it.
    printf '%s\n' "$new" > "$CMDLINE.new"
    [ "$(wc -l < "$CMDLINE.new")" = 1 ] || { echo "refusing: cmdline would not be one line" >&2; rm -f "$CMDLINE.new"; exit 1; }
    mv "$CMDLINE.new" "$CMDLINE"
    echo "kernel command line updated (backup $CMDLINE.bak-$stamp); reboot to apply"
  else
    echo "CPU isolation already on the kernel command line"
  fi
fi

# 3. Optional: performance governor from boot.
if [ "$governor" = 1 ]; then
  cat > "$GOVERNOR_UNIT" <<'UNIT'
[Unit]
Description=OpenAutoTurret: CPU governor performance (docs/operations/os-setup.md)
After=multi-user.target

[Service]
Type=oneshot
ExecStart=/bin/sh -c 'for g in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do echo performance > "$g"; done'
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
UNIT
  systemctl daemon-reload
  systemctl enable --now ota-cpu-governor.service
  echo "governor: $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor)"
fi

# 4. Optional: start at boot. The service itself is a user unit that every deploy installs and
#    refreshes (tools/deploy_station.py); what needs root, once, is letting the user manager run
#    without a login and giving it -- and so the station it starts -- the real-time grant above.
if [ "$autostart" = 1 ]; then
  uid=$(id -u "$user")
  loginctl enable-linger "$user"
  dropin="/etc/systemd/system/user@$uid.service.d"
  mkdir -p "$dropin"
  cat > "$dropin/ota-realtime.conf" <<CONF
# OpenAutoTurret (Firmware/docs/operations/os-setup.md): the station's boot service is a user unit,
# and a user unit inherits its limits from this manager. Same grant as $LIMITS.
[Service]
LimitRTPRIO=$RTPRIO_MAX
LimitMEMLOCK=infinity
CONF
  systemctl daemon-reload
  echo "autostart: linger on for $user; user manager real-time grant installed (from the next boot)"
fi

# 5. Optional: no desktop session.
if [ "$headless" = 1 ]; then
  systemctl set-default multi-user.target
  echo "boots to multi-user.target from the next reboot (undo: systemctl set-default graphical.target)"
fi

check
