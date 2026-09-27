#!/usr/bin/env bash
# Boot-time CAN link configuration. The launcher remains the sole motor owner.
set -euo pipefail

configure_link() {
  local dev="$1" flags details
  [[ -e "/sys/class/net/$dev" ]] || { echo "$dev is missing" >&2; return 1; }
  read -r flags < "/sys/class/net/$dev/flags"
  details="$(/usr/sbin/ip -details link show dev "$dev")"
  if (( (flags & 1) != 0 )); then
    # Do not interrupt an already running bus to change an incorrect bitrate.
    [[ "$details" =~ bitrate[[:space:]]1000000([[:space:]]|$) ]] || {
      echo "$dev is up at an unexpected bitrate" >&2
      return 1
    }
  else
    /usr/sbin/ip link set dev "$dev" type can bitrate 1000000
    /usr/sbin/ip link set dev "$dev" up
    details="$(/usr/sbin/ip -details link show dev "$dev")"
  fi
  [[ "$details" =~ mtu[[:space:]]16([[:space:]]|$) ]] &&
  [[ "$details" =~ can[[:space:]]state[[:space:]]ERROR-ACTIVE ]] &&
  [[ "$details" =~ bitrate[[:space:]]1000000([[:space:]]|$) ]] || {
    echo "$dev is not healthy classical CAN at 1 Mbps" >&2
    return 1
  }
}

configure_link can0
configure_link can1
