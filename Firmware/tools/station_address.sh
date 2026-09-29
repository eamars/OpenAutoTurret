#!/usr/bin/env bash
# Resolve the station's address. No literal IP in a command, a doc or a script.
#
# Why a script rather than a constant: the station's address changed once this week (.100 → .103)
# and every place that had copied the old one kept confidently diagnosing the wrong box. mDNS is the
# answer on a host that runs an mDNS client, and this container does not — so the fallback is not a
# hardcoded guess, it is **the last address we verified by connecting**, and a candidate we cannot
# ssh to is not an address at all.
#
# Usage:  station_address.sh print                 → the verified address, or a named failure
#         station_address.sh deploy -- [flags]     → runs deploy_station.py against it
#
# Override for one run:  OTA_STATION_ADDRESS=192.168.2.77 station_address.sh print
set -euo pipefail

here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
checkout=$(cd "$here/../.." && pwd)               # .../OpenAutoTurret（这份现在住在 Firmware/tools）
ws=$(cd "$checkout/../.." && pwd)                # the workspace root; secrets and the venv live here
identity=${OTA_SSH_IDENTITY:-$ws/.secrets/ssh/id_ed25519}
pinned=${OTA_KNOWN_HOSTS:-$ws/.secrets/ssh/known_hosts_station}
cache=${OTA_STATION_ADDRESS_CACHE:-$ws/.secrets/station_address}
user=${OTA_STATION_USER:-eamars}
alias_name=${OTA_STATION_ALIAS:-rpi-turret}      # the name the pinned key is recorded under

die() { printf 'station-address: %s: %s\n' "$1" "$2" >&2; exit 1; }

# Every identity check goes through the alias, so an address change is a non-event: the pinned
# fingerprint is looked up under `rpi-turret` no matter which number the box answers on today.
verify() {
  local addr=$1
  [[ -n "$addr" ]] || return 1
  local out
  out=$(ssh -o ConnectTimeout=5 -o BatchMode=yes \
      -o "HostName=$addr" -o "HostKeyAlias=$alias_name" \
      -i "$identity" -o IdentitiesOnly=yes \
      -o "UserKnownHostsFile=$pinned" -o StrictHostKeyChecking=yes \
      -o GlobalKnownHostsFile=/dev/null \
      "$user@$alias_name" 'echo ok' 2>&1) && return 0
  printf '  %s 被拒：%s\n' "$addr" "${out##*$'\n'}" >&2   # ssh's own last line, not my guess
  return 1
}

candidates=()
[[ -n "${OTA_STATION_ADDRESS:-}" ]] && candidates+=("$OTA_STATION_ADDRESS")
if command -v avahi-resolve >/dev/null 2>&1; then
  resolved=$(avahi-resolve -4 -n "$alias_name.local" 2>/dev/null | awk '{print $2}' || true)
  [[ -n "$resolved" ]] && candidates+=("$resolved")
fi
resolved=$(getent hosts "$alias_name.local" 2>/dev/null | awk '{print $1; exit}' || true)
[[ -n "$resolved" ]] && candidates+=("$resolved")
[[ -f "$cache" ]] && candidates+=("$(tr -d '[:space:]' < "$cache")")

(( ${#candidates[@]} )) || die "no candidate" \
  "mDNS is not available here and no cache exists. Either start an mDNS client, point
  OTA_STATION_ADDRESS at the box, or write the address once into $cache. Do not paste a literal IP
  into a command and call it a fix — that is the thing this script exists to prevent."

for addr in "${candidates[@]}"; do
  if verify "$addr"; then
    printf '%s\n' "$addr" > "$cache"       # the verified answer replaces the stale one
    case "${1:-print}" in
      print) echo "$addr" ;;
      deploy)
        python_bin=${OTA_PYTHON:-$ws/.venv/bin/python}
        [[ -x "$python_bin" ]] || die "no interpreter" "set OTA_PYTHON; nothing is installed globally"
        shift
        [[ "${1:-}" == "--" ]] && shift      # the separator is for us, not for the deployer
        exec "$python_bin" "$here/deploy_station.py" \
          --host "$user@$alias_name" --connect-address "$addr" \
          --identity "$identity" --known-hosts "$pinned" "$@"
        ;;
      *) die "unknown subcommand" "use print or deploy -- [deploy_station.py flags]" ;;
    esac
    exit 0
  fi
done

die "every candidate refused the pinned connection" \
  "tried: ${candidates[*]}. A changed fingerprint is a finding to investigate, not a line to edit —
  check the pinned fingerprint against the box (ssh-keygen -lf $pinned) before believing anything
  else this tool tells you."
