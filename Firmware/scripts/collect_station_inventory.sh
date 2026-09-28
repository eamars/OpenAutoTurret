#!/usr/bin/env bash
# Phase 1 of the dual-camera -> Hailo validation: record what this machine actually is.
#
# Written for the architect's rule "use the commands actually available on the machine, and
# check --help rather than inventing HailoRT commands". Every section is therefore guarded, and
# a missing tool prints that it is missing instead of the section disappearing quietly -- an
# inventory with silent holes is how "we validated the hardware" ends up meaning "we assumed
# the hardware".
#
# Read-only. No camera is opened, no motor is touched, nothing is written outside /tmp.
set -u

section() { printf '\n===== %s =====\n' "$1"; }
run() {
  # run <label> <cmd...>  -- prints the label, then the output, or why there is none.
  local label="$1"; shift
  printf -- '-- %s\n' "$label"
  if command -v "$1" >/dev/null 2>&1; then
    "$@" 2>&1 | head -n 14 || printf '   (exit %s)\n' "$?"
  else
    printf '   不存在: %s\n' "$1"
  fi
}

VENV="${STATION_VENV:-$HOME/workspace/OpenAutoTurret/run/station-venv}"
PY="$VENV/bin/python"
[ -x "$PY" ] || PY="$(command -v python3)"

section "board / OS / kernel"
[ -r /sys/firmware/devicetree/base/model ] && { printf 'model: '; tr -d '\0' </sys/firmware/devicetree/base/model; echo; }
grep -E '^(PRETTY_NAME|VERSION_ID)' /etc/os-release
printf 'kernel: '; uname -r
printf 'arch:   '; uname -m

section "throttling / temperature (before any load)"
if command -v vcgencmd >/dev/null 2>&1; then
  printf 'get_throttled: '; vcgencmd get_throttled
  printf 'measure_temp : '; vcgencmd measure_temp
else
  echo '   不存在: vcgencmd'
fi
[ -r /sys/class/thermal/thermal_zone0/temp ] && printf 'thermal_zone0: %s C\n' \
  "$(awk '{printf "%.2f", $1/1000}' /sys/class/thermal/thermal_zone0/temp)"

section "camera stack versions"
run "rpicam-hello --version" rpicam-hello --version
if command -v dpkg >/dev/null 2>&1; then
  printf -- '-- packages\n'
  dpkg -l 2>/dev/null | awk '/libcamera|rpicam|rp-camera|imx500|piprobe/ {printf "   %s %s\n", $2, $3}' | head -10
fi
printf -- '-- Picamera2 / libcamera as the station venv sees them\n'
"$PY" - <<'PYEOF'
try:
    from importlib.metadata import version
    for name in ("picamera2", "libcamera", "numpy", "PIL"):
        try:
            print(f"   {name} == {version(name)}")
        except Exception as exc:
            print(f"   {name}: 未安装 ({type(exc).__name__})")
except Exception as exc:
    print("   importlib.metadata 不可用:", exc)
try:
    from picamera2 import Picamera2
    print("   libcamera reports:", Picamera2.get_libcamera_version()
          if hasattr(Picamera2, "get_libcamera_version") else "(该版本没暴露这个函数)")
except Exception as exc:
    print(f"   picamera2 导入失败: {type(exc).__name__}: {exc}")
PYEOF

section "cameras as the Raspberry Pi stack enumerates them (native modes)"
run "rpicam-hello --list-cameras" rpicam-hello --list-cameras
printf -- '-- /dev/v4l/by-path (identity we key on, not an index)\n'
ls -l /dev/v4l/by-path 2>/dev/null | sed -n '2,14p' | awk '{printf "   %s -> %s\n", $(NF-2), $NF}'
printf -- '-- i2c sensor nodes\n'
for f in /sys/bus/i2c/devices/*/name; do
  n=$(cat "$f" 2>/dev/null); case "$n" in imx*|*sensor*) printf '   %s = %s\n' "${f%/name}" "$n";; esac
done

section "Hailo: what is installed, and what its own help says"
run "hailortcli --help (可用子命令)" bash -c "hailortcli --help 2>&1 | sed -n '1,26p'"
run "hailortcli scan" hailortcli scan
run "hailortcli fw-control identify" hailortcli fw-control identify
printf -- '-- python platform module\n'
"$PY" - <<'PYEOF'
try:
    import hailo_platform
    print("   hailo_platform:", getattr(hailo_platform, "__version__", "无 __version__"),
          hailo_platform.__file__)
    from hailo_platform import Device
    ids = Device.scan()
    print("   Device.scan():", ids if ids else "没有设备")
    if ids:
        with Device(ids[0]) as d:
            info = d.control.identify()
            for k in ("device_architecture", "protocol_version", "device_type"):
                if hasattr(info, "device_architecture") and k == "device_architecture":
                    print("   architecture:", info.device_architecture)
                if hasattr(info, "board_id"):
                    print("   board_id:", info.board_id); break
except Exception as exc:
    print(f"   读不到: {type(exc).__name__}: {exc}")
PYEOF
printf -- '-- Hailo 应用层工具（多源管线是否现成）\n'
for t in hailoapp hailomux hailo_perf hailo_camera_tool; do
  if command -v "$t" >/dev/null 2>&1; then echo "   有: $(command -v $t)"; else echo "   无: $t"; fi
done
ls -d /opt/hailo* /usr/share/hailo* 2>/dev/null | sed 's/^/   /' || true

section "GStreamer + Hailo plugins"
run "gst-inspect-1.0 --version" gst-inspect-1.0 --version
printf -- '-- 已注册的 hailo 相关 element\n'
if command -v gst-inspect-1.0 >/dev/null 2>&1; then
  gst-inspect-1.0 2>/dev/null | grep -iE "hailo|rpicam|libcamsrc|v4l2" | sed 's/^/   /' | head -12
else
  echo "   不存在: gst-inspect-1.0"
fi

section "PCIe link actually in use (read-only; nothing is re-negotiated here)"
HBUS="$(lspci -d 1e60: 2>/dev/null | awk '{print $1; exit}')"
if [ -n "${HBUS:-}" ]; then
  printf 'Hailo PCIe endpoint: %s\n' "$HBUS"
  lspci -vv -s "$HBUS" 2>&1 | grep -E 'LnkCap:|LnkSta:|Width|Speed' | head -6 | sed 's/^/   /'
  lspci -vv -s "$HBUS" 2>&1 | grep -qE 'LnkSta:' || echo "   LnkSta 需要 root 才看得到（本站无 sudo）"
else
  echo "   lspci 里找不到 Hailo (vendor 1e60)；设备可能不在 PCIe 上或 lspci 缺失"
fi

section "memory / cores"
printf 'cores: '; nproc
free -m | awk 'NR<=2{printf "   %s\n", $0}'

printf '\n===== Phase 1 inventory ends =====\n'
