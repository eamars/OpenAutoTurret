"""打印这台机器上几把钟的读数和它们之间的关系（只读，标准库）。

时间戳语义的唯一一手依据：控制路径的时间戳是 CLOCK_MONOTONIC 纳秒
（control/src/common/time.hpp），而单调钟每次开机归零。任何活得比一次开机久的
东西（跳闸文件、归档、black-box）都必须自带"我是哪把钟、哪一次开机、换算到墙上钟
差多少"。跑法：在站上和容器里各跑一次，比较 BOOTTIME-MONOTONIC（=累计 suspend 时间）。
"""
import ctypes
libc = ctypes.CDLL("libc.so.6", use_errno=True)
class TS(ctypes.Structure):
    _fields_ = [("sec", ctypes.c_long), ("nsec", ctypes.c_long)]
IDS = {"MONOTONIC": 1, "REALTIME": 0, "BOOTTIME": 7, "REALTIME_COARSE": 5}
def read(cid):
    t = TS()
    if libc.clock_gettime(cid, ctypes.byref(t)) != 0:
        raise OSError(ctypes.get_errno(), "clock_gettime")
    return t.sec + t.nsec * 1e-9
vals = {}
for k, cid in IDS.items():
    try:
        vals[k] = read(cid)
    except OSError as e:
        vals[k] = "不可用（%s）" % e
for k in IDS:
    print("  %-16s %s" % (k, vals[k]))
if all(isinstance(vals[k], float) for k in ("MONOTONIC", "BOOTTIME", "REALTIME")):
    print("  BOOTTIME - MONOTONIC = %.3f s   ← 这段就是“睡过去又被抹掉”的时间（0 = 从未 suspend）"
          % (vals["BOOTTIME"] - vals["MONOTONIC"]))
    print("  REALTIME - MONOTONIC = %.1f s   ← 单调钟换算成墙上钟的锚点（本机现在）"
          % (vals["REALTIME"] - vals["MONOTONIC"]))
print("  /proc/uptime:", open("/proc/uptime").read().split()[0])
try:
    print("  /sys/power/suspend_stats/active_time:", open("/sys/power/suspend_stats/active_time").read().strip())
except OSError:
    print("  /sys/power/suspend_stats：无（这台 Pi 没有 suspend 统计）")
import subprocess
print("  时间同步：", subprocess.run(["timedatectl", "show", "-p", "NTPSynchronized,TimeUSec"],
      capture_output=True, text=True).stdout.strip().replace("\n", "  ") or "取不到")
print("  boot_id:", open("/proc/sys/kernel/random/boot_id").read().strip())
