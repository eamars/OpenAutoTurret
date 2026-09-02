#!/usr/bin/env python3
"""OTA CAN supervisor — unattended recovery watchdog for can0 (MCP2515 HAT).

Implements the recovery ladder from the CAN-lockup debug procedure:
    L0 SocketCAN auto-restart re-arm (restart-ms)
    L1 Explicit SocketCAN restart          (BUS-OFF only)
    L2 Interface down/up with bitrate
    L3 mcp251x SPI driver unbind/rebind    (hardware reset via driver re-probe)
    L4 MCP2515 physical RESET GPIO         (config-gated; not wired on the
                                            stock Waveshare HAT — skip unless
                                            configured)
    L5 Raspberry Pi reboot                 (config-gated; default OFF — a Pi
                                            reboot does not power-cycle the HAT)

Design rules (from the procedure):
  * Every system call is bounded by an external `timeout`.
  * The supervisor never blocks forever; one loop iteration is bounded.
  * Escalate one level at a time; verify the interface is genuinely healthy
    (state ERROR-ACTIVE, error counters flat) after each step.
  * Rate-limit recovery rounds so a persistent fault latches instead of
    reset-looping.
  * Persist counters + last-failure/last-recovery to a state file for the
    daemon/web/operator.
  * On fault, snapshot diagnostics BEFORE touching the interface.

Failure classification (the 2026-09-02 P6 incident):
  If can0 cannot hold ERROR-ACTIVE on an idle bus after L2 and L3 (fresh
  driver re-probe), the controller cannot be software-recovered — classify as
  a hardware/controller fault (e.g. MCP2515 CAN engine dead; see
  docs/can_hardware_fault_report.md) and require operator action (power-cycle
  the drives, replace/repair the HAT). Do NOT reboot-loop.

Python 3 stdlib only (no third-party deps -> no venv required).
"""

import argparse
import json
import os
import re
import shutil
import subprocess
import sys
import time
from datetime import datetime, timezone

# ---------------------------------------------------------------------------
# Constants / defaults
# ---------------------------------------------------------------------------
IFACE = "can0"
BITRATE = 1000000
RESTART_MS = 100
DRIVER = "mcp251x"          # SPI driver owning the HAT controller
POLL_S = 2.0                # bus-state poll interval
ACTIVE_SAMPLES = 2          # consecutive ERROR-ACTIVE polls required to verify
CMD_TIMEOUT_S = 5           # per-command timeout (external `timeout`)
DIAG_DIR = "/tmp/ota_can_diag"
LOG_PATH = "/tmp/ota_can_supervisor.log"
STATE_PATH = "/tmp/ota_can_supervisor.json"
MAX_ROUNDS = 3              # recovery rounds before latching
ROUND_WINDOW_S = 600        # latch window (s)
SNAPSHOT_MAX = 8            # keep at most this many diagnostic snapshots

STATE_RE = re.compile(r"can state (\S+)")


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


# ---------------------------------------------------------------------------
# Bounded subprocess helpers
# ---------------------------------------------------------------------------
def run(cmd, timeout=CMD_TIMEOUT_S, capture=True):
    """Run a command under `timeout`; never raise on non-zero exit."""
    full = ["timeout", str(timeout)] + cmd
    try:
        p = subprocess.run(
            full,
            stdout=subprocess.PIPE if capture else None,
            stderr=subprocess.PIPE if capture else None,
            text=True,
        )
        return p.returncode, (p.stdout or ""), (p.stderr or "")
    except FileNotFoundError as e:
        return 127, "", f"command not found: {e}"
    except Exception as e:  # noqa: BLE001 - supervisor must not die
        return 126, "", str(e)


def sh(cmd):
    """Shlex-quoted run, returns (rc, combined output)."""
    rc, out, err = run(cmd)
    return rc, (out + err).strip()


# ---------------------------------------------------------------------------
# Bus state queries (all read-only)
# ---------------------------------------------------------------------------
def query_bus(iface=IFACE):
    """Return dict of can0 state: exists, up, state, counters, bitrate.

    Never raises. All fields best-effort.
    """
    info = {
        "exists": False, "up": False, "state": None, "bitrate": 0,
        "rx_packets": 0, "tx_packets": 0, "rx_errors": 0, "tx_errors": 0,
        "bus_errors": 0, "bus_off": 0, "error_pass": 0, "error_warn": 0,
        "restart_ms": 0,
    }
    rc, out = sh(["ip", "-details", "-statistics", "link", "show", iface])
    if rc != 0:
        return info  # interface missing / ip error
    m = STATE_RE.search(out)
    if m:
        info["state"] = m.group(1)
    info["exists"] = True
    info["up"] = "state UP" in out
    m = re.search(r"bitrate (\d+)", out)
    if m:
        info["bitrate"] = int(m.group(1))
    m = re.search(r"restart-ms (\d+)", out)
    if m:
        info["restart_ms"] = int(m.group(1))
    # Event counters line: "re-started bus-errors arbit-lost error-warn error-pass bus-off"
    m = re.search(
        r"re-started\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)", out)
    if m:
        info["bus_errors"] = int(m.group(2))
        info["error_warn"] = int(m.group(4))
        info["error_pass"] = int(m.group(5))
        info["bus_off"] = int(m.group(6))
    m = re.search(r"RX:\s+bytes\s+packets\s+errors", out)
    if m:
        # the counter row follows the RX: header
        m2 = re.search(
            r"RX:\s+bytes\s+packets\s+errors\s+dropped\s+missed\s+mcast\s*\n"
            r"\s+(\d+)\s+(\d+)\s+(\d+)", out)
        if m2:
            info["rx_packets"] = int(m2.group(2))
            info["rx_errors"] = int(m2.group(3))
        m3 = re.search(
            r"TX:\s+bytes\s+packets\s+errors\s+dropped\s+carrier\s+collsns\s*\n"
            r"\s+(\d+)\s+(\d+)\s+(\d+)", out)
        if m3:
            info["tx_packets"] = int(m3.group(2))
            info["tx_errors"] = int(m3.group(3))
    return info


def healthy(info):
    """A genuinely healthy interface for verification purposes."""
    return info["exists"] and info["state"] == "ERROR-ACTIVE" and info["up"]


def ack_probe(iface=IFACE):
    """Send 2 probe frames and read the netdev counters.

    Returns (acked: bool, detail: str). A healthy bus with at least one
    other node ACKs the frames (tx_packets rises, tx_errors flat). A lone
    node (drives off / fault-latched) raises tx_errors (no ACK). A broken
    controller TX path completes neither.
    """
    before = query_bus(iface)
    rc, out = sh(["cangen", iface, "-I", "0x777", "-L", "8",
                  "-D", "0x00000000", "-n", "2", "-g", "20"])
    time.sleep(0.3)
    after = query_bus(iface)
    tx_ok = after["tx_packets"] - before["tx_packets"] >= 2
    tx_err = after["tx_errors"] - before["tx_errors"] >= 1
    if tx_ok and not tx_err:
        return True, "probe frame ACKed — at least one other CAN node on the bus"
    if tx_err:
        return False, ("probe frame got NO ACK — no other CAN node responding "
                       "(drives off / fault-latched / not wired)")
    return False, "probe TX did not complete — controller TX path failing"


# ---------------------------------------------------------------------------
# Diagnostics snapshot (taken BEFORE any recovery action)
# ---------------------------------------------------------------------------
def snapshot(iface=IFACE, reason=""):
    """Capture the pre-recovery state to DIAG_DIR; return the file path."""
    os.makedirs(DIAG_DIR, exist_ok=True)
    ts = time.strftime("%Y%m%d-%H%M%S")
    path = os.path.join(DIAG_DIR, f"diag_{ts}.txt")
    lines = [f"=== OTA CAN diagnostic snapshot {now_iso()} ===",
             f"reason: {reason}", ""]
    lines.append("-- uname --")
    rc, out = sh(["uname", "-a"])
    lines.append(out)
    lines.append("-- ip -details -statistics link show " + iface + " --")
    rc, out = sh(["ip", "-details", "-statistics", "link", "show", iface])
    lines.append(out)
    lines.append("-- /proc/interrupts (spi/mcp/can) --")
    try:
        with open("/proc/interrupts") as f:
            for ln in f:
                if re.search(r"spi|mcp|can|gpio", ln, re.I):
                    lines.append(ln.rstrip())
    except OSError as e:
        lines.append(f"(unreadable: {e})")
    lines.append("-- dmesg (can/spi/irq/voltage) --")
    rc, out = sh(["dmesg", "-T"])
    for ln in out.splitlines():
        if re.search(r"mcp251|spi|can[0-9]|under-voltage|voltage", ln, re.I):
            lines.append(ln)
    lines.append("-- ps (daemons) --")
    rc, out = sh(["ps", "-eo", "pid,stat,wchan:30,comm"])
    for ln in out.splitlines():
        if re.search(r"controld|visiond|turret", ln, re.I):
            lines.append(ln)
    lines.append("")
    try:
        with open(path, "w") as f:
            f.write("\n".join(lines))
    except OSError as e:
        return f"(snapshot failed: {e})"
    # keep the newest SNAPSHOT_MAX
    snaps = sorted(f for f in os.listdir(DIAG_DIR) if f.startswith("diag_"))
    for old in snaps[:-SNAPSHOT_MAX]:
        try:
            os.remove(os.path.join(DIAG_DIR, old))
        except OSError:
            pass
    return path


# ---------------------------------------------------------------------------
# Recovery ladder (each step bounded; returns True if verified healthy)
# ---------------------------------------------------------------------------
class Recovery:
    def __init__(self, iface, bitrate, restart_ms, driver, log):
        self.iface = iface
        self.bitrate = bitrate
        self.restart_ms = restart_ms
        self.driver = driver
        self.log = log
        self.last_success = None

    def _log(self, msg):
        self.log(msg)

    def _verify(self, wait_s=ACTIVE_SAMPLES * POLL_S):
        """Verify the interface holds ERROR-ACTIVE with flat error counters."""
        before = query_bus(self.iface)
        if not healthy(before):
            return False, before
        deadline = time.monotonic() + wait_s
        while time.monotonic() < deadline:
            time.sleep(POLL_S)
            after = query_bus(self.iface)
            if not healthy(after):
                return False, after
        if (after["bus_errors"] != before["bus_errors"] or
                after["rx_errors"] != before["rx_errors"] or
                after["tx_errors"] != before["tx_errors"]):
            return False, after
        return True, after

    def level0_rearm(self):
        self._log("L0: re-arm SocketCAN restart-ms")
        rc, out = sh(["ip", "link", "set", self.iface, "type", "can",
                      "restart-ms", str(self.restart_ms)])
        self._log(f"L0 rc={rc} {out}")
        return True  # never the sole fix; always continue

    def level1_restart(self):
        self._log("L1: explicit SocketCAN restart (BUS-OFF only)")
        rc, out = sh(["ip", "link", "set", self.iface, "type", "can", "restart"])
        self._log(f"L1 rc={rc} {out}")
        return True

    def level2_down_up(self):
        self._log("L2: interface down/up")
        rc, out = sh(["ip", "link", "set", self.iface, "down"])
        if rc != 0:
            self._log(f"L2 down rc={rc} {out}")
        rc, out = sh(["ip", "link", "set", self.iface, "up", "type", "can",
                      "bitrate", str(self.bitrate),
                      "restart-ms", str(self.restart_ms)])
        self._log(f"L2 up rc={rc} {out}")
        ok, after = self._verify()
        if ok:
            self.last_success = "L2 down/up"
        return ok

    def level3_rebind(self):
        self._log("L3: mcp251x SPI driver unbind/rebind")
        dev = self.iface
        # resolve the SPI device behind the netdev
        rc, out = sh(["readlink", "-f", f"/sys/class/net/{self.iface}/device"])
        if rc == 0 and out:
            dev = os.path.basename(out.strip())
        self._log(f"L3 spi device: {dev}")
        rc, out = sh(["sh", "-c",
                      f'echo {dev} > /sys/bus/spi/drivers/{self.driver}/unbind'])
        if rc != 0:
            self._log(f"L3 unbind rc={rc} {out} (sysfs may be read-only)")
            return False
        time.sleep(0.3)
        rc, out = sh(["sh", "-c",
                      f'echo {dev} > /sys/bus/spi/drivers/{self.driver}/bind'])
        if rc != 0:
            self._log(f"L3 bind rc={rc} {out}")
            return False
        time.sleep(1.0)
        # interface is recreated DOWN; bring it up at the correct bitrate
        rc, out = sh(["ip", "link", "set", self.iface, "up", "type", "can",
                      "bitrate", str(self.bitrate),
                      "restart-ms", str(self.restart_ms)])
        self._log(f"L3 bring-up rc={rc} {out}")
        ok, after = self._verify()
        if ok:
            self.last_success = "L3 driver rebind"
        return ok

    def level4_gpio_reset(self, gpio=None):
        """Optional physical RESET (open-drain style) if configured."""
        if not gpio:
            self._log("L4: skipped (no RESET GPIO configured)")
            return False
        self._log(f"L4: MCP2515 physical RESET via GPIO {gpio}")
        return False  # not wired on the stock HAT; reserved for a modified HAT

    def level5_reboot(self, enabled):
        if not enabled:
            self._log("L5: skipped (reboot disabled in config — operator "
                      "decision; note a Pi reboot does NOT power-cycle the HAT)")
            return False
        self._log("L5: requesting Raspberry Pi reboot")
        rc, out = sh(["sh", "-c", "systemctl reboot"])
        self._log(f"L5 rc={rc} {out}")
        return True


# ---------------------------------------------------------------------------
# Main supervisor loop
# ---------------------------------------------------------------------------
class Supervisor:
    def __init__(self, args):
        self.args = args
        self.iface = args.iface
        self.bitrate = args.bitrate
        self.restart_ms = args.restart_ms
        self.driver = args.driver
        self.state = {
            "interface": self.iface,
            "bitrate": self.bitrate,
            "restart_ms": self.restart_ms,
            "berr_reporting_supported": None,
            "counters": {
                "bus_off_events": 0,
                "error_passive_events": 0,
                "interface_restarts": 0,
                "driver_rebinds": 0,
                "hardware_resets": 0,
                "pi_reboots": 0,
                "unrecovered_failures": 0,
                "recovery_rounds": 0,
            },
            "last_failure": None,
            "last_failure_time": None,
            "last_recovery_level": None,
            "last_recovery_time": None,
            "verdict": "monitoring",
            "last_check": None,
            "snapshot": None,
        }
        self.round_times = []  # recent recovery-round start times
        self.cooldown_until = 0.0  # after an unrecovered/latched fault

    # -- logging / state persistence --------------------------------------
    def log(self, msg):
        line = f"{now_iso()} {msg}"
        print(line, flush=True)
        try:
            with open(LOG_PATH, "a") as f:
                f.write(line + "\n")
        except OSError:
            pass

    def save_state(self):
        self.state["last_check"] = now_iso()
        try:
            tmp = STATE_PATH + ".tmp"
            with open(tmp, "w") as f:
                json.dump(self.state, f, indent=2, sort_keys=True)
            os.replace(tmp, STATE_PATH)
        except OSError as e:
            self.log(f"state write failed: {e}")

    def rate_limited(self):
        now = time.monotonic()
        self.round_times = [t for t in self.round_times
                            if now - t < ROUND_WINDOW_S]
        return len(self.round_times) >= MAX_ROUNDS

    # -- fault handling ----------------------------------------------------
    def handle_fault(self, info, daemon_ok):
        reason = []
        if info["state"] == "BUS-OFF":
            reason.append("BUS-OFF")
            self.state["counters"]["bus_off_events"] += 1
        if info["state"] == "ERROR-PASSIVE":
            reason.append("ERROR-PASSIVE")
            self.state["counters"]["error_passive_events"] += 1
        if not info["exists"]:
            reason.append("interface missing")
        if not info["up"]:
            reason.append("interface DOWN")
        reason.append(f"daemon_ok={daemon_ok}")
        reason_str = ", ".join(reason)
        self.log(f"FAULT: {reason_str}")

        if self.rate_limited():
            self.log(f"RATE-LIMIT: {MAX_ROUNDS} rounds per "
                     f"{ROUND_WINDOW_S // 60} min exhausted — latching.")
            self.state["verdict"] = "latched-rate-limit"
            self.state["last_failure"] = reason_str
            self.state["last_failure_time"] = now_iso()
            self.cooldown_until = time.monotonic() + ROUND_WINDOW_S
            self.save_state()
            return

        # snapshot BEFORE touching anything
        snap = snapshot(self.iface, reason=reason_str)
        self.state["snapshot"] = snap
        self.log(f"diagnostics -> {snap}")

        self.round_times.append(time.monotonic())
        self.state["counters"]["recovery_rounds"] += 1

        rec = Recovery(self.iface, self.bitrate, self.restart_ms,
                       self.driver, self.log)

        # L0/L1 first (cheap, safe)
        rec.level0_rearm()
        rec.level1_restart()
        ok, after = rec._verify(wait_s=2 * POLL_S)
        if ok:
            self.state["last_recovery_level"] = "L0/L1 SocketCAN restart"
            self.state["verdict"] = "recovered"
            self._report_probe()
            self.log("RECOVERED via L0/L1")
            return
        self.log(f"not recovered (state={after['state']}) — escalating")

        # L2 down/up
        if rec.level2_down_up():
            self.state["counters"]["interface_restarts"] += 1
            self.state["last_recovery_level"] = "L2 down/up"
            self.state["verdict"] = "recovered"
            self._report_probe()
            self.log("RECOVERED via L2")
            return
        self.state["counters"]["interface_restarts"] += 1  # attempted
        self.log("L2 failed to hold ERROR-ACTIVE — escalating")

        # L3 driver rebind (requires writable /sys; root)
        if rec.level3_rebind():
            self.state["counters"]["driver_rebinds"] += 1
            self.state["last_recovery_level"] = "L3 driver rebind"
            self.state["verdict"] = "recovered"
            self._report_probe()
            self.log("RECOVERED via L3")
            return
        self.state["counters"]["driver_rebinds"] += 1  # attempted
        self.log("L3 failed to hold ERROR-ACTIVE — controller cannot be "
                 "software-recovered")

        # L4 (config-gated physical RESET) and L5 (reboot, default off)
        if rec.level4_gpio_reset(getattr(self.args, "reset_gpio", None)):
            self.state["counters"]["hardware_resets"] += 1
            self.state["last_recovery_level"] = "L4 GPIO RESET"
            self.state["verdict"] = "recovered"
            self._report_probe()
            return
        if rec.level5_reboot(self.args.allow_reboot):
            self.state["counters"]["pi_reboots"] += 1

        # Unrecovered: classify.
        self.state["counters"]["unrecovered_failures"] += 1
        self.state["last_failure"] = reason_str
        self.state["last_failure_time"] = now_iso()
        acked, probe_detail = (None, "probe disabled")
        if self.args.probe:
            acked, probe_detail = ack_probe(self.iface)
            self.log(f"ACK probe: {probe_detail}")
        if info["exists"]:
            if acked is False:
                self.state["verdict"] = (
                    "interface cannot hold ERROR-ACTIVE and probe got NO ACK — "
                    "consistent with an external CAN fault: no other node "
                    "responding (drives powered off / fault-latched) and/or "
                    "an external receive-error source on CANH/CANL. Root "
                    "cause unresolved (see docs/can_hardware_fault_report.md "
                    "v2; run the physical isolation ladder). Operator action: "
                    "run Test A (HAT disconnected, 5 min), then power-cycle "
                    "the drives.")
            else:
                self.state["verdict"] = (
                    "can0 cannot hold ERROR-ACTIVE even after driver re-probe; "
                    "not software-recoverable. Likely external CAN fault "
                    "(receive-error source on the wire or HAT RX path; root "
                    "cause unresolved — see docs/can_hardware_fault_report.md "
                    "v2, isolation ladder). Operator action: Test A (HAT "
                    "disconnected), then power-cycle the drives.")
        else:
            self.state["verdict"] = (
                "interface missing — driver failed to create can0; check "
                "dtoverlay + dmesg.")
        self.cooldown_until = time.monotonic() + ROUND_WINDOW_S
        self.log("UNRECOVERED: " + self.state["verdict"])
        self.save_state()

    def _report_probe(self):
        """Log the ACK probe result after a successful interface recovery."""
        if not self.args.probe:
            return
        acked, detail = ack_probe(self.iface)
        self.log(f"ACK probe: {detail}")
        if not acked:
            self.log("NOTE: interface is up but no other CAN node ACKs — "
                     "check drive power/discovery before re-running the queue.")

    # -- main loop ---------------------------------------------------------
    def run(self):
        self.log(f"OTA CAN supervisor starting: {self.iface} @ "
                 f"{self.bitrate} bit/s restart-ms {self.restart_ms} "
                 f"poll {POLL_S}s")
        # Probe berr-reporting support once (finding: NOT supported by this
        # kernel's mcp251x driver). Requires the link DOWN, so only probe when
        # it is safe to do so at startup (interface down).
        info0 = query_bus(self.iface)
        if info0["exists"] and not info0["up"]:
            rc, out = sh(["ip", "link", "set", self.iface, "type", "can",
                          "berr-reporting", "on"])
            self.state["berr_reporting_supported"] = (rc == 0)
            self.log(f"berr-reporting on: "
                     f"{'supported' if rc == 0 else 'NOT supported'} "
                     f"({out[:80]})")
        else:
            self.state["berr_reporting_supported"] = None
            self.log("berr-reporting probe skipped (link up; probe needs the "
                     "link down). Known from testing: this kernel's mcp251x "
                     "does NOT support berr-reporting.")
        self.save_state()

        prev_passive = False
        prev_busoff = False
        while True:
            info = query_bus(self.iface)
            self.state["last_check"] = now_iso()
            self.state["last_bus_state"] = info["state"]

            # daemon health (optional; process-name check, exact match)
            daemon_ok = True
            if self.args.daemon_proc:
                rc, out = sh(["pgrep", "-x", self.args.daemon_proc])
                daemon_ok = (rc == 0)

            fault = (not info["exists"] or not info["up"] or
                     info["state"] in ("ERROR-PASSIVE", "BUS-OFF") or
                     (info["state"] is None and self.args.treat_unknown_fault))

            if fault:
                if time.monotonic() < self.cooldown_until:
                    # latched fault; hold off re-escalating (rate limit)
                    self.save_state()
                    time.sleep(POLL_S)
                    continue
                self.handle_fault(info, daemon_ok)
                prev_passive = False
                prev_busoff = False
            else:
                # clear a one-shot recovery latch once the bus is healthy
                if self.state.get("verdict", "").startswith(("recovered",
                                                             "monitoring")):
                    pass
                if healthy(info):
                    prev_passive = False
                    prev_busoff = False
            self.save_state()
            time.sleep(POLL_S)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--iface", default=IFACE)
    ap.add_argument("--bitrate", type=int, default=BITRATE)
    ap.add_argument("--restart-ms", type=int, default=RESTART_MS)
    ap.add_argument("--driver", default=DRIVER)
    ap.add_argument("--daemon-proc", default="controld",
                    help="pgrep pattern for the control daemon (health check); "
                         "empty disables")
    ap.add_argument("--allow-reboot", action="store_true",
                    help="allow L5 reboot (default: off — operator decision)")
    ap.add_argument("--reset-gpio", type=int, default=None,
                    help="BCM GPIO wired to the MCP2515 RESET (open-drain mod); "
                         "requires hardware modification — see report")
    ap.add_argument("--probe", action="store_true", default=True,
                    help="send a 2-frame ACK probe after recovery / before "
                         "classification (default: on; use --no-probe to "
                         "disable on busy production buses)")
    ap.add_argument("--no-probe", dest="probe", action="store_false",
                    help="disable the ACK probe frames")
    ap.add_argument("--once", action="store_true",
                    help="run one poll cycle and exit (diagnostics)")
    args = ap.parse_args()

    sup = Supervisor(args)
    if args.once:
        info = query_bus(args.iface)
        rc, out = sh(["ip", "link", "set", args.iface, "type", "can",
                      "berr-reporting", "on"])
        berr = (rc == 0)
        print(json.dumps({**info, "berr_reporting_supported": berr},
                         indent=2, sort_keys=True))
        if not info["exists"] or info["state"] in ("ERROR-PASSIVE", "BUS-OFF"):
            print("FAULT PRESENT")
            return 1
        print("OK")
        return 0
    try:
        sup.run()
    except KeyboardInterrupt:
        sup.log("supervisor stopped (SIGINT)")
        return 0


if __name__ == "__main__":
    sys.exit(main())
