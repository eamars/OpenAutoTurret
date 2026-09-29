#!/usr/bin/env python3
"""Rehearse the launcher's stop-cause attribution against a real stack.

Runs on the station. Every scenario starts the stack (so homing runs), applies
exactly one trigger, and then checks that `$RUN/shutdown.cause` tells the truth
about why the stack came down. No motor command is sent beyond the stack's own
controlled stop, and no process outside this stack is signalled.

The scenarios are the four claims the launcher makes:

  operator  an operator stop leaves a credential          cause=operator_stop
  signal    a bare SIGTERM has no credential              cause=external_signal
  child     a child dying first is blamed by name         cause=child_exit
  archive   the previous round's logs survive a restart   logs-history/<stamp>/

`--selftest` checks the assertion logic against canned cause lines, so this tool
has a runnable acceptance path on a machine with no station (a container).
"""
import argparse
import json
import os
import re
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

APP = Path(__file__).resolve().parents[1]
SCRIPT = APP / "scripts" / "run_application.sh"
RUN = Path(os.environ.get("OTA_RUN_DIR", f"/tmp/ota-stack-{os.getuid()}"))
CAUSE = RUN / "shutdown.cause"
READY: list[str] = []  # one entry per scenario: did that stack actually come up


def bash(*args: str) -> subprocess.CompletedProcess:
    return subprocess.run(["bash", str(SCRIPT), *args], capture_output=True, text=True, timeout=300)


def read(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return ""


def cause_text() -> str:
    return read(CAUSE).strip()


def field(text: str, key: str) -> str:
    match = re.search(rf"\b{key}=(\S+)", text)
    return match.group(1) if match else ""


def child_pid(name: str) -> int:
    """Find this stack's own child by the command line recorded in /proc."""
    info = read(RUN / "stack.info")
    match = re.search(r"^Children: (.+)$", info, re.MULTILINE)
    pids = match.group(1).split() if match else []
    for pid in pids:
        try:
            cmdline = Path(f"/proc/{pid}/cmdline").read_bytes().decode(errors="replace")
        except OSError:
            continue
        if name in cmdline:
            return int(pid)
    raise SystemExit(f"rehearsal: no {name} child among stack children {pids} — the stack is not what I started")


def wait_ready(timeout_s: int) -> str:
    # /api/state, not /api/health: health answers before the controller has a
    # phase, and a rehearsal that cannot prove the stack actually came up would
    # be rehearsing nothing. Readiness is an assertion, reported per scenario.
    port = read(RUN / "web.port").strip()
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if port.isdigit():
            try:
                with urllib.request.urlopen(f"http://127.0.0.1:{port}/api/state", timeout=2) as reply:
                    body = json.loads(reply.read().decode())
                    if body.get("phase") in ("hold", "parked", "idle") and not body.get("fault"):
                        return f"ready phase={body['phase']} mode={body.get('operating_mode')}"
            except Exception:  # noqa: BLE001 - during startup any failure means "not yet"
                pass
        port = read(RUN / "web.port").strip() or port
        time.sleep(1)
    return "NOT READY"


def start(timeout_s: int) -> str:
    result = bash("start")
    if result.returncode != 0:
        raise SystemExit(f"rehearsal: start failed rc={result.returncode}: {result.stdout}{result.stderr}")
    ready = wait_ready(timeout_s)
    READY.append(ready)
    return ready


def archive_count() -> int:
    root = RUN / "logs-history"
    return sum(1 for entry in root.iterdir() if entry.is_dir()) if root.is_dir() else 0


# --- assertion logic, shared with --selftest -------------------------------------------------

def check(scenario: str, cause: str, extra: str = "") -> tuple[bool, str]:
    if scenario == "operator":
        ok = field(cause, "cause") == "operator_stop" and "operator=" in cause
        return ok, "cause=operator_stop with a credential" if ok else f"got: {cause}"
    if scenario == "signal":
        ok = field(cause, "cause") == "external_signal" and field(cause, "signal") == "TERM" \
            and "operator=" not in cause
        return ok, "cause=external_signal signal=TERM, no credential" if ok else f"got: {cause}"
    if scenario == "child":
        ok = field(cause, "cause") == "child_exit" and field(cause, "exited_child") == "visiond" \
            and "signal 9" in cause
        return ok, "cause=child_exit blamed visiond with signal 9" if ok else f"got: {cause}"
    if scenario == "archive":
        ok = "archives=" in extra and int(re.sub(r"\D", "", extra) or 0) >= 2
        return ok, f"previous rounds kept on disk ({extra})" if ok else f"got: {extra}"
    if scenario == "ready":
        want, got = int(extra.split("/")[0]), int(extra.split("/")[1])
        ok = want == got and want > 0
        return ok, f"every scenario's stack came up ({extra})" if ok else f"got: {extra}"
    return False, f"unknown scenario {scenario}"


SELFTEST_CASES = [
    ("operator", "cause=operator_stop utc=x launcher=1 uptime_s=9 operator=\"who=operator pid=5\" ", "", True),
    ("signal", "cause=external_signal utc=x launcher=1 uptime_s=9 signal=TERM", "", True),
    ("child", "cause=child_exit utc=x launcher=1 uptime_s=9 exited_child=visiond exited_pid=7 wait_status=137(signal 9)", "", True),
    ("child", "cause=child_exit utc=x launcher=1 uptime_s=9 exited_child=unknown exited_pid=7 wait_status=137(signal 9)", "", False),
    ("operator", "cause=child_exit utc=x launcher=1 uptime_s=9 exited_child=visiond exited_pid=7 wait_status=137(signal 9)", "", False),
    ("archive", "", "archives=3", True),
    ("archive", "", "archives=1", False),
    ("ready", "", "3/3", True),
    ("ready", "", "2/3", False),
    # A credential outranks the signal label: an operator stop *is* a SIGTERM, and
    # naming the person beats naming the syscall. Pinned here so precedence changes
    # out loud instead of quietly rewriting history.
    ("signal", "cause=operator_stop utc=x launcher=1 uptime_s=9 signal=TERM operator=\"who=operator pid=5\"", "", False),
]


def selftest() -> int:
    failures = 0
    for scenario, cause, extra, expected in SELFTEST_CASES:
        ok, detail = check(scenario, cause, extra)
        verdict = "ok" if ok == expected else "WRONG"
        if ok != expected:
            failures += 1
        print(f"[{verdict}] scenario={scenario} expected={expected} got={ok} :: {detail}")
    print(f"selftest: {len(SELFTEST_CASES) - failures}/{len(SELFTEST_CASES)} assertion paths behave")
    return 1 if failures else 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--selftest", action="store_true", help="check the pass/fail logic without a station")
    parser.add_argument("--ready-timeout", type=int, default=240)
    args = parser.parse_args()
    if args.selftest:
        return selftest()

    results = []
    # Normalise: the scenarios each own one start, so whatever is running now goes
    # down first and every cause line below belongs to this run.
    if read(RUN / "launcher.pid").strip():
        bash("stop")
    print("=== scenario 1: operator stop leaves a credential ===")
    print(start(args.ready_timeout))
    bash("stop")
    results.append(("operator", cause_text(), ""))
    print("  " + cause_text())

    print("=== scenario 2: bare SIGTERM has no credential ===")
    print(start(args.ready_timeout))
    launcher_pid = int(read(RUN / "launcher.pid").split()[0])
    subprocess.run(["kill", "-TERM", str(launcher_pid)], check=True)
    time.sleep(6)
    results.append(("signal", cause_text(), ""))
    print("  " + cause_text())

    print("=== scenario 3: a child dying first is blamed by name ===")
    print(start(args.ready_timeout))
    victim = child_pid("perception.visiond")
    subprocess.run(["kill", "-KILL", str(victim)], check=True)
    time.sleep(8)
    results.append(("child", cause_text(), ""))
    print(f"  killed visiond {victim}")
    print("  " + cause_text())

    print("=== scenario 4: earlier rounds survived every restart ===")
    extra = f"archives={archive_count()}"
    results.append(("archive", "", extra))
    print("  " + extra)

    up = sum(1 for entry in READY if not entry.startswith("NOT READY"))
    results.append(("ready", "", f"{up}/{len(READY)}"))
    for index, entry in enumerate(READY, start=1):
        print(f"  scenario {index}: {entry}")

    print("=== verdict ===")
    failed = 0
    for scenario, cause, extra in results:
        ok, detail = check(scenario, cause, extra)
        failed += 0 if ok else 1
        print(f"[{'PASS' if ok else 'FAIL'}] {scenario}: {detail}")
    print(f"rehearsal: {len(results) - failed}/{len(results)} claims hold; the station is currently stopped")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
