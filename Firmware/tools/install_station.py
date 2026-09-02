#!/usr/bin/env python3
"""Stage, render and VERIFY a station install (P11 / §52), without touching motors.

The systemd units in `Firmware/systemd/` are templates: their comment says
"CHANGE before deploy: User/Group and the /opt/open_auto_turret install root",
and the README then lists hand-typed `cp`/`sed`/`systemctl` steps. Hand-typed
deployment on a station whose failure modes are SILENT is how P11 loses an
afternoon:

  * visiond's `--socket` and controld's vision socket disagree -> telemetry just
    says `vision_connected=false` forever, with a healthy-looking daemon;
  * `OTA_VIDEO_ORIENTATION` on web != `--orientation` on vision -> the operator
    sees an upside-down preview while the tracker aims correctly (or the
    reverse), and the two disagree about which one is wrong;
  * the web service leaves its video stream on -> webd holds /dev/video1 and
    visiond gets nothing to look at;
  * `TimeoutStartSec` finite -> systemd SIGTERMs the daemon mid-homing, and
    because a timeout stop is not a "failure", `Restart=on-failure` never brings
    it back: `systemctl status` looks deployed, the station sits unhomed.

This tool stages the runtime tree, renders the units from the templates with the
install root and user/group substituted, and then CHECKS the result — including
the four agreements above. It never starts, stops, or enables a service, and it
never copies anything without `--apply`, because `systemctl start
turret-control` homes the turret: that is the operator's call, in the supervised
window, with the run sheet open.

    python3 -m tools.install_station check  --root .            # pre-deploy lint
    python3 -m tools.install_station stage  --stage /opt/open_auto_turret --dry-run
    python3 -m tools.install_station stage  --stage /opt/open_auto_turret --apply
    sudo tools/install_station check --root /opt/open_auto_turret --installed-units /etc/systemd/system

`check` is read-only and safe at any time, including mid-run.
"""
from __future__ import annotations

import argparse
import grp
import os
import pwd
import re
import shutil
import stat
import subprocess
import sys

FIRMWARE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# What the unit templates claim as the install root. `check` verifies the tree at
# this path, because the honest question is "does everything the units reference
# exist", not "does this development checkout exist".
DEFAULT_ROOT = "/opt/open_auto_turret"

# What the running daemons actually need. Chosen deliberately narrow: a station
# install that carries the test suite and the venv is a station install that
# gets "fixed" by editing files under /opt.
RUNTIME_DIRS = ("vision", "web", "common", "config", "calibration", "tools")
RUNTIME_FILES = ("build/control/controld",)
UNITS = ("can0.service", "turret-control.service", "turret-vision.service",
         "turret-web.service")

PASS, FAIL, WARN, INFO = "PASS", "FAIL", "WARN", "INFO"


class Report:
    def __init__(self) -> None:
        self.rows: list[tuple[str, str, str]] = []

    def add(self, level: str, what: str, detail: str = "") -> None:
        self.rows.append((level, what, detail))

    @property
    def failed(self) -> bool:
        return any(lvl == FAIL for lvl, _, _ in self.rows)

    def print(self) -> None:
        for lvl, what, detail in self.rows:
            print(f"  [{lvl}] {what}" + (f" — {detail}" if detail else ""))


# --------------------------------------------------------------- unit rendering

def render_unit(text: str, root: str, user: str, group: str) -> str:
    """Substitute the install root and the run-as identity into a template.

    Only the four known keys are touched. Anything else in the unit is left
    exactly as reviewed: silently rewriting an `After=` or a sandbox directive is
    how a deploy tool ends up owning someone's outage.
    """
    text = re.sub(r"^(WorkingDirectory=).*$",
                  rf"\g<1>{root.rstrip('/')}", text, flags=re.M)
    # Capture the WHOLE line, not just the key: the first version of this
    # captured `(ExecStart)` and returned only the key, which silently DELETED
    # the command from every rendered unit. A rendered unit that starts nothing
    # is the worst kind of deploy artifact, so exec_command is now also checked
    # for emptiness below.
    text = re.sub(r"^(ExecStart(?:Pre)?=.*)$",
                  lambda m: m.group(1).replace("/opt/open_auto_turret",
                                              root.rstrip("/")),
                  text, flags=re.M)
    text = re.sub(r"^User=.*$", rf"User={user}", text, flags=re.M)
    text = re.sub(r"^Group=.*$", rf"Group={group}", text, flags=re.M)
    text = re.sub(r"^(ExecStartPre=/usr/bin/chown ).*$",
                  rf"\g<1>{user}:{group} /run/ota", text, flags=re.M)
    return text


def exec_command(text: str) -> str:
    """The real ExecStart, continuations joined, comments excluded.

    Two traps, both hit while writing this: searching the whole unit text for
    `--socket` matches the COMMENT that explains the socket flag (a false
    "vision socket disagrees" alarm on a correct unit), and reading only the
    `ExecStart=` line misses `\\`-continued flags like --orientation, which is
    where the orientation actually lives in turret-vision.service.
    """
    out: list[str] = []
    for line in text.splitlines():
        stripped = line.strip()
        if not stripped.startswith("ExecStart="):
            continue
        chunk = stripped[len("ExecStart="):]
        while True:
            out.append(chunk.rstrip("\\").strip())
            break
    # Join any physical continuation lines that follow the ExecStart=.
    lines = text.splitlines()
    for i, line in enumerate(lines):
        if line.strip().startswith("ExecStart="):
            joined = [line.strip()[len("ExecStart="):]]
            j = i
            while joined[-1].endswith("\\") and j + 1 < len(lines):
                j += 1
                joined.append(lines[j].strip())
            return " ".join(x.rstrip("\\").strip() for x in joined)
    return " ".join(out)


def unit_field(text: str, key: str) -> str:
    m = re.search(rf"^{key}=(.*)$", text, flags=re.M)
    return m.group(1).strip() if m else ""


def unit_env(text: str, name: str) -> str:
    vals = re.findall(rf"^Environment={name}=(.*)$", text, flags=re.M)
    return vals[-1].strip() if vals else ""


# ------------------------------------------------------------------- the checks

def check_layout(root: str, rep: Report) -> dict:
    """Files the daemons need must exist at the path the units claim."""
    controld = os.path.join(root, "build", "control", "controld")
    if os.path.isfile(controld) and os.access(controld, os.X_OK):
        rep.add(PASS, "controld present and executable", controld)
    else:
        rep.add(FAIL, "controld missing or not executable", controld)
    cfg = os.path.join(root, "config", "turret.yaml")
    if os.path.isfile(cfg):
        rep.add(PASS, "config/turret.yaml present", cfg)
    else:
        rep.add(FAIL, "config/turret.yaml missing", cfg)
    for d in ("vision", "web"):
        p = os.path.join(root, d)
        rep.add(PASS if os.path.isdir(p) else FAIL, f"{d}/ package installed", p)
    # controld resolves payload.profile_dir (and the calibration files) against
    # the WorkingDirectory, not against the config file's location.
    text = open(cfg, errors="replace").read() if os.path.isfile(cfg) else ""
    m = re.search(r"profile_dir:\s*[\"']?([^\"'\n#]+)", text)
    if m:
        rel = m.group(1).strip()
        target = rel if os.path.isabs(rel) else os.path.join(root, rel)
        rep.add(PASS if os.path.isdir(target) else FAIL,
                "payload.profile_dir resolves from WorkingDirectory",
                f"{target} (relative to {root}, per WorkingDirectory=)")
    else:
        rep.add(INFO, "payload.profile_dir", "not set in turret.yaml (default)")
    return {"config_text": text}


def check_units(root: str, units_dir: str, rep: Report) -> None:
    """The four silent-failure agreements, plus the §52/§33 restart guarantees."""
    texts: dict[str, str] = {}
    for u in UNITS:
        for base in (units_dir, os.path.join(root, "systemd")):
            path = os.path.join(base, u)
            if os.path.isfile(path):
                texts[u] = open(path, errors="replace").read()
                texts[u + "__path"] = path
                break
        if u not in texts:
            # can0 is the MCP2515 HAT bring-up. On THIS station the primary PHY
            # is the yousee USB-CAN adapter (config/turret.yaml can.backend), so
            # a missing can0 unit is not a broken install — claiming FAIL would
            # make the check cry wolf on the exact station it was written for.
            rep.add(INFO if u == "can0.service" else FAIL,
                    f"unit {u} not found",
                    "can0 is the MCP2515 HAT only; the yousee USB-CAN adapter "
                    "needs no unit (turret-can-supervisor would be witness-only)"
                    if u == "can0.service" else
                    f"looked in {units_dir} and {os.path.join(root, 'systemd')}")

    ctrl = texts.get("turret-control.service", "")
    vis = texts.get("turret-vision.service", "")
    web = texts.get("turret-web.service", "")
    if not ctrl:
        return

    # A unit whose ExecStart vanished renders an "active" service that runs
    # nothing (or fails to parse). Cheapest check in this file.
    for u, text in (("turret-control.service", ctrl),
                    ("turret-vision.service", vis),
                    ("turret-web.service", web)):
        if text and not exec_command(text):
            rep.add(FAIL, f"{u}: ExecStart is empty",
                    "the unit would activate and run nothing")
        elif text:
            rep.add(PASS, f"{u}: ExecStart", exec_command(text)[:96])

    # §52: homing must not be cut short, and a stop must park, not abort.
    tss = unit_field(ctrl, "TimeoutStartSec")
    rep.add(PASS if tss == "infinity" else FAIL,
            "control TimeoutStartSec lets homing finish",
            f"TimeoutStartSec={tss or '(unset: systemd default 90 s)'}; homing "
            "measured ~2.5 min, and a timeout stop is not a 'failure' so "
            "Restart=on-failure would NOT bring it back")
    rep.add(PASS if unit_field(ctrl, "Restart") == "on-failure" else WARN,
            "control restarts on failure",
            f"Restart={unit_field(ctrl, 'Restart') or '(none)'} — safe only "
            "because controld always boots UNHOMED (§52)")
    rep.add(PASS if unit_field(ctrl, "KillSignal") == "SIGINT" else FAIL,
            "stop maps to the safe-park path",
            f"KillSignal={unit_field(ctrl, 'KillSignal') or '(default TERM)'}")

    vis_cmd, web_cmd = exec_command(vis), exec_command(web)
    # Agreement 1: the vision IPC socket, both sides.
    sock_vision = re.search(r"--socket\s+(\S+)", vis_cmd)
    ctrl_vision_sock = unit_env(ctrl, "OTA_VISION_SOCKET") or "/tmp/ota_vision.sock"
    if sock_vision:
        sv = sock_vision.group(1)
        rep.add(PASS if sv == ctrl_vision_sock else FAIL,
                "vision socket agrees on both sides",
                f"visiond --socket {sv} vs controld {ctrl_vision_sock}"
                + ("" if sv == ctrl_vision_sock else
                   "  <- SILENT failure: telemetry reports "
                   "vision_connected=false and the daemon looks healthy"))
    else:
        rep.add(WARN, "visiond --socket not explicit",
                f"it will use its own default; controld uses "
                f"{ctrl_vision_sock}")

    # Agreement 2: the web socket.
    cw = unit_env(ctrl, "OTA_WEB_SOCKET") or "/run/ota/controld-web.sock"
    ww = unit_env(web, "OTA_WEB_SOCKET") or "/run/ota/controld-web.sock"
    rep.add(PASS if cw == ww else FAIL, "web socket agrees on both sides",
            f"control {cw} vs web {ww}")

    # Agreement 3: install orientation, both image consumers.
    ov = re.search(r"--orientation\s+(\S+)", vis_cmd)
    ow = unit_env(web, "OTA_VIDEO_ORIENTATION")
    if ov:
        same = (ov.group(1) == (ow or "none"))
        rep.add(PASS if same else WARN, "orientation agrees (IMX500 is flipped)",
                f"vision --orientation {ov.group(1)} vs web "
                f"OTA_VIDEO_ORIENTATION {ow or '(unset: none)'}")
    # The exclusive-camera question, stated as the code actually behaves
    # (measured in the P12 rehearsal: /api/video/state says running=False until
    # something starts it, and the camera is opened only by that call).
    video_on = (unit_env(web, "OTA_VIDEO_ENABLE") or "1").lower() not in (
        "0", "false", "off")
    rep.add(INFO if video_on else PASS,
            "webd preview availability flag",
            ("OTA_VIDEO_ENABLE on: webd does NOT open /dev/video1 at startup — "
             "it opens it when the dashboard switch turns the preview on "
             "(measured: running=False before /api/video/start). The device is "
             "exclusive, so previewing while visiond runs fails with a visible "
             "error in the panel rather than stealing frames (§42.3). Set "
             "OTA_VIDEO_ENABLE=0 if visiond must own the camera unconditionally."
             ) if video_on else
            "preview disabled: visiond always owns the camera")

    # Identity + the witness-only supervisor trap.
    for u, text in ((("turret-control.service", ctrl),
                     ("turret-vision.service", vis),
                     ("turret-web.service", web))):
        if not text:
            continue
        user = unit_field(text, "User")
        try:
            pwd.getpwnam(user)
            rep.add(PASS, f"{u}: User={user} exists")
        except KeyError:
            rep.add(FAIL, f"{u}: User={user} does not exist",
                    "systemd fails the unit at start; fix the template or the "
                    "station user")
        if u == "turret-vision.service":
            groups = unit_field(text, "SupplementaryGroups")
            rep.add(PASS if "video" in groups else FAIL,
                    "vision can open the camera device",
                    f"SupplementaryGroups={groups or '(none)'} — needs video")

    # Interpreter: the units must use the SYSTEM python (no picamera2 in a venv).
    for u, text in (("turret-vision.service", vis), ("turret-web.service", web)):
        ex = exec_command(text)
        if ex.startswith("/usr/bin/python3"):
            rep.add(PASS, f"{u}: runs the system python")
        elif "python" in ex:
            rep.add(FAIL, f"{u}: wrong interpreter",
                    f"ExecStart={ex} — picamera2/uvicorn live in "
                    "/usr/bin/python3, not in .venv")

    # WorkingDirectory is what relative config paths resolve against.
    for u, text in (("turret-control.service", ctrl),
                    ("turret-vision.service", vis),
                    ("turret-web.service", web)):
        wd = unit_field(text, "WorkingDirectory")
        if wd and wd.rstrip("/") != root.rstrip("/"):
            rep.add(FAIL, f"{u}: WorkingDirectory is not the install root",
                    f"{wd} vs {root} — relative paths (config, calibration, "
                    "payload profiles) resolve against the wrong tree")
        elif wd:
            rep.add(PASS, f"{u}: WorkingDirectory={wd}")


def verify_python(root: str, rep: Report) -> None:
    """Import the real modules with the interpreter the units use."""
    for label, code in (("picamera2 (visiond)", "import picamera2"),
                        ("uvicorn+fastapi (webd)", "import uvicorn, fastapi"),
                        ("numpy+cv2 (visiond)", "import numpy, cv2")):
        try:
            r = subprocess.run(["/usr/bin/python3", "-c", code],
                               capture_output=True, text=True, timeout=30)
            if r.returncode == 0:
                rep.add(PASS, f"system python imports {label}")
            else:
                rep.add(FAIL, f"system python cannot import {label}",
                        r.stderr.strip().splitlines()[-1][:120])
        except (OSError, subprocess.TimeoutExpired) as e:
            rep.add(WARN, f"could not test {label}", str(e)[:80])


# ------------------------------------------------------------------- staging

def stage(source: str, stage_dir: str, user: str, group: str, apply: bool,
          rep: Report) -> None:
    """Copy the runtime tree and render the units into <stage>/systemd."""
    if os.path.realpath(source) == os.path.realpath(stage_dir):
        rep.add(FAIL, "refusing to stage onto the source checkout", source)
        return
    plan: list[tuple[str, str]] = []
    for rel in RUNTIME_FILES:
        src = os.path.join(source, rel)
        if os.path.isfile(src):
            plan.append((src, os.path.join(stage_dir, rel)))
        else:
            rep.add(FAIL, f"nothing to stage: {rel} missing",
                    "build it first: ninja -C build")
    for d in RUNTIME_DIRS:
        src = os.path.join(source, d)
        if not os.path.isdir(src):
            rep.add(WARN, f"optional runtime dir missing: {d}")
            continue
        for dirpath, names, files in os.walk(src):
            names[:] = [n for n in names if n not in
                        ("__pycache__", ".pytest_cache", "tests")]
            for f in files:
                if f.endswith((".pyc", ".pdf")):
                    continue
                a = os.path.join(dirpath, f)
                plan.append((a, os.path.join(stage_dir, os.path.relpath(
                    a, source))))
    tpl = os.path.join(source, "systemd")
    rendered: dict[str, str] = {}
    for u in UNITS:
        path = os.path.join(tpl, u)
        if not os.path.isfile(path):
            rep.add(FAIL, f"unit template missing: {u}", path)
            continue
        rendered[u] = render_unit(open(path).read(), stage_dir, user, group)

    rep.add(INFO, "staging plan", f"{len(plan)} files -> {stage_dir}, "
            f"{len(rendered)} units rendered for root={stage_dir}")
    if not apply:
        rep.add(INFO, "dry run", "nothing written (pass --apply to write); "
                "no systemctl call is ever made — starting turret-control homes "
                "the turret")
        return
    for src, dst in plan:
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        shutil.copy2(src, dst)
    os.makedirs(os.path.join(stage_dir, "systemd"), exist_ok=True)
    for u, text in rendered.items():
        with open(os.path.join(stage_dir, "systemd", u), "w") as f:
            f.write(text)
    if not plan or not rendered:
        rep.add(FAIL, "staged nothing", f"{len(plan)} files, {len(rendered)} "
                "units — an empty install is not an install (check --source)")
    else:
        rep.add(PASS, "staged", f"{len(plan)} files, {len(rendered)} units "
                f"under {stage_dir}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("action", choices=("check", "stage"),
                    help="check = read-only verification; stage = copy + render")
    ap.add_argument("--root", default=DEFAULT_ROOT,
                    help="install tree the units claim (default: "
                         "/opt/open_auto_turret, what the templates say). "
                         "Local pre-deploy lint: stage to a scratch dir first, "
                         "then check --root that dir --units its systemd/")
    ap.add_argument("--units", default="",
                    help="directory holding the INSTALLED units "
                         "(e.g. /etc/systemd/system); default: <root>/systemd")
    ap.add_argument("--source", default=FIRMWARE,
                    help="stage: checkout to copy FROM (default: this repo)")
    ap.add_argument("--stage", default=DEFAULT_ROOT,
                    help="stage: tree to build (default: %(default)s)")
    ap.add_argument("--user", default=None)
    ap.add_argument("--group", default=None)
    ap.add_argument("--apply", action="store_true",
                    help="actually write during stage (default is a dry run)")
    ap.add_argument("--verify-python", action="store_true",
                    help="import picamera2/uvicorn with /usr/bin/python3")
    args = ap.parse_args(argv)

    user = args.user or os.environ.get("USER") or pwd.getpwuid(os.getuid()).pw_name
    group = args.group or grp.getgrgid(pwd.getpwnam(user).pw_gid).gr_name

    rep = Report()
    if args.action == "check":
        rep.add(INFO, "checking install tree", args.root)
        check_layout(args.root, rep)
        check_units(args.root, args.units or os.path.join(args.root, "systemd"),
                    rep)
        if args.verify_python:
            verify_python(args.root, rep)
    else:
        stage(args.source, args.stage, user, group, args.apply, rep)
    rep.print()
    print(f"\n{'FAILURES PRESENT' if rep.failed else 'no failures'} "
          f"({sum(1 for l, _, _ in rep.rows if l == FAIL)} fail, "
          f"{sum(1 for l, _, _ in rep.rows if l == WARN)} warn)")
    return 1 if rep.failed else 0


if __name__ == "__main__":
    sys.exit(main())
