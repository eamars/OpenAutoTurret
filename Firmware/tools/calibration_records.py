#!/usr/bin/env python3
"""The station's calibration records (ADR-005): seed #1, import a record, list them.

A record is a payload set in run/state/calibration/records/<id>/: the yaw and pitch servo assets,
the yaw accuracy limits, the camera->pitch extrinsic and the bore. controld reads them at start and
is the only writer of records/ and active.json; this tool writes only an empty store's first record
and records staged in incoming/, which controld adopts at its next start under the next free id.

    calibration_records.py seed   --state DIR --config CONFIG [--root DIR] [--revision REV]
    calibration_records.py import --state DIR --name NAME [--from ID] [--yaw F] [--pitch F]
                                  [--accuracy F] [--extrinsic F] [--bore F]
    calibration_records.py list   --state DIR
    calibration_records.py pull   [--host USER@HOST] [--address IP] [--state DIR] [--to DIR] [ID ...]

There are no hashes (owner, 2026-10-24): the committed files seed #1 once, on an empty store, and a
deploy never compares them with the records again. A record is usable when its files parse.
"""
from __future__ import annotations

import argparse
import datetime
import io
import json
import os
import re
import shlex
import shutil
import subprocess
import sys
import tarfile
import tempfile
from pathlib import Path

import yaml

SCHEMA = "ota.calibration-record/1"
PARTS = {"yaw": "yaw_servo.json", "pitch": "pitch_servo.json", "accuracy": "yaw_accuracy.json",
         "extrinsic": "camera_extrinsics.yaml", "bore": "bore.yaml"}
NAME = re.compile(r"^[A-Z0-9 _-]{1,24}$")
BORE_KEYS = ("right_mm", "up_mm", "forward_mm", "right_deg", "up_deg")
FIRMWARE = Path(__file__).resolve().parents[1]
STATION_STATE = "/home/eamars/workspace/OpenAutoTurret/run/state"


def store_dirs(state: Path) -> tuple:
    root = state / "calibration"
    return root, root / "records", root / "incoming"


def record_ids(records: Path) -> list:
    return sorted(int(p.name) for p in records.iterdir() if p.is_dir() and p.name.isdigit()) \
        if records.is_dir() else []


def now_local() -> str:
    return datetime.datetime.now().astimezone().isoformat(timespec="seconds")


def write_json(path: Path, value) -> None:
    """Temp file, then rename: a reader never sees half a record."""
    fd, tmp = tempfile.mkstemp(dir=path.parent, prefix=path.name + ".")
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        json.dump(value, f, indent=1)
        f.write("\n")
    os.replace(tmp, path)


def asset_provenance(path: Path) -> dict:
    """The date and run a servo or accuracy asset records about itself, when it does."""
    try:
        doc = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}
    prov = doc.get("provenance") or {}
    run = str(prov.get("run") or doc.get("calibration_run") or "")
    stamp = str(prov.get("commissioned") or "")
    if not stamp:                              # the run id ends in its own timestamp
        m = re.search(r"\d{8}-\d{6}$", run)
        stamp = m.group(0) if m else ""
    out = {"run": run} if run else {}
    if re.fullmatch(r"\d{8}(-\d{6})?", stamp):
        out["date"] = f"{stamp[0:4]}-{stamp[4:6]}-{stamp[6:8]}"
    return out


def bore_yaml(alignment: dict) -> str:
    """bore.yaml from turret_mixed.yaml's alignment block: the mode, the zeroing distance (the
    config's assumed depth) and each camera's offsets, with no trims yet (ADR-005 D9)."""
    offsets = alignment.get("camera_offset") or {}
    depth = alignment.get("assumed_depth_m", 10)
    # YAML 1.1 (PyYAML) reads the config's bare `off` as False; controld's reader keeps the word.
    # Normalise it, and quote it here so both readers agree.
    mode = alignment.get("mode", "off")
    mode = "off" if mode in (False, None, "off") else str(mode)
    lines = ["# ADR-005 D9: this record's bore. mode off = the tracker aims the camera centre.",
             "schema: 1",
             f'mode: "{mode}"',
             f"zero_m: {'.inf' if depth in (None, float('inf')) else depth}   # zeroing distance; .inf = no parallax"]
    for cam in ("wide", "detail"):
        o = offsets.get(cam) or {}
        fields = ", ".join(f"{k}: {o.get(k, 0)}" for k in BORE_KEYS)
        lines.append(f"{cam}: {{{fields}, trim_right_mrad: 0, trim_up_mrad: 0}}")
    return "\n".join(lines) + "\n"


def committed_set(config: Path, root: Path) -> dict:
    """Where the release keeps each part, read from the configuration controld runs with."""
    cfg = yaml.safe_load(config.read_text(encoding="utf-8")) or {}
    profile_path = root / cfg["hardware_profile"]
    servo = (yaml.safe_load(profile_path.read_text(encoding="utf-8")) or {}).get("servo") or {}
    yaw = profile_path.parent / servo["yaw_asset"]
    return {"yaw": yaw, "pitch": profile_path.parent / servo["pitch_asset"],
            "accuracy": yaw.parent / "yaw_accuracy.json",
            "extrinsic": root / cfg["camera"]["extrinsics_file"],
            "alignment": cfg.get("alignment") or {}}


def revision_of(root: Path) -> str:
    for candidate in (root.parent / "REVISION", root / "REVISION"):
        if candidate.is_file():
            return candidate.read_text(encoding="utf-8").strip()[:12]
    try:
        return subprocess.run(["git", "rev-parse", "--short=7", "HEAD"], cwd=root, capture_output=True,
                              text=True, check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def seed(state: Path, config: Path, root: Path, revision: str) -> str:
    store, records, _ = store_dirs(state)
    have = record_ids(records)
    if have:
        return f"calibration: {len(have)} record(s) in {records}; nothing seeded"
    if "hardware_profile" not in (yaml.safe_load(config.read_text(encoding="utf-8")) or {}):
        return f"calibration: {config.name} has no hardware profile (no servo assets); nothing seeded"
    parts = committed_set(config, root)
    records.mkdir(parents=True, exist_ok=True)   # the state directory may not exist on a first boot
    staging = Path(tempfile.mkdtemp(dir=store, prefix=".seed-"))
    try:
        target = staging / "0001"
        target.mkdir()
        meta = {}
        for part in ("yaw", "pitch", "accuracy", "extrinsic"):
            shutil.copyfile(parts[part], target / PARTS[part])
            meta[part] = {"file": PARTS[part], "origin": "shipped", "rev": revision,
                          **(asset_provenance(parts[part]) if part != "extrinsic" else {})}
        (target / PARTS["bore"]).write_text(bore_yaml(parts["alignment"]), encoding="utf-8")
        meta["bore"] = {"file": PARTS["bore"], "origin": "shipped", "rev": revision}
        write_json(target / "record.json", {"schema": SCHEMA, "name": "AS SHIPPED", "created": now_local(),
                                            "source": f"seeded from release {revision}",
                                            "extrinsic_frame": "pitch_from_homed_low", "parts": meta,
                                            "last_run": None})
        write_json(staging / "active.json", {"schema": 1, "active": 1})
        os.replace(target, records / "0001")
        os.replace(staging / "active.json", store / "active.json")
    finally:
        shutil.rmtree(staging, ignore_errors=True)
    return f"calibration: seeded #1 AS SHIPPED from release {revision} into {records}"


def import_record(state: Path, name: str, source: int | None, files: dict) -> str:
    _, records, incoming = store_dirs(state)
    name = name.strip().upper()
    if not NAME.match(name):
        raise SystemExit("a name is 1-24 letters, digits, spaces, - or _")
    if source is None and set(files) != set(PARTS):
        raise SystemExit("give every part, or --from ID to inherit the parts not given")
    src = records / f"{source:04d}" if source is not None else None
    if src is not None and not (src / "record.json").is_file():
        raise SystemExit(f"no record #{source} in {records}")
    incoming.mkdir(parents=True, exist_ok=True)
    slug = re.sub(r"[^A-Z0-9]+", "-", name).strip("-") or "RECORD"
    staging = Path(tempfile.mkdtemp(dir=incoming, prefix=".staging-"))
    meta = {}
    src_meta = json.loads((src / "record.json").read_text(encoding="utf-8"))["parts"] if src else {}
    for part, filename in PARTS.items():
        if part in files:
            shutil.copyfile(files[part], staging / filename)
            meta[part] = {"file": filename, "origin": "measured",
                          **(asset_provenance(Path(files[part])) if part in ("yaw", "pitch", "accuracy") else {})}
        else:
            shutil.copyfile(src / src_meta[part]["file"], staging / filename)
            meta[part] = {"file": filename, "origin": "inherited", "from": source}
    write_json(staging / "record.json", {"schema": SCHEMA, "name": name, "created": now_local(),
                                         "source": "imported" + (f" from #{source}" if src else ""),
                                         "extrinsic_frame": "pitch_from_homed_low", "parts": meta,
                                         "last_run": None})
    final = incoming / f"{datetime.datetime.now():%Y%m%d-%H%M%S}-{slug}"
    os.replace(staging, final)
    return f"calibration: staged {name} in {final}; controld adopts it at its next start (not in force)"


def listing(state: Path) -> str:
    root, records, incoming = store_dirs(state)
    try:
        active = json.loads((root / "active.json").read_text(encoding="utf-8")).get("active")
    except (OSError, ValueError):
        active = None
    out = []
    for rid in record_ids(records):
        try:
            rec = json.loads((records / f"{rid:04d}" / "record.json").read_text(encoding="utf-8"))
            parts = ", ".join(f"{k} {v.get('origin')}" + (f" #{v['from']}" if v.get("from") else "")
                              for k, v in rec.get("parts", {}).items())
            out.append(f"{'*' if rid == active else ' '} #{rid} {rec.get('name')}: {parts}")
        except (OSError, ValueError) as e:
            out.append(f"  #{rid} unreadable: {e}")
    for p in sorted(incoming.iterdir()) if incoming.is_dir() else ():
        if p.is_dir() and not p.name.startswith("."):
            out.append(f"  incoming {p.name}")
    return "\n".join(out) or f"no records in {records}"


def place(fetched: Path, dest: Path, ids=None) -> str:
    """Records fetched from a station into <dest>/<id>-<name>/, without journals (runtime captures,
    outside git). A record pulled again replaces its earlier pull, renamed or not."""
    out = []
    dest.mkdir(parents=True, exist_ok=True)
    for rid in record_ids(fetched):
        if ids and rid not in ids:
            continue
        src = fetched / f"{rid:04d}"
        try:
            name = str(json.loads((src / "record.json").read_text(encoding="utf-8")).get("name") or "")
        except (OSError, ValueError) as e:
            out.append(f"#{rid} skipped: record.json unreadable ({e})")
            continue
        slug = re.sub(r"[^A-Z0-9]+", "-", name.upper()).strip("-") or "UNNAMED"
        for old in dest.glob(f"{rid:04d}-*"):
            shutil.rmtree(old)
        target = dest / f"{rid:04d}-{slug}"
        shutil.copytree(src, target, ignore=shutil.ignore_patterns("journals", "*.tmp", ".*"))
        out.append(f"#{rid} {name} -> {target}")
    return "\n".join(out) or f"no records in {fetched}"


def pull(host: str, state: str, dest: Path, ids=None, address: str | None = None) -> str:
    """Copies the station's records into the repository as evidence (ADR-005 D3). Read-only on the
    station: one ssh that streams a tar of records/."""
    ssh = ["ssh", "-o", "ConnectTimeout=10", "-o", "ServerAliveInterval=15", "-o", "ServerAliveCountMax=4"]
    if address:   # mDNS goes stale after a reboot; the key is still the station's
        ssh += ["-o", f"HostName={address}", "-o", f"HostKeyAlias={host.rsplit('@', 1)[-1]}"]
    remote = shlex.quote(f"{state.rstrip('/')}/calibration/records")
    data = subprocess.run([*ssh, host, f"tar -C {remote} --exclude=journals -cf - ."],
                          check=True, capture_output=True).stdout
    with tempfile.TemporaryDirectory() as tmp:
        with tarfile.open(fileobj=io.BytesIO(data)) as tar:
            tar.extractall(tmp, filter="data")
        return place(Path(tmp), dest, ids)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("seed")
    s.add_argument("--state", required=True, type=Path)
    s.add_argument("--config", required=True, type=Path)
    s.add_argument("--root", type=Path, help="the Firmware directory (default: the config's grandparent)")
    s.add_argument("--revision")
    i = sub.add_parser("import")
    i.add_argument("--state", required=True, type=Path)
    i.add_argument("--name", required=True)
    i.add_argument("--from", dest="source", type=int)
    for part in PARTS:
        i.add_argument("--" + part, type=Path)
    lst = sub.add_parser("list")
    lst.add_argument("--state", required=True, type=Path)
    pl = sub.add_parser("pull", help="copy the station's records into the repository, without journals")
    pl.add_argument("--host", default="eamars@rpi-turret")
    pl.add_argument("--address", help="connect here instead of the host's name (e.g. 192.168.2.103)")
    pl.add_argument("--state", default=STATION_STATE, help="the station's state directory")
    pl.add_argument("--to", type=Path, default=FIRMWARE / "calibration" / "records")
    pl.add_argument("ids", nargs="*", type=int, help="only these records (default: all)")
    a = ap.parse_args(argv)
    if a.cmd == "seed":
        root = (a.root or a.config.resolve().parents[1]).resolve()
        print(seed(a.state, a.config.resolve(), root, a.revision or revision_of(root)))
    elif a.cmd == "import":
        files = {p: getattr(a, p) for p in PARTS if getattr(a, p) is not None}
        print(import_record(a.state, a.name, a.source, files))
    elif a.cmd == "pull":
        print(pull(a.host, a.state, a.to, a.ids, a.address))
    else:
        print(listing(a.state))
    return 0


if __name__ == "__main__":
    sys.exit(main())
