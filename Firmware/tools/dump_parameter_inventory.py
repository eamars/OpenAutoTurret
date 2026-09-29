#!/usr/bin/env python3
"""Derive parameter_inventory.json from the binary that enforces it, and prove nothing is unlisted.

Why generated: ADR-002.1 §6 asks for real-tunable coverage, and a hand-typed inventory is a copy of
the code — the first new field makes it a stale copy that still reads convincingly. controld prints
the registry (``--dump-parameter-registry <path>``) from the same member accesses that apply the
values, so the document cannot claim a field the firmware does not read. This tool adds what a
running process cannot know about itself — the binary digest and the source revision — and then does
the part only a source scan can do: enumerate every field the config headers declare and refuse if
one is neither bound to an entry nor excluded by a stated reason.

Run it after changing a control parameter, and commit the result with the change:

    .venv/bin/python Firmware/tools/dump_parameter_inventory.py --write
"""
from __future__ import annotations

import argparse
import datetime
import hashlib
import json
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
FIRMWARE = os.path.dirname(HERE)
CONTROL = os.path.join(FIRMWARE, "control", "src")
HEADERS = [
    os.path.join(CONTROL, "config", "turret_config.hpp"),
    os.path.join(CONTROL, "config", "mixed_hardware_profile.hpp"),
    os.path.join(CONTROL, "can", "gm6020_friction.hpp"),
    os.path.join(CONTROL, "control", "motion_profile.hpp"),
]
INVENTORY = os.path.join(FIRMWARE, "docs", "ADR-002.1", "manifests", "parameter_inventory.json")
BINARY_CANDIDATES = ["build/control/controld", "build-arm64/control/controld"]
# MotionProfile/MotionRates are deliberately NOT scalars: they are nested structs, and a
# struct that counts as its own field loses its parent, which is how a rule on the parent
# would silently stop covering it.
FIELD_TYPES = r"double|float|int|long|unsigned|uint\d+_t|int\d+_t|bool|char|std::string|std::vector<[^>]*>|std::array<[^>]*>|gm6020::FrictionConfig"
PSEUDO_RULES = {"campaign": "not a config struct: the campaign knobs live in campaign.lock.json"}
# A config header declares both what can be configured and what the controller computes per cycle.
# gm6020_friction.hpp also holds the compensation output struct: runtime state nobody configures, so
# counting it as configuration would turn "coverage" into a list of everything in the tree and stop
# meaning anything. Coverage is scoped to the types a config or profile file can actually carry.
COVERED_STRUCTS = {"gm6020_friction.hpp": {"FrictionConfig"}}


def source_revision(root: str) -> tuple:
    """Which source tree produced this binary — or an honest empty string.

    `git -C <release-dir> rev-parse HEAD` walks *up* the tree when the directory has no checkout of its
    own, and on the station that finds the Pi's long-dirty working copy, whose HEAD has nothing to do
    with the release being measured: the release reported `source_rev=6a47f1d…` for a build made from
    `4c0b047…`. A binding leg that quietly describes someone else's tree is worse than a missing one,
    because it reads like evidence. Naming the git directory exactly means "unknown" is what we get
    when it is unknown.
    """
    outcome = subprocess.run(["git", "--git-dir", os.path.join(root, ".git"), "rev-parse", "HEAD"],
                             capture_output=True, text=True)
    if outcome.returncode != 0:
        return "", ("no git checkout at " + root + ": the digest still binds the binary, the revision "
                    "is unknown here rather than borrowed from a parent directory")
    return outcome.stdout.strip(), "read from " + os.path.join(root, ".git")


def sha256(path: str) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for block in iter(lambda: handle.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def find_binary(explicit: str = "") -> str:
    candidates = [explicit] if explicit else [os.path.join(FIRMWARE, p) for p in BINARY_CANDIDATES]
    for path in candidates:
        if path and os.path.isfile(path) and os.access(path, os.X_OK):
            return path
    raise SystemExit(
        "no controld binary to ask. Build one first (this document is generated from the binary "
        f"that enforces it, not from a wish): looked in {', '.join(candidates)}")


def parse_structs() -> dict:
    """struct name -> declared scalar fields + child struct types, read out of the headers.

    Nested declarations count as children of the struct that declares them (AxisLimitsConfig owns
    its TravelDeg pair), and so does any member whose declared type is another declared struct --
    including through std::array/std::vector, which is how the motion tables hang together. A
    ``*Result`` type is a load outcome rather than a configuration surface, so it is not under
    coverage unless something explicitly asks for it by name.
    """
    structs = {}
    for path in HEADERS:
        allowed = COVERED_STRUCTS.get(os.path.basename(path))
        text = open(path, encoding="utf-8").read()
        text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
        text = "\n".join(re.sub(r"//.*$", "", line) for line in text.splitlines())
        for match in re.finditer(r"\b(?:struct|class)\s+(\w+)[^{]*\{(.*?)\n\s*\}", text, flags=re.S):
            name, body = match.group(1), match.group(2)
            if allowed is not None and name not in allowed:
                continue
            fields, children = [], []
            for nested in re.finditer(r"\bstruct\s+(\w+)\s*\{", body):
                children.append(nested.group(1))
            for line in body.splitlines():
                field = re.match(r"\s*(?:static\s+)?(?:const\s+)?([\w:<>,\[\]0-9 ]+?)\s+(\w+)\s*(?:\{[^;]*\})?\s*(?:=|;)", line)
                if not field:
                    continue
                declared, field_name = field.group(1).strip(), field.group(2)
                if declared == "enum" or "(" in declared:
                    continue
                if re.fullmatch(FIELD_TYPES, declared):
                    fields.append(field_name)
                    continue
                for candidate in re.findall(r"[A-Za-z_][\w:]*", declared):
                    simple = candidate.split("::")[-1]
                    if simple and simple[0].isupper() and not simple.startswith("std"):
                        children.append(simple)
            entry = structs.setdefault(name, {"fields": [], "children": []})
            entry["fields"] += fields
            entry["children"] += children
    return {name: entry for name, entry in structs.items()
            if not (name.endswith("Result") and entry["fields"])}


def ancestors(structs: dict) -> dict:
    """child struct -> the structs that contain it, so a rule on an owner covers what it owns.

    Without this a rule on MotionConfig would not cover MotionProfile, and the honest alternative
    (naming every nested type in C++) would make the exclusion list longer than the entry list.
    """
    owners = {}
    for name, entry in structs.items():
        for child in entry["children"]:
            if child != name:
                owners.setdefault(child, set()).add(name)
    changed = True
    while changed:                        # transitive; these graphs are small
        changed = False
        for child, parents in list(owners.items()):
            for parent in list(parents):
                for grand in owners.get(parent, ()):
                    if grand != child and grand not in owners.setdefault(child, set()):
                        owners[child].add(grand)
                        changed = True
    return owners


def bound_fields(document: dict) -> set:
    """Field names the registry binds, taken from source_binding's last path element.

    The join is by field name, and the limitation is stated instead of hidden: a binding names the
    member path (``Profile::Yaw::current_kp_a_per_rad_s``) while the parse only knows the type
    (``Axis``), so coverage asks "is any entry bound to a field of this name". Two same-named fields
    in different structs therefore vouch for each other -- which is why every row spells out name,
    group, unit and mutability: the suffix decides what is counted, the row is what a reader trusts.
    """
    bound = set()
    for entry in document.get("entries", []):
        binding = entry.get("source_binding", "")
        tail = re.split(r"[.:]", binding)[-1]
        tail = re.sub(r"^\[\d+\]$", "", tail)
        if tail:
            bound.add(tail)
    return bound


def check(document: dict) -> list:
    """Every declared field must be bound or excluded. Returns the problems, empty when clean."""
    problems = []
    structs = parse_structs()
    owners = ancestors(structs)
    bound = bound_fields(document)
    rules = {rule["config_struct"]: rule.get("reason", "") for rule in document.get("exclusion_rules", [])}
    for rule_name in rules:
        if rule_name not in structs and rule_name not in PSEUDO_RULES:
            problems.append(f"exclusion rule {rule_name!r} names no struct in the headers: a rule "
                            "that matches nothing will also block nothing when the code moves on")
    for name, entry in sorted(structs.items()):
        for field in entry["fields"]:
            if field in bound:
                continue
            chain = {name} | owners.get(name, set())
            covered = sorted(chain & set(rules))
            if covered:
                continue
            problems.append(
                f"{name}::{field} is a declared config field with no registry entry and no "
                "exclusion rule. Either it is tunable (add it to build_parameter_registry with "
                "unit, bounds, mutability and readback source) or it is not (add a rule saying why). "
                "Silently unlisted is what let a firmware field go unmentioned.")
    for entry in document.get("entries", []):
        missing = [key for key in ("name", "group", "type", "unit", "supported_range", "mode",
                                   "mutability", "apply_condition", "source_binding",
                                   "readback_source", "encoding")
                   if not entry.get(key)]
        if missing:
            problems.append(f"entry {entry.get('name', '?')} is missing {missing}")
        if entry.get("mutability") not in ("experiment_writable", "fixed_in_campaign",
                                           "protected_read_only", "unsupported"):
            problems.append(f"entry {entry.get('name')} has mutability "
                            f"{entry.get('mutability')!r}; the four names are the contract")
        if entry.get("mutability") != "experiment_writable" and not entry.get("reason"):
            problems.append(f"entry {entry.get('name')} is not searchable and gives no reason")
        if entry.get("mutability") == "unsupported" and entry.get("readback_source") != "none":
            problems.append(f"entry {entry.get('name')} is unsupported but claims a readback source; "
                            "that is how a no-op write gets reported as applied")
    return problems


def generate(binary: str, config_path: str, out_path: str) -> dict:
    with subprocess_tmp() as path:
        result = subprocess.run([binary, config_path, "--dump-parameter-registry", path],
                                capture_output=True, text=True, timeout=60, cwd=FIRMWARE)
        if result.returncode != 0:
            raise SystemExit(f"controld refused to dump its registry (exit {result.returncode}): "
                             + (result.stderr.strip().splitlines() or ["no explanation"])[-1])
        document = json.load(open(path, encoding="utf-8"))
    generated = document.get("generated_from", {})
    # A release tree has no checkout, but the deployment records the revision it built; that file is
    # a better authority than a parent directory's git state, so the caller may hand it over.
    recorded = ""
    for candidate in (os.path.join(os.path.dirname(FIRMWARE), "REVISION"),
                      os.path.join(FIRMWARE, "REVISION")):
        if os.path.exists(candidate):
            recorded = open(candidate, encoding="utf-8").read().strip()
            break
    revision, revision_how = source_revision(os.path.dirname(FIRMWARE))
    if not revision and recorded:
        revision, revision_how = recorded, ("read from the release's own REVISION file, written by the "
                                            "deployment that built this tree")
    document["generated_from"] = {
        **generated,
        "binary": os.path.relpath(binary, FIRMWARE),
        "binary_sha256": sha256(binary),
        "source_rev": revision,
        "source_rev_how": revision_how,
        "generated_at_utc": datetime.datetime.now(datetime.timezone.utc)
                            .strftime("%Y-%m-%dT%H:%M:%SZ"),
    }
    problems = check(document)
    if problems:
        for line in problems:
            print(f"parameter inventory: {line}", file=sys.stderr)
        raise SystemExit(f"parameter inventory: {len(problems)} field(s) unaccounted for")
    with open(out_path, "w", encoding="utf-8") as handle:
        json.dump(document, handle, indent=2, ensure_ascii=False, sort_keys=True)
        handle.write("\n")
    return document


class subprocess_tmp:
    """contextlib.NamedTemporaryFile would work; the name has to exist for the child process."""

    def __enter__(self):
        import tempfile
        self._handle = tempfile.NamedTemporaryFile(suffix=".json", delete=False)
        return self._handle.name

    def __exit__(self, *_exc):
        try:
            os.unlink(self._handle.name)
        except OSError:
            pass


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--binary", default="", help="controld to ask (default: build/, build-arm64/)")
    parser.add_argument("--config", default="config/turret_mixed.yaml",
                        help="config whose values are reported as the boot values")
    parser.add_argument("--write", action="store_true", help=f"write {os.path.relpath(INVENTORY, FIRMWARE)}")
    parser.add_argument("--check", action="store_true",
                        help="regenerate into a temp file and compare with the checked-in inventory")
    args = parser.parse_args()
    binary = find_binary(args.binary)
    if args.check:
        import tempfile
        with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as handle:
            path = handle.name
        try:
            fresh = generate(binary, args.config, path)
            committed = json.load(open(INVENTORY, encoding="utf-8"))
            for section in ("entries", "exclusion_rules"):
                if fresh[section] != committed.get(section):
                    raise SystemExit(
                        f"parameter_inventory.json is stale in {section!r}: regenerate with "
                        "`Firmware/tools/dump_parameter_inventory.py --write` and commit it with "
                        "the code change it describes")
            print(f"parameter inventory: up to date ({len(fresh['entries'])} entries, "
                  f"{len(fresh['exclusion_rules'])} exclusion rules, all declared fields accounted for)")
        finally:
            os.unlink(path)
        return 0
    document = generate(binary, args.config, args.write and INVENTORY or "/dev/stdout")
    if args.write:
        print(f"wrote {os.path.relpath(INVENTORY, FIRMWARE)}: {len(document['entries'])} entries, "
              f"{len(document['exclusion_rules'])} exclusion rules")
    return 0


if __name__ == "__main__":
    sys.exit(main())
