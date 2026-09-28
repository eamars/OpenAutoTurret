"""Load the calibration manifest and answer one question: may anything be qualified on it?

The manifest (``manifest.yaml``) records provenance, not numbers -- a duplicated calibration
constant is a second way to be wrong. This module's job is the refusal: a missing file, a file
that no longer matches its hash, an entry with no calibration session, or an entry whose
coordinate closure was never demonstrated must all stop a qualification, with a reason that says
which entry and why. Silent degradation is what let a 180-degree-wrong preview look healthy.
"""
from __future__ import annotations

import hashlib
from pathlib import Path
from typing import Dict, List, Optional

import yaml

CLOSURE_ACCEPTED = ("measured",)          # 'claimed' is honest but not enough to qualify
SCHEMA = "ota-calibration-manifest-v1"


class Status:
    def __init__(self) -> None:
        self.reasons: List[str] = []
        self.entries: Dict[str, Dict[str, str]] = {}

    @property
    def qualified(self) -> bool:
        return not self.reasons

    def qualified_for(self, mode: str) -> bool:
        return self.qualified and all(mode in e["applies_to"] for e in self.entries.values())


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load(manifest_path: str, root: Optional[str] = None) -> Status:
    status = Status()
    origin = Path(manifest_path)
    base = Path(root) if root else origin.parent
    if not origin.is_file():
        status.reasons.append(f"calibration manifest is missing: {manifest_path}")
        return status
    doc = yaml.safe_load(origin.read_text(encoding="utf-8")) or {}
    if doc.get("schema") != SCHEMA:
        status.reasons.append(f"calibration manifest schema is {doc.get('schema')!r}, expected {SCHEMA!r}")
        return status
    for entry in doc.get("entries") or ():
        ident = str(entry.get("id") or "<unnamed>")
        path = base / str(entry.get("path") or "")
        if not path.is_file():
            status.reasons.append(f"{ident}: referenced file is missing ({entry.get('path')})")
            continue
        if _sha256(path) != str(entry.get("sha256") or ""):
            status.reasons.append(f"{ident}: contents no longer match the manifest hash")
        if not str(entry.get("session") or "").strip():
            status.reasons.append(f"{ident}: no calibration session recorded, so its provenance is unknown")
        if str(entry.get("closure") or "unknown") not in CLOSURE_ACCEPTED:
            status.reasons.append(
                f"{ident}: coordinate closure is {entry.get('closure') or 'unknown'!r}, "
                f"and qualification needs {CLOSURE_ACCEPTED}")
        else:
            status.entries[ident] = {"path": str(path),
                                     "applies_to": [str(m) for m in entry.get("applies_to") or ()]}
    return status


def selftest() -> int:
    """The refusals, exercised. A gate that has never been seen to fail is decoration."""
    import shutil
    import tempfile

    checks = []
    work = tempfile.mkdtemp()
    target = Path(work) / "cal.yaml"
    target.write_text("a: 1\n", encoding="utf-8")
    good = ("schema: %s\nentries:\n  - id: intrinsics\n    path: cal.yaml\n    sha256: %s\n"
            "    session: \"cal-x\"\n    applies_to: [tracking]\n    closure: measured\n"
            % (SCHEMA, _sha256(target)))
    manifest = Path(work) / "manifest.yaml"

    def reload(text):
        manifest.write_text(text, encoding="utf-8")
        return load(str(manifest))

    checks.append(("a complete manifest qualifies", reload(good).qualified))
    checks.append(("and qualifies the mode it lists", reload(good).qualified_for("tracking")))
    checks.append(("without pretending about modes it never listed",
                   not reload(good).qualified_for("manual")))
    tampered = good.replace('closure: measured', 'closure: measured')   # keep hash line intact
    target.write_text("a: 2\n", encoding="utf-8")                       # content drift
    checks.append(("a drifted file is refused by name",
                   "intrinsics" in " ".join(reload(tampered).reasons)
                   and any("hash" in r for r in reload(tampered).reasons)))
    target.write_text("a: 1\n", encoding="utf-8")
    checks.append(("a blank session is refused -- unknown provenance is not a pass",
                   any("session" in r for r in reload(good.replace('session: "cal-x"', 'session: ""')).reasons)))
    checks.append(("an unproven closure is refused rather than assumed",
                   any("closure" in r for r in reload(good.replace("closure: measured", "closure: claimed")).reasons)))
    (Path(work) / "cal.yaml").unlink()
    checks.append(("a missing referenced file is refused with its path",
                   any("missing" in r for r in reload(good).reasons)))
    checks.append(("a wrong schema version is refused outright",
                   any("schema" in r for r in reload(good.replace(SCHEMA, "other")).reasons)))
    shutil.rmtree(work, ignore_errors=True)

    failed = [name for name, ok in checks if not ok]
    for name, ok in checks:
        print(("  ok   " if ok else "  FAIL ") + name)
    print("calibration manifest selftest: %d/%d passed" % (len(checks) - len(failed), len(checks)))
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(selftest())
