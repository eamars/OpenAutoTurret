"""§9.1 — the environment manifest: record the whole compatibility set *before* changing it.

§9.1's closing sentence is the reason this module exists: *"the agent shall not 'upgrade until
it works' without recording the full compatibility set."* The station has already been through
an upgrade-by-accident — a working IMX500 pipeline stopped working after package churn, and
nobody could say which of `python3-picamera2`, `imx500-firmware`, `imx500-models` or
`rpicam-apps-imx500-postprocess` moved. A "works on my station" claim is only evidence when the
station is described.

So the manifest records: OS, Python, kernel, the six Debian packages that jointly define the
IMX500 stack, numpy/opencv, and the model's own identity (path, sha256, task, labels hash,
intrinsics). It is written at startup, before inference begins, and it is written even when
everything is fine — a record that only exists after a failure cannot tell you what changed.

Every collector is defensive: an unavailable `dpkg-query`, a missing package, an unreadable
sysfs node each record as `None`/`"absent"` rather than raising. A manifest that cannot be
collected must still be produced, because the manifest's job is to describe reality including
the parts of reality that are broken.
"""
from __future__ import annotations

import hashlib
import os
import platform
import subprocess
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Mapping, Optional, Sequence

from ..protocol.jsonio import atomic_write_text, dumps

#: The Debian packages that together *are* the IMX500 stack (§9.1's list), plus the two
#: Python libraries whose APIs this subsystem touches directly.
DEBIAN_PACKAGES: tuple = (
    "python3-picamera2",
    "imx500-all",
    "imx500-firmware",
    "imx500-models",
    "imx500-tools",
    "rpicam-apps-imx500-postprocess",
)

PYTHON_DISTRIBUTIONS: tuple = ("picamera2", "numpy", "opencv-python", "PyYAML")


def _run(argv: Sequence[str], timeout: float = 4.0) -> Optional[str]:
    """Run a helper command, returning stdout or ``None``. Never raises.

    Version collection runs before the camera is configured. A subprocess that can throw —
    because a binary is missing, or because `subprocess` is sandboxed — must not be able to
    stop the daemon from starting.
    """
    try:
        completed = subprocess.run(list(argv), capture_output=True, text=True,
                                   timeout=timeout, check=False)
    except (OSError, subprocess.SubprocessError):
        return None
    if completed.returncode != 0:
        return None
    return completed.stdout.strip() or None


def _dpkg_version(name: str) -> str:
    out = _run(["dpkg-query", "-W", "-f=${Version}", name])
    return out or "absent"


def _distribution_version(name: str) -> Optional[str]:
    try:
        from importlib.metadata import PackageNotFoundError, version
    except ImportError:                               # pragma: no cover - py<3.8
        return None
    try:
        return version(name)
    except PackageNotFoundError:
        return None


def sha256_file(path: str, *, chunk: int = 1 << 20) -> Optional[str]:
    """Hash a model file. Returns ``None`` when it cannot be read, which is a finding."""
    if not path:
        return None
    try:
        digest = hashlib.sha256()
        with open(path, "rb") as handle:
            while True:
                block = handle.read(chunk)
                if not block:
                    break
                digest.update(block)
    except OSError:
        return None
    return digest.hexdigest()


def labels_hash(names: Optional[Sequence[str]]) -> Optional[str]:
    """A stable short hash of the label list, so a re-export that reordered classes shows up
    in the manifest without storing eighty strings beside every other copy."""
    if not names:
        return None
    joined = "\n".join(str(name) for name in names)
    return hashlib.sha256(joined.encode("utf-8")).hexdigest()[:16]


@dataclass
class EnvironmentManifest:
    """§9.1's document, as data."""

    os: str = ""
    python: str = ""
    kernel: str = ""
    hostname: str = ""
    machine: str = ""
    collected_at_ns: int = 0
    packages: Dict[str, Optional[str]] = field(default_factory=dict)
    model: Dict[str, Any] = field(default_factory=dict)
    collector_notes: List[str] = field(default_factory=list)

    # -- collection ---------------------------------------------------------
    @classmethod
    def collect(cls, *, model_path: str = "", model_id: str = "", task: str = "",
                labels: Optional[Sequence[str]] = None,
                network_intrinsics: Optional[Mapping[str, Any]] = None,
                extra_packages: Sequence[str] = ()) -> "EnvironmentManifest":
        packages: Dict[str, Optional[str]] = {}
        notes: List[str] = []
        for name in tuple(DEBIAN_PACKAGES) + tuple(extra_packages):
            packages[name] = _dpkg_version(name)
        if all(value == "absent" for value in packages.values()):
            notes.append("dpkg-query unavailable or no matching packages: this is not a "
                         "Debian station, or the IMX500 stack is not installed by dpkg")
        for name in PYTHON_DISTRIBUTIONS:
            version = _distribution_version(name)
            if version is not None:
                packages[f"python:{name}"] = version
        if not any(pip for pip in packages if pip):
            notes.append("no python distributions importable under this interpreter")

        model: Dict[str, Any] = {"path": model_path or None, "model_id": model_id or None,
                                 "task": task or None,
                                 "sha256": sha256_file(model_path) if model_path else None,
                                 "labels_hash": labels_hash(labels),
                                 "network_intrinsics": _plain(network_intrinsics)}
        if model_path and model["sha256"] is None:
            notes.append(f"model file {model_path} could not be read for hashing")

        return cls(
            os=f"{platform.system()} {platform.release()}",
            python=platform.python_version(),
            kernel=platform.release() or (_run(["uname", "-r"]) or ""),
            hostname=platform.node(),
            machine=platform.machine(),
            collected_at_ns=time.time_ns(),
            packages=packages,
            model=model,
            collector_notes=notes)

    # -- output -------------------------------------------------------------
    def missing_required_packages(self) -> List[str]:
        return [name for name in DEBIAN_PACKAGES if self.packages.get(name) in (None, "absent")]

    def to_dict(self) -> Dict[str, Any]:
        return {"os": self.os, "python": self.python, "kernel": self.kernel,
                "hostname": self.hostname, "machine": self.machine,
                "collected_at_ns": int(self.collected_at_ns),
                "packages": dict(self.packages),
                "model": dict(self.model),
                "collector_notes": list(self.collector_notes)}

    def to_yaml(self) -> str:
        """§9.1 shows YAML; emit it without requiring PyYAML.

        The shape is flat enough that writing it by hand is fewer moving parts than an
        optional dependency, and this file is read by humans and `diff`, not by code.
        """
        lines = [f"os: {self.os}", f"python: {self.python}", f"kernel: {self.kernel}",
                 f"hostname: {self.hostname}", f"machine: {self.machine}",
                 f"collected_at_ns: {int(self.collected_at_ns)}", "packages:"]
        for name in sorted(self.packages):
            value = self.packages[name]
            lines.append(f"  {_quote(name)}: {_quote(value) if value else 'null'}")
        lines.append("model:")
        for key, value in self.model.items():
            if isinstance(value, dict):
                lines.append(f"  {key}:")
                for inner, inner_value in sorted(value.items()):
                    lines.append(f"    {_quote(inner)}: {_quote(inner_value)}")
            else:
                lines.append(f"  {key}: {_quote(value) if value is not None else 'null'}")
        if self.collector_notes:
            lines.append("collector_notes:")
            lines.extend(f"  - {_quote(note)}" for note in self.collector_notes)
        return "\n".join(lines) + "\n"

    def write(self, path: str) -> str:
        """Write YAML (by extension) or JSON, atomically (§43's durability rule)."""
        text = self.to_yaml() if path.endswith((".yaml", ".yml")) else \
            dumps(self.to_dict(), indent=2)
        atomic_write_text(path, text)
        return path


def _quote(value: Any) -> str:
    text = str(value)
    if text == "" or any(char in text for char in ":#{}[]&*?|>\"'%@`,") or text != text.strip():
        escaped = text.replace('"', '""')
        return f'"{escaped}"'
    return text


def _plain(value: Any) -> Any:
    """``NetworkIntrinsics`` is an object; the manifest must be serialisable."""
    if value is None:
        return None
    if isinstance(value, Mapping):
        return {str(key): _plain(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_plain(item) for item in value]
    if isinstance(value, bool) or isinstance(value, (int, float, str)) or value is None:
        return value
    return str(value)
