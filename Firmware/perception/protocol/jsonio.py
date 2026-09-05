"""JSON / JSON-lines plumbing for the published messages, the recorder and diagnostics.

Two details are worth having in one place because getting them wrong is invisible until
it costs an afternoon:

**Writes that a reader can never half-see.** Diagnostic dumps and recordings are written
to a temporary file in the same directory and moved with ``os.replace``, which is atomic
within a filesystem. A recording truncated by a power cut mid-write is not a truncated
recording, it is a recording whose last frames are missing and whose first frames are
fine — which is the worst possible shape for the tool that is trying to explain why a
target was lost.

**Reading a JSONL file reports what it refused.** ``iter_jsonl`` counts malformed lines
instead of swallowing them. A replay that silently skipped 3 % of its frames would
reproduce the exact defect class §41 exists to end, with a cleaner-looking console.
"""
from __future__ import annotations

import json
import os
import tempfile
from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, Iterable, Iterator, List, Optional, Tuple

from ..errors import ValidationError


def _json_default(value: Any) -> Any:
    """Enums and dataclass-ish objects that already know how to describe themselves."""
    if isinstance(value, Enum):
        return value.value
    to_dict = getattr(value, "to_dict", None)
    if callable(to_dict):
        return to_dict()
    raise TypeError(f"{type(value).__name__} is not serializable; give it a to_dict()")


def dumps(record: Any, indent: Optional[int] = None) -> str:
    """Stable-key JSON. Stability matters in a recording: two runs of the same frames
    should diff cleanly, and a key order that varies with dict insertion history makes
    that impossible."""
    return json.dumps(record, sort_keys=True, separators=(",", ":"), indent=indent,
                      default=_json_default, allow_nan=False)


def loads(text: str) -> Any:
    """Strict JSON: ``allow_nan`` is False on the write side, so refuse it on the read
    side too. ``NaN`` in a recorded timestamp is not a missing value, it is a value that
    escaped validation, and the loader must not launder it into ``None``."""
    return json.loads(text)


def load_mapping_document(path: str, *, what: str = "document") -> Dict[str, Any]:
    """Read a YAML or JSON mapping from disk; the extension decides which.

    YAML is optional on purpose: the daemon runs under the system interpreter, which has
    PyYAML, while the test venv does not. A subsystem that cannot be imported because a
    config parser is missing is a subsystem that cannot be tested offline — and §55.18
    requires the acceptance tests to run with the motors disconnected.
    """
    with open(path, "r", encoding="utf-8") as handle:
        text = handle.read()
    if path.endswith((".yaml", ".yml")):
        try:
            import yaml  # type: ignore
        except ImportError as exc:                          # pragma: no cover
            raise ValidationError(
                f"{path}: {what} is YAML but PyYAML is unavailable "
                f"(install python3-yaml, or save the file as .json)") from exc
        data = yaml.safe_load(text) or {}
    else:
        data = loads(text)
    if not isinstance(data, dict):
        raise ValidationError(f"{path}: expected a mapping at the top level of the {what}")
    return data


@dataclass
class ScanStats:
    """What a JSONL read actually saw. ``malformed`` is the no-silent-drop counter."""

    lines_read: int = 0
    records: int = 0
    malformed: int = 0
    skipped_empty: int = 0

    def to_dict(self) -> Dict[str, int]:
        return {"lines_read": self.lines_read, "records": self.records,
                "malformed": self.malformed, "skipped_empty": self.skipped_empty}


def iter_jsonl(path: str, stats: Optional[ScanStats] = None,
               skip_malformed: bool = True) -> Iterator[Dict[str, Any]]:
    """Yield decoded records from a JSONL file, one dict per line.

    ``skip_malformed=False`` raises on the first bad line: use it when the file is an
    input to a claim (§46's acceptance gates), because a run that quietly lost frames
    proves less than a run that refuses to start.
    """
    stats = stats if stats is not None else ScanStats()
    with open(path, "r", encoding="utf-8") as handle:
        for line in handle:
            stats.lines_read += 1
            stripped = line.strip()
            if not stripped:
                stats.skipped_empty += 1
                continue
            try:
                record = loads(stripped)
            except (ValueError, TypeError):
                stats.malformed += 1
                if skip_malformed:
                    continue
                raise
            if not isinstance(record, dict):
                stats.malformed += 1
                if skip_malformed:
                    continue
                raise ValueError(f"{path}: expected an object per line")
            stats.records += 1
            yield record


def read_jsonl(path: str) -> Tuple[List[Dict[str, Any]], ScanStats]:
    stats = ScanStats()
    records = list(iter_jsonl(path, stats=stats))
    return records, stats


class JsonlWriter:
    """Append-only JSONL writer with an explicit flush policy.

    ``flush_every`` bounds how much data a crash can lose without making every frame a
    filesystem sync: recordings are written on the frame path, and a daemon that blocks
    on fsync at 17 Hz has been given a latency problem it did not need.
    """

    def __init__(self, path: str, flush_every: int = 32) -> None:
        parent = os.path.dirname(os.path.abspath(path))
        os.makedirs(parent, exist_ok=True)
        self.path = path
        self._flush_every = max(1, int(flush_every))
        self._handle = open(path, "a", encoding="utf-8")
        self._pending = 0
        self.records_written = 0
        self.bytes_written = 0

    def write(self, record: Any) -> None:
        line = dumps(record) if not isinstance(record, str) else record
        blob = line + "\n"
        self._handle.write(blob)
        self.records_written += 1
        self.bytes_written += len(blob.encode("utf-8"))
        self._pending += 1
        if self._pending >= self._flush_every:
            self.flush()

    def flush(self) -> None:
        if self._pending:
            self._handle.flush()
            os.fsync(self._handle.fileno())
            self._pending = 0

    def close(self) -> None:
        try:
            self.flush()
        finally:
            if not self._handle.closed:
                self._handle.close()

    def __enter__(self) -> "JsonlWriter":
        return self

    def __exit__(self, *exc_info) -> None:
        self.close()


def atomic_write_text(path: str, text: str) -> None:
    """Write a whole file atomically (temp in the same directory, then rename)."""
    parent = os.path.dirname(os.path.abspath(path)) or "."
    os.makedirs(parent, exist_ok=True)
    handle = tempfile.NamedTemporaryFile(
        "w", encoding="utf-8", delete=False, dir=parent, prefix=".tmp-",
        suffix=os.path.basename(path))
    try:
        with handle:
            handle.write(text)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(handle.name, path)
    except BaseException:
        try:
            os.unlink(handle.name)
        except OSError:
            pass
        raise
