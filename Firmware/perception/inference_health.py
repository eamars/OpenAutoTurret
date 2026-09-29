"""Publish what the inference backend actually is and how it is doing, once a second.

The owner's rule is "what I cannot see has not been updated", and the station's web surface was built
around one sensor's on-sensor network. Switching the production backend has to be visible on that
surface, not inferred from which model file happens to be open. This module is the publisher: a daemon
thread beside the capture owner, writing the adapter's own self-report plus a timestamp.

Every field comes from the adapter, so nothing here needs calibration to mean something and nothing
here is a constant pretending to be a measurement. A backend that cannot be reached produces an absent
file or a stale one, which is a published state, not a fallback: the point of the switch is that losing
Hailo means losing perception, loudly.
"""

from __future__ import annotations

import json
import os
import threading
import time
from typing import Any, Callable, Dict, Optional

INTERVAL_S = 1.0
STALE_AFTER_S = 3.0          # three missed beats: a paused daemon must not look healthy


def report(adapter: Any, *, now_ns: int) -> Dict[str, Any]:
    """The adapter's self-report, stamped. Missing pieces stay missing rather than becoming zero."""
    described: Dict[str, Any] = {}
    describe = getattr(adapter, "describe", None)
    if callable(describe):
        try:
            described = dict(describe() or {})
        except Exception as exc:                                    # noqa: BLE001
            described = {"describe_failed": f"{type(exc).__name__}: {exc}"}
    described["updated_ns"] = int(now_ns)
    described["pid"] = os.getpid()
    return described


class HealthPublisher:
    """Writes the self-report every INTERVAL_S so the web layer never has to open the camera."""

    def __init__(self, *, adapter: Any, path: str, sink: Optional[Callable[[str], None]] = None) -> None:
        self.adapter = adapter
        self.path = str(path)
        self._sink = sink                       # tests inject a collector; production writes the file
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self.published = 0
        self.last_error = ""

    def _write_once(self) -> None:
        body = json.dumps(report(self.adapter, now_ns=time.time_ns()), sort_keys=True)
        if self._sink is not None:
            self._sink(body)
        else:
            tmp = self.path + ".tmp"
            with open(tmp, "w", encoding="utf-8") as handle:
                handle.write(body)
            os.replace(tmp, self.path)          # a reader never sees half a document
        self.published += 1

    def start(self) -> "HealthPublisher":
        def run() -> None:
            while not self._stop.wait(INTERVAL_S):
                try:
                    self._write_once()
                except Exception as exc:                            # noqa: BLE001
                    # A counter is not a failure (B45): the first reason has to be heard out loud.
                    if not self.last_error:
                        print(f"inference-health: first failure while publishing: "
                              f"{type(exc).__name__}: {exc}", flush=True)
                    self.last_error = f"{type(exc).__name__}: {exc}"

        self._thread = threading.Thread(target=run, name="inference-health", daemon=True)
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()


def _selftest() -> int:
    class Fake:
        def describe(self):
            return {"adapter": "hailo", "model_id": "yolov8n", "inferences": 7,
                    "model_inference_ms": 8.1}

    got = []
    pub = HealthPublisher(adapter=Fake(), path="/unused", sink=got.append).start()
    time.sleep(INTERVAL_S * 1.6)
    pub.stop()
    checks = [
        ("published at least once", len(got) >= 1),
        ("the backend names itself", '"adapter": "hailo"' in (got[0] if got else "")),
        ("a measured number survives", '"model_inference_ms": 8.1' in (got[0] if got else "")),
        ("every beat is stamped", all("updated_ns" in line for line in got)),
    ]
    broken = [name for name, ok in checks if not ok]
    for name, ok in checks:
        print(f"  {'✓' if ok else '✗'} {name}")
    print(f"inference-health selftest: {len(checks) - len(broken)}/{len(checks)}"
          + (f" -- failed: {broken}" if broken else ""))
    return 1 if broken else 0


if __name__ == "__main__":
    raise SystemExit(_selftest())
