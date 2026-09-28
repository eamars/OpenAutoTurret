"""One worker per camera: its own thread, its own latest-tap, its own failure.

Why this exists (WP3, docs/ADR-001/docs/07). The station has one camera today and the pipeline
runs one thread inside ``visiond``; "dual worker lifecycle" and "the wide stream survives the
detail worker dying" were therefore not properties of the code but wishes. A second camera added
to that shape shares a thread, so a hung detail camera takes the wide stream with it -- which is
exactly the boundary case 08 lists for A3. This module makes the unit real before the second
camera exists, so the isolation is architectural rather than hoped for.

Three decisions worth their comments:

* The tap holds **one** frame and counts overwrites, copying ``pipeline.LatestJsonPublisher``.
  A slow consumer must not push a bounded queue into unbounded memory, and "we dropped 4000
  frames" has to be a number someone can see, not a rumour.
* ``generation`` increments every time an owner (re)starts. A frame carrying a retired generation
  is rejected, which is what "no old owner running concurrently" means in practice: the old
  owner's in-flight output is discarded, not merged with the new one's.
* ``CameraWorker.run`` never raises into the supervisor. A worker that dies takes only its own
  tap offline; saying so through ``state``/``last_error`` is the supervisor's whole interface to
  failure, so one camera's exception cannot become the other camera's exception.
"""
from __future__ import annotations

import threading
import time
from typing import Any, Callable, Dict, Optional

from perception.camera_id import CameraId


# Why this is not ``pipeline.PreviewTap``. That tap belongs to the preview path: it is rate-limited
# to ``fps`` and carries preview-specific counters, because §39's whole point is that preview must
# not sit on the control critical path. Applying that limiter to the tracking hand-off would
# silently discard frames the tracker has not seen yet -- turning a throughput problem into a
# tracking problem. Different path, different policy; the shared shape (depth one, counted
# overwrites) is deliberate and stays that way.
class LatestTap:
    """One-slot hand-off: the newest frame wins and every overwrite is counted."""

    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._item: Optional[Any] = None
        self.overwritten = 0

    def push(self, item: Any) -> None:
        with self._condition:
            self.overwritten += int(self._item is not None)
            self._item = item
            self._condition.notify()

    def take(self, timeout_s: float = 0.0) -> Optional[Any]:
        """Newest item or None. Never blocks a caller that asked for timeout 0."""
        with self._condition:
            if self._item is None and timeout_s > 0:
                self._condition.wait(timeout_s)
            item, self._item = self._item, None
            return item

    def depth(self) -> int:
        with self._condition:
            return 0 if self._item is None else 1


class CameraWorker:
    """One camera's owner thread. ``step`` is called repeatedly; returning or raising is a death,
    not an error the supervisor has to survive."""

    def __init__(self, camera_id: CameraId, step: Callable[[], Any],
                 clock: Callable[[], float] = time.monotonic) -> None:
        self.camera_id = camera_id
        self.tap = LatestTap()
        self.generation = 0
        self.state = "created"          # created | running | dead | stopped
        self.last_error = ""
        self.frames = 0
        self._step = step
        self._clock = clock
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self) -> int:
        """(Re)start the owner. The generation bump is the point: anything the previous owner
        still has in flight belongs to a retired generation."""
        self.generation += 1
        self.state = "running"
        self.last_error = ""
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name=f"camera-{self.camera_id.id}",
                                        daemon=True)
        self._thread.start()
        return self.generation

    def _run(self) -> None:
        generation = self.generation
        try:
            while not self._stop.is_set():
                item = self._step()
                if item is None:
                    continue
                # Tagged here, at the owner's own boundary, so a consumer can reject a frame from
                # an owner that has already been restarted.
                self.tap.push((generation, item))
                self.frames += 1
        except Exception as exc:                                    # noqa: BLE001
            self.last_error = f"{type(exc).__name__}: {exc}"
            self.state = "dead"
            return
        self.state = "stopped" if self._stop.is_set() else "dead"

    def stop(self, join_s: float = 1.0) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(join_s)
            if self._thread.is_alive():
                # A hung worker is *left* hung, deliberately: joining harder would mean the
                # supervisor waits on the camera that is misbehaving.
                self.last_error = self.last_error or "owner thread did not exit within the join window"

    def alive(self) -> bool:
        return self.state == "running" and self._thread is not None and self._thread.is_alive()

    def latest(self, expect_generation: Optional[int] = None) -> Optional[Any]:
        """Newest frame, dropping anything from a retired generation."""
        item = self.tap.take()
        if item is None:
            return None
        generation, payload = item
        if expect_generation is not None and generation != expect_generation:
            return None
        if generation != self.generation:
            return None
        return payload


class WorkerSupervisor:
    """Keeps one worker per camera id and answers the only question that matters when one fails:
    did the others keep serving?"""

    def __init__(self) -> None:
        self._workers: Dict[str, CameraWorker] = {}

    def add(self, worker: CameraWorker) -> "WorkerSupervisor":
        """Returns self so callers can chain. Keyed by the durable camera id: two paths that turn
        out to be the same camera get one worker, not two owners arguing over one device."""
        if worker.camera_id.id in self._workers:
            raise ValueError(f"a worker for {worker.camera_id.id} is already registered")
        self._workers[worker.camera_id.id] = worker
        return self

    def start_all(self) -> Dict[str, int]:
        return {key: worker.start() for key, worker in self._workers.items()}

    def ids(self):
        return sorted(self._workers)

    def worker(self, camera_id: str) -> Optional[CameraWorker]:
        return self._workers.get(camera_id)

    def status(self) -> Dict[str, Dict[str, Any]]:
        return {key: {"state": worker.state, "generation": worker.generation,
                      "frames": worker.frames, "dropped": worker.tap.overwritten,
                      "alive": worker.alive(), "error": worker.last_error}
                for key, worker in self._workers.items()}


def selftest() -> int:
    """The A3 properties, checked with mocks so they can be checked at all.

    Real-camera rates belong to T2 on the station; what is checkable here is the shape of the
    failure: one worker dying, hanging, or flooding must not touch its sibling.
    """
    from perception.camera_id import derive_camera_id

    wide_id = derive_camera_id("/dev/v4l/by-path/platform-wide-capture-video0")
    detail_id = derive_camera_id("/dev/v4l/by-path/platform-detail-capture-video1")
    checks = []

    # 1. A worker that raises takes only its own tap offline.
    def explode(_n=[0]):
        _n[0] += 1
        if _n[0] > 2:
            raise RuntimeError("detail sensor wedged")
        return _n[0]

    wide = CameraWorker(wide_id, lambda: {"seq": 1})
    detail = CameraWorker(detail_id, explode)
    sup = WorkerSupervisor()
    sup.add(wide).add(detail)
    sup.start_all()
    time.sleep(0.2)
    checks.append(("a dead worker is reported dead, not silently alive", detail.state == "dead"))
    checks.append(("the surviving worker is still alive and producing",
                   wide.alive() and wide.latest() is not None))
    checks.append(("the dead worker names its own failure", "wedged" in detail.last_error))

    # 2. A hung worker must not be able to delay the supervisor's answer.
    gate = threading.Event()
    hung = CameraWorker(detail_id, lambda: gate.wait(2.0) or "late")
    hung.start()
    time.sleep(0.05)
    t0 = time.monotonic()
    sup2 = WorkerSupervisor()
    sup2.add(hung)
    sup2.status()
    checks.append(("status stays answerable while a worker hangs",
                   (time.monotonic() - t0) < 0.05))
    hung.stop(join_s=0.05)
    checks.append(("stopping a hung worker records why it is still there",
                   "join window" in hung.last_error))
    gate.set()

    # 3. Backpressure: the newest frame wins, the queue never grows, and drops are counted.
    flooded = CameraWorker(wide_id, lambda: {"n": 1})
    flooded.start()
    time.sleep(0.15)
    flooded.stop(join_s=0.2)
    checks.append(("the tap stays bounded at one item under a flood", flooded.tap.depth() <= 1))
    checks.append(("overwrites are counted rather than assumed", flooded.tap.overwritten > 0))

    # 4. A restarted owner retires its generation; the old owner's output cannot be merged in.
    retired = CameraWorker(wide_id, lambda: "first-owner")
    retired.start()
    time.sleep(0.05)
    first_generation = retired.generation
    retired.stop(join_s=0.2)
    second_generation = retired.start()
    time.sleep(0.05)
    checks.append(("restart increments the generation", second_generation > first_generation))
    checks.append(("a frame from a retired generation is refused, not merged",
                   retired.latest(expect_generation=first_generation) is None))

    failed = [name for name, ok in checks if not ok]
    for name, ok in checks:
        print(("  ok   " if ok else "  FAIL ") + name)
    print("camera_worker selftest: %d/%d passed" % (len(checks) - len(failed), len(checks)))
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(selftest())
