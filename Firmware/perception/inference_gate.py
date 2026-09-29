"""One accelerator, two cameras: the arbiter the dual-worker cut needs.

Why this exists as its own piece. The production shape (B path) feeds both sensors' inference
streams into **one** Hailo, and the station measured that as comfortable: aggregate 59.9 fps with
the device busy ~50 % at 640x360 inputs. What it does *not* measure is what happens when one camera
arrives faster than the shared device can serve both, or when a consumer is slow: that is where a
naive design grows an unbounded queue and quietly turns live latency into stored latency.

So the queueing rule is fixed here, in one place, and testable without hardware:

* **bounded, newest-wins, counted.** Depth is a parameter (the measured production number is 2).
  A frame that finds the buffer full is dropped and *named* — dropped-by-buffer-full and
  dropped-because-the-device-was-busy are different diagnoses.
* **round-robin, not arrival-order.** A wide camera at 30 fps must not starve a detail camera at
  30 fps by producing its frames a microsecond earlier every time. Fairness is a property, so it
  gets a test.
* **one device, one caller.** The accelerator is called from this thread only. Two threads
  sharing one Hailo group without a plan is how you get a device that appears healthy and serves
  whoever shouts loudest.

What is deliberately *not* here: per-camera tracking and selection. `PerceptionPipeline.process_frame`
has no `camera_id` argument yet, so a frame from the second sensor would be folded into the first
sensor's tracks. That belongs to the dual-worker cut (the same one `visiond`'s identity comment and
`telemetry.hpp`'s §20 note both point at). This module hands that cut a scheduler it can trust.
"""
from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Callable, Deque, Dict, List, Optional, Tuple

DEFAULT_QUEUE_DEPTH = 2


@dataclass(frozen=True)
class InferenceJob:
    """One frame asking for the shared device, carrying the identity it must come back with.

    The identity travels with the job: a result that arrives without a camera_id cannot be
    attributed, and "we forgot which sensor this came from" is not a state worth representing as
    ``None`` deep inside a hot loop — it is a bug, so the type refuses to construct it.
    """

    camera_id: str
    frame_sequence: int
    sensor_timestamp_ns: int
    image: Any
    metadata: Any = None

    def __post_init__(self) -> None:
        if not self.camera_id:
            raise ValueError("an inference job with no camera_id cannot be attributed to a stream")


@dataclass
class InferenceArbiter:
    """Round-robins bounded per-camera queues into one accelerator, on one thread.

    ``infer`` is called as ``infer(job) -> result`` and is expected to raise for a frame-level
    failure; a raise counts, names itself once, and keeps the thread alive. A device that fails
    forever should degrade the streams, not retire the daemon.
    """

    infer: Callable[[InferenceJob], Any]
    queue_depth: int = DEFAULT_QUEUE_DEPTH
    clock: Callable[[], float] = time.monotonic
    idle_sleep_s: float = 0.002
    roles: Dict[str, str] = field(default_factory=dict)
    served: Dict[str, int] = field(default_factory=dict)
    dropped_full: Dict[str, int] = field(default_factory=dict)
    failures: int = 0
    first_error: str = ""
    _queues: Dict[str, Deque[InferenceJob]] = field(default_factory=dict, repr=False)
    _lock: threading.Lock = field(default_factory=threading.Lock, repr=False)
    _wake: threading.Condition = field(default_factory=threading.Condition, repr=False)
    _stop: threading.Event = field(default_factory=threading.Event, repr=False)
    _thread: Optional[threading.Thread] = field(default=None, repr=False)
    _order: List[str] = field(default_factory=list, repr=False)

    def __post_init__(self) -> None:
        if int(self.queue_depth) < 1:
            raise ValueError("queue_depth must be >= 1; a zero-depth queue cannot hold a frame")
        self.queue_depth = int(self.queue_depth)
        self._wake = threading.Condition(self._lock)

    def submit(self, job: InferenceJob, *, role: Optional[str] = None) -> bool:
        """Offer a frame. Returns False when it was dropped because the buffer was full.

        Newest-wins on purpose: the operator is watching *now*, and a frame from 200 ms ago is not
        history worth memory — it is a latency lie. The drop is counted per camera, so a stream
        that is chronically too fast to be served shows up as a number rather than a rumour.
        """
        key = job.camera_id
        with self._wake:
            queue = self._queues.get(key)
            if queue is None:
                queue = deque(maxlen=self.queue_depth)
                self._queues[key] = queue
                self._order.append(key)
                self.served.setdefault(key, 0)
                self.dropped_full.setdefault(key, 0)
            if role:
                self.roles[key] = role
            full = len(queue) == queue.maxlen
            queue.append(job)                              # deque(maxlen) evicts the oldest
            if full:
                self.dropped_full[key] += 1
                return False
            self._wake.notify()
            return True

    def pending(self) -> Dict[str, int]:
        with self._lock:
            return {key: len(queue) for key, queue in self._queues.items()}

    def run_once(self) -> Optional[Tuple[str, Any]]:
        """Serve one queued job, round-robin across cameras. Returns (camera_id, result)."""
        job = self._take_next_round_robin()
        if job is None:
            return None
        try:
            result = self.infer(job)
        except Exception as exc:                                            # noqa: BLE001
            self.failures += 1
            self.first_error = self.first_error or f"{type(exc).__name__}: {exc}"
            if self.failures == 1:
                # B45: a counter nobody reads is not a failure. The first one speaks.
                import sys
                print(f"inference-arbiter: first failure for {job.camera_id}: {self.first_error}",
                      file=sys.stderr)
            return (job.camera_id, None)
        self.served[job.camera_id] = self.served.get(job.camera_id, 0) + 1
        return (job.camera_id, result)

    def _take_next_round_robin(self) -> Optional[InferenceJob]:
        with self._lock:
            if not self._order:
                return None
            for _ in range(len(self._order)):
                key = self._order[0]
                self._order = self._order[1:] + [key]        # rotate: fairness is the point
                queue = self._queues.get(key)
                if queue:
                    return queue.popleft()
            return None

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._run, name="inference-arbiter", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.is_set():
            if self.run_once() is None:
                with self._wake:
                    if any(self._queues.values()):
                        continue
                    self._wake.wait(self.idle_sleep_s)

    def stop(self, join_s: float = 2.0) -> None:
        self._stop.set()
        with self._wake:
            self._wake.notify_all()
        if self._thread is not None:
            self._thread.join(join_s)

    def stats(self) -> dict:
        with self._lock:
            return {"served": dict(self.served), "dropped_full": dict(self.dropped_full),
                    "pending": {key: len(queue) for key, queue in self._queues.items()},
                    "failures": self.failures, "first_error": self.first_error,
                    "queue_depth": self.queue_depth,
                    "fairness": _fairness(self.served)}


def _fairness(served: Dict[str, int]) -> Optional[float]:
    """Ratio of the least-served camera to the most-served. 1.0 is perfectly fair, None if idle.

    Published rather than asserted in the UI because a *measured* 0.4 tells the operator that one
    sensor is being starved; the shape alone would just say "there are two streams".
    """
    counts = [count for count in served.values() if count]
    if not counts or max(counts) == 0:
        return None
    return round(min(counts) / max(counts), 3)


def _selftest() -> int:
    """No device, no /dev, no picamera2: fairness and boundedness are testable claims."""
    checks = 0

    def job(camera: str, n: int, image: Any = None) -> InferenceJob:
        return InferenceJob(camera_id=camera, frame_sequence=n,
                            sensor_timestamp_ns=n * 33_333_333, image=image)

    # 1) Round-robin: a camera that submits twice as often must not take twice the device.
    seen: List[str] = []

    def recorder(incoming: InferenceJob) -> str:
        seen.append(incoming.camera_id)
        return "ok"

    arbiter = InferenceArbiter(infer=recorder, queue_depth=2)
    for n in range(8):
        # Both sensors produce one frame per round; the detail side also produces a second one,
        # so the arbiter is asked to be fair while one queue is chronically full.
        arbiter.submit(job("wide", n))
        arbiter.submit(job("detail", n))
        arbiter.submit(job("detail", n + 100))
        assert arbiter.run_once() is not None
        assert arbiter.run_once() is not None
    wide = seen.count("wide")
    detail = seen.count("detail")
    assert wide >= 3 and detail >= 3, f"一条路饿死了：{seen}"
    assert abs(wide - detail) <= 1, f"轮转不公平：wide {wide} / detail {detail}"
    assert arbiter.stats()["dropped_full"]["detail"] >= 1, "溢出的旧帧必须计下来"
    checks += 1

    # 2) Newest wins, and the buffer never grows.
    slow = InferenceArbiter(infer=lambda j: "ok", queue_depth=2)
    for n in range(50):
        slow.submit(job("wide", n))
    assert slow.pending()["wide"] == 2, f"有界队列长到了 {slow.pending()}"
    drained: List[InferenceJob] = []
    while True:
        taken = slow._take_next_round_robin()
        if taken is None:
            break
        drained.append(taken)
    assert [j.frame_sequence for j in drained] == [48, 49], \
        f"latest-wins 应该留下最后两帧，实得 {[j.frame_sequence for j in drained]}"
    assert slow.stats()["dropped_full"]["wide"] == 48
    checks += 1

    # 3) A device that raises degrades the streams; it does not retire the thread (B45).
    def explodes(incoming: InferenceJob) -> str:
        raise RuntimeError("hailo said no")

    angry = InferenceArbiter(infer=explodes, queue_depth=2)
    angry.submit(job("wide", 1))
    assert angry.run_once() == ("wide", None)
    angry.submit(job("wide", 2))
    assert angry.run_once() == ("wide", None)
    assert angry.failures == 2 and "RuntimeError" in angry.first_error
    assert angry.stats()["served"].get("wide", 0) == 0, "失败不能记成服务过"
    checks += 1

    # 4) A result carries the identity of the sensor it came from.
    tagged = InferenceArbiter(infer=lambda j: j.camera_id, queue_depth=1)
    tagged.submit(job("detail", 7))
    camera, result = tagged.run_once()
    assert (camera, result) == ("detail", "detail")
    checks += 1

    try:
        InferenceJob(camera_id="", frame_sequence=1, sensor_timestamp_ns=1, image=None)
    except ValueError as exc:
        assert "camera_id" in str(exc)
    else:
        raise AssertionError("没有 camera_id 的作业必须被拒，不能归属不到就算成功")
    checks += 1

    for depth in (0, -1):
        try:
            InferenceArbiter(infer=lambda j: "ok", queue_depth=depth)
        except ValueError as exc:
            assert "queue_depth" in str(exc)
        else:
            raise AssertionError(f"queue_depth={depth} 必须被拒")
    checks += 1
    print(f"inference arbiter selftest: {checks} checks passed（不碰设备）")
    return 0


if __name__ == "__main__":
    raise SystemExit(_selftest())
