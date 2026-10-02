"""The second physical sensor, owned by visiond, published as the `detail` stream.

Why this file exists instead of a loop inside ``_run_camera``: the station's production shape
(§ architect's (b)) has one process holding **both** CSI sensors, and the day there are three the
only thing that should change is the role table. So the second sensor gets an owner of its own, a
bounded latest-only buffer of its own, and a failure that stays its own.

When the role is configured with its own ``lores`` leg, each frame also carries ``inference_image``,
which the daemon infers only while this camera is on the main display (owner, 2026-10-02; see
``inference_switch.py``). This file does not own an inference backend, a tracker or a scheduler: it
owns one sensor, one bounded latest-only buffer, and one preview.
"""
from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Any, Callable, Deque, Optional, Tuple

from .camera_id import CameraId
from .preview import JpegPreviewWorker
from .stream_manifest import StreamDescriptor, publish, publish_merged


@dataclass(frozen=True)
class DetailFrame:
    """One delivered frame of the secondary stream, already carrying its own identity.

    `sensor_timestamp_ns` is the SOF stamp of *this* frame, from *this* sensor: two cameras on one
    board can never share a clock reading, and a detail frame stamped with the wide sensor's clock
    would produce latencies that look fine and mean nothing.
    """

    camera_id: str
    frame_sequence: int
    #: This sensor's own SOF stamp. ``None`` means the sensor did not report one — a detail frame
    #: must never inherit the wide camera's clock, and "unknown" must not arrive downstream as 0.
    sensor_timestamp_ns: Optional[int]
    image: Any
    metadata: dict
    #: This camera's own inference leg, or ``None`` when the role is preview-only. It is a separate
    #: field and not "the smaller image" because the two have different lifetimes inside one
    #: capture request: both come out of the same request, and neither may be resized by hand --
    #: the whole point of the ISP leg is that the scaling happened in the sensor, not on the host.
    inference_image: Any = None
    inference_size: Tuple[int, int] = (0, 0)


class SecondaryCameraStream:
    """Owns one sensor, publishes its preview, and keeps at most `queue_depth` frames waiting.

    The buffer is bounded and newest-wins by construction: a preview that is two seconds old is
    not history, it is a lie about the present. Depth is a parameter because the station measured
    depth 2 as the production number for the inference path; the preview wants 1 (§39).
    """

    def __init__(self, *, role: str, ident: CameraId, poll: Callable[[], Optional[DetailFrame]],
                 queue_depth: int = 1, clock: Callable[[], float] = time.monotonic) -> None:
        if queue_depth < 1:
            raise ValueError("a stream with no buffer cannot be polled; use depth >= 1")
        if role not in ("wide", "detail"):
            raise ValueError(f"unknown stream role {role!r}")
        self.role = role
        self.ident = ident
        self._poll = poll
        self._queue: Deque[DetailFrame] = deque(maxlen=int(queue_depth))
        self._clock = clock
        self._lock = threading.Lock()
        self._condition = threading.Condition(self._lock)
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        # Counted separately, because "the sensor stopped" and "we are faster than the consumer"
        # look identical from a single dropped counter and need opposite answers.
        self.delivered = 0
        self.dropped_stale = 0
        self.errors = 0
        self.last_error = ""
        self.started_ns = 0

    def start(self) -> None:
        self.started_ns = int(self._clock() * 1e9)
        self._thread = threading.Thread(target=self._run, name=f"stream-{self.role}", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                frame = self._poll()
            except Exception as exc:                                    # noqa: BLE001
                # A sensor that raises is a stream that is down, not a daemon that must die: the
                # wide camera keeps serving, and this says so with a name instead of vanishing.
                self.errors += 1
                self.last_error = f"{type(exc).__name__}: {exc}"
                if self.errors == 1:
                    # Counted is not enough: a stream that fails 30 times a second and never says
                    # a word looks exactly like a stream nobody asked for. First failure speaks.
                    import sys
                    print(f"stream-{self.role}: owner thread's first failure: {self.last_error}",
                          file=sys.stderr)
                self._stop.wait(0.05)
                continue
            if frame is None:
                self._stop.wait(0.002)
                continue
            self._push(frame)

    def _push(self, frame: "DetailFrame") -> None:
        """Newest-wins into a bounded slot, counted. Split out of ``_run`` so the buffer's
        behaviour is testable without starting a thread that races it."""
        with self._condition:
            if len(self._queue) == self._queue.maxlen:
                self.dropped_stale += 1
            self._queue.append(frame)
            self.delivered += 1
            self._condition.notify()

    def latest(self, timeout_s: float = 0.0) -> Optional[DetailFrame]:
        deadline = self._clock() + max(0.0, float(timeout_s))
        with self._condition:
            while not self._queue:
                remaining = deadline - self._clock()
                if remaining <= 0:
                    return None
                self._condition.wait(remaining)
                if self._stop.is_set():
                    return None
            # Newest wins and the rest go with it: popping only the newest left an OLDER frame
            # for the next call, which would hand inference a frame from before the one it just ran.
            frame = self._queue.pop()
            self._queue.clear()
            return frame

    def stop(self, join_s: float = 2.0) -> None:
        self._stop.set()
        with self._condition:
            self._condition.notify_all()
        if self._thread is not None:
            self._thread.join(join_s)

    def delivered_fps(self, window_ns: int, last_count: int, last_ns: int) -> Optional[float]:
        """Measured rate over the caller's window — never the frame rate we asked the sensor for."""
        now_ns = int(self._clock() * 1e9)
        if now_ns - last_ns < window_ns or self.delivered <= last_count:
            return None
        return round((self.delivered - last_count) * 1e9 / max(1, now_ns - last_ns), 2)

    def stats(self) -> dict:
        with self._lock:
            return {"role": self.role, "camera_id": self.ident.id,
                    "identity_source": self.ident.source, "durable": self.ident.durable,
                    "delivered": self.delivered, "dropped_stale": self.dropped_stale,
                    "errors": self.errors, "last_error": self.last_error,
                    "queue_depth": len(self._queue)}


class DetailStreamAnnouncer:
    """Publishes the `detail` entry alongside whatever else is in the manifest.

    One writer per manifest file would make the two streams race each other's file, so this keeps
    the entries it does not own: it reads the published manifest, replaces only its own role, and
    writes the set back. Two streams on one board must not be able to erase each other.
    """

    def __init__(self, *, path: str, stream: SecondaryCameraStream, preview: JpegPreviewWorker,
                 size: Tuple[int, int], interval_s: float = 1.0) -> None:
        self.path = str(path)
        self.stream = stream
        self.preview = preview
        self.size = (int(size[0]), int(size[1]))
        self.interval_ns = max(1, int(float(interval_s) * 1_000_000_000))
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._last_count = 0
        self._last_ns = int(time.monotonic() * 1e9)
        self.failures = 0
        self.last_error = ""

    def start(self) -> None:
        self.publish_once()
        self._thread = threading.Thread(target=self._run, name="detail-manifest", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.wait(self.interval_ns / 1e9):
            self.publish_once()

    def publish_once(self) -> None:
        count = int(getattr(self.preview, "published", 0))
        now = int(time.monotonic() * 1e9)
        elapsed = now - self._last_ns
        fps = None
        if elapsed >= self.interval_ns and count > self._last_count:
            fps = round((count - self._last_count) * 1e9 / elapsed, 2)
        if elapsed >= self.interval_ns:
            self._last_count, self._last_ns = count, now
        mine = StreamDescriptor(role=self.stream.role, camera_id=self.stream.ident.id,
                                identity_source=self.stream.ident.source,
                                durable=bool(self.stream.ident.durable),
                                path=str(self.preview.path), width=self.size[0],
                                height=self.size[1], delivered_fps=fps,
                                dropped=int(getattr(self.preview, "failures", 0)), updated_ns=now)
        try:
            publish_merged(path=self.path, descriptor=mine)
            self.last_error = ""
        except (OSError, ValueError) as exc:
            self.failures += 1
            self.last_error = f"{type(exc).__name__}: {exc}"

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2)


def StreamManifest_read(path: str) -> dict:
    """Read the published manifest as {role: descriptor}, tolerating absence.

    Indirected through the module so a test can watch how often the secondary writer looks at the
    file the primary one owns.
    """
    from . import stream_manifest as sm
    manifest = sm.StreamManifest.read(path)
    return dict(manifest.streams) if manifest else {}


def _selftest() -> int:
    """No sensor, no /dev: the shape is testable, so it gets tested here and not only on hardware."""
    import os
    import tempfile
    import types

    checks = 0
    ident = types.SimpleNamespace(id="cam-1f2e3d4c", source="by-path", durable=True)
    counter = iter(range(1, 10 ** 9))

    def poll():
        # A sensor that keeps producing: newest-wins is only observable against a producer that
        # is still there after the consumer drains the slot once.
        n = next(counter)
        return DetailFrame(camera_id=ident.id, frame_sequence=n,
                           sensor_timestamp_ns=n * 33_333_333, image=object(), metadata={})

    stream = SecondaryCameraStream(role="detail", ident=ident, poll=poll, queue_depth=1)
    stream.start()
    deadline = time.monotonic() + 2.0
    seen = 0
    while time.monotonic() < deadline and seen < 3:
        if stream.latest(timeout_s=0.2) is not None:
            seen += 1
    stream.stop()
    stats = stream.stats()
    assert seen >= 3, f"三条帧都没送到，只到 {seen}"
    assert stats["camera_id"] == ident.id and stats["identity_source"] == "by-path"
    assert stats["delivered"] >= stats["dropped_stale"], "丢旧帧要计得比交付少，否则这条流在自嗨"
    checks += 1

    # A sensor that raises must mark this stream down without taking the process with it.
    def broken():
        raise RuntimeError("i2c went away")

    down = SecondaryCameraStream(role="detail", ident=ident, poll=broken)
    down.start()
    time.sleep(0.05)
    down.stop()
    assert down.stats()["errors"] > 0 and "RuntimeError" in down.last_error
    assert down.latest() is None, "挂掉的一路不能伪装成有一条帧在等"
    checks += 1

    # Newest wins: a producer faster than the consumer must not grow memory or serve the past.
    backlog = iter([DetailFrame(camera_id=ident.id, frame_sequence=n, sensor_timestamp_ns=n,
                                image=n, metadata={}) for n in range(1, 200)])
    fast = SecondaryCameraStream(role="detail", ident=ident,
                                 poll=lambda: next(backlog, None), queue_depth=1)

    def at(n):
        return DetailFrame(camera_id=ident.id, frame_sequence=n, sensor_timestamp_ns=n, image=n,
                           metadata={})

    fast._push(at(0))
    fast._push(at(1))
    before = fast.dropped_stale
    fast._push(at(2))
    assert len(fast._queue) == 1, f"有界缓冲长出了第二个槽位：{len(fast._queue)}"
    assert fast.dropped_stale == before + 1, "丢旧帧没被计上，UI 就会把积压说成健康"
    assert fast.latest().frame_sequence == 2, "最新一帧必须是最新的那帧"
    checks += 1

    with tempfile.TemporaryDirectory() as box:
        target = os.path.join(box, "video_streams.json")
        preview = types.SimpleNamespace(path=os.path.join(box, "detail.jpg"), published=7,
                                        failures=0)
        quiet = SecondaryCameraStream(role="detail", ident=ident, poll=lambda: None)
        announcer = DetailStreamAnnouncer(path=target, stream=quiet, preview=preview,
                                          size=(1920, 1080))
        publish(path=target, descriptors=[StreamDescriptor(
            role="wide", camera_id="cam-baa28c2a", identity_source="by-path", durable=True,
            path=os.path.join(box, "preview.jpg"), delivered_fps=30.01, updated_ns=1)])
        announcer.publish_once()
        back = StreamManifest_read(target)
        assert set(back) == {"wide", "detail"}, f"detail 把 wide 挤掉了：{sorted(back)}"
        assert back["detail"].delivered_fps is None, "第一个窗口没测到，不能写 0"
        assert back["wide"].delivered_fps == 30.01, "另一路的实测值必须原样留着"
        checks += 1

    try:
        SecondaryCameraStream(role="left", ident=ident, poll=poll)
    except ValueError as exc:
        assert "left" in str(exc)
    else:
        raise AssertionError("新角色名必须被点名拒绝")
    checks += 1
    print(f"detail stream selftest: {checks} checks passed（不碰 /dev，不碰真相机）")
    return 0


if __name__ == "__main__":
    raise SystemExit(_selftest())
