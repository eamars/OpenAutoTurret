"""The camera owner: §39's "one camera owner", plus §19's insistence on ``SensorTimestamp``.

Two failures this file exists to prevent, both from the handover:

**A second owner of the camera.** §39's diagram has exactly one box above the inference and
preview branches. Picamera2 permits a process to configure a device twice and simply do the
unpredictable thing; here the owner is constructed once, owns the request lifecycle, and hands
out frames and a latest-only preview slot from that single stream.

**Wall clock standing in for the sensor clock.** §19's lifecycle thresholds are intervals on the
*sensor* timeline, and a frame whose metadata has no ``SensorTimestamp`` has no position on that
timeline. Substituting ``time.monotonic_ns()`` at that moment makes every downstream age wrong by
whatever the pipeline latency happens to be that second — which is exactly the shape of bug that
survives a demo and appears on station. So a frame without a sensor stamp is counted and
dropped, and the counter is published.

The Picamera2 calls themselves are one line each and cannot be exercised off-hardware (the
import needs ``libcamera``). Everything *around* them — stamp extraction, stall detection,
request release, preview hand-off — is written against an injected capture object and is
therefore covered by the offline suite.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Optional, Tuple

from .errors import ConfigError
from .events import EventLog, EventType
from .measure import ms_from_ns
from .pipeline import PreviewTap

#: The metadata key that puts a frame on the sensor timeline (§19, §40).
SENSOR_TIMESTAMP_KEY = "SensorTimestamp"


@dataclass
class CapturedFrame:
    """One frame plus the two clocks §40 asks to keep separate."""

    image: Any
    metadata: Any
    sensor_timestamp_ns: int
    metadata_receive_ns: int
    frame_sequence: int
    stream_size: Tuple[int, int] = (0, 0)
    #: Why this frame cannot be used, when it cannot be. Never a silent ``None`` return.
    unusable_reason: str = ""

    @property
    def usable(self) -> bool:
        return not self.unusable_reason


@dataclass
class CameraStats:
    requested: int = 0
    delivered: int = 0
    missing_sensor_timestamp: int = 0
    stalled: int = 0
    last_frame_gap_ms: float = 0.0
    max_gap_ms: float = 0.0
    notes: list = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        return {"requested": self.requested, "delivered": self.delivered,
                "missing_sensor_timestamp": self.missing_sensor_timestamp,
                "stalled": self.stalled,
                "last_frame_gap_ms": round(self.last_frame_gap_ms, 3),
                "max_gap_ms": round(self.max_gap_ms, 3), "notes": list(self.notes)}


class CameraOwner:
    """Single owner of one Picamera2 device, producing ``CapturedFrame``s.

    ``device`` is anything with ``capture_request()`` returning an object with
    ``make_array(stream)``, ``get_metadata()`` and ``release()`` — which is Picamera2's real
    interface, and also what a test provides.

    Stalls are measured between arrivals, so a camera that stops delivering is only observed as
    a gap by the frame that eventually comes through (``capture_request`` blocks otherwise).
    ``stalled`` therefore proves that frames resumed, never that they never stopped; liveness of
    a run that produces nothing at all belongs to the supervisor, not to this class.
    """

    def __init__(self, device: Any, *, stream_size: Tuple[int, int],
                 preview: Optional[PreviewTap] = None,
                 events: Optional[EventLog] = None,
                 stall_timeout_ms: float = 1500.0,
                 clock: Optional[Callable[[], int]] = None,
                 main_stream: str = "main") -> None:
        if min(int(stream_size[0]), int(stream_size[1])) <= 0:
            raise ConfigError(
                f"CameraOwner needs a real stream size, got {stream_size}. The retired code's "
                f"hard-coded 1920x1080 is what §14's coordinate checks exist to catch; asking "
                f"the caller to read it from the intrinsics is cheaper.")
        self.device = device
        self.stream_size = (int(stream_size[0]), int(stream_size[1]))
        self.preview = preview
        self.events = events if events is not None else EventLog()
        self.stall_timeout_ms = float(stall_timeout_ms)
        self.clock = clock or time.monotonic_ns
        self.main_stream = main_stream
        self.stats = CameraStats()
        self.frame_sequence = 0
        self._previous_sensor_ns = 0
        self._previous_receive_ns = 0
        self._closed = False

    # -- frames -------------------------------------------------------------
    def next_frame(self) -> CapturedFrame:
        """Take one frame. Never raises for a missing stamp; reports it instead."""
        self.stats.requested += 1
        receive_ns = int(self.clock())
        gap_ms = ms_from_ns(receive_ns, self._previous_receive_ns) if self._previous_receive_ns \
            else 0.0
        if gap_ms > self.stall_timeout_ms > 0.0:
            self.stats.stalled += 1
            self.events.emit(EventType.CAMERA_FRAME_STALLED, gap_ms=round(gap_ms, 3),
                             threshold_ms=self.stall_timeout_ms)
        if gap_ms:
            self.stats.last_frame_gap_ms = gap_ms
            self.stats.max_gap_ms = max(self.stats.max_gap_ms, gap_ms)
        self._previous_receive_ns = receive_ns

        if self._closed:
            return CapturedFrame(None, None, 0, receive_ns, self.frame_sequence,
                                 self.stream_size, unusable_reason="camera is closed")

        request = self.device.capture_request()
        try:
            image = request.make_array(self.main_stream)
            metadata = request.get_metadata() or {}
            sensor_ns = _sensor_timestamp_ns(metadata)
            if sensor_ns <= 0:
                self.stats.missing_sensor_timestamp += 1
                self.events.emit(EventType.CAMERA_FRAME_STALLED,
                                 reason="metadata has no SensorTimestamp",
                                 keys=sorted(str(key) for key in metadata)[:12])
                # Deliberately not "use now()": see the module docstring.
                return CapturedFrame(None, metadata, 0, receive_ns, self.frame_sequence,
                                     self.stream_size,
                                     unusable_reason="metadata has no SensorTimestamp")
            self.frame_sequence += 1
            self.stats.delivered += 1
            self._previous_sensor_ns = sensor_ns
            if self.preview is not None:
                # §39: one reference into a one-slot buffer. Safe to hand on and to keep after
                # release() below, because Picamera2's make_array() copies the camera buffer
                # (request.py: "we don't want to send out an exported handle to the camera
                # buffer, so we're going to have to do a copy"). A view instead of a copy would
                # have made the encoder's frame a function of when libcamera recycled it.
                self.preview.offer(image, now_ns=receive_ns)
            return CapturedFrame(image, metadata, sensor_ns, receive_ns,
                                 self.frame_sequence, self.stream_size)
        finally:
            # A leaked request pins a buffer, and Picamera2 stops delivering frames once it
            # runs out — which looks exactly like a scene with nothing in it.
            release = getattr(request, "release", None)
            if callable(release):
                try:
                    release()
                except Exception:                               # noqa: BLE001
                    self.stats.notes.append("capture request release() raised")

    def frames(self, *, max_frames: int = 0,
               skip_unusable: bool = True):
        """Iterate frames. An unusable one is yielded unless ``skip_unusable``."""
        delivered = 0
        while True:
            frame = self.next_frame()
            if not frame.usable:
                if skip_unusable:
                    if self._closed:
                        return
                    continue
                yield frame
                continue
            delivered += 1
            yield frame
            if max_frames and delivered >= int(max_frames):
                return

    def start(self) -> None:
        """Start the stream. Everything in ``frames()`` assumes it ran.

        ``configure()`` alone does not make Picamera2 deliver: the first on-device run hung
        silently at ``capture_request()``, because the pipeline had configured the camera but
        never called ``start()`` — which looks exactly like an empty scene, the worst kind of
        zero (a frame counter that adds up but proves nothing). Starting here, where the owner
        can pair it with the existing ``close()``'s ``stop()``, keeps camera lifecycle in one
        place.
        """
        start = getattr(self.device, "start", None)
        if not callable(start):
            raise ConfigError("camera device has no start(): this is not a live Picamera2. "
                              "Mock profiles take the synthetic path and never reach here.")
        start()

    def close(self) -> None:
        self._closed = True
        stop = getattr(self.device, "stop", None)
        if callable(stop):
            try:
                stop()
            except Exception:                                   # noqa: BLE001
                self.stats.notes.append("camera stop() raised")
        close = getattr(self.device, "close", None)
        if callable(close):
            try:
                close()
            except Exception:                                   # noqa: BLE001
                self.stats.notes.append("camera close() raised")


def _sensor_timestamp_ns(metadata: Any) -> int:
    """Pull the sensor stamp out of Picamera2's metadata, tolerating its several shapes.

    Metadata is a dict of control names whose values have moved between libcamera versions:
    sometimes an int of nanoseconds, sometimes a ``datetime`` with nanosecond precision,
    sometimes wrapped in a two-element tuple with a validity flag (§26's "metadata may be a
    little stale"). Each accepted shape is handled explicitly; anything else is *not* guessed
    at, because a wrong clock is not a missing clock.
    """
    if not metadata:
        return 0
    try:
        raw = metadata.get(SENSOR_TIMESTAMP_KEY)
    except AttributeError:
        return 0
    if raw is None:
        return 0
    if isinstance(raw, bool):
        return 0
    if isinstance(raw, int):
        return int(raw)
    if isinstance(raw, float):
        return int(raw)
    if isinstance(raw, (tuple, list)):
        return _sensor_timestamp_ns({SENSOR_TIMESTAMP_KEY: raw[0]}) if raw else 0
    timestamp = getattr(raw, "timestamp", None)
    if isinstance(timestamp, int):
        return timestamp
    iso = getattr(raw, "iso8601", None)
    if isinstance(iso, str):
        from datetime import datetime
        try:
            # libcamera's ISO stamps carry the sensor clock; §19 only needs differences, so
            # the zone it is parsed in does not matter as long as it is applied consistently.
            return int(datetime.fromisoformat(iso.replace("Z", "+00:00")).timestamp()
                       * 1_000_000_000)
        except ValueError:
            return 0
    return 0


def open_picamera2(model_path: str, *, stream_size: Optional[Tuple[int, int]] = None,
                   preview_size: Optional[Tuple[int, int]] = None) -> Tuple[Any, Any, Dict[str, Any]]:
    """Build ``(imx500, picam2, info)`` for a model on this station. Import-guarded.

    The construction order is §9's, from the current upstream example: ask the model first, then
    open the camera it names — ``Picamera2(imx500.camera_num)``. Opening a default camera and
    hoping it is the one holding the network is another retired-code assumption this replaces.
    """
    try:
        from picamera2 import Picamera2                            # type: ignore
        from picamera2.devices.imx500.imx500 import IMX500         # type: ignore
    except Exception as exc:                                      # noqa: BLE001
        raise ConfigError(
            f"cannot import Picamera2/IMX500 on this interpreter ({exc}). The IMX500 path needs "
            f"the station's system Python; use --replay or the mock profile elsewhere.") from exc

    imx500 = IMX500(model_path)
    intrinsics = imx500.network_intrinsics
    if intrinsics is None:
        raise ConfigError(f"{model_path} reports no network_intrinsics (§9.2)")
    input_size = imx500.get_input_size()
    camera_num = getattr(imx500, "camera_num", 0)
    main_size = tuple(stream_size) if stream_size else (int(input_size[0]), int(input_size[1]))
    picam2 = Picamera2(int(camera_num))
    configuration = picam2.create_preview_configuration(
        # Uppercase: this picamera2 build's stream validator is case-sensitive and rejects the
        # lowercase alias, which the first on-device run hit as "Bad format rgb888 in stream main".
        main={"size": main_size, "format": "RGB888"},
        buffer_count=2)
    picam2.configure(configuration)
    info = {"camera_num": int(camera_num), "input_size": (int(input_size[0]),
                                                          int(input_size[1])),
            "stream_size": (int(main_size[0]), int(main_size[1])),
            "task": getattr(intrinsics, "task", None),
            "inference_rate_hz": getattr(intrinsics, "inference_rate", None)}
    return imx500, picam2, info
