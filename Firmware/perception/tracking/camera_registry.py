"""One tracker per camera, and one camera's bad day stays its own.

``PerceptionPipeline`` carries the tracking state inside the instance, so "per-camera tracker" is
not a new abstraction to invent here -- it is one pipeline instance per camera, keyed by the
*durable* camera id. Keying by ``/dev/videoN`` would reintroduce exactly what ``camera_id.py`` was
written to remove: a re-enumeration would silently hand a camera's accumulated tracks to a
different sensor.

Two behaviours are deliberate and should not be "optimised" away:

* A tracker that raises is isolated: the sibling keeps running, and the failed camera keeps
  reporting its **last known** result with a ``stale`` flag. Reporting "no targets" because the
  tracker threw is a lie with consequences -- the turret would treat a broken sensor as an empty
  room and could swing toward something it can no longer see.
* Results are keyed to the worker generation. A result from a retired owner is refused, not
  merged: merging is how a restarted camera's first frame lands on top of a track the new owner
  has never observed.
"""
from __future__ import annotations

from types import SimpleNamespace
from typing import Any, Callable, Dict, Optional

from perception.camera_id import CameraId


class CameraTracker:
    """One camera's tracker: owns the pipeline instance, the last known result, and the counters."""

    def __init__(self, camera_id: CameraId, factory: Callable[[], Any]) -> None:
        self.camera_id = camera_id
        self.result: Any = None
        self.state = "created"          # created | tracking | degraded
        self.last_error = ""
        self.errors = 0
        self.stale_results = 0
        self._factory = factory
        self._pipeline: Optional[Any] = None

    def _ensure_pipeline(self) -> Any:
        if self._pipeline is None:
            self._pipeline = self._factory()
        return self._pipeline

    def process(self, frame: Any, generation: int,
                expect_generation: Optional[int] = None) -> bool:
        """Route one frame. Returns False when the frame was refused rather than processed."""
        if expect_generation is not None and generation != expect_generation:
            self.stale_results += 1
            return False
        try:
            self.result = self._ensure_pipeline().process_frame(frame)
        except Exception as exc:                                    # noqa: BLE001
            # Keep the last known result: see the module docstring.
            self.errors += 1
            self.state = "degraded"
            self.last_error = f"{type(exc).__name__}: {exc}"
            return False
        self.state = "tracking"
        self.last_error = ""
        return True

    def snapshot(self) -> Dict[str, Any]:
        return {"state": self.state, "generation_stale": self.stale_results,
                "errors": self.errors, "error": self.last_error,
                "has_result": self.result is not None}


class CameraTrackRegistry:
    """Routes frames to the right tracker and answers the per-camera health question."""

    def __init__(self, factory: Callable[[], Any]) -> None:
        self._factory = factory
        self._trackers: Dict[str, CameraTracker] = {}

    def tracker_for(self, camera_id: CameraId) -> CameraTracker:
        key = camera_id.id
        if key not in self._trackers:
            self._trackers[key] = CameraTracker(camera_id, self._factory)
        return self._trackers[key]

    def ids(self):
        return sorted(self._trackers)

    def process(self, camera_id: CameraId, frame: Any, generation: int,
                expect_generation: Optional[int] = None) -> bool:
        return self.tracker_for(camera_id).process(frame, generation, expect_generation)

    def status(self) -> Dict[str, Dict[str, Any]]:
        return {key: tracker.snapshot() for key, tracker in self._trackers.items()}


def selftest() -> int:
    """A3's isolation properties at the tracking layer, with mock pipelines."""
    from perception.camera_id import derive_camera_id

    wide = derive_camera_id("/dev/v4l/by-path/platform-wide-capture-video0")
    detail = derive_camera_id("/dev/v4l/by-path/platform-detail-capture-video1")
    checks = []

    built = []

    def make_pipeline():
        obj = {"frames": 0}

        def process_frame(frame):
            obj["frames"] += 1
            return {"n": obj["frames"], "frame": frame}

        built.append(obj)
        return SimpleNamespace(process_frame=process_frame)

    mgr = CameraTrackRegistry(make_pipeline)
    checks.append(("a tracker is created per durable camera id",
                   mgr.process(wide, "f1", 1) and mgr.process(detail, "f2", 1)))
    checks.append(("two paths for one camera are one tracker, not two",
                   mgr.tracker_for(wide) is mgr.tracker_for(
                       derive_camera_id("/dev/v4l/by-path/platform-wide-capture-video0"))))
    checks.append(("each tracker keeps its own state", len(mgr.ids()) == 2))

    # One camera's failure must not touch the sibling, and must not erase what we last knew.
    def explode_pipeline():
        # Counted per pipeline instance, not globally: a shared counter would make the wide
        # camera fail because the detail camera ran, which is the very coupling under test.
        calls = [0]

        def process_frame(frame):
            calls[0] += 1
            if calls[0] > 1:
                raise RuntimeError("detail model output is not finite")
            return {"n": 1}
        return SimpleNamespace(process_frame=process_frame)

    mgr2 = CameraTrackRegistry(explode_pipeline)
    mgr2.process(detail, "a", 1)
    before = mgr2.tracker_for(detail).result
    mgr2.process(detail, "b", 1)
    mgr2.process(wide, "c", 1)
    checks.append(("a throwing tracker is degraded, not gone",
                   mgr2.tracker_for(detail).state == "degraded"))
    checks.append(("a failed tracker keeps its last known result instead of claiming an empty room",
                   mgr2.tracker_for(detail).result == before and before is not None))
    checks.append(("the sibling keeps tracking", mgr2.tracker_for(wide).state == "tracking"))
    checks.append(("the failure is named", "not finite" in mgr2.tracker_for(detail).last_error))

    # A retired generation's result is refused, and the refusal is counted.
    mgr3 = CameraTrackRegistry(make_pipeline)
    mgr3.tracker_for(wide).result = {"n": "previous-owner"}
    accepted = mgr3.process(wide, "late-frame", 1, expect_generation=2)
    checks.append(("a frame from a retired owner is refused", not accepted))
    checks.append(("the refusal is counted rather than swallowed",
                   mgr3.tracker_for(wide).snapshot()["generation_stale"] == 1))
    checks.append(("a refused frame leaves the last known result alone",
                   mgr3.tracker_for(wide).result == {"n": "previous-owner"}))

    failed = [name for name, ok in checks if not ok]
    for name, ok in checks:
        print(("  ok   " if ok else "  FAIL ") + name)
    print("camera_registry selftest: %d/%d passed" % (len(checks) - len(failed), len(checks)))
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(selftest())
