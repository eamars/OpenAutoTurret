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

import copy
import time
from types import SimpleNamespace
from typing import Any, Callable, Dict, Optional

from perception.camera_id import CameraId
from perception.protocol.track_set import TrackSet, TrackSetCounters


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


class MergedTrackSetView:
    """The newest TrackSet from each camera, merged on demand with a freshness budget.

    The wide camera's loop is the only publisher, so a document appears at the wide camera's cadence
    and each camera's contribution is whatever it last said. That is the honest shape of a two-camera
    system with one publisher -- but it needs two rules, both enforced here:

    * a contribution is only believed while it is young (``max_age_ns``), and a dropped one is named;
    * ``merge()`` never returns ``None`` after the first offer. "There is no document yet" and
      "this document is empty" are different claims, and the caller has to be able to tell them apart.
    """

    def __init__(self, *, max_age_ns: int,
                 clock: Callable[[], int] = time.monotonic_ns) -> None:
        if int(max_age_ns) <= 0:
            raise ValueError("a merge with no freshness budget would happily publish a box from "
                             "whenever the camera last felt like speaking")
        self.max_age_ns = int(max_age_ns)
        self._clock = clock
        self._latest: Dict[str, Any] = {}
        self.offers = 0
        self.dropped_stale = 0
        self.refusals = 0
        self.last_refusal = ""

    def offer(self, track_set: Any) -> bool:
        """Take one camera's newest document. An unattributed one is refused by name."""
        from ..errors import ValidationError
        cid = str(getattr(track_set, "camera_id", "") or "")
        if not cid:
            self.refusals += 1
            self.last_refusal = "a TrackSet reached the merge view without a camera_id"
            return False
        self._latest[cid] = track_set
        self.offers += 1
        return True

    def merge(self):
        """The merged document, or ``None`` while nobody has spoken yet."""
        from ..errors import ValidationError
        if not self._latest:
            return None
        try:
            merged = merge_track_sets(list(self._latest.values()), max_age_ns=self.max_age_ns,
                                      now_ns=int(self._clock()))
        except ValidationError as exc:
            # The view must not end the publisher's frame over a stale set: the frame that could
            # not be merged is reported, counted, and the previous document simply stays published.
            self.refusals += 1
            self.last_refusal = f"{type(exc).__name__}: {exc}"
            self.dropped_stale += 1
            return None
        self.dropped_stale += len(merged.stale_sources)
        return merged

    def stats(self) -> Dict[str, Dict[str, Any]]:
        now = int(self._clock())
        return {cid: {"age_ms": round((now - int(s.publish_timestamp_ns)) / 1e6, 3),
                      "tracks": len(s.tracks),
                      "stale": (now - int(s.publish_timestamp_ns)) > self.max_age_ns}
                for cid, s in self._latest.items()}


def merge_track_sets(sets, *, max_age_ns: Optional[int] = None,
                     now_ns: Optional[int] = None):
    """One document out of several cameras' TrackSets, with the refusals written down.

    Merging is a *union of measurements*, not a reconciliation: each tracker owns its own ids and
    its own lifecycle, and nothing here decides that the person in the narrow view and the person
    in the wide view are the same human. That question needs a cross-camera identity model and the
    calibration to support it; pretending it here would silently invent a person.

    What this function will not do is hand back a document whose meaning is unclear:

    * every set must say which camera it came from, and no camera may appear twice -- two sets from
      one sensor means a frame was routed twice, which is a bug and not something to average;
    * all sets must declare the same picture geometry, because a merged set declares exactly one.
      Two legs declaring different pictures need a mapping, and a mapping is calibration;
    * track uuids must be unique across the merge. A collision would mean two identities in one
      document, which every consumer that looks up ``by_uuid`` would silently halve.

    Counters are summed, and the stamps come from the newest set: an older document's sequence
    number is not a lie about the newer one.
    """
    from ..errors import ValidationError

    given = [s for s in (sets or ()) if s is not None]
    if not given:
        raise ValidationError("merge_track_sets([]) would publish an empty set that means "
                              "'nothing was measured', which is not the same claim")
    stale_sources: Tuple[str, ...] = ()
    if max_age_ns is not None:
        # A camera that has not published inside the budget is not silent because the room is
        # empty; it is silent because it is not there any more. Its boxes are a memory, and a
        # memory in a live document makes the control loop turn toward something nobody is
        # measuring. So: dropped from the tracks, and named, so "stopped contributing" never
        # reads as "sees nobody".
        now = int(now_ns if now_ns is not None else time.monotonic_ns())
        fresh, stale = [], []
        for s in given:
            age = now - int(s.publish_timestamp_ns)
            (fresh if age <= int(max_age_ns) else stale).append(s)
        if not fresh:
            raise ValidationError(
                f"every camera's newest TrackSet was older than {int(max_age_ns)} ns "
                f"(oldest ages: {sorted(now - int(s.publish_timestamp_ns) for s in given)}); "
                "publishing an all-stale merge would claim a picture nobody has taken")
        given, stale_sources = fresh, tuple(sorted(str(s.camera_id) for s in stale))
    seen = {}
    for s in given:
        cid = str(getattr(s, "camera_id", "") or "")
        if not cid:
            raise ValidationError("a TrackSet reached the merge without a camera_id: an "
                                  "unattributed box cannot be placed in a room with two cameras")
        if cid in seen:
            raise ValidationError(f"two TrackSets from camera {cid} reached the merge; one sensor "
                                  "contributing twice is a routing bug, not a second opinion")
        seen[cid] = s
    geometries = {(int(s.stream_width), int(s.stream_height)) for s in given}
    if len(geometries) > 1:
        raise ValidationError(
            f"the merge declares one picture but the sets declare {sorted(geometries)}; "
            "cross-camera boxes need a mapping, and a mapping has to be measured, not assumed")
    uuids = {}
    for s in given:
        for t in s.tracks:
            if t.track_uuid in uuids:
                raise ValidationError(
                    f"track {t.track_uuid} arrived from both {uuids[t.track_uuid]} and "
                    f"{s.camera_id}; one document cannot hold an identity twice")
            uuids[t.track_uuid] = s.camera_id

    newest = max(given, key=lambda s: int(s.publish_timestamp_ns))
    merged = copy.deepcopy(newest)
    merged.tracks = [t for s in given for t in s.tracks]
    # Attribution is stamped per source, so a merged document says which optic each box belongs to
    # rather than only that several optics are involved somewhere in it.
    for s in given:
        for t in s.tracks:
            t.camera_id = s.camera_id
    merged.camera_id = ""
    merged.source_cameras = tuple(sorted(seen))
    merged.stale_sources = stale_sources
    total = {}
    for s in given:
        for key, value in s.counters.to_dict().items():
            total[key] = total.get(key, 0) + int(value)
    merged.counters = TrackSetCounters.from_dict(total)
    merged.events = [e for s in given for e in s.events]
    # Sorted so a reader can attribute a box without opening two documents: the camera id travels
    # on the track's own record in the published dictionary, which is what the web layer reads.
    return merged


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

    # -- merge_track_sets: one document, two optics, no invented person ----
    from perception.errors import ValidationError
    from perception.protocol.track_set import TrackSet, TrackSetCounters
    from perception.tracking.track import Track

    def made(cid, uuids, *, width=1920, height=1080, publish=1, created=0, confirmed=0):
        set_ = TrackSet(session_uuid=cid, stream_width=width, stream_height=height,
                        publish_timestamp_ns=publish,
                        tracks=[Track(track_uuid=u) for u in uuids])
        set_.stamp_camera(cid)
        set_.counters = TrackSetCounters(tracks_created=created, tracks_confirmed=confirmed,
                                        detections_in=len(uuids))
        return set_

    wide_set = made(wide.id, ("w1", "w2"), publish=10, created=2, confirmed=1)
    detail_set = made(detail.id, ("d1",), publish=20, created=1, confirmed=1)
    merged = merge_track_sets([wide_set, detail_set])
    checks.append(("a merge is the union of both cameras' tracks",
                   sorted(t.track_uuid for t in merged.tracks) == ["d1", "w1", "w2"]))
    checks.append(("every merged track says which camera it came from",
                   {t.track_uuid: t.camera_id for t in merged.tracks}
                   == {"w1": wide.id, "w2": wide.id, "d1": detail.id}))
    checks.append(("the merged document names its sources and claims no single camera",
                   merged.source_cameras == tuple(sorted([wide.id, detail.id]))
                   and merged.camera_id == ""))
    checks.append(("counters are summed, not averaged into meaninglessness",
                   merged.counters.tracks_created == 3 and merged.counters.tracks_confirmed == 2
                   and merged.counters.detections_in == 3))
    checks.append(("the stamps come from the newer set", merged.publish_timestamp_ns == 20))
    checks.append(("the declared geometry survives the merge",
                   (merged.stream_width, merged.stream_height) == (1920, 1080)))

    def refused(build, says):
        try:
            merge_track_sets(build())
        except ValidationError as exc:
            return says in str(exc)
        except Exception:                                                 # noqa: BLE001
            return False
        return False

    checks.append(("a set that does not say its camera is refused, not guessed",
                   refused(lambda: [TrackSet(stream_width=1920, stream_height=1080)],
                           "camera_id")))
    checks.append(("one camera contributing twice is refused as a routing bug",
                   refused(lambda: [wide_set, made(wide.id, ("x",))], "routing bug")))
    checks.append(("sets declaring different pictures are refused, not rescaled",
                   refused(lambda: [wide_set, made(detail.id, ("d9",), width=1280, height=720)],
                           "one picture")))
    checks.append(("an identity appearing in two documents is refused, not halved",
                   refused(lambda: [wide_set, made(detail.id, ("w1",))], "cannot hold")))
    checks.append(("merging nothing is a refusal, not an empty set",
                   refused(lambda: [], "nothing was measured")))

    # -- the freshness gate: a memory is not a sighting ---------------------
    clock = {"now": 1_000_000_000_000}
    fresh_set = made(wide.id, ("w1",), publish=999_999_000_000)
    old_set = made(detail.id, ("d1",), publish=999_000_000_000)      # 1000 ms old
    gated = merge_track_sets([fresh_set, old_set], max_age_ns=150_000_000, now_ns=clock["now"])
    checks.append(("a stale contribution is left out of the tracks",
                   [t.track_uuid for t in gated.tracks] == ["w1"]))
    checks.append(("but it is named as stale, not silently missing",
                   gated.stale_sources == (detail.id,)))
    checks.append(("a merge where nothing is fresh is refused with the ages",
                   refused(lambda: merge_track_sets([old_set], max_age_ns=1_000_000,
                                                    now_ns=clock["now"]),
                           "every camera's newest TrackSet was older")))

    view = MergedTrackSetView(max_age_ns=150_000_000, clock=lambda: clock["now"])
    checks.append(("the view says 'no document yet' instead of publishing an empty one",
                   view.merge() is None))
    checks.append(("an unattributed set is refused by the view and counted",
                   not view.offer(TrackSet(stream_width=1920, stream_height=1080))
                   and view.refusals == 1 and "camera_id" in view.last_refusal))
    view.offer(made(wide.id, ("w1", "w2"), publish=clock["now"]))
    view.offer(made(detail.id, ("d1",), publish=clock["now"]))
    both = view.merge()
    checks.append(("while both are young, the view merges both",
                   sorted(t.track_uuid for t in both.tracks) == ["d1", "w1", "w2"]))
    clock["now"] += 400_000_000                            # only the wide camera keeps talking
    view.offer(made(wide.id, ("w1", "w2"), publish=clock["now"]))
    aged = view.merge()
    checks.append(("when one camera falls silent, the next merge drops it and says so",
                   [t.track_uuid for t in aged.tracks] == ["w1", "w2"]
                   and aged.stale_sources == (detail.id,)))
    checks.append(("the drop is counted where the operator can be pointed to it",
                   view.dropped_stale == 1 and view.stats()[detail.id]["stale"]))
    try:
        MergedTrackSetView(max_age_ns=0)
    except ValueError:
        checks.append(("a merge with no freshness budget is a refusal at construction", True))
    else:
        checks.append(("a merge with no freshness budget is a refusal at construction", False))

    failed = [name for name, ok in checks if not ok]
    for name, ok in checks:
        print(("  ok   " if ok else "  FAIL ") + name)
    print("camera_registry selftest: %d/%d passed" % (len(checks) - len(failed), len(checks)))
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(selftest())
