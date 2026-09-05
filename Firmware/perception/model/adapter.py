"""``ModelAdapter`` — the one interface between a neural network and the pipeline.

Everything below this line in the perception subsystem consumes ``DetectionSet`` and knows
nothing about which network produced it. That boundary is what makes §7's bake-off possible
without touching the tracker, and what makes §52's offline replay an honest test rather than a
simulation of the pipeline: a replay adapter and a hardware adapter are interchangeable if the
contract is the same, and the contract is ``DetectionSet`` plus §14's geometry.

Two rules live here rather than in each adapter:

**``model_generation`` is bumped by the adapter, not by whoever notices.** §33 uses it to mean
"the score semantics changed underneath the tracker". If a reload could happen without the
counter moving, a tentative identity's threshold history would silently describe a different
model, and the thresholds that confirmed it would no longer be the thresholds in force.

**An adapter that cannot produce a valid ``DetectionSet`` must produce a *counted* failure.**
``infer`` either returns a validated set or raises; returning an empty set on a dropped frame
would look identical to an empty room, and §40's timing report would keep looking healthy while
the station saw nothing (§26's "report, don't absorb", applied to the input stage).
"""
from __future__ import annotations

import os
import time

from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

from ..config import ModelConfig, VisionConfig
from ..detection.normalize import InferenceGeometry, normalize_rows
from ..detection.types import DetectionSet
from ..errors import ConfigError, ModelRejected
from .manifest import ModelManifest


class ModelAdapter:
    """Base contract: turn one captured frame into a validated ``DetectionSet``."""

    name = "abstract"

    def __init__(self, manifest: ModelManifest, *, generation: int = 1) -> None:
        self.manifest = manifest
        self.generation = int(generation)
        self.opened = False
        self.inferences = 0
        self.failures = 0
        self._stream: Tuple[int, int] = (0, 0)
        #: Stage timings only the adapter can see (§40). Empty until the first inference: the
        #: pipeline records what exists and says what is missing, rather than inventing zeros.
        self.last_timings_ms: Dict[str, float] = {}
        #: Time spent reading the model's output tensors, 0.0 meaning "this adapter has no
        #: read to time" (a mock does not wait on an NPU). Not a measurement of zero.
        self.last_read_ms = 0.0
        self._roi: Optional[Tuple[int, int, int, int]] = None

    # -- lifecycle ----------------------------------------------------------
    def configure_stream(self, width: int, height: int,
                         roi: Optional[Tuple[int, int, int, int]] = None) -> None:
        """Tell the adapter the size of the stream it will be shown (§14 needs both sizes)."""
        if min(int(width), int(height)) <= 0:
            raise ConfigError(f"stream size must be positive, got {width}x{height}")
        self._stream = (int(width), int(height))
        self._roi = roi

    @property
    def stream_size(self) -> Tuple[int, int]:
        return self._stream

    def geometry(self) -> InferenceGeometry:
        if min(self._stream) <= 0:
            raise ConfigError("adapter used before configure_stream(): §14 cannot map "
                              "model coordinates onto a stream of unknown size")
        return self.manifest.geometry(self._stream[0], self._stream[1], self._roi)

    def open(self) -> None:
        """Acquire the device/model. Subclasses raise ``ModelRejected`` on refusal."""
        raise NotImplementedError

    def close(self) -> None:
        self.opened = False

    def __enter__(self) -> "ModelAdapter":
        self.open()
        return self

    def __exit__(self, *_exc: Any) -> None:
        self.close()

    # -- inference ----------------------------------------------------------
    def infer(self, image: Any, metadata: Optional[Any] = None, *,
              frame_sequence: int, sensor_timestamp_ns: int,
              publish_timestamp_ns: int) -> DetectionSet:
        raise NotImplementedError

    def note_model_generation(self, generation: int) -> None:
        """Record a reload performed elsewhere (§33)."""
        self.generation = int(generation)

    # -- reporting ----------------------------------------------------------
    def describe(self) -> Dict[str, Any]:
        return {"adapter": self.name, "model_id": self.manifest.model_id,
                "model_generation": int(self.generation),
                "task": self.manifest.task,
                "input_size": [self.manifest.input_width, self.manifest.input_height],
                "postprocess": self.manifest.postprocess,
                "stream": list(self._stream), "opened": bool(self.opened),
                "inferences": int(self.inferences), "failures": int(self.failures)}

    def _rows_to_set(self, rows: Sequence[Sequence[float]], *, frame_sequence: int,
                     sensor_timestamp_ns: int, publish_timestamp_ns: int,
                     anchor_cfg: Any = None) -> DetectionSet:
        """The single normalization path (§14, §9.2's oracle compares exactly this output).

        It also owns the one timing the pipeline cannot measure itself: §40 asks for
        ``model_output_parse`` and ``coordinate_normalization`` as separate stages, and both
        happen in here. An adapter that reported neither would force the pipeline to publish a
        zero — and a zero in a percentile report is indistinguishable from a stage that is fast.
        """
        score_index, class_index, box_index = self.manifest.score_indices()
        started = time.monotonic_ns()
        out = normalize_rows(
            rows, geometry=self.geometry(), model_id=self.manifest.model_id,
            model_generation=self.generation, frame_sequence=int(frame_sequence),
            sensor_timestamp_ns=int(sensor_timestamp_ns),
            publish_timestamp_ns=int(publish_timestamp_ns),
            label_map=self.manifest.label_map(), score_index=score_index,
            class_index=class_index, box_index=box_index, anchor_cfg=anchor_cfg)
        elapsed = (time.monotonic_ns() - started) / 1_000_000.0
        timings = {"coordinate_normalization_ms": round(elapsed, 6)}
        if self.last_read_ms > 0.0:
            # Only the on-sensor path can time the read itself, and `model_output_parse_ms` for
            # it means "time in get_outputs()" — the row→Detection parsing happens inside
            # normalize_rows and is reported with the normalization stage above. Reported only
            # when a read was actually timed: a mock has no read to measure, and 0.0 in §40's
            # table would claim a free stage where there is simply no number (§40).
            timings["model_output_parse_ms"] = float(self.last_read_ms)
        self.last_timings_ms = timings
        return out


class MockAdapter(ModelAdapter):
    """A scripted adapter: rows per frame, through the real normalization path.

    Used by the offline tests, the replay source, and §52's determinism checks. It is not a
    stand-in for "good enough to ship": it exists so that the tracker, the selector and the
    record/replay path can be tested with known inputs on a machine with no camera attached
    (§55.18's offline acceptance requirement).
    """

    name = "mock"

    def __init__(self, manifest: ModelManifest,
                 rows_by_frame: Optional[Any] = None, *,
                 default_rows: Optional[Sequence[Sequence[float]]] = None,
                 generation: int = 1) -> None:
        super().__init__(manifest, generation=generation)
        #: Either a mapping of frame_sequence → rows, or a callable(frame_sequence) → rows.
        self.rows_by_frame: Any = rows_by_frame or {}
        self.default_rows = list(default_rows or [])
        self.requested_frames: List[int] = []

    def open(self) -> None:
        self.manifest.validate()
        self.opened = True

    def rows_for(self, frame_sequence: int) -> Sequence[Sequence[float]]:
        source = self.rows_by_frame
        if callable(source):
            return list(source(frame_sequence)) or []
        if frame_sequence in source:
            return list(source[frame_sequence])
        return list(self.default_rows)

    def infer(self, image: Any, metadata: Optional[Any] = None, *, frame_sequence: int,
              sensor_timestamp_ns: int, publish_timestamp_ns: int) -> DetectionSet:
        if not self.opened:
            raise ModelRejected("MockAdapter.infer() before open()")
        self.requested_frames.append(int(frame_sequence))
        self.inferences += 1
        return self._rows_to_set(self.rows_for(int(frame_sequence)),
                                 frame_sequence=int(frame_sequence),
                                 sensor_timestamp_ns=int(sensor_timestamp_ns),
                                 publish_timestamp_ns=int(publish_timestamp_ns))


def manifest_for(config: VisionConfig, profile: Optional[str] = None) -> ModelManifest:
    """Build the manifest a profile describes, reading the checked-in file when named (§9.3).

    **A manifest file is authoritative for what the model *is***: task, input dimensions, box
    order, box normalization, labels, nominal rate, hash. The configuration may relocate the
    ``.rpk`` (a deployment fact) and choose ``permitted_classes`` (a policy decision), and
    nothing else.

    The reason is a bug class this subsystem is full of: ``ModelConfig``'s fields have
    defaults, so an overlay from the config cannot tell "the engineer wrote 640" from "nobody
    wrote anything and the dataclass said 640". Overlaying would let a stale config silently
    rewrite a reviewed artefact — and the one thing §9.3 needs is an artefact that cannot be
    rewritten by whoever last edited a different file. Where the config does disagree with the
    file, the ignored value is recorded in the manifest's notes rather than dropped.
    """
    model: ModelConfig = config.model_for(profile or config.profile)
    if model.manifest_path:
        manifest = ModelManifest.from_file(resolve_artifact(model.manifest_path,
                                                           config.source_path))
        ignored = [
            f"config models.{model.profile_name}.{attribute}="
            f"{getattr(model, attribute, None)!r} ignored: {manifest.source_path or 'manifest'} "
            f"says {getattr(manifest, attribute, None)!r}"
            for attribute in ("model_id", "task", "input_width", "input_height",
                              "bbox_order", "postprocess", "inference_rate_hz")
            if attribute == "task"
            or (attribute != "task"
                and getattr(manifest, attribute, None) != getattr(model, attribute, None)
                and not (attribute == "model_id" and model.model_id == model.profile_name))
        ]
        overlay: Dict[str, Any] = {"permitted_classes": tuple(model.permitted_classes)}
        if model.path:
            overlay["path"] = model.path
        if model.sha256:
            overlay["sha256"] = model.sha256
        manifest = manifest.with_runtime(**overlay)
        if ignored:
            manifest = manifest.with_runtime(
                notes=(manifest.notes + "\n" if manifest.notes else "")
                + "; ".join(dict.fromkeys(ignored)))
        return manifest

    manifest = ModelManifest(
        model_id=model.model_id or model.profile_name, task=model.task,
        input_width=model.input_width, input_height=model.input_height,
        bbox_order=model.bbox_order, bbox_normalized=model.bbox_normalized,
        postprocess=model.postprocess, inference_rate_hz=float(model.inference_rate_hz),
        labels=model.labels, permitted_classes=tuple(model.permitted_classes),
        path=model.path, sha256=model.sha256, license=model.license)
    return manifest


def resolve_artifact(path: str, source_path: str = "") -> str:
    """Find a shipped artefact named by a config value, without depending on the cwd.

    systemd starts the daemon with some working directory it was configured with, and a
    relative ``manifest_path`` resolved against *that* either fails obscurely or — worse,
    because it is silent — picks up a different file left in the launch directory. Relative
    names are therefore tried against the package directory first (the artefacts ship inside
    ``perception/``) and then next to the config file that named them.
    """
    if not path or os.path.isabs(path):
        return path
    package_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    candidates = [
        os.path.join(package_root, path),                     # relative to perception/
        os.path.join(os.path.dirname(package_root), path),    # relative to Firmware/
    ]
    if source_path:
        candidates.append(os.path.join(os.path.dirname(os.path.abspath(source_path)), path))
    candidates.append(path)                                   # finally, the cwd's opinion
    for candidate in candidates:
        if os.path.exists(candidate):
            return candidate
    return path


#: Adapter kinds that need no sensor and no artifact. A single tuple, so that whatever
#: §50 treats as "offline" cannot drift from whatever `build_adapter` is willing to build: the
#: drift would show up as a mock profile being refused for a missing .rpk, or — worse — a
#: station profile being waived.
OFFLINE_ADAPTERS: Tuple[str, ...] = ("mock", "replay", "offline")


def offline_scene_rows(frame_sequence: int) -> List[List[float]]:
    """The default offline scene: one person crossing, a second entering at frame 10.

    Small, deterministic, and shaped to touch the behaviours the offline suite claims to cover
    — a moving identity that must stay one identity, and a second identity arriving while the
    first is live (which is what makes §25's duplicate logic and §27's labels observable at
    all). Coordinates are fractions of the network input, matching §9.3's
    ``bbox_normalized: true``.
    """
    rows: List[List[float]] = [[0.85, 0.0, 0.30 + 0.006 * frame_sequence, 0.30,
                               0.36 + 0.006 * frame_sequence, 0.72]]
    if frame_sequence >= 10:
        rows.append([0.70, 0.0, 0.72, 0.34, 0.77, 0.70])
    # Not at frame 0, even though 0 % 37 == 0: in the first frames no identity exists yet, so a
    # duplicate arriving there exercises §25's creation guard rather than §16's NMS, and — with
    # §16 uncommissioned, as the shipped profile is — it mints an identity that dies two frames
    # later and leaves the run's first published label #2. A fixture whose labels start at two
    # makes §27 harder to read than it is.
    if frame_sequence and frame_sequence % 37 == 0:
        # A near-duplicate: a few thousandths of the frame away from the row above, which is what
        # a second detector head or a second anchor actually produces. It has to be *tight*, not
        # merely overlapping — the point is that §16's NMS eats it, and a loose box would test
        # the tracker's duplicate handling instead of the stage under observation.
        shift = 0.006 * frame_sequence
        rows.append([0.66, 0.0, 0.304 + shift, 0.305, 0.364 + shift, 0.715])
    return rows


def build_adapter(config: VisionConfig, *, profile: Optional[str] = None,
                  manifest: Optional[ModelManifest] = None,
                  mock_rows: Optional[Any] = None,
                  imx500_factory: Optional[Callable[[str], Any]] = None) -> ModelAdapter:
    """Construct the adapter a profile names (§5: one model owns inference at a time)."""
    from .imx500_yolo import Imx500YoloAdapter

    model: ModelConfig = config.model_for(profile or config.profile)
    manifest = manifest or manifest_for(config, profile or config.profile)
    kind = (model.adapter or "").strip().lower()
    if kind in OFFLINE_ADAPTERS:
        # An unscripted mock still has to produce *something*: a daemon whose mock profile
        # emits empty frames records an empty dataset and reports green metrics. The default
        # scene below is fixed by definition, so it is also a determinism fixture (§52).
        return MockAdapter(manifest, mock_rows if mock_rows is not None
                           else offline_scene_rows)
    if kind in ("imx500", "imx500_yolo", "imx500_yolo_pp"):
        return Imx500YoloAdapter(manifest, roi=None, imx500_factory=imx500_factory,
                                 anchor_cfg=config.anchor)
    raise ConfigError(
        f"unknown adapter {model.adapter!r} for profile {model.profile_name!r}. "
        f"known: mock, imx500. A typo here must stop startup: an adapter chosen by "
        f"fallback would silently change what 'detection' means.")


def rows_for_moving_target(rows: int = 3, *, normalized: bool = True) -> List[List[float]]:
    """A deterministic synthetic frame: ``rows`` person boxes, evenly spaced.

    The coordinate unit is the point of this helper. §9.3's manifest declares
    ``bbox_normalized: true`` for the on-sensor YOLO11n baseline, so that is what the default
    emits — fractions of the network input. A fixture that emitted pixels against that flag
    would be rejected by §14's validity check as out-of-range fractions, which is the check
    doing its job, and would look like a broken helper.
    """
    output: List[List[float]] = []
    for index in range(rows):
        centre = (index + 1) / (rows + 1)
        half_w, half_h = 0.05, 0.15
        if not normalized:                       # same box, as pixels of a 640x640 input
            centre, half_w, half_h = centre * 640.0, 32.0, 96.0
        y_centre = 0.5 if normalized else 320.0
        output.append([round(0.9 - 0.1 * index, 6), 0,
                       centre - half_w, y_centre - half_h,
                       centre + half_w, y_centre + half_h])
    return output
