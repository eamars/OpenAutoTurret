"""§9.3 — the checked-in model manifest, and §9.3's one rule: fail fast on disagreement.

The manifest is the file that says what the ``.rpk`` on this station actually is: its task, its
input size, its box convention, its labels, its rate. The runtime can ask the sensor most of
those questions through ``network_intrinsics``, and when the two answers differ, **the model
is not the one the configuration believes it is**. Loading it anyway is how a station ends up
with boxes that are 640× scaled, or with class 0 named "person" when the network was retrained
to put "background" there — both of which look like tracking bugs from the outside and are
neither.

§9.2's oracle is the other half: a model is admitted when the upstream reference example works
*and* this subsystem's adapter produces identical normalized detections on a captured sequence.
The manifest is what makes "identical" checkable — without declared conventions there is
nothing to compare against.

Note ``preserve_aspect_ratio``. The IMX500 resizes to its network input; if that resize was a
stretch and this subsystem assumes a letterbox (or the reverse), every box is wrong in a way
that scales with distance from the image centre. §14's synthetic coordinate tests exist because
this class of error is invisible in a demo photo.
"""
from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Any, Dict, List, Mapping, Optional, Tuple

from ..detection.normalize import InferenceGeometry
from ..errors import ValidationError
from ..protocol.jsonio import dumps, load_mapping_document
from .label_maps import LabelMap, resolve as resolve_labels

MANIFEST_SCHEMA_VERSION = 1

#: The IMX500 intrinsics spell tasks with a space (§ Appendix D). Accept both spellings on
#: the way in and normalise on the way through, so a hand-written manifest and the sensor's
#: own report can be compared without one of them being "wrong".
TASK_ALIASES: Dict[str, str] = {
    "object detection": "object_detection",
    "object_detection": "object_detection",
    "pose estimation": "pose_estimation",
    "pose_estimation": "pose_estimation",
    "semantic segmentation": "segmentation",
    "segmentation": "segmentation",
    "classification": "classification",
}
SUPPORTED_TASKS = ("object_detection", "pose_estimation")
POSTPROCESS_MODES = ("on_sensor", "host")
BBOX_ORDERS = ("xy", "yxyx", "cxcywh")


def _notes(value: Any) -> str:
    """A manifest may carry its notes as one string or as a list of lines (§9.3's files do)."""
    if isinstance(value, (list, tuple)):
        return "\n".join(str(line) for line in value)
    return str(value or "")


def normalise_task(value: Any) -> str:
    if value is None:
        return ""
    return TASK_ALIASES.get(str(value).strip().lower(), str(value).strip().lower())


def _layout_index(layout: Mapping[str, Any], key: str, default: int) -> int:
    """Which column of a model output row holds the score, the class, the first box value.

    A *missing* key keeps its documented default, because that is how every manifest written
    before row layouts existed asks for 0/1/2. A key that is present and says ``COMMISSION`` does
    not: that is the probe's draft saying "nobody has looked at the column order yet", and the
    parse would otherwise read a person's confidence out of the box's x_min.
    """
    value = layout.get(key, None)
    if value is None:
        return default
    try:
        return int(value)
    except (TypeError, ValueError):
        raise ValidationError(
            f"row_layout.{key} is {value!r}. A probe-drafted manifest leaves the column order "
            f"unresolved, because no runtime query can see it — open the model's upstream "
            f"example, fill the three indices, and only then load this file (§50)") from None


def _output_tensors(value: Any) -> Any:
    """Validate the parallel-tensor descriptor, or ``None`` when the field is absent.

    A malformed descriptor must not silently degrade to the single-tensor path: that is how a
    model that emits boxes/scores/classes/get-count gets parsed as one row of columns and every
    box comes out as ``[score, class, y0, x0…]``. Refusing is the §9.3 discipline applied to a
    shape the probe cannot see.
    """
    if value is None:
        return None
    if not isinstance(value, Mapping):
        raise ValidationError(f"output_tensors must be an object, got {value!r}")
    return dict(value)


@dataclass
class Disagreement:
    """One mismatch between what the manifest claims and what the sensor reports."""

    field: str
    manifest_value: Any
    runtime_value: Any
    fatal: bool = True
    note: str = ""

    def __str__(self) -> str:
        severity = "fatal" if self.fatal else "warning"
        return (f"{severity}: {self.field}: manifest says {self.manifest_value!r}, "
                f"model reports {self.runtime_value!r}"
                + (f" ({self.note})" if self.note else ""))


@dataclass
class ModelManifest:
    """A model's declared contract. Every field is checked against the runtime (§9.3)."""

    model_id: str = ""
    task: str = "object_detection"
    input_width: int = 0
    input_height: int = 0
    bbox_order: str = "xy"
    bbox_normalized: bool = True
    postprocess: str = "on_sensor"
    inference_rate_hz: float = 0.0
    #: A built-in name (``coco``) or an explicit list. §9.3 requires it either way.
    labels: Any = "coco"
    #: §15: the only classes this model may create tracks for.
    permitted_classes: Tuple[str, ...] = ("person",)
    preserve_aspect_ratio: bool = True

    # Row layout of the on-sensor post-processed output (§13's contract).
    score_index: int = 0
    class_index: int = 1
    box_index: int = 2
    #: Columns per row as declared by the export. 0 means "whatever the tensor turns out to
    #: be", which the adapter still checks at runtime (§9.3's disagreement found late).
    row_columns: int = 0

    #: Descriptor for a model whose on-sensor post-processor emits its result as several
    #: parallel tensors rather than as one row-major ``[score, class, box…]`` matrix. The
    #: IMX500 ``_pp`` zoo does exactly that: a boxes tensor ``(N, 4)``, a scores vector ``(N,)``,
    #: a classes vector ``(N,)`` and a scalar count. ``None`` means the legacy single-tensor
    #: ``row_layout`` applies. Measured on this station 2026-09-05 for ssd/nanodet/efficientdet
    #: ``_pp``; a model that disagrees must say so here rather than be guessed at (§9.3).
    output_tensors: Any = None

    path: str = ""
    sha256: str = ""
    license: str = ""
    notes: str = ""
    source_path: str = ""

    # -- loading ------------------------------------------------------------
    @classmethod
    def from_dict(cls, data: Mapping[str, Any],
                  source_path: str = "") -> "ModelManifest":
        raw = dict(data or {})
        layout = raw.get("row_layout") or {}
        permitted = raw.get("permitted_classes", ("person",))
        if isinstance(permitted, str):
            permitted = [permitted]
        return cls(
            model_id=str(raw.get("model_id", "")),
            task=normalise_task(raw.get("task", "object_detection")),
            input_width=int(raw.get("input_width", 0) or 0),
            input_height=int(raw.get("input_height", 0) or 0),
            bbox_order=str(raw.get("bbox_order", "xy")).strip().lower(),
            bbox_normalized=bool(raw.get("bbox_normalized",
                                         raw.get("bbox_normalization", True))),
            postprocess=str(raw.get("postprocess", "on_sensor")).strip().lower(),
            inference_rate_hz=float(raw.get("inference_rate_hz",
                                            raw.get("inference_rate", 0.0)) or 0.0),
            labels=raw.get("labels", "coco"),
            permitted_classes=tuple(str(name).strip().lower() for name in permitted),
            preserve_aspect_ratio=bool(raw.get("preserve_aspect_ratio", True)),
            score_index=_layout_index(layout, "score_index", 0),
            class_index=_layout_index(layout, "class_index", 1),
            box_index=_layout_index(layout, "box_index", 2),
            row_columns=int(raw.get("row_columns", layout.get("columns", 0)) or 0),
            output_tensors=_output_tensors(raw.get("output_tensors")),
            path=str(raw.get("path", "")),
            sha256=str(raw.get("sha256", "")),
            license=str(raw.get("license", "")),
            notes=_notes(raw.get("notes", "")),
            source_path=source_path)

    @classmethod
    def from_file(cls, path: str) -> "ModelManifest":
        return cls.from_dict(load_mapping_document(path, what="model manifest"),
                             source_path=path)

    # -- validation ---------------------------------------------------------
    def validate(self) -> "ModelManifest":
        problems: List[str] = []
        if not self.model_id:
            problems.append("model_id is required (§9.3)")
        if self.task not in SUPPORTED_TASKS:
            problems.append(
                f"task {self.task!r} is not supported by Vision 1.0 "
                f"(known: {', '.join(SUPPORTED_TASKS)}). A segmentation model's output "
                f"rows are not boxes, and §14's coordinate contract would be violated "
                f"downstream rather than here.")
        if min(self.input_width, self.input_height) <= 0:
            problems.append("input_width/input_height must be positive "
                            "(§14 normalizes against them)")
        if self.bbox_order not in BBOX_ORDERS:
            problems.append(f"bbox_order must be one of {BBOX_ORDERS}, got {self.bbox_order!r}")
        if self.postprocess not in POSTPROCESS_MODES:
            problems.append(f"postprocess must be one of {POSTPROCESS_MODES}")
        if self.inference_rate_hz <= 0.0:
            problems.append("inference_rate_hz must be positive (§40's timing budgets "
                            "and §19's time-based lifecycle both derive from it)")
        try:
            self.label_map()
        except ValueError as exc:
            problems.append(str(exc))
        if not self.permitted_classes:
            problems.append("permitted_classes is empty: §15 would filter out every "
                            "detection, which is a silent way to see nothing")
        indices = {"score_index": self.score_index, "class_index": self.class_index,
                   "box_index": self.box_index}
        if any(value < 0 for value in indices.values()):
            problems.append(f"row layout indices must be non-negative, got {indices}")
        if len(set(indices.values())) != 3:
            # Reading the class id out of the box's x_min is not a crash — it is a model
            # whose every detection is either filtered away or filed under a wrong class.
            problems.append(f"row layout indices must name distinct columns, got {indices}")
        if self.row_columns and self.box_index + 4 > self.row_columns:
            problems.append(f"box_index {self.box_index} leaves fewer than 4 coordinates "
                            f"in a {self.row_columns}-column row")
        if self.output_tensors is not None:
            spec = self.output_tensors
            if spec.get("format") != "imx500_pp_split":
                problems.append(f"output_tensors.format must be 'imx500_pp_split', "
                                f"got {spec.get('format')!r}")
            required = ("boxes", "scores", "classes", "count")
            for key in required:
                if not isinstance(spec.get(key), int) or spec[key] < 0:
                    problems.append(f"output_tensors.{key} must be a non-negative tensor index, "
                                    f"got {spec.get(key)!r}")
            distinct = {spec.get(key) for key in required if isinstance(spec.get(key), int)}
            if len(distinct) != len(required):
                problems.append(f"output_tensors tensors must be distinct, got {sorted(distinct)}")
            if not int(spec.get("max_rows", 0) or 0) > 0:
                problems.append("output_tensors.max_rows must be positive")
            if (int(self.score_index), int(self.class_index), int(self.box_index)) != (0, 1, 2):
                problems.append("split outputs assemble rows as [score, class, box…], so "
                                "score_index/class_index/box_index must be 0/1/2")
        if problems:
            where = f" in {self.source_path}" if self.source_path else ""
            raise ValidationError(f"model manifest {self.model_id or '(unnamed)'} is "
                                  f"not admissible{where}:\n  - " + "\n  - ".join(problems))
        return self

    def commissioning_gaps(self, *, requires_artifact: bool = True) -> List[str]:
        """Fields left as ``COMMISSION``: work items, checked in production mode (§50).

        ``validate()`` checks the manifest is *coherent*; this checks it is *finished*. A
        declared sha256 of "COMMISSION" is coherent and useless — the hash is what proves the
        .rpk on this station is the .rpk this manifest describes, which is the only reason §9.1
        records it.

        ``requires_artifact=False`` waives the artifact questions, because there is nothing to
        hash: a mock adapter reads scripted rows, so "path is empty" is a fact about the test
        rig rather than an open work item. Callers derive the flag from the adapter kind
        (:data:`perception.model.adapter.OFFLINE_ADAPTERS`), never from a profile name.
        """
        gaps: List[str] = []
        for name in ("sha256", "license"):
            value = getattr(self, name, "")
            if "COMMISSION" in str(value).upper():
                gaps.append(f"{name} is still COMMISSION for {self.model_id}")
        if requires_artifact and not self.path:
            gaps.append(f"path is empty for {self.model_id}")
        return gaps

    @property
    def row_width(self) -> int:
        """Minimum columns a row must carry for this layout to be readable at all."""
        return max(self.box_index + 4, self.class_index + 1, self.score_index + 1,
                   self.row_columns)

    # -- derived ------------------------------------------------------------
    def label_map(self) -> LabelMap:
        return resolve_labels(self.labels)

    def geometry(self, stream_width: int, stream_height: int,
                 roi: Optional[Tuple[int, int, int, int]] = None) -> InferenceGeometry:
        if min(self.input_width, self.input_height) <= 0:
            raise ValidationError("cannot build geometry: manifest has no input dimensions")
        return InferenceGeometry(input_width=self.input_width,
                                 input_height=self.input_height,
                                 stream_width=int(stream_width),
                                 stream_height=int(stream_height),
                                 preserve_aspect_ratio=self.preserve_aspect_ratio,
                                 roi=roi,
                                 bbox_normalized=self.bbox_normalized,
                                 bbox_order=self.bbox_order)

    def score_indices(self) -> Tuple[int, int, int]:
        return (self.score_index, self.class_index, self.box_index)

    def uses_split_outputs(self) -> bool:
        """Whether the on-sensor post-processor emits parallel tensors (§13).

        The adapter branches on this: split output is assumed for the IMX500 ``_pp`` zoo, where
        boxes, scores, classes and the valid count are separate tensors. The alternative — trying
        every possible assembly at run-time — is how a garbage detection occasionally looks
        plausible, and plausible is the worst kind of wrong.
        """
        return self.output_tensors is not None

    # -- §9.3 disagreement check -------------------------------------------
    def disagreements_with_intrinsics(self, intrinsics: Mapping[str, Any]) -> List[Disagreement]:
        """Compare this manifest with ``network_intrinsics`` read from the sensor.

        Accepts a mapping or an object with the corresponding properties, because
        ``NetworkIntrinsics`` is a property-bearing class in current picamera2 and a plain
        dict in fixtures and captured manifests.
        """
        def read(key: str, default: Any = None) -> Any:
            if isinstance(intrinsics, Mapping):
                return intrinsics.get(key, default)
            return getattr(intrinsics, key, default)

        found: List[Disagreement] = []

        runtime_task = normalise_task(read("task"))
        if runtime_task and runtime_task != self.task:
            found.append(Disagreement("task", self.task, runtime_task, True,
                                      "the profile's thresholds and anchor policy are "
                                      "for a different task"))

        declared = (self.input_width, self.input_height)
        reported = read("input_size") or read("dims")
        if reported and len(reported) >= 2:
            # ``input_size`` arrives as (w, h); ``dims`` commonly as [channels, h, w].
            # Normalising here means one comparison, not two conventions argued about later.
            if isinstance(read("dims"), (list, tuple)) and len(reported) >= 3:
                shape = (int(reported[-1]), int(reported[-2]))
            else:
                shape = (int(reported[0]), int(reported[1]))
            if shape != declared:
                found.append(Disagreement("input_size", declared, shape, True,
                                          "§14: normalized coordinates are computed "
                                          "against these dimensions"))

        order = read("bbox_order")
        if order and str(order).strip().lower() != self.bbox_order:
            found.append(Disagreement("bbox_order", self.bbox_order, str(order).lower(), True,
                                      "swapped coordinates put a person's box in the "
                                      "wrong half of the frame"))

        normalization = read("bbox_normalization")
        if normalization is not None and bool(normalization) != self.bbox_normalized:
            found.append(Disagreement("bbox_normalized", self.bbox_normalized,
                                      bool(normalization), True,
                                      "a fraction read as pixels is a 640x error"))

        aspect = read("preserve_aspect_ratio")
        if aspect is not None and bool(aspect) != self.preserve_aspect_ratio:
            found.append(Disagreement("preserve_aspect_ratio", self.preserve_aspect_ratio,
                                      bool(aspect), True,
                                      "letterbox and stretch are different mappings; "
                                      "guessing wrong skews boxes away from the centre"))

        labels = read("labels")
        if labels:
            try:
                mine = list(self.label_map().names)
            except ValueError:
                mine = []
            theirs = [str(name) for name in labels]
            if mine and theirs != mine:
                # Name the first index where they part: "80 labels vs 80 labels" is not a
                # finding an operator can act on, "index 3: 'car' vs 'c'" is.
                delta = next((index for index in range(min(len(mine), len(theirs)))
                              if mine[index] != theirs[index]), min(len(mine), len(theirs)))
                fatal = len(theirs) != len(mine) or mine[:1] != theirs[:1]
                found.append(Disagreement(
                    "labels", f"{len(mine)} labels; index {delta} = {mine[delta] if delta < len(mine) else None!r}",
                    f"{len(theirs)} labels; index {delta} = {theirs[delta] if delta < len(theirs) else None!r}",
                    fatal,
                    "class indices name the wrong objects, so §15's person filter would "
                    "select whatever the network calls zero"))

        rate = read("inference_rate") or read("fps")
        if rate and self.inference_rate_hz > 0.0:
            ratio = float(rate) / self.inference_rate_hz
            if not 0.75 <= ratio <= 1.35:
                # Not fatal: the declared figure is a nominal capability and the measured
                # rate legitimately differs. It is still reported, because §40's budgets and
                # §19's timeouts were sized against the declared one.
                found.append(Disagreement("inference_rate_hz", self.inference_rate_hz,
                                          float(rate), False,
                                          "timing budgets were derived from the declared rate"))
        return found

    def require_agreement(self, intrinsics: Mapping[str, Any]) -> List[Disagreement]:
        """Raise on fatal disagreement; return the non-fatal ones for logging."""
        found = self.disagreements_with_intrinsics(intrinsics)
        fatal = [item for item in found if item.fatal]
        if fatal:
            raise ValidationError(
                f"model {self.model_id} disagrees with its manifest (§9.3); refuse to run "
                f"rather than reinterpret its output:\n  - "
                + "\n  - ".join(str(item) for item in fatal))
        return [item for item in found if not item.fatal]

    # -- serialization ------------------------------------------------------
    def to_dict(self) -> Dict[str, Any]:
        return {
            "schema_version": MANIFEST_SCHEMA_VERSION,
            "model_id": self.model_id,
            "task": self.task,
            "input_width": int(self.input_width),
            "input_height": int(self.input_height),
            "bbox_order": self.bbox_order,
            "bbox_normalized": bool(self.bbox_normalized),
            "postprocess": self.postprocess,
            "inference_rate_hz": float(self.inference_rate_hz),
            "labels": self.labels if isinstance(self.labels, str)
            else list(self.labels),
            "permitted_classes": list(self.permitted_classes),
            "preserve_aspect_ratio": bool(self.preserve_aspect_ratio),
            "row_layout": {"score_index": int(self.score_index),
                           "class_index": int(self.class_index),
                           "box_index": int(self.box_index),
                           "columns": int(self.row_columns)},
            "output_tensors": dict(self.output_tensors) if self.output_tensors is not None
            else None,
            "path": self.path,
            "sha256": self.sha256,
            "license": self.license,
            "notes": self.notes,
        }

    def to_json(self) -> str:
        return dumps(self.to_dict(), indent=2)

    def with_runtime(self, **overrides) -> "ModelManifest":
        """A copy corrected by the runtime's own report. Used only for *warnings*.

        Deliberately not used for fatal fields: silently adopting the sensor's answer is how
        a manifest stops being a check and becomes a transcription of whatever was loaded.
        """
        return replace(self, **overrides)
