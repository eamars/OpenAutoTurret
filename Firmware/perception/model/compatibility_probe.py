"""§9.2 and Appendix D — the compatibility probe: what the model actually is.

Appendix D sketches this as four `assert`s inside a function. Asserts are the wrong shape for
a daemon: the first failure aborts, and the operator learns one fact at a time until they have
run the tool eight times. This version *collects findings* and reports them together, then the
caller raises once — because the questions the probe answers ("what task, what rate, what box
convention, what labels") are exactly the questions a commissioning sheet asks at once.

§9.2's oracle has two halves and this module serves both:

* the upstream reference example works — outside this codebase, but the probe records the
  runtime facts that let a run be reproduced;
* **this subsystem's adapter produces identical normalized detections** on a captured sequence
  — which requires a normalization path that exists in exactly one place (§14's
  ``normalize_rows``) and a probe that runs *that* path rather than a private copy of it.

The factory parameter is not test decoration. The station's own hardware cannot be imported
off-hardware (``picamera2`` needs ``libcamera``), and a probe that could only run against a
physical IMX500 could never be part of a test suite — so the object it queries is injected, and
the default factory does the real import lazily.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Mapping, Optional, Sequence, Tuple

from ..errors import ModelRejected
from .manifest import ModelManifest, normalise_task


@dataclass
class ProbeResult:
    """Everything the runtime said about one model file, plus what was wrong with it."""

    model_path: str = ""
    model_id: str = ""
    probed: bool = False
    source: str = "imx500"
    duration_s: float = 0.0

    task: str = ""
    inference_rate_hz: Optional[float] = None
    bbox_order: Optional[str] = None
    bbox_normalized: Optional[bool] = None
    preserve_aspect_ratio: Optional[bool] = None
    labels: Optional[Tuple[str, ...]] = None
    input_width: int = 0
    input_height: int = 0

    output_shapes: Tuple[Tuple[int, ...], ...] = ()
    row_count: int = 0
    row_width: int = 0
    sample_rows: Tuple[Tuple[float, ...], ...] = ()
    postprocess: Optional[str] = None
    raw_intrinsics: Dict[str, Any] = field(default_factory=dict)

    findings: List[str] = field(default_factory=list)
    error: str = ""

    @property
    def usable(self) -> bool:
        return bool(self.probed and not self.findings and not self.error)

    def intrinsics(self) -> Dict[str, Any]:
        """The runtime's own declarations, in the shape §9.3's comparison expects."""
        payload = dict(self.raw_intrinsics)
        payload.setdefault("task", self.task or None)
        payload.setdefault("inference_rate", self.inference_rate_hz)
        payload.setdefault("bbox_order", self.bbox_order)
        payload.setdefault("bbox_normalization", self.bbox_normalized)
        payload.setdefault("preserve_aspect_ratio", self.preserve_aspect_ratio)
        payload.setdefault("labels", list(self.labels) if self.labels else None)
        if self.input_width and self.input_height:
            payload.setdefault("input_size", [self.input_width, self.input_height])
        return payload

    def sha256(self) -> str:
        """Hash the probed file, if it is still where the probe found it."""
        import hashlib
        import os

        if not self.model_path or not os.path.isfile(self.model_path):
            return ""
        digest = hashlib.sha256()
        with open(self.model_path, "rb") as handle:
            for block in iter(lambda: handle.read(1 << 20), b""):
                digest.update(block)
        return digest.hexdigest()

    def to_manifest_document(self, *, model_id: str = "",
                             permitted_classes: Sequence[str] = ("person",)) -> Dict[str, Any]:
        """A manifest **draft** built from what the runtime declared, for a human to finish.

        §9.3's rule is that an artefact is described by measurement, so this is the only honest
        direction of writing: probe → file. The alternative — typing an ``xy``/``yxyx`` guess
        and letting the probe reject it at every start-up — is how a station ends up with a
        manifest nobody trusts and a probe everyone works around.

        It is a draft, not a manifest. The probe sees the *shapes* of a model's outputs, not
        their column order, so ``row_layout`` stays ``COMMISSION`` and everything a human owns
        (``license``) is named in ``_needs``. Committing this file unreviewed would replace a
        human's guess with a machine's, which is not an improvement (§50).
        """
        from .label_maps import COCO, COCO_RPK

        labels: Any = list(self.labels) if self.labels else []
        needs: List[str] = [
            "license — not measurable; see /usr/share/doc/imx500-models/copyright",
            "row_layout.score_index / class_index / box_index — the probe sees the row's width, "
            "not its column order; confirm against the upstream example",
            "permitted_classes — a policy choice, not a model property",
            "review _provisional and delete _needs before committing",
        ]
        if tuple(labels) == COCO_RPK:
            labels = "coco_rpk"
            needs.append("labels matched the built-in 'coco_rpk' map, so the literal list was "
                         "shortened to that name")
        elif tuple(labels) == COCO:
            labels = "coco"

        return {
            "_comment": [
                "Drafted by the compatibility probe at "
                + time.strftime("%Y-%m-%d %H:%M UTC", time.gmtime()) + ".",
                "",
                "Every field here was read from the running model except the ones _needs names.",
                "Fill _needs, review _provisional, then move this file into",
                "perception/model/manifests/ and reference it from a config profile.",
            ],
            "_needs": needs,
            "_provisional": [name for name, value in (
                ("bbox_order", self.bbox_order),
                ("bbox_normalized", self.bbox_normalized),
                ("postprocess", self.postprocess)) if value in (None, "")],
            "schema_version": 1,
            "model_id": model_id or self.model_id or self.model_path,
            "task": normalise_task(self.task) if self.task else "object_detection",
            "input_width": int(self.input_width),
            "input_height": int(self.input_height),
            "bbox_order": self.bbox_order or "COMMISSION",
            "bbox_normalized": self.bbox_normalized if self.bbox_normalized is not None
            else "COMMISSION",
            "postprocess": self.postprocess or "COMMISSION",
            "inference_rate_hz": self.inference_rate_hz,
            "labels": labels,
            "permitted_classes": [str(name) for name in permitted_classes],
            "preserve_aspect_ratio": bool(self.preserve_aspect_ratio),
            "row_layout": {"score_index": "COMMISSION", "class_index": "COMMISSION",
                           "box_index": "COMMISSION"},
            "path": self.model_path,
            "sha256": self.sha256(),
            "license": "COMMISSION",
            "notes": [
                f"probe saw {self.row_count} output rows of {self.row_width} columns",
                "observed output shapes: " + (
                    ", ".join(str(list(shape)) for shape in self.output_shapes) or "none"),
            ],
        }

    def to_dict(self) -> Dict[str, Any]:
        return {"model_path": self.model_path, "model_id": self.model_id,
                "probed": bool(self.probed), "source": self.source,
                "duration_s": round(float(self.duration_s), 4),
                "task": self.task, "inference_rate_hz": self.inference_rate_hz,
                "bbox_order": self.bbox_order,
                "bbox_normalized": self.bbox_normalized,
                "preserve_aspect_ratio": self.preserve_aspect_ratio,
                "labels": list(self.labels) if self.labels else None,
                "input_size": [self.input_width, self.input_height],
                "output_shapes": [list(shape) for shape in self.output_shapes],
                "row_count": int(self.row_count), "row_width": int(self.row_width),
                "sample_rows": [list(row) for row in self.sample_rows],
                "postprocess": self.postprocess,
                "findings": list(self.findings), "error": self.error,
                "usable": self.usable}


def default_imx500_factory(model_path: str):
    """Build the real ``IMX500`` object. Imported lazily, so the module stays importable.

    ``picamera2`` imports ``libcamera`` at module scope, which exists only on the station. A
    top-level import here would make ``perception.model`` unimportable in every offline
    context — including the tests that check this module's logic.
    """
    try:
        from picamera2.devices.imx500.imx500 import IMX500      # type: ignore
    except Exception as exc:                                    # noqa: BLE001
        raise ModelRejected(
            f"cannot import picamera2's IMX500 support ({exc}). This interpreter has no "
            f"libcamera; run on the station under the system Python, or use the mock "
            f"adapter for offline work.") from exc
    return IMX500(model_path)


def _read(intrinsics: Any, key: str, default: Any = None) -> Any:
    if intrinsics is None:
        return default
    if isinstance(intrinsics, Mapping):
        return intrinsics.get(key, default)
    return getattr(intrinsics, key, default)


def probe_model(model_path: str, *, model_id: str = "",
                imx500_factory: Optional[Callable[[str], Any]] = None,
                metadata: Optional[Mapping[str, Any]] = None,
                sample_rows: int = 3) -> ProbeResult:
    """Ask the runtime what ``model_path`` is, and report every problem found.

    ``metadata`` is a captured request's metadata: with it, the probe also reads the output
    tensors, which is the only way to learn the real row width and row count rather than the
    ones the manifest hopes for.
    """
    started = time.monotonic()
    result = ProbeResult(model_path=model_path, model_id=model_id)
    factory = imx500_factory or default_imx500_factory

    try:
        device = factory(model_path)
    except ModelRejected as exc:
        result.error = str(exc)
        result.duration_s = time.monotonic() - started
        return result
    except Exception as exc:                                    # noqa: BLE001
        result.error = f"IMX500({model_path!r}) failed: {exc}"
        result.duration_s = time.monotonic() - started
        return result

    result.probed = True
    intrinsics = getattr(device, "network_intrinsics", None)
    if intrinsics is None:
        result.findings.append("network_intrinsics is None: the .rpk declares nothing, so "
                               "no output convention can be trusted (§9.2)")
    else:
        result.raw_intrinsics = _flatten_intrinsics(intrinsics)
        result.task = normalise_task(_read(intrinsics, "task"))
        rate = _read(intrinsics, "inference_rate") or _read(intrinsics, "fps")
        result.inference_rate_hz = float(rate) if rate else None
        order = _read(intrinsics, "bbox_order")
        result.bbox_order = str(order).strip().lower() if order else None
        result.bbox_normalized = _read(intrinsics, "bbox_normalization")
        result.preserve_aspect_ratio = _read(intrinsics, "preserve_aspect_ratio")
        labels = _read(intrinsics, "labels")
        result.labels = tuple(str(name) for name in labels) if labels else None
        result.postprocess = _read(intrinsics, "postprocess")

    if not result.task:
        result.findings.append("no task reported")
    elif result.task not in ("object_detection", "pose_estimation"):
        result.findings.append(f"task {result.task!r} is not object detection or pose "
                               f"estimation (Appendix D)")
    if result.inference_rate_hz in (None, 0.0):
        result.findings.append("inference_rate is not reported: §19's time-based lifecycle "
                               "and §40's budgets both need a nominal rate")
    if not result.labels:
        result.findings.append("labels are not reported: §15's class filter cannot name "
                               "what it is filtering on")

    try:
        size = device.get_input_size()
        if size and len(size) >= 2:
            result.input_width, result.input_height = int(size[0]), int(size[1])
    except Exception as exc:                                    # noqa: BLE001
        result.findings.append(f"get_input_size() failed: {exc}")

    if metadata is not None:
        _probe_outputs(device, result, metadata, sample_rows)

    result.duration_s = time.monotonic() - started
    return result


def _probe_outputs(device: Any, result: ProbeResult, metadata: Mapping[str, Any],
                   sample_rows: int) -> None:
    """Read the output tensors once, to learn the row layout the manifest claims to know."""
    try:
        shapes = getattr(device, "get_output_shapes", lambda _md: []) (metadata)
        result.output_shapes = tuple(tuple(int(d) for d in shape) for shape in shapes or ())
    except Exception as exc:                                    # noqa: BLE001
        result.findings.append(f"get_output_shapes() failed: {exc}")
        return
    try:
        outputs = device.get_outputs(metadata)
    except Exception as exc:                                    # noqa: BLE001
        result.findings.append(f"get_outputs() failed: {exc}")
        return
    if not outputs:
        result.findings.append("get_outputs() returned nothing for the captured metadata")
        return
    rows = _as_rows(outputs[0])
    if rows is None:
        result.findings.append("the first output tensor is not row-shaped; this model's "
                               "post-processed layout is not [score, class, x1, y1, x2, y2]")
        return
    result.row_count, result.row_width = len(rows), (len(rows[0]) if rows else 0)
    result.sample_rows = tuple(tuple(float(value) for value in row)
                               for row in rows[:sample_rows])


def _as_rows(tensor: Any) -> Optional[List[Sequence[float]]]:
    """Flatten a post-processed tensor to rows of equal length, or say it cannot be done.

    The shapes seen in practice are ``(N, 6)`` and ``(1, N, 6)``. An earlier revision stripped
    *any* leading singleton dimension, which quietly turned a one-detection frame — shape
    ``(1, 6)`` — into a flat six-vector and then into "not row-shaped": the network saw a
    person and the adapter reported a layout error. Unwrapping now requires the payload to
    still be rows, and a bare row is accepted as the one row it is.
    """
    try:
        data = tensor.tolist() if hasattr(tensor, "tolist") else list(tensor)
    except Exception:                                           # noqa: BLE001
        return None
    while (isinstance(data, list) and len(data) == 1 and isinstance(data[0], list)
           and data[0] and isinstance(data[0][0], (list, tuple))):
        data = data[0]                                            # drop a batch dimension
    if not data:
        return []
    if all(isinstance(value, (int, float)) for value in data):
        return [data] if len(data) >= 4 else None                 # one bare row
    if not isinstance(data[0], (list, tuple)):
        return None
    width = len(data[0])
    if width < 4:
        return None
    if any(not isinstance(row, (list, tuple)) or len(row) != width for row in data):
        return None
    return data


def _flatten_intrinsics(intrinsics: Any) -> Dict[str, Any]:
    payload: Dict[str, Any] = {}
    if isinstance(intrinsics, Mapping):
        payload.update({str(key): _plain(value) for key, value in intrinsics.items()})
        return payload
    inner = getattr(intrinsics, "intrinsics", None)
    if isinstance(inner, Mapping):
        payload.update({str(key): _plain(value) for key, value in inner.items()})
    for key in ("task", "inference_rate", "fps", "bbox_order", "bbox_normalization",
                "preserve_aspect_ratio", "postprocess", "softmax", "ignore_dash_labels"):
        value = getattr(intrinsics, key, None)
        if value is not None:
            payload[key] = _plain(value)
    return payload


def _plain(value: Any) -> Any:
    if isinstance(value, (bool, int, float, str)) or value is None:
        return value
    if isinstance(value, Mapping):
        return {str(key): _plain(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_plain(item) for item in value]
    return str(value)


def admit(manifest: ModelManifest, probe: ProbeResult, *,
          strict: bool = True) -> List[str]:
    """§9.3's gate: return the warnings, or raise with every reason the model is inadmissible."""
    problems: List[str] = list(probe.findings)
    if probe.error:
        problems.append(probe.error)
    if not probe.probed:
        problems.append("model could not be probed at all")
    disagreements = manifest.disagreements_with_intrinsics(probe.intrinsics())
    problems.extend(str(item) for item in disagreements if item.fatal)
    warnings = [str(item) for item in disagreements if not item.fatal]

    if problems and (strict or not probe.probed):
        raise ModelRejected(
            f"model {manifest.model_id or probe.model_path} is not admissible on this "
            f"station:\n  - " + "\n  - ".join(dict.fromkeys(problems)))
    return warnings
