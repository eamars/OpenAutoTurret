"""Label maps: turning a model's class *index* into the name the policy filters on.

§15's class filter and §13's ``class_name`` both need this, and it matters more than it looks:
the YOLO11n COCO export says class 0 is ``person``, while a fine-tuned one-class export
(§11) may say class 0 is ``person`` too — or index into a completely different set. A map
that silently returned ``f"class_{id}"`` on a miss would let a permitted-class filter pass
whatever it liked, because "class_0" is not "person" and yet looks like a name downstream.

So every lookup here is explicit about whether a name was actually found.
"""
from __future__ import annotations

from typing import Any, Dict, Mapping, Optional, Sequence, Tuple

#: COCO 80-class order, as used by the IMX500 zoo exports and §6's YOLO11n baseline.
COCO: Tuple[str, ...] = (
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
    "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
    "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
    "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
    "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
    "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake",
    "chair", "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop",
    "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
    "toothbrush",
)

#: The class list as embedded in the ``.rpk`` metadata by ``imx500-models``: measured
#: 2026-09-05 from ``imx500-models 1:1.0.0-1``, and byte-identical across the three installed
#: ``_pp`` detectors. 90 entries, of which 10 are ``"-"`` placeholders (indices 11, 25, 28,
#: 29, 44, 65, 67, 68, 70, 82) — the COCO category ids with the gaps kept, so index 0 is
#: ``person`` and index 89 is ``toothbrush``.
#:
#: This is a different map from :data:`COCO`, not a formatting variant of it. The installed
#: models emit *these* indices, so a manifest that declares the 80-name contiguous list
#: disagrees with the device on label length — and §9.3's probe treats a length difference as
#: fatal, refusing to start. That refusal is the safety net working; declaring ``coco_rpk`` is
#: what lets a correct manifest pass instead of training people to disable the net.
COCO_RPK: Tuple[str, ...] = (
    'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat',
    'traffic light', 'fire hydrant', '-', 'stop sign', 'parking meter', 'bench', 'bird', 'cat',
    'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe', '-', 'backpack',
    'umbrella', '-', '-', 'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard',
    'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard',
    'tennis racket', 'bottle', '-', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl',
    'banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut',
    'cake', 'chair', 'couch', 'potted plant', 'bed', '-', 'dining table', '-', '-', 'toilet',
    '-', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone', 'microwave', 'oven',
    'toaster', 'sink', 'refrigerator', '-', 'book', 'clock', 'vase', 'scissors', 'teddy bear',
    'hair drier', 'toothbrush',
)

_BUILTIN: Dict[str, Tuple[str, ...]] = {"coco": COCO, "coco_rpk": COCO_RPK}


class LabelMap:
    """An explicit class-index → name mapping, with the source it came from recorded."""

    def __init__(self, names: Sequence[str], *, source: str = "explicit") -> None:
        self.names: Tuple[str, ...] = tuple(str(name) for name in names)
        self.source = source

    def __len__(self) -> int:
        return len(self.names)

    def __bool__(self) -> bool:
        """Always true: an empty label map is a manifest problem to report, not a missing one."""
        return True

    def __getitem__(self, class_id: int) -> str:
        """Indexable, so ``normalize_rows``'s lookup works on a map as well as a list.

        Raising for an out-of-range id rather than returning a placeholder keeps §15's
        distinction intact: an unknown class is a rejection upstream, not a name.
        """
        name = self.name(int(class_id))
        if name is None:
            raise IndexError(f"class id {class_id} is outside this label map")
        return name

    def __contains__(self, name: object) -> bool:
        return str(name).strip().lower() in [item.strip().lower() for item in self.names]

    def name(self, class_id: int) -> Optional[str]:
        """The name for an index, or ``None`` — never a synthetic placeholder.

        A miss is a contract violation between the manifest and the model, and §15's filter
        must be able to tell "this class is not a person" from "we do not know what this
        class is". Returning ``class_0`` would collapse those two into one, and the second
        one is a bug we want to surface.
        """
        if 0 <= int(class_id) < len(self.names):
            return self.names[int(class_id)]
        return None

    def name_or_unknown(self, class_id: int) -> str:
        return self.name(class_id) or ""

    def index_of(self, name: str) -> Optional[int]:
        target = str(name).strip().lower()
        for index, candidate in enumerate(self.names):
            if candidate.strip().lower() == target:
                return index
        return None

    def ids_for(self, names: Sequence[str]) -> Tuple[int, ...]:
        return tuple(index for index in (self.index_of(name) for name in names)
                     if index is not None)

    def to_dict(self) -> Dict[str, Any]:
        return {"source": self.source, "count": len(self.names), "names": list(self.names)}


def resolve(spec: Any) -> LabelMap:
    """Accept a built-in name (``coco``), a list of names, or a mapping of index → name."""
    if spec is None:
        raise ValueError("no label map specified (§9.3 requires labels)")
    if isinstance(spec, LabelMap):
        return spec
    if isinstance(spec, str):
        key = spec.strip().lower()
        if key in _BUILTIN:
            return LabelMap(_BUILTIN[key], source=key)
        raise ValueError(f"unknown label map '{spec}'. known: {sorted(_BUILTIN)}")
    if isinstance(spec, Mapping):
        size = max(int(key) for key in spec) + 1
        names = [str(spec.get(index, spec.get(str(index), ""))) for index in range(size)]
        return LabelMap(names, source="explicit_map")
    if isinstance(spec, (list, tuple)):
        return LabelMap(spec, source="explicit_list")
    raise ValueError(f"cannot interpret a label map from {type(spec).__name__}")
