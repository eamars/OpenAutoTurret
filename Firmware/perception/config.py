"""Subsystem configuration (§50), with ``COMMISSION`` treated as "not configured".

Every tunable that the architecture document refuses to invent lives here as ``None``
rather than as a plausible-looking default. §50 writes the production values as the
literal string ``COMMISSION`` and then says the implementation "shall reject unresolved
``COMMISSION`` values for a production profile" — and §3.5 records the cost of ignoring
that: one universal ``0.50`` was applied across detectors whose scores did not mean the
same thing, so a motion-blob detector scoring 0.31–0.45 sat permanently below a threshold
that a YOLO head would clear by accident. A default that reads like a measured value is
how that happened; ``None`` cannot be misread as one.

Values fall into three tiers, and the difference is enforced rather than commented:

**Frozen by measurement** — §19's lifecycle timings (120 ms visible, 180 ms gap, 350 ms
occlusion, 3000 ms TTL) ship as the document's stated starting values. They are starting
values, and ``validate()`` says so in the warning it emits, because a starting value
nobody re-measured is what a station ends up running for a year.

**Structural** — ``max_tracks``, dwell times, the anchor fraction. Defaults are safe and
the reason for each is written next to it.

**Commissioned** — every score threshold and every dedup IoU. These require the station's
own recordings (§16.1: "do not copy an arbitrary IoU threshold from a generic demo"), and
``validate(production=True)`` refuses to start a production profile without them.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Mapping, Optional, Tuple

from .errors import ConfigError, ConfigPlaceholderError, ValidationError

#: §50's placeholder. Accepting it as a *string* in a config file and then refusing to
#: run is deliberate: the file stays readable as a work list.
COMMISSION = "COMMISSION"


class SelectionPolicy(str, Enum):
    """§28. ``AUTO_SELECT_BEST`` is deliberately absent: §28.3 rejects it as a default
    and §54 lists it as an approach the project has already refused."""

    EXPLICIT_ONLY = "explicit_only"
    AUTO_SELECT_SINGLE = "auto_select_single"


class AssociationEngine(str, Enum):
    """§22's pluggable association. ``OC_SORT`` is an interface slot, not a promise."""

    BYTE_STYLE = "byte_style"
    OC_SORT = "oc_sort"


class CameraMotionProviderName(str, Enum):
    """§23. ``NONE`` is the baseline; frame differencing is not offered (§23.3, §54)."""

    NONE = "none"
    IMAGE_GMC = "image_gmc"
    EXTERNAL_POSE_HINT = "external_pose_hint"


def _as_number_opt(value: Any, name: str) -> Optional[float]:
    """Parse a possibly-``COMMISSION`` number. ``COMMISSION`` -> ``None``, junk -> error."""
    if value is None:
        return None
    if isinstance(value, str):
        if value.strip().upper() == COMMISSION:
            return None
        try:
            value = float(value)
        except ValueError:
            raise ConfigError(f"{name}: expected a number or {COMMISSION}, got {value!r}")
    try:
        return float(value)
    except (TypeError, ValueError):
        raise ConfigError(f"{name}: expected a number, got {value!r}") from None


def _as_int(value: Any, name: str, default: int) -> int:
    if value is None:
        return int(default)
    try:
        return int(value)
    except (TypeError, ValueError):
        raise ConfigError(f"{name}: expected an integer, got {value!r}") from None


def _as_float(value: Any, name: str, default: float) -> float:
    if value is None:
        return float(default)
    try:
        return float(value)
    except (TypeError, ValueError):
        raise ConfigError(f"{name}: expected a number, got {value!r}") from None


def _as_bool(value: Any, name: str, default: bool) -> bool:
    if value is None:
        return bool(default)
    if isinstance(value, str):
        lowered = value.strip().lower()
        if lowered in ("1", "true", "yes", "on"):
            return True
        if lowered in ("0", "false", "no", "off"):
            return False
        raise ConfigError(f"{name}: expected a boolean, got {value!r}")
    return bool(value)


def _as_enum(enum_cls, value: Any, name: str):
    if value is None or value == "":
        return list(enum_cls)[0]
    if isinstance(value, enum_cls):
        return value
    try:
        return enum_cls(str(value).lower())
    except ValueError:
        allowed = ", ".join(str(m.value) for m in enum_cls)
        raise ConfigError(f"{name}: {value!r} is not one of {allowed}") from None


# --------------------------------------------------------------------------
# Score thresholds (§37.2)
# --------------------------------------------------------------------------

@dataclass
class ScoreThresholds:
    """§37.2's per-model threshold set. All four must be commissioned (§50)."""

    low_association: Optional[float] = None    # §20 pass 2: may rescue, may not create
    new_track: Optional[float] = None          # §20 pass 3: may create a tentative track
    confirmed_update: Optional[float] = None   # §20 pass 1: may confirm/update
    selectable: Optional[float] = None         # §37.1's gate for operator selection

    def is_complete(self) -> bool:
        return all(v is not None for v in (self.low_association, self.new_track,
                                           self.confirmed_update, self.selectable))

    def missing(self) -> List[str]:
        return [name for name in ("low_association", "new_track", "confirmed_update",
                                  "selectable")
                if getattr(self, name) is None]

    def validate(self, where: str) -> List[str]:
        """Returns human-readable complaints rather than raising on the first one.

        Commissioning four thresholds is one tuning session; discovering them one at a
        time by restarting the daemon is four.
        """
        problems = [f"{where}: unresolved COMMISSION threshold(s): {', '.join(self.missing())}"
                    if self.missing() else ""]
        values = {name: getattr(self, name) for name in
                  ("low_association", "new_track", "confirmed_update", "selectable")
                  if getattr(self, name) is not None}
        for name, value in values.items():
            if not 0.0 <= value <= 1.0:
                problems.append(f"{where}.{name}={value} is outside [0,1]")
        # A detector score below which a track may not be *created* must not exceed the
        # score at which it may be *selected*: that inversion is unsatisfiable, and it
        # reads to an operator as "the person is right there and the turret refuses".
        low, new = values.get("low_association"), values.get("new_track")
        conf, sel = values.get("confirmed_update"), values.get("selectable")
        if low is not None and new is not None and new < low:
            problems.append(f"{where}: new_track({new}) < low_association({low}) "
                            f"inverts §20's pass ordering")
        if conf is not None and sel is not None and sel < conf:
            problems.append(f"{where}: selectable({sel}) < confirmed_update({conf}) "
                            f"would make every confirmed track selectable")
        return [p for p in problems if p]

    def to_dict(self) -> Dict[str, Any]:
        return {name: (COMMISSION if getattr(self, name) is None else getattr(self, name))
                for name in ("low_association", "new_track", "confirmed_update",
                             "selectable")}

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "ScoreThresholds":
        data = data or {}
        return cls(**{name: _as_number_opt(data.get(name), f"thresholds.{name}")
                      for name in ("low_association", "new_track", "confirmed_update",
                                   "selectable")})


# --------------------------------------------------------------------------
# Tracking (§18, §19, §21, §22, §26)
# --------------------------------------------------------------------------

@dataclass
class TentativeConfig:
    """§19's TENTATIVE timings — §19's own starting values."""

    min_observations: int = 3
    min_visible_ms: float = 120.0
    max_gap_ms: float = 180.0

    def to_dict(self) -> Dict[str, Any]:
        return {"min_observations": self.min_observations,
                "min_visible_ms": self.min_visible_ms,
                "max_gap_ms": self.max_gap_ms}

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "TentativeConfig":
        data = data or {}
        return cls(
            min_observations=_as_int(data.get("min_observations"), "tentative.min_observations", 3),
            min_visible_ms=_as_float(data.get("min_visible_ms"), "tentative.min_visible_ms", 120.0),
            max_gap_ms=_as_float(data.get("max_gap_ms"), "tentative.max_gap_ms", 180.0))


@dataclass
class OccludedConfig:
    max_ms: float = 350.0

    def to_dict(self) -> Dict[str, Any]:
        return {"max_ms": self.max_ms}

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "OccludedConfig":
        return cls(max_ms=_as_float((data or {}).get("max_ms"), "occluded.max_ms", 350.0))


@dataclass
class LostConfig:
    """§18.4/§19: how long a LOST identity stays reacquirable. §31's selection TTL."""

    retain_ms: float = 3000.0

    def to_dict(self) -> Dict[str, Any]:
        return {"retain_ms": self.retain_ms}

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "LostConfig":
        return cls(retain_ms=_as_float((data or {}).get("retain_ms"), "lost.retain_ms", 3000.0))


@dataclass
class AppearanceConfig:
    """§24. Off by default: association is tried without it first (§54)."""

    enabled: bool = False
    type: str = "hsv_upper_lower"
    persist: bool = False
    # Cosine distance below which two descriptors look like the same clothing, and above
    # which §21's appearance term is a veto rather than a tiebreaker. Structural, not
    # commissioned: a coarse HSV histogram cannot distinguish two people in the same
    # jacket, so this term may only ever down-weight a match.
    max_distance: float = 0.35

    def to_dict(self) -> Dict[str, Any]:
        return {"enabled": self.enabled, "type": self.type,
                "persist": self.persist, "max_distance": self.max_distance}

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "AppearanceConfig":
        data = data or {}
        return cls(
            enabled=_as_bool(data.get("enabled"), "appearance.enabled", False),
            type=str(data.get("type", "hsv_upper_lower")),
            persist=_as_bool(data.get("persist"), "appearance.persist", False),
            max_distance=_as_float(data.get("max_distance"), "appearance.max_distance", 0.35))


@dataclass
class CameraMotionConfig:
    """§23. The interface carries no motor brand and no drive detail (§23.2)."""

    provider: CameraMotionProviderName = CameraMotionProviderName.NONE
    # A hint older than this is treated as absent: a stale pose hint shifts every box by
    # the previous frame's delta, which manufactures motion rather than compensating it.
    hint_max_age_ms: float = 150.0
    # Horizontal focal length in pixels, used to turn delta_yaw into a normalized image
    # shift. Commissioned from §29's calibration work; without it EXTERNAL_POSE_HINT
    # cannot scale a yaw into pixels and must decline to compensate at all.
    focal_px: Optional[float] = None
    # Sparse-flow GMC (§23.1) is CPU-costly and profiled, not assumed free.
    gmc_max_features: int = 200
    gmc_min_inliers: int = 12

    def to_dict(self) -> Dict[str, Any]:
        return {"provider": self.provider.value,
                "hint_max_age_ms": self.hint_max_age_ms,
                "focal_px": (COMMISSION if self.focal_px is None else self.focal_px),
                "gmc_max_features": self.gmc_max_features,
                "gmc_min_inliers": self.gmc_min_inliers}

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "CameraMotionConfig":
        data = data or {}
        return cls(
            provider=_as_enum(CameraMotionProviderName, data.get("provider"),
                              "camera_motion.provider"),
            hint_max_age_ms=_as_float(data.get("hint_max_age_ms"),
                                      "camera_motion.hint_max_age_ms", 150.0),
            focal_px=_as_number_opt(data.get("focal_px"), "camera_motion.focal_px"),
            gmc_max_features=_as_int(data.get("gmc_max_features"),
                                     "camera_motion.gmc_max_features", 200),
            gmc_min_inliers=_as_int(data.get("gmc_min_inliers"),
                                    "camera_motion.gmc_min_inliers", 12))


@dataclass
class AssociationWeights:
    """§21's cost terms. A term is only enabled when the data for it exists."""

    motion: float = 1.0
    iou: float = 0.40
    scale: float = 0.25
    shape: float = 0.15
    appearance: float = 0.0      # §24: enabled only after box/motion proves insufficient
    pose: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        return {"motion": self.motion, "iou": self.iou, "scale": self.scale,
                "shape": self.shape, "appearance": self.appearance, "pose": self.pose}

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "AssociationWeights":
        data = data or {}
        return cls(**{_snake(name): _as_float(data.get(_snake(name)),
                                              f"weights.{_snake(name)}", default)
                      for name, default in (("motion", 1.0), ("iou", 0.40),
                                            ("scale", 0.25), ("shape", 0.15),
                                            ("appearance", 0.0), ("pose", 0.0))})


@dataclass
class AssociationGates:
    """§21's hard gates, applied BEFORE any cost is considered.

    These are physics, not tuning. A person cannot cross the image at five body-widths a
    second, and a 12-pixel-tall box cannot triple in one frame; a match that violates
    either is not a low-probability association, it is a different object. Gating is
    separate from cost because a cost can be outvoted by other terms and a gate cannot.
    """

    #: Maximum anchor speed a track may follow, in normalized image heights per second.
    #: A walking person at typical station range is well under this; a camera pan
    #: compensated by §23 must not need it.
    max_speed_norm_s: float = 2.5
    #: One-frame area ratio beyond which the two boxes cannot be the same person.
    max_scale_ratio: float = 3.0
    #: Aspect (w/h) relative change beyond which the pose changed impossibly fast.
    max_aspect_change: float = 1.0
    #: A LOST identity is matched by §21's gates plus a wider displacement allowance,
    #: multiplied onto the speed-derived distance for the reacquisition window.
    reacquire_margin: float = 1.6
    #: §32: two reacquisition candidates whose scores differ by less than this fraction
    #: of the better one are ambiguous, and §32 says choose neither.
    ambiguity_margin: float = 0.15

    def to_dict(self) -> Dict[str, Any]:
        return {"max_speed_norm_s": self.max_speed_norm_s,
                "max_scale_ratio": self.max_scale_ratio,
                "max_aspect_change": self.max_aspect_change,
                "reacquire_margin": self.reacquire_margin,
                "ambiguity_margin": self.ambiguity_margin}

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "AssociationGates":
        data = data or {}
        return cls(
            max_speed_norm_s=_as_float(data.get("max_speed_norm_s"), "gates.max_speed_norm_s", 2.5),
            max_scale_ratio=_as_float(data.get("max_scale_ratio"), "gates.max_scale_ratio", 3.0),
            max_aspect_change=_as_float(data.get("max_aspect_change"), "gates.max_aspect_change", 1.0),
            reacquire_margin=_as_float(data.get("reacquire_margin"), "gates.reacquire_margin", 1.6),
            ambiguity_margin=_as_float(data.get("ambiguity_margin"), "gates.ambiguity_margin", 0.15))


@dataclass
class TrackingConfig:
    """§17–§26's tracker settings."""

    engine: AssociationEngine = AssociationEngine.BYTE_STYLE
    tentative: TentativeConfig = field(default_factory=TentativeConfig)
    occluded: OccludedConfig = field(default_factory=OccludedConfig)
    lost: LostConfig = field(default_factory=LostConfig)
    weights: AssociationWeights = field(default_factory=AssociationWeights)
    gates: AssociationGates = field(default_factory=AssociationGates)
    appearance: AppearanceConfig = field(default_factory=AppearanceConfig)
    camera_motion: CameraMotionConfig = field(default_factory=CameraMotionConfig)
    #: §26's explicit capacity.
    max_tracks: int = 16
    #: §25: do not merge after one frame.
    duplicate_dwell_ms: float = 300.0
    #: §22's prediction horizon clamp. A detector that stalls for four seconds must not
    #: extrapolate a track across the frame on its next result.
    max_predict_dt_ms: float = 250.0
    #: §26: never evict the selected identity while its TTL is active.
    protect_selected: bool = True
    diagnostics_enabled: bool = False
    diagnostics_capacity: int = 512

    def to_dict(self) -> Dict[str, Any]:
        return {"engine": self.engine.value,
                "tentative": self.tentative.to_dict(),
                "occluded": self.occluded.to_dict(),
                "lost": self.lost.to_dict(),
                "weights": self.weights.to_dict(),
                "gates": self.gates.to_dict(),
                "appearance": self.appearance.to_dict(),
                "camera_motion": self.camera_motion.to_dict(),
                "max_tracks": self.max_tracks,
                "duplicate_dwell_ms": self.duplicate_dwell_ms,
                "max_predict_dt_ms": self.max_predict_dt_ms,
                "protect_selected": self.protect_selected,
                "diagnostics_enabled": self.diagnostics_enabled,
                "diagnostics_capacity": self.diagnostics_capacity}

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "TrackingConfig":
        data = data or {}
        return cls(
            engine=_as_enum(AssociationEngine, data.get("engine"), "tracking.engine"),
            tentative=TentativeConfig.from_dict(data.get("tentative")),
            occluded=OccludedConfig.from_dict(data.get("occluded")),
            lost=LostConfig.from_dict(data.get("lost")),
            weights=AssociationWeights.from_dict(data.get("weights")),
            gates=AssociationGates.from_dict(data.get("gates")),
            appearance=AppearanceConfig.from_dict(data.get("appearance")),
            camera_motion=CameraMotionConfig.from_dict(data.get("camera_motion")),
            max_tracks=_as_int(data.get("max_tracks"), "tracking.max_tracks", 16),
            duplicate_dwell_ms=_as_float(data.get("duplicate_dwell_ms"),
                                         "tracking.duplicate_dwell_ms", 300.0),
            max_predict_dt_ms=_as_float(data.get("max_predict_dt_ms"),
                                        "tracking.max_predict_dt_ms", 250.0),
            protect_selected=_as_bool(data.get("protect_selected"),
                                      "tracking.protect_selected", True),
            diagnostics_enabled=_as_bool(data.get("diagnostics_enabled"),
                                         "tracking.diagnostics_enabled", False),
            diagnostics_capacity=_as_int(data.get("diagnostics_capacity"),
                                         "tracking.diagnostics_capacity", 512))

    def validate(self) -> List[str]:
        problems: List[str] = []
        if self.max_tracks < 1:
            problems.append("tracking.max_tracks must be >= 1")
        if self.tentative.min_observations < 1:
            problems.append("tracking.tentative.min_observations must be >= 1")
        if self.tentative.min_visible_ms < 0 or self.tentative.max_gap_ms < 0:
            problems.append("tracking.tentative timings must be >= 0")
        if self.occluded.max_ms <= 0:
            problems.append("tracking.occluded.max_ms must be > 0")
        if self.lost.retain_ms <= self.occluded.max_ms:
            problems.append(
                f"tracking.lost.retain_ms({self.lost.retain_ms}) must exceed "
                f"occluded.max_ms({self.occluded.max_ms}) or OCCLUDED is unreachable")
        if self.max_predict_dt_ms <= 0:
            problems.append("tracking.max_predict_dt_ms must be > 0")
        if self.engine is AssociationEngine.OC_SORT:
            problems.append("tracking.engine=oc_sort has no implementation in this "
                            "revision; §22 keeps it as an interface slot only")
        return problems


# --------------------------------------------------------------------------
# Deduplication (§16)
# --------------------------------------------------------------------------

@dataclass
class DedupConfig:
    """§16's host-side defensive layer. The on-sensor ``_pp``/``nms=True`` output is
    still preferred; this catches what it misses (§16's opening rule)."""

    class_aware_nms: bool = True
    #: §16.1: model-specific, evaluated on the station dataset. Not defaulted.
    nms_iou: Optional[float] = None
    containment: bool = True
    #: §16.2's ``intersection / min(area)`` threshold. Not defaulted.
    containment_ratio: Optional[float] = None
    #: Normalized-image-unit centre distance under which a nested pair may be merged at
    #: all. Without it, two people standing one behind the other at 0.9 containment would
    #: be merged into whichever box scored higher — a plausible-looking way to lose a
    #: person, which is the outcome §16 exists to prevent.
    center_distance_norm: Optional[float] = None
    #: §16.2's "scale relationship is plausible": how much bigger the outer box may be.
    containment_max_scale_ratio: float = 4.0
    pose_nms: bool = False
    #: OKS-style keypoint sigma as a fraction of box diagonal (§16.3).
    pose_oks_sigma: float = 0.10

    def to_dict(self) -> Dict[str, Any]:
        def show(value):
            return COMMISSION if value is None else value
        return {"class_aware_nms": self.class_aware_nms, "nms_iou": show(self.nms_iou),
                "containment": self.containment,
                "containment_ratio": show(self.containment_ratio),
                "center_distance_norm": show(self.center_distance_norm),
                "containment_max_scale_ratio": self.containment_max_scale_ratio,
                "pose_nms": self.pose_nms, "pose_oks_sigma": self.pose_oks_sigma}

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "DedupConfig":
        data = data or {}
        return cls(
            class_aware_nms=_as_bool(data.get("class_aware_nms"), "dedup.class_aware_nms", True),
            nms_iou=_as_number_opt(data.get("nms_iou"), "dedup.nms_iou"),
            containment=_as_bool(data.get("containment"), "dedup.containment", True),
            containment_ratio=_as_number_opt(data.get("containment_ratio"),
                                             "dedup.containment_ratio"),
            center_distance_norm=_as_number_opt(data.get("center_distance_norm"),
                                                "dedup.center_distance_norm"),
            containment_max_scale_ratio=_as_float(
                data.get("containment_max_scale_ratio"),
                "dedup.containment_max_scale_ratio", 4.0),
            pose_nms=_as_bool(data.get("pose_nms"), "dedup.pose_nms", False),
            pose_oks_sigma=_as_float(data.get("pose_oks_sigma"), "dedup.pose_oks_sigma", 0.10))

    def validate(self) -> List[str]:
        problems: List[str] = []
        if self.class_aware_nms and self.nms_iou is None:
            problems.append("dedup.nms_iou is COMMISSION while class_aware_nms is on")
        if self.containment and self.containment_ratio is None:
            problems.append("dedup.containment_ratio is COMMISSION while containment is on")
        if self.containment and self.center_distance_norm is None:
            problems.append("dedup.center_distance_norm is COMMISSION while containment is on")
        for name in ("nms_iou", "containment_ratio"):
            value = getattr(self, name)
            if value is not None and not 0.0 < value <= 1.0:
                problems.append(f"dedup.{name}={value} must be in (0,1]")
        if self.center_distance_norm is not None and self.center_distance_norm < 0:
            problems.append("dedup.center_distance_norm must be >= 0")
        if self.pose_nms and not 0.0 < self.pose_oks_sigma <= 1.0:
            problems.append("dedup.pose_oks_sigma must be in (0,1] when pose_nms is on")
        return problems


# --------------------------------------------------------------------------
# Aim anchor (§35)
# --------------------------------------------------------------------------

@dataclass
class AnchorConfig:
    """§35. The aim anchor is deliberately not the identity box's centre."""

    #: §35's initial candidate is 0.42–0.48 of the box height from its top: upper torso,
    #: which is steadier than a head and means something more than a pair of legs.
    torso_fraction: float = 0.45
    use_pose_anchors: bool = True
    #: §12.1's "use confidence-weighted keypoints": below this a keypoint does not
    #: contribute, and a pose profile with nothing above it falls back to the box anchor.
    min_keypoint_score: float = 0.30

    def to_dict(self) -> Dict[str, Any]:
        return {"torso_fraction": self.torso_fraction,
                "use_pose_anchors": self.use_pose_anchors,
                "min_keypoint_score": self.min_keypoint_score}

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "AnchorConfig":
        data = data or {}
        return cls(
            torso_fraction=_as_float(data.get("torso_fraction"), "anchor.torso_fraction", 0.45),
            use_pose_anchors=_as_bool(data.get("use_pose_anchors"),
                                      "anchor.use_pose_anchors", True),
            min_keypoint_score=_as_float(data.get("min_keypoint_score"),
                                         "anchor.min_keypoint_score", 0.30))

    def validate(self) -> List[str]:
        problems: List[str] = []
        if not 0.0 < self.torso_fraction < 1.0:
            problems.append("anchor.torso_fraction must be inside (0,1)")
        if not 0.0 <= self.min_keypoint_score <= 1.0:
            problems.append("anchor.min_keypoint_score must be inside [0,1]")
        return problems


# --------------------------------------------------------------------------
# Model profile (§5, §9.3)
# --------------------------------------------------------------------------

@dataclass
class ModelConfig:
    """One model profile (§5) plus the manifest fields §9.3 requires."""

    profile_name: str = ""
    model_id: str = ""
    adapter: str = ""
    path: str = ""
    manifest_path: str = ""
    task: str = "object_detection"
    input_width: int = 640
    input_height: int = 640
    bbox_order: str = "xy"
    bbox_normalized: bool = True
    postprocess: str = "on_sensor"
    inference_rate_hz: int = 16
    labels: str = "coco"
    #: §15: the only classes this profile may create tracks for.
    permitted_classes: Tuple[str, ...] = ("person",)
    sha256: str = ""
    license: str = ""
    thresholds: ScoreThresholds = field(default_factory=ScoreThresholds)

    def to_dict(self) -> Dict[str, Any]:
        return {"model_id": self.model_id, "adapter": self.adapter, "path": self.path,
                "manifest": self.manifest_path, "task": self.task,
                "input_width": self.input_width, "input_height": self.input_height,
                "bbox_order": self.bbox_order, "bbox_normalized": self.bbox_normalized,
                "postprocess": self.postprocess,
                "inference_rate_hz": self.inference_rate_hz, "labels": self.labels,
                "permitted_classes": list(self.permitted_classes),
                "sha256": self.sha256, "license": self.license,
                "thresholds": self.thresholds.to_dict()}

    @classmethod
    def from_dict(cls, profile_name: str, data: Mapping[str, Any]) -> "ModelConfig":
        data = data or {}
        permitted = data.get("permitted_classes")
        if permitted is None:
            permitted = ("person",)
        elif isinstance(permitted, str):
            permitted = (permitted,)
        return cls(
            profile_name=profile_name,
            model_id=str(data.get("model_id", profile_name)),
            adapter=str(data.get("adapter", "")),
            path=str(data.get("path", "")),
            manifest_path=str(data.get("manifest", "")),
            task=str(data.get("task", "object_detection")),
            input_width=_as_int(data.get("input_width"), "input_width", 640),
            input_height=_as_int(data.get("input_height"), "input_height", 640),
            bbox_order=str(data.get("bbox_order", "xy")),
            bbox_normalized=_as_bool(data.get("bbox_normalized"), "bbox_normalized", True),
            postprocess=str(data.get("postprocess", "on_sensor")),
            inference_rate_hz=_as_int(data.get("inference_rate_hz"),
                                      "inference_rate_hz", 16),
            labels=str(data.get("labels", "coco")),
            permitted_classes=tuple(str(c) for c in permitted),
            sha256=str(data.get("sha256", "")),
            license=str(data.get("license", "")),
            thresholds=ScoreThresholds.from_dict(data.get("thresholds")))

    def validate(self) -> List[str]:
        problems: List[str] = []
        if not self.adapter:
            problems.append(f"models.{self.profile_name}.adapter is required")
        if self.input_width <= 0 or self.input_height <= 0:
            problems.append(f"models.{self.profile_name}: input dimensions must be > 0")
        if self.bbox_order not in ("xy", "yxyx", "cxcywh"):
            problems.append(f"models.{self.profile_name}.bbox_order={self.bbox_order!r} "
                            f"is not one of xy, yxyx, cxcywh")
        if self.inference_rate_hz <= 0:
            problems.append(f"models.{self.profile_name}.inference_rate_hz must be > 0")
        if not self.permitted_classes:
            problems.append(f"models.{self.profile_name}.permitted_classes is empty; "
                            f"§15 requires an explicit class list")
        problems.extend(self.thresholds.validate(f"models.{self.profile_name}.thresholds"))
        return problems


# --------------------------------------------------------------------------
# Selection (§28, §31, §32)
# --------------------------------------------------------------------------

@dataclass
class SelectionConfig:
    """§28's policy switch plus §28.2/§32's thresholds."""

    policy: SelectionPolicy = SelectionPolicy.EXPLICIT_ONLY
    #: §28.2's dwell: 400–700 ms, mid-range as the starting value.
    auto_select_single_dwell_ms: float = 500.0
    #: §28.2: "detector/identity confidence passes high threshold". Commissioned: 0.75
    #: would auto-select a target the station's own detector does not consider certain.
    auto_select_min_detector_score: Optional[float] = None
    auto_select_min_identity_confidence: Optional[float] = None
    #: §37.1's identity half of the selectable gate. Also structural for the same reason.
    select_min_identity_confidence: float = 0.50

    def to_dict(self) -> Dict[str, Any]:
        def show(value):
            return COMMISSION if value is None else value
        return {"policy": self.policy.value,
                "auto_select_single_dwell_ms": self.auto_select_single_dwell_ms,
                "auto_select_min_detector_score": show(self.auto_select_min_detector_score),
                "auto_select_min_identity_confidence": show(
                    self.auto_select_min_identity_confidence),
                "select_min_identity_confidence": self.select_min_identity_confidence}

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "SelectionConfig":
        data = data or {}
        return cls(
            policy=_as_enum(SelectionPolicy, data.get("policy"), "selection.policy"),
            auto_select_single_dwell_ms=_as_float(
                data.get("auto_select_single_dwell_ms"),
                "selection.auto_select_single_dwell_ms", 500.0),
            auto_select_min_detector_score=_as_number_opt(
                data.get("auto_select_min_detector_score"),
                "selection.auto_select_min_detector_score"),
            auto_select_min_identity_confidence=_as_number_opt(
                data.get("auto_select_min_identity_confidence"),
                "selection.auto_select_min_identity_confidence"),
            select_min_identity_confidence=_as_float(
                data.get("select_min_identity_confidence"),
                "selection.select_min_identity_confidence", 0.50))

    def validate(self) -> List[str]:
        problems: List[str] = []
        if self.auto_select_single_dwell_ms <= 0:
            problems.append("selection.auto_select_single_dwell_ms must be > 0")
        if not 0.0 <= self.select_min_identity_confidence <= 1.0:
            problems.append("selection.select_min_identity_confidence must be in [0,1]")
        # §28.2: AUTO_SELECT_SINGLE auto-selects on the operator's behalf, so it demands MORE
        # evidence than a manual click — a separate, higher confidence floor that must be
        # commissioned (§50). A station that auto-selects on an uncommissioned threshold would
        # be picking whatever the detector liked least, which is the refusal §50 exists for.
        if self.policy is SelectionPolicy.AUTO_SELECT_SINGLE:
            for name in ("auto_select_min_detector_score",
                         "auto_select_min_identity_confidence"):
                if getattr(self, name) is None:
                    problems.append(
                        f"selection.{name} is COMMISSION but the policy is "
                        f"AUTO_SELECT_SINGLE, which cannot run without it (§50)")
        return problems


# --------------------------------------------------------------------------
# Camera / preview / recording
# --------------------------------------------------------------------------

@dataclass
class CameraConfig:
    """§50's camera node. ``model_intrinsics`` means "whatever the loaded network says"."""

    frame_rate: str = "model_intrinsics"
    preserve_aspect_ratio: bool = True
    width: Optional[int] = None
    height: Optional[int] = None

    def to_dict(self) -> Dict[str, Any]:
        return {"frame_rate": self.frame_rate,
                "preserve_aspect_ratio": self.preserve_aspect_ratio,
                "width": self.width, "height": self.height}

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "CameraConfig":
        data = data or {}
        return cls(
            frame_rate=str(data.get("frame_rate", "model_intrinsics")),
            preserve_aspect_ratio=_as_bool(data.get("preserve_aspect_ratio"),
                                           "camera.preserve_aspect_ratio", True),
            width=(None if data.get("width") is None
                   else _as_int(data.get("width"), "camera.width", 0)),
            height=(None if data.get("height") is None
                    else _as_int(data.get("height"), "camera.height", 0)))


@dataclass
class PreviewConfig:
    """§39: queue depth one, latest frame only, preview may never block inference."""

    enabled: bool = True
    fps: float = 10.0
    latest_queue_depth: int = 1
    tap_path: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return {"enabled": self.enabled, "fps": self.fps,
                "latest_queue_depth": self.latest_queue_depth, "tap_path": self.tap_path}

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "PreviewConfig":
        data = data or {}
        return cls(
            enabled=_as_bool(data.get("enabled"), "preview.enabled", True),
            fps=_as_float(data.get("fps"), "preview.fps", 10.0),
            latest_queue_depth=_as_int(data.get("latest_queue_depth"),
                                       "preview.latest_queue_depth", 1),
            tap_path=str(data.get("tap_path", "")))

    def validate(self) -> List[str]:
        # §39 freezes queue depth at one. A configurable depth is how "the preview may
        # not block inference" quietly becomes true again after someone raises it to 8.
        return ([] if self.latest_queue_depth == 1
                else ["preview.latest_queue_depth must be 1 (§39: latest-only buffer)"])


@dataclass
class RecordConfig:
    """§43's recorder settings."""

    dataset_path: str = ""            # --record-dataset target
    replay_path: str = ""             # --replay source
    record_detections: bool = True    # normalized DetectionSets (Level B input)
    record_images: bool = False       # Level A needs images; large, opt-in
    event_log_path: str = ""          # §42's persisted critical events
    flush_every: int = 32

    def to_dict(self) -> Dict[str, Any]:
        return {"dataset_path": self.dataset_path, "replay_path": self.replay_path,
                "record_detections": self.record_detections,
                "record_images": self.record_images,
                "event_log_path": self.event_log_path, "flush_every": self.flush_every}

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "RecordConfig":
        data = data or {}
        return cls(
            dataset_path=str(data.get("dataset_path", "")),
            replay_path=str(data.get("replay_path", "")),
            record_detections=_as_bool(data.get("record_detections"),
                                       "record.record_detections", True),
            record_images=_as_bool(data.get("record_images"), "record.record_images", False),
            event_log_path=str(data.get("event_log_path", "")),
            flush_every=_as_int(data.get("flush_every"), "record.flush_every", 32))


def _snake(name: str) -> str:
    return name.replace("-", "_")


# --------------------------------------------------------------------------
# Root
# --------------------------------------------------------------------------

@dataclass
class VisionConfig:
    """§50's ``vision:`` document as a typed object."""

    profile: str = "person_detect"
    camera: CameraConfig = field(default_factory=CameraConfig)
    preview: PreviewConfig = field(default_factory=PreviewConfig)
    models: Dict[str, ModelConfig] = field(default_factory=dict)
    dedup: DedupConfig = field(default_factory=DedupConfig)
    tracking: TrackingConfig = field(default_factory=TrackingConfig)
    anchor: AnchorConfig = field(default_factory=AnchorConfig)
    selection: SelectionConfig = field(default_factory=SelectionConfig)
    record: RecordConfig = field(default_factory=RecordConfig)
    #: §40's per-stage timing window (samples retained for p50/p95/p99).
    timing_window: int = 512
    #: Where the file came from, so an error message can name it.
    source_path: str = ""

    # -- loading ------------------------------------------------------------
    @classmethod
    def from_dict(cls, data: Mapping[str, Any],
                  source_path: str = "") -> "VisionConfig":
        """Accept both §50's nested form (``{"vision": {...}}``) and the bare node."""
        root = dict(data or {})
        vision = root.get("vision")
        if isinstance(vision, Mapping):
            root = dict(vision)
        models_raw = root.get("models") or {}
        if not isinstance(models_raw, Mapping):
            raise ConfigError("models: must be a mapping of profile name to model")
        models = {str(name): ModelConfig.from_dict(str(name), model)
                  for name, model in models_raw.items()}
        return cls(
            profile=str(root.get("profile", "person_detect")),
            camera=CameraConfig.from_dict(root.get("camera")),
            preview=PreviewConfig.from_dict(root.get("preview")),
            models=models,
            dedup=DedupConfig.from_dict(root.get("dedup")),
            tracking=TrackingConfig.from_dict(root.get("tracking")),
            anchor=AnchorConfig.from_dict(root.get("anchor")),
            selection=SelectionConfig.from_dict(root.get("selection")),
            record=RecordConfig.from_dict(root.get("record")),
            timing_window=_as_int(root.get("timing_window"), "timing_window", 512),
            source_path=source_path)

    @classmethod
    def from_file(cls, path: str) -> "VisionConfig":
        """Read a YAML or JSON configuration (extension decides).

        YAML is optional on purpose: the daemon runs under the system interpreter, which
        has PyYAML, while the test venv does not. A subsystem that cannot be imported
        because a config parser is missing is a subsystem that cannot be tested offline,
        and §55.18 requires the acceptance tests to run with the motors disabled.
        """
        from .protocol.jsonio import load_mapping_document
        try:
            data = load_mapping_document(path, what="configuration")
        except ValidationError as exc:
            raise ConfigError(str(exc)) from exc
        return cls.from_dict(data, source_path=path)

    # -- access -------------------------------------------------------------
    @property
    def active_model(self) -> ModelConfig:
        """The profile selected at startup (§5: one model owns inference at a time)."""
        return self.model_for(self.profile)

    def model_for(self, profile: str) -> ModelConfig:
        try:
            return self.models[profile]
        except KeyError:
            known = ", ".join(sorted(self.models)) or "(none declared)"
            raise ConfigError(
                f"profile {profile!r} is not declared under models: (known: {known})"
                + (f" in {self.source_path}" if self.source_path else "")) from None

    def to_dict(self) -> Dict[str, Any]:
        return {"vision": {
            "profile": self.profile,
            "camera": self.camera.to_dict(),
            "preview": self.preview.to_dict(),
            "models": {name: model.to_dict() for name, model in self.models.items()},
            "dedup": self.dedup.to_dict(),
            "tracking": self.tracking.to_dict(),
            "anchor": self.anchor.to_dict(),
            "selection": self.selection.to_dict(),
            "record": self.record.to_dict(),
            "timing_window": self.timing_window,
        }}

    # -- validation ---------------------------------------------------------
    def validate(self, production: bool = True) -> List[str]:
        """Collect every problem with the configuration, then let the caller decide.

        ``production=False`` downgrades unresolved ``COMMISSION`` values from errors to
        warnings — the mode a replay or a unit test runs in, where §16.1's IoU is not
        being used to point a weapon at anything. It is still a *list* of warnings that
        gets printed, because §50's placeholder is a work item, and a silent work item is
        how the station ended up running on demo thresholds before.
        """
        problems: List[str] = []
        if not self.models:
            problems.append("no model profiles are declared under models:")
        else:
            self.active_model.validate()  # raises ConfigError if the profile is unknown
            problems.extend(self.active_model.validate())
        problems.extend(self.dedup.validate())
        problems.extend(self.tracking.validate())
        problems.extend(self.anchor.validate())
        problems.extend(self.selection.validate())
        problems.extend(self.preview.validate())
        if not production:
            return [p for p in problems if "COMMISSION" not in p]
        placeholders = [p for p in problems if "COMMISSION" in p]
        if placeholders:
            raise ConfigPlaceholderError(
                "production profile cannot start with unresolved COMMISSION values "
                "(§50):\n  - " + "\n  - ".join(placeholders))
        return problems

    def warnings(self) -> List[str]:
        """Non-fatal notes, including the §19 "these are starting values" reminder."""
        return [
            "tracking timings are §19's starting engineering values; re-measure them "
            "against station recordings (§19)",
        ]
