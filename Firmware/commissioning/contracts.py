"""Typed mathematical assets. Unknown, simulated and measured are distinct states."""
from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import Enum
import hashlib
import json
import math
import numbers
from pathlib import Path
from typing import Any

import numpy as np


class Reason(str, Enum):
    DATA_INVALID = "DATA_INVALID"
    MEASUREMENT_LIMITED = "MEASUREMENT_LIMITED"
    INSUFFICIENT_EXCITATION = "INSUFFICIENT_EXCITATION"
    OPERATING_POINT_CHANGED = "OPERATING_POINT_CHANGED"
    MODEL_INADEQUATE = "MODEL_INADEQUATE"
    ENVELOPE_LIMITED = "ENVELOPE_LIMITED"
    INTEGRATION_MISMATCH = "INTEGRATION_MISMATCH"
    HARD_ABORT = "HARD_ABORT"


class Rejected(ValueError):
    def __init__(self, reason: Reason, detail: str):
        self.reason, self.detail = reason, detail
        super().__init__(f"{reason.value}: {detail}")


def require(condition: bool, reason: Reason, detail: str) -> None:
    if not condition:
        raise Rejected(reason, detail)


def digest(value: Any) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                    allow_nan=False, ensure_ascii=False).encode()).hexdigest()


def array(value: Any, shape: tuple[int, ...], name: str) -> np.ndarray:
    try:
        if not (isinstance(value,np.ndarray) and value.dtype.kind in 'iuf'):
            raw=np.asarray(value,dtype=object)
            require(all(isinstance(v,numbers.Real) and not isinstance(v,(bool,np.bool_)) for v in raw.flat),
                    Reason.DATA_INVALID,f"{name}: finite real numbers required, not booleans or strings")
        out = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as exc:
        raise Rejected(Reason.DATA_INVALID, f"{name}: numeric array required") from exc
    require(out.shape == shape and bool(np.isfinite(out).all()), Reason.DATA_INVALID,
            f"{name}: expected finite shape {shape}")
    return out


@dataclass(frozen=True)
class ModelSpec:
    axis: str
    q_nodes: tuple[float, ...]
    z_nodes: tuple[float, ...]
    periodic: bool = False
    version: str = "adr0022.model/2"
    frame: str = "output_shaft_rad"

    def __post_init__(self):
        require(type(self.periodic) is bool,Reason.DATA_INVALID,"periodic must be an explicit boolean")
        require(self.version == "adr0022.model/2" and self.frame == "output_shaft_rad",
                Reason.DATA_INVALID, "unsupported model version or coordinate frame")
        require(self.axis in ("yaw", "pitch"), Reason.DATA_INVALID, "axis must be yaw/pitch")
        q = array(self.q_nodes, (8 if self.periodic else 5,), "q_nodes")
        z = array(self.z_nodes, (3,), "z_nodes")
        require(np.all(np.diff(q) > 0) and np.all(np.diff(z) > 0), Reason.DATA_INVALID,
                "strictly increasing position and posture knots required")
        require(np.allclose(np.diff(q), np.diff(q)[0]), Reason.DATA_INVALID,
                "position knots must be equally spaced")
        if self.periodic:
            require(self.axis == "yaw" and math.isclose(np.diff(q)[0] * 8, 2 * math.pi),
                    Reason.DATA_INVALID, "periodic yaw requires eight knots over a full turn")

    @property
    def size(self) -> int:
        return 6 + 6 * len(self.q_nodes) + 1  # a[3], b[3], h[2,3,nq], delay

    @property
    def identity(self) -> str:
        return digest(asdict(self))


@dataclass(frozen=True)
class Identity:
    hardware: str
    measurement: str
    operating_point: str
    provenance: str  # SYNTHETIC or MEASURED; neither implies physical qualification
    provisional_labels: bool = False

    def __post_init__(self):
        require(self.provenance in ("SYNTHETIC", "MEASURED"), Reason.DATA_INVALID,
                "explicit data provenance required")
        require(type(self.provisional_labels) is bool, Reason.DATA_INVALID,
                "provisional label status must be explicit")
        for key in (self.hardware, self.measurement, self.operating_point):
            if self.provisional_labels:
                require(isinstance(key, str) and bool(key.strip()), Reason.DATA_INVALID,
                        "provisional hardware, measurement and operating-point labels required")
                continue
            require(isinstance(key, str) and len(key) == 64 and
                    all(c in "0123456789abcdef" for c in key), Reason.DATA_INVALID,
                    "identities must be canonical content hashes")


@dataclass
class PlantSnapshot:
    spec: ModelSpec
    identity: Identity
    theta: np.ndarray
    uncertainty: np.ndarray
    train_hashes: tuple[str, ...]
    holdout_hashes: tuple[str, ...]
    frequency_band_hz: tuple[float, float]
    start_intervals: np.ndarray  # [negative/positive, z, q, not_moving/sustained]
    start_censored: np.ndarray
    fit_report: dict

    def __post_init__(self):
        self.theta = array(self.theta, (self.spec.size,), "theta")
        require(np.all(self.theta[:3] > 0) and np.all(self.theta[3:6] >= 0)
                and self.theta[-1] >= 0, Reason.DATA_INVALID, "a>0, b>=0, delay>=0 required")
        self.uncertainty = array(self.uncertainty, (128, self.spec.size), "joint bootstrap")
        require(np.all(self.uncertainty[:, :3] > 0) and
                np.all(self.uncertainty[:, 3:6] >= 0) and
                np.all(self.uncertainty[:, -1] >= 0), Reason.DATA_INVALID,
                "bootstrap contains nonphysical parameters")
        shape = (2, 3, len(self.spec.q_nodes))
        self.start_intervals = np.asarray(self.start_intervals, dtype=float)
        self.start_censored = np.asarray(self.start_censored)
        require(self.start_censored.shape == shape and self.start_censored.dtype == bool,
                Reason.DATA_INVALID, "explicit start censoring required")
        require(self.start_intervals.shape == shape+(2,) and
                np.isfinite(self.start_intervals[...,0]).all() and
                np.isfinite(self.start_intervals[...,1][~self.start_censored]).all(),
                Reason.DATA_INVALID, "noncensored startup intervals must have both measured endpoints")
        require(np.isnan(self.start_intervals[...,1][self.start_censored]).all(),
                Reason.DATA_INVALID, "censored startup upper endpoint must remain unknown")
        widths=np.array([-1,1])[:,None,None]*np.diff(self.start_intervals,axis=-1)[...,0]
        require(np.all(widths[~self.start_censored]>=0),Reason.DATA_INVALID,"startup endpoints reverse direction")
        require(bool(self.train_hashes) and bool(self.holdout_hashes) and
                not set(self.train_hashes) & set(self.holdout_hashes), Reason.DATA_INVALID,
                "whole-run training/holdout overlap or missing provenance")
        f = array(self.frequency_band_hz, (2,), "frequency band")
        require(0 <= f[0] < f[1], Reason.MEASUREMENT_LIMITED, "empty valid frequency band")

    def document(self) -> dict:
        return {"version": "adr0022.snapshot/2", "model_spec": json.loads(json.dumps(asdict(self.spec))),
                "model_spec_hash": self.spec.identity, "identity": asdict(self.identity),
                "units": {"a": "A*s^2/rad", "b": "A*s/rad", "h": "A", "delay": "s"},
                "theta": self.theta.tolist(), "uncertainty": self.uncertainty.tolist(),
                "train_hashes": list(self.train_hashes), "holdout_hashes": list(self.holdout_hashes),
                "frequency_band_hz": list(self.frequency_band_hz),
                "start_intervals": np.where(np.isnan(self.start_intervals),None,self.start_intervals).tolist(),
                "start_censored": self.start_censored.tolist(), "fit_report": self.fit_report,
                "qualification": "MATHEMATICAL_CANDIDATE"}

    @property
    def identity_hash(self) -> str:
        return digest(self.document())

    @classmethod
    def bind(cls, doc: dict, spec: ModelSpec, identity: Identity, *, physical: bool = False):
        require(doc.get("version") == "adr0022.snapshot/2" and
                doc.get("model_spec_hash") == spec.identity and
                doc.get("model_spec") == json.loads(json.dumps(asdict(spec))),
                Reason.INTEGRATION_MISMATCH, "model version, dimensions or coordinates differ")
        require(doc.get("identity") == asdict(identity), Reason.OPERATING_POINT_CHANGED,
                "hardware, calibration, operating point or provenance differs")
        require(doc.get("units") == {"a": "A*s^2/rad", "b": "A*s/rad", "h": "A", "delay": "s"},
                Reason.DATA_INVALID, "parameter units differ")
        require(not physical or identity.provenance == "MEASURED", Reason.DATA_INVALID,
                "synthetic parameters cannot enter a physical session")
        try:
            return cls(spec, identity, doc["theta"], doc["uncertainty"],
                       tuple(doc["train_hashes"]), tuple(doc["holdout_hashes"]),
                       tuple(doc["frequency_band_hz"]), doc["start_intervals"],
                       doc["start_censored"], doc["fit_report"])
        except KeyError as exc:
            raise Rejected(Reason.DATA_INVALID, f"missing parameter asset {exc}") from exc


def write_immutable(directory: Path, document: dict) -> Path:
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / f"{digest(document)}.json"
    encoded = json.dumps(document, indent=2, sort_keys=True, allow_nan=False) + "\n"
    if path.exists():
        require(path.read_text(encoding="utf-8") == encoded, Reason.DATA_INVALID,
                "immutable asset collision")
    else:
        with path.open("x", encoding="utf-8") as stream:
            stream.write(encoded)
    return path
