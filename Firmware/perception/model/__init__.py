"""The model layer: what the network is, what it claims, and what it produced.

Three artefacts and one interface:

``ModelManifest`` (§9.3) declares the contract of a specific ``.rpk`` — task, input size, box
order, box normalization, labels, nominal rate. It is checked in so it can be reviewed, diffed
and held against a station's actual install.

``EnvironmentManifest`` (§9.1) describes the station around it: OS, Python, kernel, the six
Debian packages that jointly *are* the IMX500 stack, and the model's hash. §9.1's rule is that
this is recorded before anything is upgraded, because "it works now" is not evidence unless the
machine is described.

``probe_model`` (Appendix D) asks the runtime what it thinks, and ``admit`` refuses a model
whose answer disagrees with the manifest.

``ModelAdapter`` is where all of that stops mattering: below it, the pipeline sees validated
``DetectionSet`` objects and one ``model_generation`` counter (§33).
"""
from __future__ import annotations

from .adapter import (OFFLINE_ADAPTERS, MockAdapter, ModelAdapter, build_adapter, manifest_for,
                      resolve_artifact, rows_for_moving_target)
from .compatibility_probe import ProbeResult, admit, probe_model
from .environment import EnvironmentManifest, labels_hash, sha256_file
from .label_maps import COCO, LabelMap, resolve
from .manifest import (MANIFEST_SCHEMA_VERSION, Disagreement, ModelManifest,
                       normalise_task)

__all__ = ["COCO", "Disagreement", "EnvironmentManifest", "LabelMap",
           "MANIFEST_SCHEMA_VERSION", "MockAdapter", "ModelAdapter", "ModelManifest",
           "OFFLINE_ADAPTERS",
           "ProbeResult", "admit", "build_adapter", "labels_hash", "manifest_for",
           "normalise_task", "probe_model", "resolve", "resolve_artifact",
           "rows_for_moving_target",
           "sha256_file"]
