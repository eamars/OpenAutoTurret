"""Perception subpackage: model output contract, normalization, class filtering, dedup.

Everything downstream of ``TrackManager`` consumes ``DetectionSet`` (``types.py``) and
never a raw model tensor, which is what makes §9.2's compatibility oracle possible: the
reference example and this subsystem's adapter are compared on *normalized detections*,
not on tensor layouts.
"""
