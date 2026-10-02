"""One Hailo context, one pipeline, fed by whichever camera is on the main display.

Owner, 2026-10-02: only the main display's camera is inferred; the PIP is display only. Both
cameras keep capturing (each has its own preview), and each camera's frame loop asks this switch,
under its lock, whether it is the one being inferred. The first frame of a newly selected camera
moves the adapter's binding, the pipeline's view and the wide preview's feed in one step, so no
frame is ever inferred against the other camera's geometry -- and tells the tracker the source
changed, which is where "re-acquire at aim" happens (TrackManager.note_source_change).

The lock is the whole concurrency story: the wide loop and the detail worker never run the adapter
at the same time, and the wire and document publishers are only ever called while it is held.
"""
from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple


@dataclass(frozen=True)
class View:
    camera_id: str
    leg: Tuple[int, int]            # the inference input (the ISP's small leg)
    declared: Tuple[int, int]       # the picture the adapter maps the network's boxes onto
    scale: float = 1.0              # 1: the wide frame itself; >1: narrower, mapped into the wide frame
    canvas: Optional[Tuple[int, int]] = None


class InferenceSwitch:
    def __init__(self, main_camera: Any, pipeline: Any, adapter: Any, views: Dict[str, View], *,
                 wide_preview: Any = None, handoff_gate_norm: float = 0.05,
                 handoff_window_ms: float = 1000.0) -> None:
        self.main_camera = main_camera
        self.pipeline = pipeline
        self.adapter = adapter
        self.views = dict(views)
        self.wide_preview = wide_preview
        self.handoff_gate_norm = float(handoff_gate_norm)
        self.handoff_window_ms = float(handoff_window_ms)
        self.lock = threading.Lock()
        self.serving: Optional[Tuple[str, int]] = None
        self.switches = 0
        self.last_sensor_ns = 0

    def acquire(self, role: str) -> bool:
        """With ``lock`` held: is ``role`` the camera to infer now? Re-binds on the first frame of a swap."""
        current, generation = self.main_camera.state()
        if current != role:
            return False
        view = self.views.get(role)
        if view is None:
            return False
        if self.serving != (current, generation):
            if self.serving is None or self.serving[0] != role:
                serve = getattr(self.adapter, "serve_camera", None)
                if callable(serve):
                    serve(view.camera_id, view.leg[0], view.leg[1], declared=view.declared)
                self.pipeline.view_scale = float(view.scale)
                self.pipeline.view_canvas = view.canvas
                # The wide preview is fed by the pipeline only while wide is inferred; otherwise the
                # wide loop offers its frames itself, so the PIP never shows the other camera.
                self.pipeline.preview = self.wide_preview if role == "wide" else None
                if self.serving is not None:
                    self.pipeline.manager.note_source_change(
                        gate_norm=self.handoff_gate_norm, window_ms=self.handoff_window_ms)
                    self.switches += 1
            self.serving = (current, generation)
        return True

    def fresh(self, sensor_ns: int) -> bool:
        """A frame older than the last one inferred (the other camera's, at a swap) is not offered:
        the tracker and the control loop both treat capture time as the order of the world."""
        if int(sensor_ns) <= self.last_sensor_ns:
            return False
        self.last_sensor_ns = int(sensor_ns)
        return True

    def stats(self) -> Dict[str, Any]:
        return {"serving": self.serving[0] if self.serving else None, "switches": self.switches,
                "views": {role: {"camera_id": v.camera_id, "leg": list(v.leg),
                                 "declared": list(v.declared), "scale": v.scale}
                          for role, v in self.views.items()}}
