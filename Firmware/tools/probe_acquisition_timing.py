#!/usr/bin/env python3
"""Offline production association/selection timing with ideal synthetic detections.

No camera, sockets or motors. Sensor time is synthetic; this proves software
dwell behavior, not neural-network accuracy or physical acquisition latency.
"""
import json
from pathlib import Path
import sys

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from perception.config import VisionConfig
from perception.detection.types import AnchorSource, BBox, Detection, DetectionCounters, DetectionSet, PointNorm
from perception.tracking.track_manager import TrackManager
from perception.tracking.track import TrackState
from perception.selection.target_selection_manager import TargetSelectionManager


def run(rate):
    cfg=VisionConfig.from_file(str(Path(__file__).resolve().parents[1]/'perception/configs/perception_v1.json'))
    cfg.profile='person_detect_available'
    tracker=TrackManager(cfg)
    selector=TargetSelectionManager(cfg,alias_map=tracker.aliases,on_selected=tracker.set_selected_uuid)
    first_confirmed=first_selectable=first_selected=None
    frames=[]
    for index in range(int(rate*2)):
        ns=10_000_000_000+round(index*1e9/rate)
        d=Detection(detection_id_in_frame=0,class_id=0,class_name='person',detector_score=.9,
            bbox=BBox(.4,.3,.6,.8),measured_anchor=PointNorm(.5,.525),anchor_source=AnchorSource.BBOX_CENTER_FALLBACK)
        ds=DetectionSet(model_id=cfg.active_model.model_id,model_generation=1,frame_sequence=index+1,
            sensor_timestamp_ns=ns,publish_timestamp_ns=ns+60_000_000,stream_width=1920,stream_height=1080,
            detections=[d],counters=DetectionCounters(raw_outputs=1,post_model_nms=1))
        tracks=tracker.update(ds,ns+60_000_000)
        selector.update(tracks,ns+60_000_000,auto_roam_enabled=True)
        tr=tracks.tracks[0]
        elapsed=index*1000/rate
        if tr.state is TrackState.CONFIRMED_VISIBLE and first_confirmed is None:
            first_confirmed=elapsed
        if tr.selectable and first_selectable is None:
            first_selectable=elapsed
        if selector.state.has_selection and first_selected is None:
            first_selected=elapsed
        frames.append(dict(ms=elapsed,state=tr.state.label,selectable=tr.selectable,
            identity_confidence=tr.identity_confidence,selected=selector.state.has_selection))
        if first_selected is not None:
            break
    return dict(rate_hz=rate,first_confirmed_ms=first_confirmed,first_selectable_ms=first_selectable,
        first_selected_ms=first_selected,frames=frames)


if __name__=='__main__':
    print(json.dumps([run(rate) for rate in (26,52,1000)],indent=2))
