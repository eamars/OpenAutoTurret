"""Apply the commissioned camera mounting convention before tracking geometry."""
from dataclasses import replace
from common.image_corrections import apply_orientation_bbox, validate_orientation
from .anchor import compute_anchor
from .types import BBox


def orient_detections(detection_set, orientation, anchor_config):
    validate_orientation(orientation)
    if orientation == 'none':
        return detection_set
    horizontal = orientation in ('rotate_180', 'flip_horizontal')
    vertical = orientation in ('rotate_180', 'flip_vertical')
    detections = []
    for detection in detection_set.detections:
        b = detection.bbox
        bbox = BBox(*apply_orientation_bbox((b.x_min, b.y_min, b.x_max, b.y_max), orientation, 1, 1))
        keypoints = tuple(replace(point,
            x=1-point.x if horizontal else point.x,
            y=1-point.y if vertical else point.y) for point in detection.keypoints)
        # A torso fraction is measured down from the upright box's top.
        anchor, source = compute_anchor(bbox, keypoints, anchor_config)
        detections.append(replace(detection, bbox=bbox, keypoints=keypoints,
                                  measured_anchor=anchor, anchor_source=source))
    return detection_set.with_detections(detections)
