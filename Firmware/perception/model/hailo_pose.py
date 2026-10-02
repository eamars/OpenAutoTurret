"""HailoRT adapter for the Hailo-8 YOLOv8s-pose HEF (Hailo Model Zoo v2.18.0, HailoRT 4.23.0).

Owner, 2026-10-02: aim at the head as the network measures it (nose, eyes, ears; else above the
shoulder line) instead of a fixed fraction of a person box, which moves with pose and with the
box's clipping at the frame edge. A pose network keeps the whole-body box for detection and
identity (it holds at range and from behind, where a face detector has nothing) and adds the
17 COCO keypoints the head anchor is measured from (detection/anchor.py, ``anchor.target``).

The HEF ends at the three YOLOv8 head scales (stride 32/16/8); decoding is on the host and follows
the Model Zoo's own post-processing for this network (``_yolov8_decoding``, v2.18):
  * per scale, three outputs: box distribution (4 sides x 16 DFL bins), person score (the
    compile script applies the sigmoid on the device), keypoints (17 x (x, y, visibility));
  * box: softmax over the 16 bins, expectation, times the stride, from the cell centre;
  * keypoint: x = stride * (2 * raw_x + column), y likewise; visibility = sigmoid(raw);
  * one class, greedy NMS at ``NMS_IOU``.
Only cells whose person score passes ``SCORE_FLOOR`` are decoded, so the host cost scales with
the people in view, not with the 8400 cells.
"""
from __future__ import annotations

from typing import Any, Dict, List, Tuple

import numpy as np

from ..errors import ModelRejected
from .hailo_yolo import HailoYoloAdapter

KEYPOINTS = 17
REG_BINS = 16
SCORE_FLOOR = 0.15     # below the lowest tracking threshold in use (low_association 0.15)
NMS_IOU = 0.45         # the Model Zoo's own evaluation setting for this network
MAX_DETECTIONS = 20
STRIDES = {20: 32, 40: 16, 80: 8}   # grid side of a 640x640 input -> stride


def _sigmoid(x: np.ndarray) -> np.ndarray:
    return 1.0 / (1.0 + np.exp(-x))


def _nms(boxes: np.ndarray, scores: np.ndarray, iou: float, keep_max: int) -> List[int]:
    """Greedy single-class NMS on [x1, y1, x2, y2] boxes."""
    order = np.argsort(-scores)
    area = np.maximum(0.0, boxes[:, 2] - boxes[:, 0]) * np.maximum(0.0, boxes[:, 3] - boxes[:, 1])
    keep: List[int] = []
    while order.size and len(keep) < keep_max:
        i = int(order[0])
        keep.append(i)
        rest = order[1:]
        xx1 = np.maximum(boxes[i, 0], boxes[rest, 0])
        yy1 = np.maximum(boxes[i, 1], boxes[rest, 1])
        xx2 = np.minimum(boxes[i, 2], boxes[rest, 2])
        yy2 = np.minimum(boxes[i, 3], boxes[rest, 3])
        inter = np.maximum(0.0, xx2 - xx1) * np.maximum(0.0, yy2 - yy1)
        union = area[i] + area[rest] - inter
        order = rest[np.where(union > 0, inter / np.maximum(union, 1e-12), 0.0) <= iou]
    return keep


def decode_yolov8_pose(outputs: Dict[int, Tuple[np.ndarray, np.ndarray, np.ndarray]], *,
                       score_floor: float = SCORE_FLOOR, nms_iou: float = NMS_IOU,
                       max_detections: int = MAX_DETECTIONS) -> List[Tuple[float, np.ndarray, np.ndarray]]:
    """{stride: (box HxWx64, score HxWx1, keypoints HxWx51)} -> [(score, [x1,y1,x2,y2], 17x3)].

    Coordinates are pixels of the 640x640 input tensor; keypoint visibility is a probability.
    """
    scores, boxes, kpts = [], [], []
    for stride, (box, score, kpt) in outputs.items():
        h, w = score.shape[0], score.shape[1]
        s = score.reshape(h * w)
        cells = np.nonzero(s >= score_floor)[0]
        if cells.size == 0:
            continue
        rows, cols = np.divmod(cells, w)
        cx = (cols + 0.5) * stride
        cy = (rows + 0.5) * stride
        dist = box.reshape(h * w, 4, REG_BINS)[cells].astype(np.float64)
        dist = np.exp(dist - dist.max(axis=-1, keepdims=True))
        dist /= dist.sum(axis=-1, keepdims=True)
        side = (dist * np.arange(REG_BINS)).sum(axis=-1) * stride      # left, top, right, bottom
        boxes.append(np.stack([cx - side[:, 0], cy - side[:, 1], cx + side[:, 2], cy + side[:, 3]], axis=1))
        raw = kpt.reshape(h * w, KEYPOINTS, 3)[cells].astype(np.float64)
        k = np.empty_like(raw)
        k[..., 0] = stride * (2.0 * raw[..., 0] + cols[:, None])
        k[..., 1] = stride * (2.0 * raw[..., 1] + rows[:, None])
        k[..., 2] = _sigmoid(raw[..., 2])
        kpts.append(k)
        scores.append(s[cells].astype(np.float64))
    if not scores:
        return []
    scores_a, boxes_a, kpts_a = np.concatenate(scores), np.concatenate(boxes), np.concatenate(kpts)
    keep = _nms(boxes_a, scores_a, nms_iou, max_detections)
    return [(float(scores_a[i]), boxes_a[i], kpts_a[i]) for i in keep]


class HailoYoloPoseAdapter(HailoYoloAdapter):
    """YOLOv8-pose on Hailo-8: person boxes with COCO-17 keypoints in the DetectionSet contract."""

    name = "hailo_pose"

    def _check_manifest(self) -> None:
        if self.manifest.task != "pose_estimation":
            raise ModelRejected(f"the pose adapter needs a pose_estimation manifest, got {self.manifest.task!r}")
        if self.manifest.labels != "coco":
            raise ModelRejected("the YOLOv8-pose HEF's single class is COCO person (label map 'coco')")
        if (self.manifest.bbox_order, self.manifest.bbox_normalized) != ("yxyx", True):
            raise ModelRejected("the pose adapter emits normalized [ymin,xmin,ymax,xmax] boxes")

    def _check_outputs(self, outputs) -> None:
        by_shape: Dict[Tuple[int, int, int], str] = {}
        for info in outputs:
            by_shape[tuple(int(v) for v in info.shape)] = info.name
        self._pose_outputs: Dict[int, Tuple[str, str, str]] = {}
        for side, stride in STRIDES.items():
            names = tuple(by_shape.get((side, side, c)) for c in (4 * REG_BINS, 1, 3 * KEYPOINTS))
            if any(n is None for n in names):
                raise ModelRejected(
                    f"YOLOv8-pose HEF lacks the {side}x{side} head (box 64, score 1, keypoints 51); "
                    f"outputs are {sorted(by_shape)}")
            self._pose_outputs[stride] = names       # type: ignore[assignment]
        if len(outputs) != 9:
            raise ModelRejected(f"expected the 9 YOLOv8-pose head outputs, got {len(outputs)}")

    def _row_layout(self) -> dict:
        return {"keypoint_index": 6, "keypoint_count": KEYPOINTS}

    def _parse(self, result, frame, pad: int) -> List[List[float]]:
        try:
            tensors = {}
            for stride, names in self._pose_outputs.items():
                arrays = []
                for name in names:
                    a = np.asarray(result[name])
                    if a.ndim == 4:
                        if a.shape[0] != 1:
                            raise ValueError(f"{name}: expected batch 1, got {a.shape}")
                        a = a[0]
                    if not np.isfinite(a).all():
                        raise ValueError(f"{name} contains non-finite values")
                    arrays.append(a)
                tensors[stride] = tuple(arrays)
            found = decode_yolov8_pose(tensors)
        except (KeyError, TypeError, ValueError, IndexError) as exc:
            self.failures += 1
            raise ModelRejected(f"unexpected YOLOv8-pose output: {exc}") from exc
        # Tensor pixels -> fractions of the frame that was fed, exactly as the NMS adapter does for
        # its boxes: x is whole-width, y undoes the letterbox pad added in infer().
        height = float(frame.shape[0])
        rows: List[List[float]] = []
        for score, (x1, y1, x2, y2), kp in found:
            lo, hi = (y1 - pad) / height, (y2 - pad) / height
            if hi <= 0.0 or lo >= 1.0:
                self.detections_pad_dropped += 1
                continue
            row = [score, 0.0, min(1.0, max(0.0, lo)), min(1.0, max(0.0, x1 / 640.0)),
                   min(1.0, max(0.0, hi)), min(1.0, max(0.0, x2 / 640.0))]
            for kx, ky, ks in kp:
                row += [kx / 640.0, (ky - pad) / height, ks]
            rows.append(row)
        return rows
