#!/usr/bin/env python3
"""Extract static-background correspondences from timestamped camera packets.

No hardware commands. Forward/backward optical flow, person masks and a robust
homography reject independently moving objects. Saves points for geometry/time fits.
"""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np

cv2.setNumThreads(1)
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('capture', type=Path)
args = parser.parse_args()
data = json.loads((args.capture / 'capture.json').read_text())
frames = data['frames']
images = [cv2.resize(cv2.imread(str(args.capture / f['path']), 0), (960, 540)) for f in frames]


def mask(frame):
    result = np.full((540, 960), 255, np.uint8)
    for det in frame['detection_set']['detections']:
        if det['class_name'] != 'person':
            continue
        b = det['bbox']
        cv2.rectangle(result, (int(b['x_min'] * 960) - 12, int(b['y_min'] * 540) - 12),
            (int(b['x_max'] * 960) + 12, int(b['y_max'] * 540) + 12), 0, -1)
    result[:8] = result[-8:] = 0
    result[:, :8] = result[:, -8:] = 0
    return result


rows = []
for i in range(0, len(frames) - 3, 2):
    j = i + 3
    a, b = images[i], images[j]
    old_mask, new_mask = mask(frames[i]), mask(frames[j])
    pts = cv2.goodFeaturesToTrack(a, 250, .015, 16, mask=old_mask)
    if pts is None or len(pts) < 15:
        continue
    nxt, ok, err = cv2.calcOpticalFlowPyrLK(a, b, pts, None, winSize=(31, 31), maxLevel=4)
    back, ok2, _ = cv2.calcOpticalFlowPyrLK(b, a, nxt, None, winSize=(31, 31), maxLevel=4)
    end = np.rint(nxt[:, 0]).astype(int)
    good = ((ok.ravel() == 1) & (ok2.ravel() == 1) &
        (np.linalg.norm(back - pts, axis=2).ravel() < .7) & (err.ravel() < 20) &
        (end[:, 0] >= 0) & (end[:, 0] < 960) & (end[:, 1] >= 0) & (end[:, 1] < 540))
    indices = np.where(good)[0]
    indices = indices[new_mask[end[indices, 1], end[indices, 0]] != 0]
    aa, bb = pts[indices, 0] * 2, nxt[indices, 0] * 2
    if len(aa) < 15:
        continue
    H, inliers = cv2.findHomography(aa, bb, cv2.RANSAC, 2.)
    if H is None:
        continue
    aa, bb = aa[inliers.ravel() == 1], bb[inliers.ravel() == 1]
    if len(aa) < 12:
        continue
    rows.append({'i': i, 'j': j, 't0_ns': frames[i]['sensor_timestamp_ns'],
        't1_ns': frames[j]['sensor_timestamp_ns'], 'a': aa.tolist(), 'b': bb.tolist(),
        'image_motion_px': float(np.median(np.linalg.norm(bb - aa, axis=1))),
        'homography_residual_px': float(np.median(np.linalg.norm(
            cv2.perspectiveTransform(aa[:, None], H)[:, 0] - bb, axis=1))),
        'probe_stage': frames[i]['probe_stage']})
(args.capture / 'correspondences.json').write_text(json.dumps(rows))
print(json.dumps({'pairs': len(rows), 'features_median': float(np.median([len(x['a']) for x in rows])),
    'flow_fit_median_px': float(np.median([x['homography_residual_px'] for x in rows]))}))
