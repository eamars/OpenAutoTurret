"""Classical bridge detector — P8 bring-up ONLY, not the production detector.

Rationale (`docs/research_vision_readiness_p7.md` option 3): the station can
stream IMX500 frames today, but the installed picamera2 has no detection API and
the platform has no RPK/Hailo stack, so no NN detector runs on the box. Closed
loop tracking (P8) still needs *something* that turns a frame into a
``Detection``, and the whole downstream chain (58-byte IPC §6.2, the selector
§12, the tracking FSM §34/§35, the reference arbitration §16) should be brought
up on real glass before the NN stack lands — not after.

This module is that placeholder: three-frame differencing + largest-blob
centroid, reported as ``class_id=<target_class_id>`` with a synthetic
confidence so the class-gating rules (§12.1) still get exercised. It has no
notion of "person". Two properties the operator must know before trusting it
in a live loop:
  * it is a **motion** detector — a target that stops moving is no longer a
    target, and the tracker will coast, then lost-search (§34) exactly as it
    would for a dropped NN detection;
  * the reported box is built from the two EDGE BANDS a moving body produces on
    a coarse grid, so its centre carries an error of order half the per-frame
    motion (a few px at bring-up speeds: on 1920 px with fx ≈ 1000 that is a
    few hundredths of a radian, well inside the tracking loop's own lag). It is
    a bring-up estimate, not a calibrated measurement — which is another reason
    the NN detector replaces it.

It is deliberately NOT named a detector model and it is never a substitute for
the §10.1 detector: when the RPK stack arrives, ``--detector rpk`` replaces it
and this file stays only for bench tests.

Pure computation on numpy arrays: NO camera, NO CAN, NO motor driver.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

from .frame_source import Detection


@dataclass
class MotionBlobConfig:
    # Coarse analysis grid (cells). The blob is found at grid resolution and
    # refined inside its own cells — cheap enough for a 200 Hz-hostile box.
    grid_cols: int = 64
    grid_rows: int = 36
    # Mean absolute luma change (0-255) within a cell to count as "moving".
    cell_threshold: float = 12.0
    # A track needs at least this many active cells (kills sensor-noise specks).
    min_cells: int = 4
    # Cells are dilated by this radius (in cells) before clustering, so a
    # target whose edges flicker stays one blob instead of a dust cloud.
    dilate_cells: int = 1
    # ...and the stitching radius may grow to this (see _target_cluster: the
    # two edge bands of one body have to be bridged). Beyond it they are
    # treated as two separate objects and the bigger one wins.
    bridge_max_cells: int = 8
    # Synthetic confidence band reported to the selector (§35 multiplies it).
    confidence_min: float = 0.30
    confidence_max: float = 0.85
    # Everything above maps to this class id so the §12.1 class gate is live.
    target_class_id: int = 1
    # First N frames only build the reference (no detections reported).
    warmup_frames: int = 2


class MotionBlobDetector:
    """Frame-difference blob detector (see module docstring for its scope)."""

    def __init__(self, cfg: Optional[MotionBlobConfig] = None) -> None:
        self._cfg = cfg or MotionBlobConfig()
        self._prev1: Optional[np.ndarray] = None
        self._prev2: Optional[np.ndarray] = None
        self._frames = 0
        self._last_energy = 0.0

    def reset(self) -> None:
        self._prev1 = None
        self._prev2 = None
        self._frames = 0
        self._last_energy = 0.0

    # -- core ---------------------------------------------------------------
    def detect(self, frame: np.ndarray) -> List[Detection]:
        """Return 0 or 1 Detection for the largest moving blob in `frame`.

        `frame` is HxWx3 (any channel order — luma is a channel mean) or HxW.
        """
        if frame is None or frame.ndim not in (2, 3):
            return []
        if frame.ndim == 3:
            gray = frame[:, :, :3].astype(np.float32).mean(axis=2)
        else:
            gray = frame.astype(np.float32)
        h, w = gray.shape
        cfg = self._cfg
        cell_h = max(1, h // cfg.grid_rows)
        cell_w = max(1, w // cfg.grid_cols)
        rows = h // cell_h
        cols = w // cell_w
        if rows < 3 or cols < 3:
            return []
        # Mean luma per cell (cropped to a whole number of cells).
        grid = gray[: rows * cell_h, : cols * cell_w]
        grid = grid.reshape(rows, cell_h, cols, cell_w).mean(axis=(1, 3))

        p1, p2 = self._prev1, self._prev2
        self._prev2, self._prev1 = p1, grid
        self._frames += 1
        if p1 is None or p2 is None or self._frames <= cfg.warmup_frames:
            return []  # three-frame differencing needs three grids

        s1 = grid - p1
        s2 = grid - p2
        diff1 = np.abs(s1)
        self._last_energy = float(diff1.mean())
        # A cell counts as "moving" only if it changed against BOTH of the last
        # two frames AND in the same direction. That rejects flicker/noise (a
        # threshold crossing that oscillates flips sign) while keeping the two
        # bands a moving body produces: the edge it is entering (brightening on
        # a bright target) and the edge it is leaving (dimming). Both belong to
        # the target's footprint, so both are kept and dilated into ONE blob.
        #
        # Consequence to state plainly: the two bands bound the footprint the
        # body swept over the last frames, so the centre is the footprint centre
        # — accurate to about half the per-frame motion (grid resolution + which
        # cells straddle the edge), never to sub-pixel. That is the whole point
        # of the §10.1 NN detector replacing this one for production.
        active = (diff1 > cfg.cell_threshold) & (np.abs(s2) > cfg.cell_threshold) \
            & ((s1 > 0) == (s2 > 0))
        cells = self._target_cluster(active, cfg)
        if not cells:
            return []
        in_cluster = np.zeros_like(active)
        in_cluster[[c[0] for c in cells], [c[1] for c in cells]] = True
        # Noise guard on REAL changed cells (not the stitched outline): a blob
        # of a few flickering cells must not become a target.
        if int(np.count_nonzero(active & in_cluster)) < cfg.min_cells:
            return []
        # Sub-cell refinement: weight the changed cells inside the cluster by
        # how much they changed, so the reported centre is smoother than the
        # 5-px grid. Only cells that genuinely changed against BOTH frames get
        # weight — the stitched outline between the two edge bands must not
        # pull the centre around.
        (r0, c0), (r1, c1) = _bounds(cells)
        weights = np.where(active & in_cluster, diff1, 0.0)
        sub = weights[r0 : r1 + 1, c0 : c1 + 1]
        total = float(sub.sum())
        if total <= 0.0:
            return []
        rr, cc = np.nonzero(sub)
        wts = sub[rr, cc]
        cy_cell = float((rr * wts).sum() / total) + r0
        cx_cell = float((cc * wts).sum() / total) + c0
        cx = (cx_cell + 0.5) * cell_w
        cy = (cy_cell + 0.5) * cell_h
        # Box: the cluster extent, so the anchor and the area both scale with
        # how much of the image the target covers.
        half_w = max(cell_w, (c1 - c0 + 1) * cell_w / 2.0)
        half_h = max(cell_h, (r1 - r0 + 1) * cell_h / 2.0)
        # Confidence grows with how much of the image actually moved (an area
        # proxy for "this is a body, not a speck"), bounded to the configured
        # band: the §35 confidence rules multiply it, it is not a probability.
        conf = float(np.clip(
            cfg.confidence_min
            + (cfg.confidence_max - cfg.confidence_min)
            * min(1.0, int(np.count_nonzero(active & in_cluster))
                  / max(1, 0.08 * rows * cols)),
            cfg.confidence_min,
            cfg.confidence_max,
        ))
        return [
            Detection(
                class_id=cfg.target_class_id,
                confidence=conf,
                bbox_x_min_px=max(0.0, cx - half_w),
                bbox_y_min_px=max(0.0, cy - half_h),
                bbox_x_max_px=min(float(w), cx + half_w),
                bbox_y_max_px=min(float(h), cy + half_h),
            )
        ]

    # -- internals ----------------------------------------------------------
    def _target_cluster(self, active: np.ndarray,
                        cfg: "MotionBlobConfig") -> List[Tuple[int, int]]:
        """Pick the target's cells, stitching a moving body's two edge bands.

        A solid body in motion lights up as TWO bands: the edge it is entering
        and the edge it is leaving. They are one object, but on the grid they
        are two clusters — and picking one of them (the classic "largest
        connected component", ties broken by scan order) would put the reported
        centre on the wake, i.e. systematically BEHIND the target. So grow the
        mask until the two biggest clusters merge (that merged footprint is the
        body), capped at ``bridge_max_cells`` in case they really are two
        different objects.
        """
        if not active.any():
            return []
        radius = max(0, cfg.dilate_cells)
        clusters = _clusters(_dilate(active, radius)) if radius else _clusters(active)
        significant = [c for c in clusters if len(c) >= cfg.min_cells]
        while len(significant) >= 2 and radius < cfg.bridge_max_cells:
            radius += 1
            merged = _clusters(_dilate(active, radius))
            a, b = set(significant[0]), set(significant[1])
            joined = [m for m in merged if (a & set(m)) and (b & set(m))]
            if joined:
                return max(joined, key=len)
            clusters = merged
            significant = [c for c in clusters if len(c) >= cfg.min_cells]
        return max(clusters, key=len) if clusters else []

    @property
    def last_motion_energy(self) -> float:
        """Mean cell change of the last frame (diagnostics / logs)."""
        return self._last_energy


# --- small helpers (no scipy dependency) ------------------------------------


def _dilate(mask: np.ndarray, radius: int) -> np.ndarray:
    out = mask.copy()
    for _ in range(radius):
        out = (
            out
            | np.roll(out, 1, axis=0)
            | np.roll(out, -1, axis=0)
            | np.roll(out, 1, axis=1)
            | np.roll(out, -1, axis=1)
        )
    return out


def _clusters(mask: np.ndarray) -> List[List[Tuple[int, int]]]:
    """All 4-connected clusters of `mask`, biggest first."""
    if not mask.any():
        return []
    seen = np.zeros_like(mask, dtype=bool)
    out: List[List[Tuple[int, int]]] = []
    rows, cols = mask.shape
    for r in range(rows):
        for c in range(cols):
            if not mask[r, c] or seen[r, c]:
                continue
            cluster: List[Tuple[int, int]] = []
            q = deque([(r, c)])
            seen[r, c] = True
            while q:
                y, x = q.popleft()
                cluster.append((y, x))
                for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    ny, nx = y + dy, x + dx
                    if 0 <= ny < rows and 0 <= nx < cols and mask[ny, nx] and not seen[ny, nx]:
                        seen[ny, nx] = True
                        q.append((ny, nx))
            out.append(cluster)
    out.sort(key=len, reverse=True)
    return out


def _bounds(cells: List[Tuple[int, int]]) -> Tuple[Tuple[int, int], Tuple[int, int]]:
    rs = [c[0] for c in cells]
    cs = [c[1] for c in cells]
    return (min(rs), min(cs)), (max(rs), max(cs))
