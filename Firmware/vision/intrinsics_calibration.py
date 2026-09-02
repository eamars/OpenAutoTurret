"""§28.2 camera intrinsics from the ChArUco board — fit, and what the daemon can use.

Why this module exists: `calibration/camera_intrinsics.yaml` did not exist, so
controld logs `UNCALIBRATED` and uses a made-up pinhole (fx=fy=1000, principal
point at the centre). Every bearing the tracker computes is then wrong by an
unknown amount, and P8/P9 can only be interpreted "qualitatively". This turns
board views into the file the daemon actually loads.

The part that needs thinking, not coding: **the C++ v1 camera model applies no
distortion** (`camera_calibration.hpp`: k1..k3/p1..p2 are "stored but the v1
camera model applies none"). A normal OpenCV calibration quietly trades focal
length against k1, so writing `fx` from that fit into a file the daemon reads as
"no distortion" would give the daemon a focal length that is only correct at the
radius where the distortion happens to cancel. So this module fits BOTH:

  * `pinhole` — distortion pinned to zero (`CALIB_FIX_K1..K3` +
    `CALIB_ZERO_TANGENT_DIST`). **This is what gets written**, because it is the
    model the daemon implements; and
  * `full`  — plumb_bob with k1..k3/p1/p2, kept in the file as provenance,
    plus `edge_error_deg`: the angular difference between the two models at the
    image edge, which is the honest size of the error the v1 model makes.

If that edge error is bigger than you can live with, the answer is to undistort
in visiond or extend the C++ model — not to smuggle a distorted-fit focal length
into a pinhole file.
"""
from __future__ import annotations

import os
import tempfile
from dataclasses import dataclass, field
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np

from .installation_calibration import BoardSpec, detect_charuco, make_detector


# How much better the with-distortion fit must be before we believe it exists.
# 0.85 (a 15 % rms drop) is deliberately conservative: ChArUco corners from a
# hand-held board already carry ~0.3-0.5 px of noise, and a fit that only trims
# 5 % is chasing it.
DISTORTION_GAIN = 0.85


@dataclass
class Fit:
    fx: float = 0.0
    fy: float = 0.0
    cx: float = 0.0
    cy: float = 0.0
    rms_px: float = 0.0
    distortion: Tuple[float, ...] = (0.0,) * 5
    n_views: int = 0
    n_corners: int = 0

    def K(self) -> np.ndarray:
        return np.array([[self.fx, 0.0, self.cx],
                         [0.0, self.fy, self.cy],
                         [0.0, 0.0, 1.0]], dtype=np.float64)


@dataclass
class IntrinsicReport:
    pinhole: Fit
    full: Fit
    width: int
    height: int
    edge_error_deg: float = 0.0
    median_view_corners: int = 0
    skipped_views: int = 0
    notes: List[str] = field(default_factory=list)

    def summary(self) -> str:
        return (f"pinhole fx={self.pinhole.fx:.2f} fy={self.pinhole.fy:.2f} "
                f"cx={self.pinhole.cx:.2f} cy={self.pinhole.cy:.2f} "
                f"(rms {self.pinhole.rms_px:.3f} px over "
                f"{self.pinhole.n_views} views / {self.pinhole.n_corners} "
                f"corners, median {self.median_view_corners} corners/view); "
                f"with-distortion rms {self.full.rms_px:.3f} px, "
                f"edge model difference {self.edge_error_deg:.3f} deg")


def _to_gray(image: np.ndarray) -> np.ndarray:
    a = np.asarray(image)
    if a.ndim == 3:
        return a[..., 0]
    return a


def collect_views(images: Iterable[np.ndarray], spec: BoardSpec,
                  min_corners: int = 12
                  ) -> Tuple[List[Tuple[np.ndarray, np.ndarray]], int, List[int],
                             Tuple[int, int]]:
    """Detect the board in each image. Returns (views, skipped, per_view_n, size).

    A view needs `min_corners` matched ChArUco corners: fewer than that and the
    pose is under-constrained enough to pull the whole fit. The size of the
    FIRST view is authoritative — an intrinsics file describes ONE stream
    geometry, so a mixed-size batch is refused, not silently averaged.
    """
    board, _obj, detector = make_detector(spec)
    views: List[Tuple[np.ndarray, np.ndarray]] = []
    per_view: List[int] = []
    skipped = 0
    size: Optional[Tuple[int, int]] = None
    for image in images:
        g = _to_gray(image)
        h, w = g.shape[:2]
        if size is None:
            size = (int(w), int(h))
        elif (int(w), int(h)) != size:
            raise ValueError(
                f"mixed image sizes ({size} then {w}x{h}): intrinsics are per "
                "stream geometry; calibrate each resolution separately")
        obj_pts, img_pts, n = detect_charuco(
            np.stack([g] * 3, axis=-1), board, detector, min_corners=2)
        if img_pts is None or n < min_corners:
            skipped += 1
            per_view.append(int(n))
            continue
        views.append((obj_pts, img_pts))
        per_view.append(int(n))
    if size is None:
        raise ValueError("no images given")
    return views, skipped, per_view, size


def _calibrate(views: Sequence[Tuple[np.ndarray, np.ndarray]],
               size: Tuple[int, int], pinhole_only: bool) -> Fit:
    import cv2

    # float32, not float64: calibrateCamera in these builds demands
    # vector<Point3f>/vector<Point2f> (float64 dies with
    # "objectPoints should contain vector of vectors of points of type
    # Point3f"). The double->float32 quantisation is ~1e-7 of a square — far
    # below anything a hand-held board achieves.
    obj = [np.asarray(o, dtype=np.float32) for o, _ in views]
    img = [np.asarray(i, dtype=np.float32) for _, i in views]
    flags = 0
    if pinhole_only:
        # No CALIB_FIX_P1/P2 in these builds (measured on 4.10 and 5.0), so
        # zero the tangential terms instead of fixing them.
        flags = (cv2.CALIB_FIX_K1 | cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_K3
                 | cv2.CALIB_ZERO_TANGENT_DIST)
    rms, K, dist, _rv, _tv = cv2.calibrateCamera(
        obj, img, (int(size[0]), int(size[1])), None, None, flags=flags)
    d = np.asarray(dist, dtype=np.float64).ravel()
    d5 = tuple(list(d[:5]) + [0.0] * (5 - min(5, d.size)))
    return Fit(fx=float(K[0, 0]), fy=float(K[1, 1]), cx=float(K[0, 2]),
               cy=float(K[1, 2]), rms_px=float(rms), distortion=d5,
               n_views=len(views),
               n_corners=int(sum(o.shape[0] for o, _ in views)))


def edge_bearing_error_deg(pinhole: Fit, full: Fit,
                           size: Tuple[int, int]) -> float:
    """Max angular difference, over the image border, between the pinhole model
    the daemon runs and the fitted distorted model.

    This is the number that says what "we ignore distortion" costs in degrees,
    at the pixels a wide-FOV camera actually hits during a search sweep.
    """
    import cv2

    w, h = size
    xs = np.concatenate([np.linspace(0, w - 1, 25), np.linspace(0, w - 1, 25)])
    ys = np.concatenate([np.zeros(25), np.full(25, h - 1)])
    xs = np.concatenate([xs, np.linspace(0, w - 1, 25)])
    ys = np.concatenate([ys, np.zeros(25)])
    pts = np.stack([xs, ys], axis=-1).astype(np.float64)
    # Undistort under the FULL model into the pinhole fit's normalized frame:
    # with P=None OpenCV returns normalized camera coordinates (x = (u-cx)/fx).
    und = cv2.undistortPoints(
        pts.reshape(-1, 1, 2), full.K(),
        np.asarray(full.distortion, dtype=np.float64).reshape(1, -1))
    true_rad = np.arctan(np.linalg.norm(und.reshape(-1, 2), axis=1))
    pin = (pts - np.array([pinhole.cx, pinhole.cy])) / np.array(
        [pinhole.fx, pinhole.fy])
    pin_rad = np.arctan(np.linalg.norm(pin, axis=1))
    return float(np.max(np.abs(np.degrees(true_rad - pin_rad))))


def solve(views: Sequence[Tuple[np.ndarray, np.ndarray]],
          size: Tuple[int, int], per_view: Optional[Sequence[int]] = None,
          skipped: int = 0) -> IntrinsicReport:
    if len(views) < 3:
        raise ValueError(
            f"only {len(views)} usable views; an intrinsics fit needs at least "
            "3, and in practice 12+ with the board at different angles and "
            "distances (a flat, single-distance set cannot separate fx from "
            "distortion at all)")
    pinhole = _calibrate(views, size, pinhole_only=True)
    full = _calibrate(views, size, pinhole_only=False)
    # Only quote an edge-angle difference when the distortion fit actually earns
    # it. On low-distortion data (a synthetic set, or a well-corrected lens seen
    # from a narrow range of poses) the "full" fit happily invents k1 to chase
    # 0.4 px of warp noise — and the invented k1 then predicts a huge phantom
    # bearing error at the image edge. Reporting that number as fact would be
    # worse than reporting none.
    k_power = abs(full.distortion[0]) + abs(full.distortion[1])
    meaningful = (k_power > 1e-3
                  and full.rms_px < pinhole.rms_px * DISTORTION_GAIN)
    edge_err = edge_bearing_error_deg(pinhole, full, size) if meaningful else 0.0
    pv = list(per_view or [0])
    notes: List[str] = []
    if meaningful:
        notes.append(
            f"distortion is resolvable here: rms drops {pinhole.rms_px:.3f} -> "
            f"{full.rms_px:.3f} px when it is allowed. The written fx/fy are "
            "still the ZERO-DISTORTION fit, because that is the daemon's v1 "
            f"model; the {edge_err:.3f} deg edge bearing difference is what "
            "ignoring distortion costs at the frame edge. If that is too much, "
            "undistort in visiond or extend the C++ model - do not write a "
            "distorted-fit focal length into a pinhole file.")
    else:
        notes.append(
            "distortion was NOT resolvable from these views (fit gain below "
            f"{1 - DISTORTION_GAIN:.0%} with the same corner noise); "
            "edge_error_deg is therefore reported as 0 rather than as noise "
            "from an unconstrained k1. Get more views at more angles/distances "
            "if you need a distortion statement.")
    return IntrinsicReport(
        pinhole=pinhole, full=full, width=int(size[0]), height=int(size[1]),
        edge_error_deg=edge_err,
        median_view_corners=int(np.median([n for n in pv if n])) if pv else 0,
        skipped_views=skipped, notes=notes)


def intrinsics_text(report: IntrinsicReport, meta: Sequence[str] = ()) -> str:
    """The file body, in the format controld's loader reads (§28.2)."""
    from .installation_calibration import CameraIntrinsics

    fit = report.pinhole
    intr = CameraIntrinsics(fx=fit.fx, fy=fit.fy, cx=fit.cx, cy=fit.cy,
                            distortion=report.full.distortion,
                            width=report.width, height=report.height)
    lines = [
        "# Produced by tools/calibrate_camera_intrinsics.py",
        f"# views={fit.n_views} corners={fit.n_corners} "
        f"rms_pinhole={fit.rms_px:.4f}px rms_with_distortion="
        f"{report.full.rms_px:.4f}px",
        f"# edge bearing difference between the v1 pinhole model and the "
        f"fitted distortion model: {report.edge_error_deg:.3f} deg",
    ]
    lines += [f"# {m}" for m in meta]
    return "".join(ln + "\n" for ln in lines) + intr.to_yaml()


def write_intrinsics(path: str, report: IntrinsicReport,
                     meta: Sequence[str] = ()) -> str:
    """Write atomically (temp + rename, §41) and return the text written."""
    text = intrinsics_text(report, meta)
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix=".intr_", suffix=".tmp", dir=directory)
    try:
        with os.fdopen(fd, "w") as f:
            f.write(text)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)
    except BaseException:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise
    return text


# --------------------------------------------------------------------------- #
# Synthetic views: prove the toolchain before trusting it on real glass        #
# --------------------------------------------------------------------------- #

def render_synthetic_views(spec: BoardSpec, truth: Fit, n_views: int = 16,
                           size: Tuple[int, int] = (640, 480),
                           distance_range: Tuple[float, float] = (0.30, 0.75),
                           seed: int = 7) -> List[np.ndarray]:
    """Render the board at N poses through a KNOWN pinhole.

    A calibration tool that has never been run against ground truth is a
    rumour-mill: this function is how the tests (and `--self-test`) check that
    the collect -> solve path recovers a fx it was handed, on this machine, with
    this OpenCV build.
    """
    import cv2

    board, _obj, _detector = make_detector(spec)
    render = np.asarray(board.generateImage((1600, 1600)))
    if render.ndim == 3:
        render = render[..., 0]

    # metres-per-pixel of the render, MEASURED by detecting the flat board and
    # comparing to the board's metric corners. Assuming it (the "squares are
    # 2N+1" folklore) is how a printed board ends up 2x wrong.
    corners, ids, _mc, _mi = _detector.detectBoard(render)
    if corners is None or ids is None:
        raise RuntimeError("cannot self-test: the flat render is not detectable")
    obj_pts, img_pts = board.matchImagePoints(corners, ids)
    # matchImagePoints hands back (N,3) metric points in these builds; the
    # board is a plane, so only x/y matter for the similarity fit.
    obj_pts = np.asarray(obj_pts, dtype=np.float64).reshape(-1, 3)[:, :2]
    img_pts = np.asarray(img_pts, dtype=np.float64).reshape(-1, 2)
    # float32, or OpenCV's RANSAC helpers assert (measured on 4.10: float64
    # inputs die in ptsetreg with checkVector). LMEDS: deterministic, unlike the
    # default RANSAC, so a synthetic self-test stays reproducible.
    aff, _inl = cv2.estimateAffinePartial2D(
        obj_pts.astype(np.float32).reshape(-1, 1, 2),
        img_pts.astype(np.float32).reshape(-1, 1, 2), method=cv2.LMEDS)
    if aff is None:
        raise RuntimeError("cannot self-test: no similarity fit for the board")
    # A: board-plane metres -> render pixels. A_inv: render pixels -> metres.
    A = np.vstack([aff, [0.0, 0.0, 1.0]])
    A_inv = np.linalg.inv(A)

    rng = np.random.default_rng(seed)
    K = truth.K()
    views: List[np.ndarray] = []
    for i in range(n_views):
        rvec = np.array([rng.uniform(-0.5, 0.5), rng.uniform(-0.5, 0.5),
                         rng.uniform(-0.25, 0.25)], dtype=np.float64)
        tvec = np.array([rng.uniform(-0.06, 0.06), rng.uniform(-0.05, 0.05),
                         rng.uniform(*distance_range)], dtype=np.float64)
        R = cv2.Rodrigues(rvec)[0]
        # image = K [r1 r2 t] X_m, and X_m = A_inv * x_render
        H = K @ np.column_stack([R[:, 0], R[:, 1], tvec]) @ A_inv
        warped = cv2.warpPerspective(render, H, size, flags=cv2.INTER_NEAREST,
                                     borderValue=255)
        views.append(cv2.cvtColor(warped, cv2.COLOR_GRAY2BGR))
    return views
