"""OpenAutoTurret — installation orientation calibration (architecture §29).

Visually calibrate the base->world rotation ``R_W_B`` from a fiducial board
(ChArUco, §29.1) that is physically levelled or plumb during commissioning.
The levelled board is the gravity reference: its frame coincides with the world
frame up to a known, usually identity, ``R_W_D``. The camera-to-base extrinsic
``R_P_C`` comes from the camera calibration (§28.3). Per observed frame we:

  1. detect the board corners (cv2 ChArUco detector),
  2. solve the camera pose relative to the board (PnP), giving ``R_C_D``/``t_C_D``
     and a reprojection error (§29.2 inputs: intrinsics + fiducial geometry +
     detected corners + current calibrated yaw/pitch + extrinsics),
  3. derive ``R_W_B = R_W_D @ (R_P_C @ R_C_D)^T``,

then collect N frames, reject outliers, and average (§29.3). The result is
committed atomically to the same text format that controld's
``FixedStoredPoseProvider`` reads (``control/src/calibration/installation_pose.hpp``).

Safety: this module NEVER touches CAN or the motor driver. Camera frames are
injected by the caller (CLI), so the math is fully testable on synthetic frames.
"""
from __future__ import annotations

import math
import os
import struct
import tempfile
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Sequence, Tuple

import numpy as np

# cv2 is imported lazily so the pure-math helpers (R_W_B derivation, outlier
# rejection, persistence) are importable and testable without OpenCV installed.
def _cv2():
    import cv2  # noqa: WPS433 (import guarded)
    return cv2


# --------------------------------------------------------------------------- #
# Board / camera configuration                                                 #
# --------------------------------------------------------------------------- #

# ArUco dictionaries available in this cv2 build.
_DICT_BY_NAME = {
    "DICT_4X4_50": "DICT_4X4_50",
    "DICT_4X4_100": "DICT_4X4_100",
    "DICT_4X4_250": "DICT_4X4_250",
    "DICT_4X4_1000": "DICT_4X4_1000",
    "DICT_5X5_50": "DICT_5X5_50",
    "DICT_5X5_100": "DICT_5X5_100",
}


@dataclass
class BoardSpec:
    """A ChArUco board with known metric geometry (§29.1 / §29.2)."""

    marker_cols: int = 3
    marker_rows: int = 3
    square_length_m: float = 0.030   # chessboard square side (must be > marker)
    marker_length_m: float = 0.020   # ArUco marker side
    dictionary: str = "DICT_4X4_50"
    marker_id_offset: int = 0

    def make_cv_board(self):
        """Build the cv2 CharucoBoard and return (board, object_points Nx3)."""
        cv2 = _cv2()
        aruco = cv2.aruco
        if self.square_length_m <= self.marker_length_m:
            raise ValueError(
                "charuco requires square_length_m > marker_length_m "
                f"({self.square_length_m} vs {self.marker_length_m})"
            )
        if self.dictionary not in _DICT_BY_NAME:
            raise ValueError(f"unknown dictionary {self.dictionary!r}")
        dict_ = aruco.getPredefinedDictionary(getattr(aruco, self.dictionary))
        # Two traps, both measured on this station's OpenCV (4.10 and 5.0 agree):
        #
        #  1. The first argument is SQUARES, not markers: markers are placed on
        #     every black square, so (3, 3) is a 3x3-square board with 4 markers
        #     and 4 ChArUco corners. Anything that sizes a *printed* board from
        #     "markers" is off by ~2.3x and the detector silently returns
        #     nothing — see vision/charuco_board.py, which sizes by measurement.
        #  2. marker_id_offset must NOT be passed positionally. In the Python
        #     binding the 6th positional parameter is the ids VECTOR, so passing
        #     an int either crashes ("Size of ids must be equal to the number of
        #     markers: 18") for bigger grids or yields a board whose ids do not
        #     match, making CharucoDetector.detectBoard() return None while
        #     ArucoDetector happily sees the markers. Both failure modes look
        #     like "the board in the room is wrong", at the station.
        board = aruco.CharucoBoard(
            (self.marker_cols, self.marker_rows),
            self.square_length_m,
            self.marker_length_m,
            dict_,
        )
        if self.marker_id_offset:
            board = self._with_id_offset(board, aruco, self.marker_id_offset)
        obj = np.asarray(board.getObjPoints(), dtype=np.float64).reshape(-1, 3)
        n_markers = obj.shape[0] // 4
        # Dictionary capacity: exceeding it aborts deep inside generateImageMarker.
        # bytesList is the dictionary's id table: a numpy array in the Python
        # binding (a Mat elsewhere) — one row per usable id.
        bl = dict_.bytesList
        capacity = (int(bl.shape[0]) if hasattr(bl, "shape") else int(bl.rows)) \
            - int(self.marker_id_offset)
        if n_markers > capacity:
            raise ValueError(
                f"board needs {n_markers} marker ids but {self.dictionary} has "
                f"{capacity} available from offset {self.marker_id_offset}; "
                f"use a bigger dictionary (e.g. DICT_4X4_1000) or a smaller grid"
            )
        return board, obj

    @staticmethod
    def _with_id_offset(board, aruco, offset: int):
        """Rebuild a board shifted by `offset` ids, or refuse.

        Only the keyword form is safe (see make_cv_board); if this binding does
        not support it, fail loudly instead of handing back a mis-id'd board.
        """
        try:
            ids = np.arange(int(np.asarray(board.getObjPoints()).shape[0] / 4),
                            dtype=np.int32) + int(offset)
            return aruco.CharucoBoard(
                board.getSize() if hasattr(board, "getSize") else
                (board.getSquaresX(), board.getSquaresY()),
                board.getSquareLength(), board.getMarkerLength(),
                board.getPredefinedDictionary() if hasattr(
                    board, "getPredefinedDictionary") else board.dictionary,
                ids,
            )
        except Exception as e:  # noqa: BLE001
            raise ValueError(
                f"this OpenCV binding cannot shift ChArUco marker ids by "
                f"{offset} ({e}); leave marker_id_offset=0") from e


def _legacy_yaml(path: str) -> dict:
    """Parse a legacy YAML-mapping calibration file, or explain why we cannot.

    The key=value format these modules WRITE needs no dependencies; YAML is
    only reached for files produced before that format was fixed. The station's
    /usr/bin/python3 has PyYAML; the test venv does not, so say what is missing
    and how to move off it rather than raising a bare ImportError.
    """
    try:
        import yaml
    except ImportError as e:  # pragma: no cover - environment dependent
        raise ValueError(
            f"{path} is in the legacy YAML mapping format and PyYAML is not "
            f"installed in this interpreter; re-save it with the key=value "
            f"format (CameraIntrinsics/CameraExtrinsics .to_yaml())") from e
    with open(path) as f:
        return yaml.safe_load(f)


def parse_key_value_file(path: str) -> dict:
    """Parse a `key=value` calibration file (the C++ contract), or {}.

    WHY THIS EXISTS: the daemon's loaders (`load_camera_intrinsics`,
    `load_camera_extrinsics`, `load_installation_pose`) read `key=value` with
    `#` comments and a trailing raw-number section. The Python writers here used
    to emit YAML mapping text (`fx: 1420.5`, `R_P_C:`), which the C++ parsers
    cannot see at all - `fx: 1.0` has no `=`, so it lands in the raw section and
    the loader reports "missing key(s); intrinsics NOT applied" / "no 3x3
    rotation found" and quietly keeps the UNCALIBRATED defaults. Strict writer,
    tolerant reader: we write the documented format and still read the old one.
    """
    out = {}
    try:
        with open(path) as f:
            text = f.read()
    except OSError:
        return out
    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        k, v = line.split("=", 1)
        out[k.strip()] = v.strip()
    return out


@dataclass
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    distortion: Sequence[float] = (0.0, 0.0, 0.0, 0.0, 0.0)
    width: int = 640
    height: int = 480

    @property
    def K(self) -> np.ndarray:
        return np.array(
            [[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )

    @property
    def dist(self) -> np.ndarray:
        return np.asarray(list(self.distortion), dtype=np.float64)

    @classmethod
    def load(cls, path: str) -> "CameraIntrinsics":
        kv = parse_key_value_file(path)
        if "fx" in kv:                      # the format controld also reads
            d = dict(kv)
            dist = tuple(float(kv.get(k, 0.0))
                         for k in ("k1", "k2", "p1", "p2", "k3"))
            d["distortion"] = dist
        else:                               # legacy YAML-mapping file
            d = _legacy_yaml(path)
        return cls(
            fx=float(d["fx"]), fy=float(d["fy"]), cx=float(d["cx"]),
            cy=float(d["cy"]),
            distortion=tuple(float(x) for x in d.get("distortion", [0.0] * 5)),
            width=int(d.get("width", 640)),
            height=int(d.get("height", 480)),
        )

    def to_yaml(self) -> str:
        """`# ota-camera-intrinsics v1` + key=value (see camera_calibration.hpp).

        The six keys fx/fy/cx/cy/width/height are MANDATORY: the loader applies
        nothing if any is missing. k1..k3/p1/p2 are stored for provenance and
        are NOT applied by the v1 C++ camera model (Part 3 item 14), which is
        why tools/calibrate_camera_intrinsics.py fits the pinhole values with
        the distortion held at zero instead of quietly writing a fit the daemon
        will only partly use.
        """
        d = list(self.distortion) + [0.0] * 5
        lines = [
            "# ota-camera-intrinsics v1",
            f"fx={self.fx:.17g}",
            f"fy={self.fy:.17g}",
            f"cx={self.cx:.17g}",
            f"cy={self.cy:.17g}",
            f"width={int(self.width)}",
            f"height={int(self.height)}",
        ]
        if any(abs(x) > 0.0 for x in d):
            lines.append("dist_model=plumb_bob")
            for name, val in zip(("k1", "k2", "p1", "p2", "k3"), d):
                lines.append(f"{name}={val:.17g}")
        return "\n".join(lines) + "\n"


@dataclass
class CameraExtrinsics:
    """Camera-to-base extrinsic (the kinematics ``R_P_C``, §28.3).

    ``R_P_C`` rotates a camera-frame vector into the base frame (the inverse of
    the base->camera rotation). ``aligned()`` is the ideal-aligned camera.
    """

    R_P_C: np.ndarray = field(default_factory=lambda: np.identity(3))
    t_P_C: np.ndarray = field(default_factory=lambda: np.zeros(3))

    @classmethod
    def aligned(cls) -> "CameraExtrinsics":
        return cls(R_P_C=np.identity(3), t_P_C=np.zeros(3))

    @classmethod
    def load(cls, path: str) -> "CameraExtrinsics":
        """Read our own format (header + 3 rows) or the legacy YAML mapping."""
        with open(path) as f:
            text = f.read()
        rows = [ln.strip() for ln in text.splitlines()
                if ln.strip() and not ln.startswith("#") and "=" not in ln]
        try:                       # our format: three rows of three numbers
            R = np.array([[float(x) for x in r.split()] for r in rows[:3]],
                         dtype=np.float64)
            if R.shape != (3, 3):
                raise ValueError(R.shape)
        except (ValueError, IndexError):
            R = None               # legacy `R_P_C:` YAML text, not raw rows
        if R is not None:
            kv = parse_key_value_file(path)
            t = np.array([float(x) for x in
                          kv.get("t_P_C", "0 0 0").replace(",", " ").split()],
                         dtype=np.float64)
            if t.size != 3:
                t = np.zeros(3)
            return cls(R_P_C=R, t_P_C=t)
        d = _legacy_yaml(path)
        R = np.array(d.get("R_P_C", np.identity(3)), dtype=np.float64)
        t = np.array(d.get("t_P_C", np.zeros(3)), dtype=np.float64)
        return cls(R_P_C=R, t_P_C=t)

    def to_yaml(self) -> str:
        """`# ota-camera-extrinsics v1` + a raw 3x3 section (camera_calibration).

        The C++ loader reads EXACTLY nine numbers from the non-key lines and
        then checks orthonormality, so the rotation must be raw rows - not
        `R_P_C:` YAML, which produced "no 3x3 rotation found (using aligned
        camera mount)" and a silently uncalibrated station. `t_P_C` is a key
        line: unknown keys are ignored by the daemon, and we keep the offset for
        our own round-trip (§28.3 treats it as commissioning data).
        """
        rows = "\n".join(
            " ".join(f"{v:.17g}" for v in row) for row in self.R_P_C)
        return ("# ota-camera-extrinsics v1\n"
                "t_P_C=" + " ".join(f"{v:.17g}" for v in self.t_P_C) + "\n"
                + rows + "\n")


# --------------------------------------------------------------------------- #
# Pose solve + R_W_B derivation                                                #
# --------------------------------------------------------------------------- #

@dataclass
class FramePose:
    """One detected board observation (§29.3 per-frame pose)."""

    R_C_D: np.ndarray        # board -> camera rotation (PnP)
    t_C_D: np.ndarray        # board origin in camera frame (m)
    reprojection_error_px: float
    timestamp_ns: int
    n_corners: int

    @property
    def R_D_C(self) -> np.ndarray:
        return self.R_C_D.T


def _rodrigues(rvec: np.ndarray) -> np.ndarray:
    return np.asarray(_cv2().Rodrigues(np.asarray(rvec).reshape(3, 1))[0])


def _geodesic_deg(R_a: np.ndarray, R_b: np.ndarray) -> float:
    """Angle (deg) between two rotations via their relative rotation."""
    c = (np.trace(R_a.T @ R_b) - 1.0) / 2.0
    c = float(np.clip(c, -1.0, 1.0))
    return math.degrees(math.acos(c))


def solve_board_pose(
    obj_points: np.ndarray,
    image_points: np.ndarray,
    intrinsics: CameraIntrinsics,
    timestamp_ns: int,
) -> FramePose:
    """Solve the board->camera pose (PnP) from 3D/2D correspondences.

    Returns a :class:`FramePose` with the recovered rotation/translation and the
    mean reprojection error in pixels (§29.3 "report reprojection error").
    """
    cv2 = _cv2()
    obj = np.asarray(obj_points, dtype=np.float64).reshape(-1, 3)
    img = np.asarray(image_points, dtype=np.float64).reshape(-1, 2)
    if len(obj) < 4 or len(obj) != len(img):
        raise ValueError(f"need >=4 matching points, got {len(obj)}/{len(img)}")
    success, rvec, tvec = cv2.solvePnP(
        obj, img, intrinsics.K, intrinsics.dist,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not success:
        raise RuntimeError("solvePnP failed")
    R = _rodrigues(rvec)
    t = np.asarray(tvec, dtype=np.float64).reshape(3)
    proj, _ = cv2.projectPoints(obj, rvec, tvec, intrinsics.K, intrinsics.dist)
    err = float(np.mean(np.linalg.norm(proj.reshape(-1, 2) - img, axis=-1)))
    return FramePose(R, t, err, timestamp_ns, len(obj))


def derive_R_W_B(
    R_C_D: np.ndarray,
    R_P_C: np.ndarray,
    R_W_D: Optional[np.ndarray] = None,
) -> np.ndarray:
    """Derive the base->world rotation from a board pose.

    Frames: C = camera, B = base, D = (levelled) board, W = world.
      * PnP gives ``R_C_D`` (board -> camera), so ``v_D = R_C_D^T v_C``.
      * ``R_P_C`` (camera -> base): ``v_C = R_P_C^T v_B``.
      * Levelled board: ``v_W = R_W_D v_D`` (identity when the board axes line
        up with the world).
    Chaining: ``R_W_B = R_W_D @ (R_P_C @ R_C_D)^T``.
    """
    if R_W_D is None:
        R_W_D = np.identity(3)
    R_W_B = R_W_D @ (np.asarray(R_P_C) @ np.asarray(R_C_D)).T
    return _re_orthonormal(R_W_B)


def _re_orthonormal(R: np.ndarray) -> np.ndarray:
    """Project a near-rotation back onto SO(3) (removes numerical drift).

    A matrix that is already orthonormal with det=+1 is returned unchanged:
    SVD is degenerate for a perfect rotation (all singular values 1) and would
    otherwise inject ~1e-8 rad of spurious error.
    """
    R = np.asarray(R, dtype=np.float64)
    if (np.linalg.norm(R @ R.T - np.identity(3)) < 1e-10
            and np.linalg.det(R) > 0):
        return R
    U, _, Vt = np.linalg.svd(R)
    out = U @ Vt
    if np.linalg.det(out) < 0:
        U[:, -1] *= -1
        out = U @ Vt
    return out


# --------------------------------------------------------------------------- #
# Multi-frame calibration (outlier rejection + average, §29.3)                 #
# --------------------------------------------------------------------------- #

@dataclass
class CalibrationResult:
    R_W_B: np.ndarray
    covariance: float          # scalar attitude variance (rad^2)
    n_frames: int              # frames that went into the result
    n_rejected: int            # frames rejected as outliers
    reprojection_error_px: float
    per_frame_R_W_B: List[np.ndarray] = field(default_factory=list)
    per_frame_error_px: List[float] = field(default_factory=list)
    timestamp_ns: int = 0

    @property
    def valid(self) -> bool:
        return self.n_frames >= 3


def _average_rotations(Rs: Sequence[np.ndarray]) -> np.ndarray:
    S = sum((np.asarray(R, dtype=np.float64) for R in Rs), start=np.zeros((3, 3)))
    U, _, Vt = np.linalg.svd(S)
    out = U @ Vt
    if np.linalg.det(out) < 0:
        U[:, -1] *= -1
        out = U @ Vt
    return out


def calibrate_from_poses(
    poses: Sequence[FramePose],
    R_P_C: np.ndarray,
    R_W_D: Optional[np.ndarray] = None,
    max_outlier_deg: float = 3.0,
    min_frames: int = 3,
) -> CalibrationResult:
    """Derive R_W_B per frame, reject outliers, and average (§29.3).

    Two passes: a first consensus (SVD mean) seeds the outlier threshold; frames
    farther than ``max_outlier_deg`` are dropped; the final R_W_B is the SVD mean
    of the accepted frames. The scalar covariance is the variance (rad^2) of the
    accepted frames' angular distance to the final mean.
    """
    if len(poses) < min_frames:
        return CalibrationResult(
            R_W_B=identity_R_W_B(), covariance=1e6, n_frames=len(poses),
            n_rejected=0, reprojection_error_px=0.0,
        )
    per_frame = [derive_R_W_B(p.R_C_D, R_P_C, R_W_D) for p in poses]
    # The plain SVD mean is sensitive to outliers: with a few gross outliers it
    # is pulled enough that even inliers can exceed a tight threshold. Anchor the
    # rejection to the frame nearest the initial consensus (a reliable inlier) so
    # the threshold is measured against an unpolluted reference (§29.3).
    consensus = _average_rotations(per_frame)
    dists = [_geodesic_deg(r, consensus) for r in per_frame]
    reference = per_frame[int(np.argmin(dists))]
    ref_dists = [_geodesic_deg(r, reference) for r in per_frame]
    keep = [i for i, d in enumerate(ref_dists) if d <= max_outlier_deg]
    if len(keep) < min_frames:
        keep = list(range(len(poses)))  # not enough inliers: keep all (flag via n)
    final = _average_rotations([per_frame[i] for i in keep])
    final_dists = [_geodesic_deg(per_frame[i], final) for i in keep]
    cov = float(np.var(np.deg2rad(final_dists))) if len(keep) >= 2 else 1e6
    reproj = float(np.mean([poses[i].reprojection_error_px for i in keep]))
    return CalibrationResult(
        R_W_B=final,
        covariance=cov,
        n_frames=len(keep),
        n_rejected=len(poses) - len(keep),
        reprojection_error_px=reproj,
        per_frame_R_W_B=[per_frame[i] for i in keep],
        per_frame_error_px=[poses[i].reprojection_error_px for i in keep],
        timestamp_ns=max((poses[i].timestamp_ns for i in keep), default=0),
    )


def identity_R_W_B() -> np.ndarray:
    return np.identity(3)


# --------------------------------------------------------------------------- #
# Detection wrapper                                                            #
# --------------------------------------------------------------------------- #

def make_detector(board_spec: BoardSpec):
    """Return (cv_board, object_points, detector) for a BoardSpec."""
    cv2 = _cv2()
    board, obj = board_spec.make_cv_board()
    detector = cv2.aruco.CharucoDetector(
        board, cv2.aruco.CharucoParameters())
    return board, obj, detector


def detect_charuco(
    frame_bgr: np.ndarray,
    board,
    detector,
    min_corners: int = 4,
):
    """Detect the board in a frame.

    Returns ``(object_points Nx3, image_points Nx2, n)`` where the two arrays
    are already matched (via ``board.matchImagePoints``), ready for PnP.
    Returns ``(None, None, 0)`` if too few corners are found.
    """
    corners, ids, _mc, _mi = detector.detectBoard(frame_bgr)
    if corners is None or ids is None:
        return None, None, 0
    n = int(np.asarray(ids).size)
    if n < min_corners:
        return None, None, 0
    obj_pts, img_pts = board.matchImagePoints(corners, ids)
    obj_pts = np.asarray(obj_pts, dtype=np.float64).reshape(-1, 3)
    img_pts = np.asarray(img_pts, dtype=np.float64).reshape(-1, 2)
    return obj_pts, img_pts, n


# --------------------------------------------------------------------------- #
# Atomic persistence (C++-compatible format, §29.3 / §41)                      #
# --------------------------------------------------------------------------- #

def commit_R_W_B(result: CalibrationResult, path: str) -> None:
    """Atomically write the calibrated R_W_B (temp file + rename, §41).

    The format matches controld's ``load_installation_pose`` so the committed
    file is directly loadable by the ``FixedStoredPoseProvider``.
    """
    R = np.asarray(result.R_W_B, dtype=np.float64)
    lines = [
        "# ota-installation-pose v1",
        "source=visual_calibration",
        # `valid=0`, not a bare `0`. Probed against the real loader: a line
        # without `=` is read as a matrix row and needs THREE doubles, so a bare
        # `0` was silently dropped - `valid` then fell back to the parser's
        # default of false, which is the right answer here only by accident, and
        # the file no longer carried its own verdict when you `cat` it. Emit the
        # documented key (installation_pose.hpp) instead of relying on defaults.
        "valid=1" if result.valid else "valid=0",
        f"timestamp_ns={result.timestamp_ns}",
        f"covariance={result.covariance:.17g}",
        f"n_frames={result.n_frames}",
        f"reprojection_error_px={result.reprojection_error_px:.17g}",
    ]
    for i in range(3):
        lines.append(
            " ".join(f"{R[i, j]:.17g}" for j in range(3)))
    payload = "\n".join(lines) + "\n"

    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix=".install_", suffix=".tmp", dir=directory)
    try:
        with os.fdopen(fd, "w") as f:
            f.write(payload)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)  # atomic on POSIX
    except BaseException:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise


def load_R_W_B(path: str) -> Optional[np.ndarray]:
    """Read a committed R_W_B (or None if missing/invalid)."""
    try:
        with open(path) as f:
            text = f.read()
    except OSError:
        return None
    matrix: List[float] = []
    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" in line:
            continue
        parts = line.split()
        if len(parts) == 3:
            try:
                matrix.extend(float(x) for x in parts)
            except ValueError:
                return None
    if len(matrix) != 9:
        return None
    R = np.array(matrix, dtype=np.float64).reshape(3, 3)
    if not np.all(np.isfinite(R)):
        return None
    return _re_orthonormal(R)


# --------------------------------------------------------------------------- #
# End-to-end (camera capture injected by the CLI)                              #
# --------------------------------------------------------------------------- #

@dataclass
class CalibrationConfig:
    board: BoardSpec = field(default_factory=BoardSpec)
    intrinsics: Optional[CameraIntrinsics] = None
    extrinsics: Optional[CameraExtrinsics] = None
    R_W_D: Optional[np.ndarray] = None   # None => identity (levelled, aligned)
    n_frames: int = 20
    max_outlier_deg: float = 3.0
    min_frames: int = 3


def run_calibration(
    capture: Callable[[], Optional[np.ndarray]],
    clock_ns: Callable[[], int],
    cfg: CalibrationConfig,
) -> CalibrationResult:
    """Collect ``n_frames`` valid observations and calibrate R_W_B.

    ``capture`` yields a BGR frame or None (no frame yet); ``clock_ns`` yields a
    monotonic timestamp. Neither is a motor/can call. This is the routine the
    webd "start installation visual calibration" command drives.
    """
    board, obj_full, detector = make_detector(cfg.board)
    poses: List[FramePose] = []
    attempts = 0
    max_attempts = max(cfg.n_frames * 20, 200)
    while len(poses) < cfg.n_frames and attempts < max_attempts:
        attempts += 1
        frame = capture()
        if frame is None:
            continue
        obj_pts, img_pts, n = detect_charuco(
            frame, board, detector, min_corners=cfg.min_frames)
        if img_pts is None:
            continue
        ts = clock_ns()
        poses.append(
            solve_board_pose(obj_pts, img_pts, cfg.intrinsics, ts))
    result = calibrate_from_poses(
        poses,
        cfg.extrinsics.R_P_C if cfg.extrinsics else np.identity(3),
        cfg.R_W_D,
        cfg.max_outlier_deg,
        cfg.min_frames,
    )
    return result
