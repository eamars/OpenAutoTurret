"""Render a print-ready ChArUco calibration board for §29 / P9.

Why this exists: P9 needs a PHYSICAL board, and the geometry lives in code
(``vision.installation_calibration.BoardSpec``) with nothing producing the sheet.
Generating it here means the paper cannot drift from the model — and the tool
**refuses to emit a sheet it cannot verify itself**: it renders, detects the
board with ``installation_calibration.detect_charuco`` (the exact function P9
runs), measures the rendered square size in pixels, resamples so one square is
exactly ``square_length_m`` at ``dpi``, then re-detects to confirm the corner
count survived and the spacing is right.

What that verification bought (measured this session, on OpenCV 4.10 AND 5.0 —
both behave the same): ``cv2.aruco.CharucoBoard((a, b), ...)`` takes
**squaresX/squaresY**, and puts a marker on every black square. So
``BoardSpec(marker_cols=3)`` builds a 3x3-**square** board (4 markers, 4 ChArUco
corners), not a 3x3-**marker** board (which would be 7x7 squares). A sheet sized
from the "markers" reading is 2.33x too big for the board object it must match,
and `detectBoard` then returns *nothing* — silently. Sizing from the render, by
measurement, is immune to how the argument is read.

Print discipline (where metric calibration dies):

  * Print at **100 % scale** — "fit to page" shrinks every square while the model
    keeps believing the original size; PnP happily absorbs that into the pose.
  * Verify with a ruler against the 50 mm check line before use.
  * Matte paper, mounted flat on card: a bowed board is deformation PnP cannot
    model, and gloss breaks the threshold at some angles.
"""
from __future__ import annotations

import io
from dataclasses import dataclass
from typing import Tuple

import numpy as np

from .installation_calibration import BoardSpec, detect_charuco, make_detector

DEFAULT_DPI = 300
_MM_PER_INCH = 25.4
A4_MM = (210.0, 297.0)
A3_MM = (297.0, 420.0)
# Verified-sheet tolerance: 1 % of a square is 0.24 mm at 24 mm squares — far
# inside the 0.5 deg repeatability P9 aims to certify.
SPACING_TOL = 0.01


@dataclass
class SheetMetrics:
    """What a rendered sheet actually is (measured, not assumed)."""

    px: Tuple[int, int]
    board_mm: Tuple[float, float]
    px_per_square: float
    dpi: int


def mm_to_px(mm: float, dpi: int = DEFAULT_DPI) -> int:
    return int(round(mm * dpi / _MM_PER_INCH))


def _corner_spacing_px(corners) -> float:
    """Median nearest-neighbour distance between ChArUco corners.

    Adjacent ChArUco corners are exactly one square apart, so this measures the
    rendered square size without knowing how many squares OpenCV built.
    """
    pts = np.asarray(corners, dtype=np.float64).reshape(-1, 2)
    if pts.shape[0] < 2:
        raise ValueError("board render has no detectable corner pairs")
    d = np.linalg.norm(pts[:, None, :] - pts[None, :, :], axis=-1)
    np.fill_diagonal(d, np.inf)
    return float(np.median(d.min(axis=1)))


def render_board(spec: BoardSpec, dpi: int = DEFAULT_DPI,
                 margin_mm: float = 8.0) -> np.ndarray:
    """Render the board (grayscale uint8) with squares exactly at `dpi` scale.

    Raises rather than returning a sheet that cannot be detected or whose scale
    is off: at the station the failure mode of a bad sheet is a calibration that
    looks converged and is wrong.
    """
    from PIL import Image  # local import: only the printer needs Pillow

    board, _obj, detector = make_detector(spec)

    def detect(arr):
        return detect_charuco(np.stack([arr] * 3, axis=-1), board, detector)

    # 1) Render generously, detect, and MEASURE the square size we got.
    raw = np.asarray(board.generateImage((2400, 2400)))
    if raw.ndim == 3:
        raw = raw[..., 0]
    _o, _i, n = detect(raw)
    if n < 4:
        raise RuntimeError(
            "the board cv2 generated is not detectable by the P9 detector; "
            "refusing to print a sheet we cannot verify")
    corners, _ids, _mc, _mi = detector.detectBoard(raw)
    got = _corner_spacing_px(corners)
    want = float(mm_to_px(spec.square_length_m * 1000.0, dpi))

    # 2) Resample so one square is exactly `want` pixels.
    size = (max(1, int(round(raw.shape[1] * want / got))),
            max(1, int(round(raw.shape[0] * want / got))))
    out = np.asarray(Image.fromarray(raw).resize(size, Image.LANCZOS))
    # Printing is binary, and LANCZOS leaves grey edges that soften corners:
    # 128 is the natural midpoint of a rendered black/white square.
    out = np.where(out >= 128, 255, 0).astype(np.uint8)

    # 3) Verify the resample: detection survived, and the spacing is on target.
    _o2, _i2, n2 = detect(out)
    if n2 < 4:
        raise RuntimeError(
            f"detection lost the board after resampling ({n} -> {n2} corners)")
    corners2, _ids2, _mc2, _mi2 = detector.detectBoard(out)
    got2 = _corner_spacing_px(corners2)
    if abs(got2 - want) > SPACING_TOL * want:
        raise RuntimeError(
            f"printed square is {got2:.2f} px, target {want:.2f} px "
            f"(off by more than {SPACING_TOL:.0%}); refusing to emit")

    margin = mm_to_px(margin_mm, dpi)
    page = Image.new("L", (out.shape[1] + 2 * margin, out.shape[0] + 2 * margin),
                     255)
    page.paste(Image.fromarray(out), (margin, margin))
    return np.asarray(page)


def sheet_metrics(spec: BoardSpec, arr: np.ndarray, dpi: int = DEFAULT_DPI,
                  margin_mm: float = 8.0) -> SheetMetrics:
    """Metrics of a rendered sheet (the board's mm size excludes its margin)."""
    h_px, w_px = arr.shape[:2]
    margin = mm_to_px(margin_mm, dpi)
    return SheetMetrics(
        px=(w_px, h_px),
        board_mm=((w_px - 2 * margin) * _MM_PER_INCH / dpi,
                  (h_px - 2 * margin) * _MM_PER_INCH / dpi),
        px_per_square=float(mm_to_px(spec.square_length_m * 1000.0, dpi)),
        dpi=dpi)


def describe(spec: BoardSpec) -> str:
    return (f"grid argument {spec.marker_cols}x{spec.marker_rows} "
            f"(OpenCV reads these as SQUARES), square "
            f"{spec.square_length_m * 1000:.0f} mm, marker "
            f"{spec.marker_length_m * 1000:.0f} mm, {spec.dictionary}")


def make_sheet(spec: BoardSpec, dpi: int = DEFAULT_DPI,
               page_mm: Tuple[float, float] = A4_MM,
               ruler: bool = True, caption: bool = True):
    """Full page: the board, then a caption and a 50 mm ruler BELOW it.

    The text sits outside the board's white margin, and the tests detect the
    annotated page to prove the text cannot confuse the detector.
    """
    from PIL import Image, ImageDraw, ImageFont

    board_px = Image.fromarray(render_board(spec, dpi=dpi))
    bw, bh = board_px.size          # PIL size is (w, h)
    page_w_mm, page_h_mm = page_mm

    # Never shrink the board to fit — shrinking is the exact error this tool
    # exists to prevent. Try the other orientation, then A3, then refuse.
    def fits(w_mm: float, h_mm: float) -> bool:
        return (bw <= mm_to_px(w_mm, dpi)
                and bh + mm_to_px(30, dpi) <= mm_to_px(h_mm, dpi))

    if not fits(page_w_mm, page_h_mm):
        for cand in ((page_mm[1], page_mm[0]), A3_MM, (A3_MM[1], A3_MM[0])):
            if fits(*cand):
                page_w_mm, page_h_mm = cand
                break
        else:
            raise ValueError(
                f"board {bw}x{bh} px does not fit a {page_mm[0]:.0f}x"
                f"{page_mm[1]:.0f} mm page in any orientation at {dpi} dpi; "
                "reduce --square-mm or the grid")
    page = Image.new("L", (mm_to_px(page_w_mm, dpi), mm_to_px(page_h_mm, dpi)), 255)
    x = (page.width - bw) // 2
    top = mm_to_px(10, dpi)
    page.paste(board_px, (x, top))

    draw = ImageDraw.Draw(page)
    try:
        font = ImageFont.load_default()
    except Exception:  # noqa: BLE001 - font availability varies
        font = None
    y = top + bh + mm_to_px(8, dpi)
    if caption:
        draw.text((x, y), f"OpenAutoTurret P9 calibration board - {describe(spec)}",
                  fill=0, font=font)
        draw.text((x, y + mm_to_px(4, dpi)),
                  "PRINT AT 100% SCALE (no 'fit to page'). Check the 50 mm line "
                  "with a ruler. Mount flat on card.", fill=0, font=font)
        y += mm_to_px(10, dpi)
    if ruler:
        # Exactly 50 mm at the sheet's nominal scale: the operator's print-scale
        # proof. If it measures wrong on paper, do not use the board.
        x0, x1 = x, x + mm_to_px(50, dpi)
        w = max(1, mm_to_px(0.4, dpi))
        draw.line((x0, y, x1, y), fill=0, width=w)
        tick = mm_to_px(2.0, dpi)
        for xa in (x0, x1):
            draw.line((xa, y - tick, xa, y + tick), fill=0, width=w)
        draw.text((x1 + mm_to_px(2, dpi), y - mm_to_px(2, dpi)), "50 mm",
                  fill=0, font=font)
    return page


def write_pdf(spec: BoardSpec, out_path: str, dpi: int = DEFAULT_DPI,
              page_mm: Tuple[float, float] = A4_MM) -> dict:
    """Write the sheet as a PDF, returning the metrics it was built to."""
    page = make_sheet(spec, dpi=dpi, page_mm=page_mm)
    # Measured from the same render the sheet used, so the number printed on the
    # terminal is the number on the paper (not the model's intention).
    board_px = render_board(spec, dpi=dpi)
    bh_px, bw_px = board_px.shape[:2]
    board_mm = (bw_px * _MM_PER_INCH / dpi, bh_px * _MM_PER_INCH / dpi)
    buf = io.BytesIO()
    # PDF carries its own units: passing `resolution` maps our pixel grid to
    # millimetres exactly, so "print at 100 %" yields the model's square size.
    page.save(buf, "PDF", resolution=float(dpi))
    data = buf.getvalue()
    with open(out_path, "wb") as f:
        f.write(data)
    return {"path": out_path, "bytes": len(data), "dpi": dpi,
            "board_mm": board_mm,
            "px_per_square": mm_to_px(spec.square_length_m * 1000.0, dpi),
            "page_mm": (page.width * _MM_PER_INCH / dpi,
                        page.height * _MM_PER_INCH / dpi),
            "spec": describe(spec)}
