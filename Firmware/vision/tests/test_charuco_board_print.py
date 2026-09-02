"""The printed P9 calibration board must be detectable by the code that uses it.

P9 needs a physical board, and the failure that matters is not an ugly PDF: it is
the sheet and the calibrator disagreeing about geometry, discovered at the station
with the rig levelled and a piece of paper in hand. So these tests close the loop
through the real path — render the sheet, then run it past
``vision.installation_calibration.detect_charuco``, the function P9 runs on camera
frames — and assert the millimetre claim holds.

They also pin the two OpenCV traps this session measured (4.10 and 5.0 behave the
same): the constructor's first argument is SQUARES, and ``marker_id_offset`` must
not be passed positionally (it is the ids vector in the Python binding, which
either crashes or yields a board whose ``detectBoard`` silently returns None).
"""
from __future__ import annotations

import os
import tempfile
import unittest

import numpy as np

from vision.charuco_board import (DEFAULT_DPI, make_sheet, mm_to_px,
                                  render_board, sheet_metrics, write_pdf)
from vision.installation_calibration import (BoardSpec, detect_charuco,
                                             make_detector)

try:
    import cv2  # noqa: F401
    HAVE_CV2 = True
except Exception:  # noqa: BLE001
    HAVE_CV2 = False


def _printable_spec() -> BoardSpec:
    """The geometry tools/make_charuco_board.py prints by default (fits A4)."""
    return BoardSpec(marker_cols=6, marker_rows=6,
                     square_length_m=0.024, marker_length_m=0.016)


@unittest.skipUnless(HAVE_CV2, "needs OpenCV")
class CharucoBoardPrintTest(unittest.TestCase):

    def test_sheet_is_detected_by_the_p9_detector(self):
        spec = _printable_spec()
        board, _obj, detector = make_detector(spec)
        sheet = render_board(spec, dpi=DEFAULT_DPI)
        bgr = np.stack([sheet] * 3, axis=-1)
        _obj_pts, _img_pts, n = detect_charuco(bgr, board, detector)
        # 6x6 squares => 25 ChArUco corners. A print that only half-detects is
        # how a station ends up with a noisy, "converged" calibration.
        self.assertGreaterEqual(n, 25,
                                f"only {n} corners detected on our own sheet")

    def test_annotations_do_not_break_detection(self):
        """The caption and the 50 mm ruler are on the page too; prove they do
        not confuse the detector (they sit outside the board's white margin)."""
        spec = _printable_spec()
        board, _obj, detector = make_detector(spec)
        page = np.asarray(make_sheet(spec, dpi=DEFAULT_DPI))
        bgr = np.stack([page] * 3, axis=-1)
        _o, _i, n = detect_charuco(bgr, board, detector)
        self.assertGreaterEqual(n, 25)

    def test_square_size_on_paper_is_the_modelled_one(self):
        """Millimetres are the whole point: at `dpi`, a square must be
        square_length_m, and the sheet metrics must agree with the pixels."""
        spec = _printable_spec()
        img = render_board(spec, dpi=DEFAULT_DPI, margin_mm=0.0)
        px_per_square = mm_to_px(spec.square_length_m * 1000.0, DEFAULT_DPI)
        m = sheet_metrics(spec, img, dpi=DEFAULT_DPI, margin_mm=0.0)
        self.assertAlmostEqual(m.px_per_square, px_per_square, delta=0)
        # Six 24 mm squares = 144 mm (measured from the render, not assumed).
        self.assertAlmostEqual(m.board_mm[0], 144.0, delta=1.0)
        self.assertAlmostEqual(m.board_mm[1], 144.0, delta=1.0)

    def test_print_is_binary(self):
        """Soft grey edges waste ink and bias corner refinement, which is the
        exact quantity P9 measures."""
        img = render_board(_printable_spec(), dpi=DEFAULT_DPI)
        vals = np.unique(img)
        self.assertTrue(np.all((vals == 0) | (vals == 255)),
                        f"render is not binary: {vals[:6]}")

    def test_pdf_is_written(self):
        spec = _printable_spec()
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "board.pdf")
            info = write_pdf(spec, path, dpi=DEFAULT_DPI)
            with open(path, "rb") as f:
                self.assertEqual(f.read(4), b"%PDF")
            self.assertGreater(info["bytes"], 1000)
            # ~144 mm board + 16 mm margin, on an A4 page (possibly rotated).
            self.assertLess(info["board_mm"][0], info["page_mm"][0])

    def test_board_too_big_for_the_page_is_refused_not_shrunk(self):
        """Shrink-to-fit is the silent killer: the model keeps believing the
        squares are 24 mm while the paper says otherwise."""
        big = BoardSpec(marker_cols=16, marker_rows=16,
                        square_length_m=0.030, marker_length_m=0.020)
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "board.pdf")
            with self.assertRaises(ValueError):
                write_pdf(big, path, dpi=DEFAULT_DPI, page_mm=(210.0, 297.0))


@unittest.skipUnless(HAVE_CV2, "needs OpenCV")
class BoardSpecCv2ContractTest(unittest.TestCase):
    """The traps in BoardSpec -> cv2 conversion, kept honest by construction."""

    def test_board_is_constructible_and_detectable(self):
        spec = BoardSpec(marker_cols=5, marker_rows=5,
                         square_length_m=0.025, marker_length_m=0.015)
        board, obj, detector = make_detector(spec)
        raw = np.asarray(board.generateImage((1600, 1600)))
        raw = raw[..., 0] if raw.ndim == 3 else raw
        _o, _i, n = detect_charuco(np.stack([raw] * 3, axis=-1), board, detector)
        self.assertGreater(n, 0,
                           "a board we build must be detectable by the same "
                           "detector — an ids mismatch looks like a bad board")

    def test_dictionary_capacity_is_checked_before_rendering(self):
        # 12x12 squares needs 72 ids; DICT_4X4_50 has 50. OpenCV aborts deep
        # inside generateImageMarker without this guard.
        spec = BoardSpec(marker_cols=12, marker_rows=12,
                         square_length_m=0.024, marker_length_m=0.016)
        with self.assertRaises(ValueError) as cm:
            spec.make_cv_board()
        self.assertIn("marker ids", str(cm.exception))


if __name__ == "__main__":
    unittest.main()
