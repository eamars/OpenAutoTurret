#!/usr/bin/env python3
"""Print a P9 ChArUco calibration board (§29.1/§29.2) at exact metric scale.

Defaults come from ``vision.installation_calibration.BoardSpec``, so the sheet
matches the board the calibrator builds in code — no hand-copied numbers to drift.

    python3 -m tools.make_charuco_board --out build/charuco_board_P9.pdf
    python3 -m tools.make_charuco_board --marker-cols 4 --square-mm 40 --out big.pdf

Then: print at 100 % (never "fit to page"), verify the 50 mm line with a ruler,
and mount the sheet flat on card. The printed board is verified detectable by
vision/tests/test_charuco_board_print.py, which runs the same
``installation_calibration.detect_charuco`` that P9 uses.
"""
from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from vision.charuco_board import DEFAULT_DPI, describe, write_pdf  # noqa: E402
from vision.installation_calibration import BoardSpec  # noqa: E402


def main(argv=None) -> int:
    import argparse

    d = BoardSpec()
    # The Printable default is NOT BoardSpec's: a 3x3 board of 30 mm squares is
    # 210 mm across, which is A4's entire short side - it cannot be printed with
    # margins, and "shrink to fit" is exactly the error that ruins metric
    # calibration. 24/16 mm keeps the same 2:3 marker ratio and lands at 168 mm,
    # which prints at 100 % on A4 portrait. Whatever is printed is stated on the
    # sheet AND echoed below as the exact flags to calibrate with.
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default="build/charuco_board_P9.pdf",
                    help="output PDF path (default: %(default)s)")
    # OpenCV's CharucoBoard takes SQUARES and drops a marker on every black
    # square, so these are squares across/down (BoardSpec's field names predate
    # that reading; make_cv_board documents the trap). 6x6 squares at 24 mm =
    # a 144 mm board with 18 markers and 25 ChArUco corners: plenty to solve
    # PnP robustly at the station, and it prints on A4 at 100 %.
    ap.add_argument("--grid-cols", type=int, default=6,
                    help="board squares across (OpenCV's first CharucoBoard "
                         "argument); BoardSpec's default is "
                         f"{d.marker_cols}, which yields only 4 markers")
    ap.add_argument("--grid-rows", type=int, default=6,
                    help="board squares down")
    ap.add_argument("--square-mm", type=float, default=24.0,
                    help="chessboard square side (must exceed --marker-mm). "
                         "BoardSpec's default is %.0f mm, which does not fit A4 "
                         "portrait at 100%%; use --page a3 for that."
                         % (d.square_length_m * 1000.0))
    ap.add_argument("--marker-mm", type=float, default=16.0,
                    help="marker side; 2/3 of --square-mm, matching BoardSpec's "
                         "ratio")
    ap.add_argument("--page", default="a4", choices=["a4", "a3", "letter"],
                    help="paper to lay the sheet out on (default: %(default)s)")
    ap.add_argument("--dict", default=d.dictionary, help="OpenCV aruco dictionary")
    ap.add_argument("--dpi", type=int, default=DEFAULT_DPI)
    ap.add_argument("--no-ruler", action="store_true",
                    help="omit the 50 mm print-scale check line (not advised)")
    args = ap.parse_args(argv)

    spec = BoardSpec(
        marker_cols=args.grid_cols,
        marker_rows=args.grid_rows,
        square_length_m=args.square_mm / 1000.0,
        marker_length_m=args.marker_mm / 1000.0,
        # BoardSpec validates the name (clear ValueError listing the known
        # dictionaries); no need to second-guess it here.
        dictionary=args.dict,
    )
    pages = {"a4": (210.0, 297.0), "a3": (297.0, 420.0),
             "letter": (215.9, 279.4)}
    out = os.path.abspath(args.out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    try:
        info = write_pdf(spec, out, dpi=args.dpi, page_mm=pages[args.page])
    except Exception as e:  # noqa: BLE001 - surface the reason, not a stack
        print(f"error: {e}", file=sys.stderr)
        return 1
    print(f"wrote {out} ({info['bytes']} bytes, {info['dpi']} dpi)")
    print(f"  board  : {describe(spec)}")
    print(f"  page   : {info['page_mm'][0]:.0f} x {info['page_mm'][1]:.0f} mm")
    print("  print  : 100% scale, NO 'fit to page'; verify the 50 mm line with a "
          "ruler; mount flat on card, matte side up.")
    print(f"  check  : board should measure {info['board_mm'][0]:.1f} x "
          f"{info['board_mm'][1]:.1f} mm on paper, and the 50 mm line 50 mm")
    # The number that must NOT drift: whatever the paper says, the calibrator has
    # to be told the same thing.
    # Real commands, real flag names (this echo once told the operator to run
    # `python3 -m vision.installation_calibration`, a module with no CLI - the
    # kind of instruction that costs an afternoon at the station). The board
    # geometry must be repeated to BOTH tools, because they build the board the
    # calibrator solves against, not the sheet.
    geo = (f"--grid-cols {spec.marker_cols} --grid-rows {spec.marker_rows} "
           f"--square-mm {args.square_mm:g} --marker-mm {args.marker_mm:g} "
           f"--dict {spec.dictionary}")
    cont = " " + chr(92)          # shell line continuation, kept out of f-strings
    print("  then calibrate with the SAME geometry (orientation is this "
          "station's: rotate_180):")
    print("    python3 -m tools.calibrate_camera_intrinsics" + cont)
    print(f"        --live --frames 40 --orientation rotate_180 {geo}")
    print("    python3 -m tools.calibrate_installation_pose" + cont)
    print(f"        --live --frames 25 --orientation rotate_180 {geo}" + cont)
    print("        --commit calibration/installation_pose.yaml")
    return 0


if __name__ == "__main__":
    sys.exit(main())
