"""The camera zero's measurement (tools/camera_zero.py) on synthetic targets: a round part's centre to a
small fraction of a pixel, an occluded lower edge, a background it must not take, and the station's
refusals said before a command is sent. No station and no video."""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import camera_zero as cz  # noqa: E402

DARK_YELLOW = (53, 45, 3)          # the 2026-10-25 part in the detail picture
TAN = (150, 135, 110)               # a chair's backlit seat: saturation 0.27


def picture(centre, radius, colour=DARK_YELLOW, background=TAN, size=(1280, 720), cut=None, ss=4):
    """A disc, anti-aliased by supersampling; `cut`: rows below this y are background (a seat in front)."""
    w, h = size
    img = np.empty((h, w, 3), float)
    img[:] = background
    cx, cy = centre
    x0, x1, y0, y1 = int(cx - radius - 2), int(cx + radius + 3), int(cy - radius - 2), int(cy + radius + 3)
    ys, xs = np.mgrid[y0:y1, x0:x1]
    sub = (np.arange(ss) + 0.5) / ss
    cover = np.zeros(xs.shape)
    for dy in sub:
        for dx in sub:
            inside = (xs + dx - cx) ** 2 + (ys + dy - cy) ** 2 <= radius ** 2
            if cut is not None:
                inside &= (ys + dy) < cut
            cover += inside
    cover /= ss * ss
    img[y0:y1, x0:x1] = cover[..., None] * np.array(colour) + (1 - cover[..., None]) * np.array(background)
    return img.round().astype(np.uint8)


def test_the_centre_to_a_twentieth_of_a_pixel():
    centre = (652.37, 351.81)
    t = cz.find_target(picture(centre, 35.0), window=150, **cz.COLOUR["detail"])
    assert abs(t["x"] - centre[0]) < 0.05 and abs(t["y"] - centre[1]) < 0.05, t
    assert abs(t["radius_px"] - 35.0) < 1.0 and t["fit_rms_px"] < 0.5


def test_an_occluded_lower_edge_is_left_out_of_the_fit():
    # 2026-10-25: the chair's seat cut the part's bottom off flat; the full outline then sits high.
    centre = (640.4, 360.6)
    img = picture(centre, 35.0, cut=centre[1] + 25.0)
    whole = cz.find_target(img, window=150, **cz.COLOUR["detail"])
    upper = cz.find_target(img, window=150, exclude_bottom=0.2, **cz.COLOUR["detail"])
    assert abs(whole["y"] - centre[1]) > 1.0 and whole["fit_rms_px"] > 1.0, "the cut shows in the fit's rms"
    assert abs(upper["x"] - centre[0]) < 0.2 and abs(upper["y"] - centre[1]) < 0.2, upper


def test_a_tan_background_and_a_yellow_thing_outside_the_window_are_not_the_target():
    img = picture((640.0, 360.0), 30.0)
    img[100:180, 100:180] = DARK_YELLOW            # e.g. a jersey elsewhere in the room
    t = cz.find_target(img, window=150, **cz.COLOUR["detail"])
    assert abs(t["x"] - 640.0) < 0.1 and abs(t["y"] - 360.0) < 0.1
    assert not cz.hsv_mask(np.full((4, 4, 3), TAN, np.uint8), **cz.COLOUR["detail"]).any()


def test_the_bright_wide_target_is_found_with_the_wide_colours():
    bright = (225, 205, 70)                        # the wide picture's part, backlit
    centre = (961.3, 541.7)
    img = picture(centre, 6.5, colour=bright, background=(205, 190, 160), size=(1920, 1080))
    t = cz.find_target(img, window=60, **cz.COLOUR["wide"])
    assert abs(t["x"] - centre[0]) < 0.3 and abs(t["y"] - centre[1]) < 0.3, t


def test_what_it_cannot_measure_it_says():
    with pytest.raises(ValueError, match="no target-coloured pixels"):
        cz.find_target(np.full((720, 1280, 3), TAN, np.uint8), window=150, **cz.COLOUR["detail"])
    with pytest.raises(ValueError, match="touches the search window"):
        cz.find_target(picture((640.0, 360.0), 60.0), window=50, **cz.COLOUR["detail"])
    with pytest.raises(ValueError, match="too small"):
        cz.find_target(picture((640.0, 360.0), 1.2), window=150, **cz.COLOUR["detail"])


def test_the_detail_picture_lands_in_the_wide_frame_and_differences_read_in_mrad():
    assert cz.wide_frame("wide", 960.0, 540.0, 1920, 1080, 5.9) == (0.5, 0.5)
    u, v = cz.wide_frame("detail", 1280.0, 0.0, 1280, 720, 5.9)
    assert abs(u - (0.5 + 0.5 / 5.9)) < 1e-12 and abs(v - (0.5 - 0.5 / 5.9)) < 1e-12
    # 1 mrad is fx / 1000 pixels: 1.389 of the 1920.
    right, down = cz.mrad(1.389 / 1920, 0.0, {"fx": 1389, "fy": 1467, "width": 1920, "height": 1080})
    assert abs(right - 1.0) < 1e-12 and down == 0.0


def test_the_stations_refusals_are_said_before_anything_is_sent():
    ok = {"calibration_record": {"id": 6}, "alignment": {"mode": "manual_depth"}, "phase": "hold",
          "operating_mode": "MANUAL"}
    assert cz.apply_refusal(ok) == ""
    for change, words in (({"calibration_record": {"id": 1}}, "#1 is the shipped backup"),
                          ({"alignment": {"mode": "off"}}, "turn the bore on"),
                          ({"operating_mode": "AUTO_ROAM"}, "MANUAL hold"),
                          ({"calibration_switching": True}, "switch"),
                          ({"rest_park": "parked"}, "MANUAL hold"),
                          ({"calibration_record": {}}, "no calibration record")):
        assert words in cz.apply_refusal(dict(ok, **change)), change


def test_the_saved_crop_shows_what_was_fitted(tmp_path):
    img = picture((640.0, 360.0), 30.0)
    t = cz.find_target(img, window=150, **cz.COLOUR["detail"])
    cz.save_crop(tmp_path / "target_detail.png", img, t)
    from PIL import Image
    assert Image.open(tmp_path / "target_detail.png").size == (300 * 4, 300 * 4)


def measured(spread=(0.02, 0.04), rms=0.4, motion=0.05, detail=(0.5, 0.5), wide=(0.5, 0.5)):
    first = {"size_px": (70, 64), "picture": (1280, 720), "fit_rms_px": rms}
    return {"intrinsics": {"fx": 1389, "fy": 1467, "width": 1920, "height": 1080}, "motion_mrad": motion,
            "first": {"detail": first, "wide": dict(first, picture=(1920, 1080))},
            "detail": {"u": detail[0], "v": detail[1], "spread_mrad": spread},
            "wide": {"u": wide[0], "v": wide[1], "spread_mrad": spread}}


def test_a_noisy_or_moving_measurement_is_not_applied():
    # Review 2026-10-25 (a cold read of the card): --apply measured and sent with no look at quality.
    assert cz.measurement_refusal(measured()) == ""
    assert "spread" in cz.measurement_refusal(measured(spread=(0.05, 0.42)))
    assert "circle fit" in cz.measurement_refusal(measured(rms=2.6))
    assert "moved" in cz.measurement_refusal(measured(motion=0.8))


def test_the_check_says_whether_both_crosshairs_are_on_the_target(capsys):
    state = {"alignment": {"wide": {"valid": True, "x_norm": 0.5, "y_norm": 0.5},
                           "detail": {"valid": True, "x_norm": 0.5, "y_norm": 0.5}}}
    assert cz.report(measured(), state) is True
    off = 0.15 * 1.389 / 1920                     # 0.15 mrad: on for the wide camera, off for the detail
    assert cz.report(measured(detail=(0.5 + off, 0.5), wide=(0.5 + off, 0.5)), state) is False
    assert "NOT on it" in capsys.readouterr().out
    assert cz.report(measured(), {"alignment": {"mode": "off"}}) is False
    assert "the bore is off" in capsys.readouterr().out


def test_what_apply_replaces_is_printed_first():
    state = {"calibration_record": {"id": 6, "name": "NO_LOAD"}, "calibration_records": [
        {"id": 6, "bore": {"range_m": 11, "wide": {"trim_right_mrad": -1.55, "trim_up_mrad": 0.82, "right_mm": 0, "up_mm": 0,
                                                   "forward_mm": 0, "zeroed_m": 11},
                           "detail": {"trim_right_mrad": -0.89, "trim_up_mrad": 5.25, "right_mm": -10, "up_mm": 60,
                                      "forward_mm": 0}}}]}
    text = cz.bore_now(state)
    assert text.startswith("record #6 NO_LOAD, range 11 m: wide trims R -1.55 U 0.82 mrad")
    assert "detail trims R -0.89 U 5.25 mrad, offsets -10/60/0 mm, zeroed no" in text
