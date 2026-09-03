"""The camera's installed orientation is a station fact, read by whoever needs it.

Why this file exists: the IMX500 on this station is bolted on upside-down, and for as long as
the correction was supplied at launch — `OTA_VIDEO_ORIENTATION` for the web daemon,
`--orientation` for the detector — it was re-forgotten often enough that an operator counted
three occurrences and asked for it to be done properly. The failure is nasty in proportion to
how quiet it is: with the correction missing, the preview looks entirely ordinary and the
detector's boxes stay plausible-looking, while the geometry the controller reasons about is
turned half a revolution from the picture. Nothing crashes. The turret simply aims away from
the target, and the log says nothing because nothing went wrong in the code.

So the mount is described once in `config/camera_install.yaml`, and the tests below are mostly
about the ways that arrangement could quietly stop being true.
"""
from __future__ import annotations

import pathlib

import pytest

from common import image_corrections as ic

REPO_CONFIG = pathlib.Path(ic.INSTALL_ORIENTATION_FILE)


def _write(tmp_path: pathlib.Path, text: str) -> pathlib.Path:
    p = tmp_path / "camera_install.yaml"
    p.write_text(text, encoding="utf-8")
    return p


def test_the_station_describes_its_own_mount(tmp_path):
    p = _write(tmp_path, "# a comment\norientation: rotate_180\n")
    orientation, where = ic.read_install_orientation(path=p)
    assert orientation == "rotate_180"
    assert "camera_install.yaml" in where, "the answer has to say where it came from"


def test_an_explicit_option_still_wins_and_says_which_one_it_was(tmp_path):
    """A flag beats the file — overriding what the station says about its own hardware is
    allowed, but it must not be invisible."""
    p = _write(tmp_path, "orientation: rotate_180\n")
    orientation, where = ic.read_install_orientation(
        path=p, explicit="none", explicit_source="OTA_VIDEO_ORIENTATION")
    assert orientation == "none"
    assert where == "OTA_VIDEO_ORIENTATION"


def test_no_description_says_so_rather_than_quietly_assuming_one(tmp_path):
    """"none" is correct for a level camera and is also exactly what a forgotten configuration
    looks like, so the caller has to be able to tell the two apart and warn about the second."""
    orientation, where = ic.read_install_orientation(path=tmp_path / "absent.yaml")
    assert orientation == ic.UNCONFIGURED_ORIENTATION
    assert "not configured" in where


def test_a_file_that_describes_nothing_says_which_file_it_read(tmp_path):
    p = _write(tmp_path, "# the mount used to be described here\n")
    orientation, where = ic.read_install_orientation(path=p)
    assert orientation == ic.UNCONFIGURED_ORIENTATION
    assert "no `orientation` key" in where


def test_a_typo_stops_the_process_that_is_about_to_reason_wrongly(tmp_path):
    """An unusable value must not fall back to "none" and keep running. A detector streaming
    with the wrong correction produces healthy frames and a turret pointing the wrong way."""
    p = _write(tmp_path, "orientation: rotate_18O\n")  # letter O, not zero
    with pytest.raises(ValueError) as excinfo:
        ic.read_install_orientation(path=p)
    message = str(excinfo.value)
    assert "rotate_18O" in message, "the bad value has to be quoted back"
    assert "camera_install.yaml" in message, "and the file it came from named"
    assert "rotate_180" in message, "and the choices it could have been"


def test_quotes_and_comments_are_tolerated(tmp_path):
    p = _write(tmp_path, "orientation: 'flip_vertical'   # swapped bracket, 2026-09-02\n")
    assert ic.read_install_orientation(path=p)[0] == "flip_vertical"


def test_the_tracked_station_file_is_valid():
    """The repo's own copy must be readable and name a real choice. A checkout that resolves to
    'not configured' would bring up an uncorrected camera on this hardware."""
    assert REPO_CONFIG.exists(), f"{REPO_CONFIG} is missing from the checkout"
    orientation, where = ic.read_install_orientation()
    assert orientation in ic.ORIENTATIONS
    assert "not configured" not in where


def test_a_daemon_started_from_the_wrong_directory_still_finds_it(monkeypatch, tmp_path):
    """The default path is anchored to the checkout. Resolving it against the working directory
    would mean a systemd unit with an odd WorkingDirectory silently runs uncorrected — the same
    silent 180-degree error again, wearing the costume of a path."""
    monkeypatch.chdir(tmp_path)
    orientation, where = ic.read_install_orientation()
    assert orientation in ic.ORIENTATIONS
    assert str(REPO_CONFIG) in where, f"resolved against the cwd instead of the checkout: {where}"


def test_the_web_daemon_asks_the_station_and_not_its_environment(monkeypatch):
    """webd used to default to "none" unless a variable happened to be set in whoever's shell
    started it. The file answers now; the variable is only an override."""
    from web.webd import config as webd_config

    monkeypatch.delenv("OTA_VIDEO_ORIENTATION", raising=False)
    assert webd_config._resolve_orientation() == ic.read_install_orientation()[0]

    monkeypatch.setenv("OTA_VIDEO_ORIENTATION", "none")
    assert webd_config._resolve_orientation() == "none", "the override must still work"


def test_the_detector_no_longer_defaults_to_none():
    """`--orientation` defaulting to "none" was the bug: leaving the flag out was a legal,
    silent, wrong answer. argparse must hand the resolver an unset value instead."""
    from vision import visiond

    parser = visiond._make_parser()
    assert parser.get_default("orientation") is None
    assert parser.parse_args([]).orientation is None
    # And a typo is still refused by argparse, with choices listed, as it always was.
    with pytest.raises(SystemExit):
        parser.parse_args(["--orientation", "sideways"])
