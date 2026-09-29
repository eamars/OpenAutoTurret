"""Two sensors on two ports must not end up with one identity.

Measured on the station today: deriving the secondary sensor's identity from libcamera's camera
index (`Num 1` -> `/dev/video1`) handed back the *wide* camera's id, because this board exposes a
whole family of nodes per PiSP host. The manifest then had two streams claiming one camera, which
is exactly the class of lie the by-path rule exists to prevent.
"""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from perception.camera_id import derive_camera_id  # noqa: E402

WIDE_FWNODE = "/base/axi/pcie@1000120000/rp1/i2c@88000/imx500@1a"
DETAIL_FWNODE = "/base/axi/pcie@1000120000/rp1/i2c@80000/imx477@1a"


def test_two_ports_two_ids_and_both_are_durable():
    wide, detail = derive_camera_id(WIDE_FWNODE), derive_camera_id(DETAIL_FWNODE)
    assert wide.id != detail.id, "两颗相机共用一个身份，归属就全错了"
    assert wide.source == detail.source == "fwnode"
    assert wide.durable and detail.durable


def test_the_same_port_keeps_the_same_id_across_boots():
    assert derive_camera_id(WIDE_FWNODE) == derive_camera_id(WIDE_FWNODE)


def test_a_different_address_on_the_same_bus_is_a_different_camera():
    other = "/base/axi/pcie@1000120000/rp1/i2c@88000/imx477@1a"
    assert derive_camera_id(other).id != derive_camera_id(WIDE_FWNODE).id


def test_an_unrelated_path_stays_a_label_rather_than_claiming_durability():
    ident = derive_camera_id("mock://synthetic")
    assert ident.source == "label" and ident.durable is False
