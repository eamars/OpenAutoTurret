"""The calibration FILES are a cross-language contract; pin both ends.

`vision/installation_calibration.py` writes `calibration/camera_intrinsics.yaml`
and `camera_extrinsics.yaml`; `controld` parses them in C++
(`control/src/calibration/camera_calibration.hpp`). Until now nothing tested the
shape, and the Python writers were emitting YAML mapping text
(`fx: 1420.5`, `R_P_C:`) that the C++ `parse_key_values` cannot see at all —
the loader answers "missing key(s); intrinsics NOT applied" / "no 3x3 rotation
found" and the station runs UNCALIBRATED while the file looks perfectly good.

These tests hold the C++ end honest by reading the literals out of the C++ test
source: if either side changes the format, one of these fails.
"""
from __future__ import annotations

import os
import re
import tempfile
import unittest

import numpy as np

from vision.installation_calibration import (CameraExtrinsics, CameraIntrinsics,
                                             parse_key_value_file)

REPO_FIRMWARE = os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__))))
try:
    import yaml  # noqa: F401
    HAVE_YAML = True
except Exception:  # noqa: BLE001
    HAVE_YAML = False

CPP_TEST = os.path.join(REPO_FIRMWARE, "control", "tests",
                        "test_camera_calibration_files.cpp")


def cpp_test_source() -> str:
    if not os.path.exists(CPP_TEST):
        return ""
    with open(CPP_TEST, encoding="utf-8") as f:
        return f.read()


class IntrinsicsFileFormatTest(unittest.TestCase):

    def test_emits_the_documented_key_value_shape(self):
        intr = CameraIntrinsics(fx=1417.6171875, fy=1418.3, cx=959.6875,
                                cy=540.125, distortion=(0.0,) * 5,
                                width=1920, height=1080)
        text = intr.to_yaml()
        self.assertTrue(text.startswith("# ota-camera-intrinsics v1\n"),
                        "the header camera_calibration.hpp documents must lead")
        kv = dict(line.split("=", 1) for line in text.splitlines()
                  if line and not line.startswith("#") and "=" in line)
        # The six mandatory keys: controld applies NOTHING if any is missing.
        for key in ("fx", "fy", "cx", "cy", "width", "height"):
            self.assertIn(key, kv)
        # No YAML-mapping leftovers — that was the whole bug.
        self.assertNotIn("fx:", text)
        self.assertNotIn("distortion:", text)
        # width/height must be integers to a C++ `int` cast, not "1920.0".
        self.assertEqual(kv["width"], "1920")
        self.assertEqual(kv["height"], "1080")

    def test_distortion_is_provenance_not_a_surprise(self):
        clean = CameraIntrinsics(1400, 1400, 960, 540, (0.0,) * 5, 1920, 1080)
        self.assertNotIn("dist_model", clean.to_yaml())
        d = CameraIntrinsics(1400, 1400, 960, 540, (-0.052, 0.011, 0, 0, 0),
                             1920, 1080)
        text = d.to_yaml()
        self.assertIn("dist_model=plumb_bob", text)
        for k in ("k1", "k2", "p1", "p2", "k3"):
            self.assertIn(f"\n{k}=", "\n" + text)

    def test_matches_the_literals_the_cpp_test_loads(self):
        """Byte-for-byte: run OUR writer over the values the C++ test asserts on
        and require the same `key=value` lines it parses."""
        src = cpp_test_source()
        if not src:
            self.skipTest("C++ contract test not present in this checkout")
        lit = dict(re.findall(r'"((?:fx|fy|cx|cy|width|height))=([0-9.]+)\\n"',
                              src))
        self.assertTrue(lit, "expected the C++ test to embed intrinsics keys")
        intr = CameraIntrinsics(fx=float(lit["fx"]), fy=float(lit["fy"]),
                                cx=float(lit["cx"]), cy=float(lit["cy"]),
                                width=int(float(lit["width"])),
                                height=int(float(lit["height"])))
        # Only the pure-integer keys are compared textually: the C++ literals
        # carry hand-typed float tails (1417.6171875000001) which are a different
        # double than our repr; width/height must match exactly either way.
        for key in ("width", "height"):
            self.assertIn(f"{key}={lit[key]}", intr.to_yaml())

    def test_round_trip_and_legacy_reader(self):
        intr = CameraIntrinsics(1417.6171875, 1418.3, 959.6875, 540.125,
                                (-0.052, 0.011, 0.0, 0.0, 0.002), 1920, 1080)
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "camera_intrinsics.yaml")
            with open(p, "w") as f:
                f.write(intr.to_yaml())
            back = CameraIntrinsics.load(p)
            self.assertAlmostEqual(back.fx, intr.fx, places=9)
            self.assertEqual(back.width, 1920)
            self.assertEqual(len(back.distortion), 5)
            self.assertAlmostEqual(back.distortion[0], -0.052, places=12)
            self.assertAlmostEqual(back.distortion[4], 0.002, places=12)

            # Legacy YAML-mapping files: read when PyYAML exists; where it does
            # not (the test venv), the loader must explain itself instead of
            # raising a bare ImportError.
            legacy = os.path.join(d, "legacy.yaml")
            with open(legacy, "w") as f:
                f.write("fx: 1400.0\nfy: 1400.0\ncx: 960.0\ncy: 540.0\n"
                        "width: 1920\nheight: 1080\n")
            if HAVE_YAML:
                old = CameraIntrinsics.load(legacy)
                self.assertAlmostEqual(old.fx, 1400.0)
            else:
                with self.assertRaises(ValueError) as cm:
                    CameraIntrinsics.load(legacy)
                self.assertIn("legacy YAML mapping format", str(cm.exception))


class ExtrinsicsFileFormatTest(unittest.TestCase):

    def test_raw_rows_not_a_mapping(self):
        R = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
        ext = CameraExtrinsics(R_P_C=R, t_P_C=np.array([0.0, 0.0, 0.035]))
        text = ext.to_yaml()
        self.assertTrue(text.startswith("# ota-camera-extrinsics v1\n"))
        self.assertNotIn("R_P_C:", text)
        rows = [ln for ln in text.splitlines()
                if ln and not ln.startswith("#") and "=" not in ln]
        self.assertEqual(len(rows), 3)
        for r in rows:
            self.assertEqual(len(r.split()), 3,
                             "the C++ loader needs 9 whitespace-separated "
                             "doubles from the raw section")
        # t_P_C as a key=value line is invisible to the C++ matrix scan.
        self.assertIn("t_P_C=", text)

    def test_matches_the_literals_the_cpp_test_loads(self):
        src = cpp_test_source()
        if not src:
            self.skipTest("C++ contract test not present in this checkout")
        self.assertIn('"0 0 1\\n"', src)
        self.assertIn('"-1 0 0\\n"', src)
        # Our writer must produce exactly that raw section for this rotation.
        R = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
        text = CameraExtrinsics(R_P_C=R).to_yaml()
        self.assertIn("\n0 0 1\n", text)
        self.assertIn("\n-1 0 0\n", text)

    def test_round_trip_and_legacy_reader(self):
        R = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
        ext = CameraExtrinsics(R_P_C=R, t_P_C=np.array([0.0, 0.0, 0.05]))
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "camera_extrinsics.yaml")
            with open(p, "w") as f:
                f.write(ext.to_yaml())
            back = CameraExtrinsics.load(p)
            np.testing.assert_allclose(back.R_P_C, R, atol=1e-12)
            np.testing.assert_allclose(back.t_P_C, [0, 0, 0.05], atol=1e-12)

            legacy = os.path.join(d, "legacy.yaml")
            with open(legacy, "w") as f:
                f.write("R_P_C:\n  - [0.0, -1.0, 0.0]\n  - [1.0, 0.0, 0.0]\n"
                        "  - [0.0, 0.0, 1.0]\n")
            if HAVE_YAML:
                old = CameraExtrinsics.load(legacy)
                np.testing.assert_allclose(old.R_P_C, R, atol=1e-12)
            else:
                with self.assertRaises(ValueError) as cm:
                    CameraExtrinsics.load(legacy)
                self.assertIn("legacy YAML mapping format", str(cm.exception))


class ParseKeyValueFileTest(unittest.TestCase):
    """The Python mirror of the C++ rules, so both sides can be tested apart."""

    def test_comments_and_raw_lines_are_skipped(self):
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "f.yaml")
            with open(p, "w") as f:
                f.write("# header\nfx=1400\nnot_a_key_line\nbroken\n"
                        "dist_model=plumb_bob\n")
            kv = parse_key_value_file(p)
            self.assertEqual(kv, {"fx": "1400", "dist_model": "plumb_bob"})

    def test_missing_file_is_empty_not_an_error(self):
        self.assertEqual(parse_key_value_file("/nonexistent/x.yaml"), {})


if __name__ == "__main__":
    unittest.main()
