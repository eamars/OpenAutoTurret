"""The zoom table refuses instead of inventing, and derives only from a measured base."""
from __future__ import annotations

import unittest

from perception.camera_zoom import (DEFAULT_TABLE, ZoomFovUnknown, derive_fov_h, fov_at,
                                    load_table)


class ZoomFovTable(unittest.TestCase):
    def test_derivation_is_the_crop_law_not_a_guess(self):
        self.assertAlmostEqual(derive_fov_h(80.0, 1.0), 80.0, places=9)
        # 裁剪掉一半宽度，角度掉的比一半少（正切不是线性的）：80° 在 2 倍下是 45.5°，不是 40°。
        # 我把这条写错过一次——凭感觉写了"约等于一半"，算出来才发现差 5.5°（而且我手算的 45.51 也错了两位，真值 45.5209），
        # 而那 5.5° 正是"目标还在不在窄视场里"的量级，所以这条要钉死在闭式解上。
        self.assertAlmostEqual(derive_fov_h(80.0, 2.0), 45.5210, delta=0.001)
        self.assertGreater(derive_fov_h(80.0, 2.0), 40.0)
        self.assertLess(derive_fov_h(80.0, 4.0), derive_fov_h(80.0, 2.0))

    def test_shipped_table_refuses_while_the_base_is_unmeasured(self):
        table = load_table(DEFAULT_TABLE)
        self.assertIsNone(table["base"]["fov_h_deg"],
                          "the base FOV is a measurement, and nobody has taken it yet")
        for level in table["levels"]:
            with self.assertRaises(ZoomFovUnknown) as ctx:
                fov_at(level["digital_zoom"], table=table)
            self.assertIn(str(level["digital_zoom"]), str(ctx.exception))

    def test_a_measured_base_is_inherited_and_says_so(self):
        table = {"sensor": "imx477",
                 "base": {"digital_zoom": 1.0, "fov_h_deg": 79.27, "status": "measured"},
                 "levels": [{"digital_zoom": 1.0, "fov_h_deg": None, "status": "measured"}]}
        got = fov_at(2.0, table=table)
        self.assertEqual(got.status, "derived_from_measured_base")
        self.assertAlmostEqual(got.fov_h_deg, derive_fov_h(79.27, 2.0), places=9)


if __name__ == "__main__":
    unittest.main()
