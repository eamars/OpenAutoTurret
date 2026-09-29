"""The Hailo manifest exists in two places, so the copies must be checked against each other.

F-WP5-1 measured that they had already drifted (model_id spelled two ways, and a `task` field that
was prose on one side and an enum on the other, read by nobody). Deleting the duplication outright
is not free: `tools/probe_hailo_camera.py` reads `artifact.sha256`, `artifact.architecture` and
`runtime.hailort_version` from the config-side copy. So the rule enforced here is the one that is
true today: any scalar the two files both name must hold the same value. It is a drift detector,
not a claim that duplication is good.
"""
import json
import pathlib
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[2]
CONFIG_SIDE = ROOT / "config" / "hailo_yolov8n_manifest.json"
PACKAGE_SIDE = ROOT / "perception" / "model" / "manifests" / "hailo_yolov8n_hailo8_coco.json"


def flat(node, prefix=""):
    out = {}
    if isinstance(node, dict):
        for key, value in node.items():
            path = f"{prefix}{key}"
            if isinstance(value, dict):
                out.update(flat(value, path + "."))
            elif not isinstance(value, list):
                out[path] = value
    return out


class TestManifestSingleTruth(unittest.TestCase):
    def test_shared_scalars_agree_between_the_two_copies(self):
        left = flat(json.loads(CONFIG_SIDE.read_text(encoding="utf-8")))
        right = flat(json.loads(PACKAGE_SIDE.read_text(encoding="utf-8")))
        shared = sorted(set(left) & set(right))
        self.assertTrue(shared, "the two copies share no scalar fields at all -- "
                                "the comparison would silently pass")
        disagreeing = [f"{key}: config={left[key]!r} package={right[key]!r}"
                       for key in shared if left[key] != right[key]]
        self.assertEqual(disagreeing, [],
                         "manifest copies have drifted; the package manifest is the one the "
                         "runtime reads")

    def test_the_prose_task_field_is_gone_from_the_config_copy(self):
        # `task` was read by nothing and was prose where the rest of the repo uses an enum.
        # Keeping it meant a branch on task could take the wrong path without an error.
        doc = json.loads(CONFIG_SIDE.read_text(encoding="utf-8"))
        self.assertNotIn("task", flat(doc))


if __name__ == "__main__":
    unittest.main()
