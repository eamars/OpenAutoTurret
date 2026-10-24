"""calibration_records.py: seeding #1 from this checkout's own configuration, importing, listing."""
import json
import tempfile
import unittest
from pathlib import Path

import yaml

from tools import calibration_records as cr

FIRMWARE = Path(__file__).resolve().parents[2]
CONFIG = FIRMWARE / "config" / "turret_mixed.yaml"


class Seed(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.state = Path(self.tmp.name) / "state"     # absent, as on a first boot

    def tearDown(self):
        self.tmp.cleanup()

    def seed(self):
        return cr.seed(self.state, CONFIG, FIRMWARE, "abc1234")

    def test_an_empty_store_gets_record_1_from_the_committed_files(self):
        self.assertIn("seeded #1 AS SHIPPED", self.seed())
        rec_dir = self.state / "calibration" / "records" / "0001"
        self.assertEqual(sorted(p.name for p in rec_dir.iterdir()),
                         sorted(list(cr.PARTS.values()) + ["record.json"]))
        rec = json.loads((rec_dir / "record.json").read_text())
        self.assertEqual(rec["name"], "AS SHIPPED")
        self.assertEqual({p["origin"] for p in rec["parts"].values()}, {"shipped"})
        self.assertEqual(rec["parts"]["yaw"]["rev"], "abc1234")
        # The assets say when they were commissioned; the page shows it.
        self.assertRegex(rec["parts"]["yaw"]["date"], r"^\d{4}-\d{2}-\d{2}$")
        self.assertTrue(rec["parts"]["yaw"]["run"].startswith("yaw-"))
        self.assertEqual((rec_dir / "yaw_servo.json").read_bytes(),
                         (FIRMWARE / "config" / "servo" / "yaw_servo.json").read_bytes())
        active = json.loads((self.state / "calibration" / "active.json").read_text())
        self.assertEqual(active, {"schema": 1, "active": 1})

    def test_the_bore_carries_the_configured_alignment_and_no_trims(self):
        self.seed()
        bore = yaml.safe_load((self.state / "calibration" / "records" / "0001" / "bore.yaml").read_text())
        configured = yaml.safe_load(CONFIG.read_text())["alignment"]
        # The word, not YAML 1.1's False for a bare `off`: controld refuses any other value.
        self.assertEqual(bore["mode"], "off" if configured["mode"] in (False, "off") else configured["mode"])
        self.assertIsInstance(bore["mode"], str)
        self.assertEqual(bore["zero_m"], configured["assumed_depth_m"])
        for cam in ("wide", "detail"):
            for key in cr.BORE_KEYS:
                self.assertEqual(bore[cam][key], configured["camera_offset"][cam][key], (cam, key))
            self.assertEqual((bore[cam]["trim_right_mrad"], bore[cam]["trim_up_mrad"]), (0, 0))

    def test_a_configuration_without_servos_seeds_nothing(self):
        # The simulator's config: no hardware profile, so no servo assets to make a record from.
        out = cr.seed(self.state, FIRMWARE / "config" / "turret_sim.yaml", FIRMWARE, "abc1234")
        self.assertIn("nothing seeded", out)
        self.assertEqual(cr.record_ids(self.state / "calibration" / "records"), [])

    def test_a_store_with_records_is_left_alone(self):
        # No comparison with the source (owner, 2026-10-24): a changed file in git creates nothing.
        self.seed()
        record = self.state / "calibration" / "records" / "0001" / "record.json"
        before = record.read_text()
        self.assertIn("nothing seeded", self.seed())
        self.assertEqual(record.read_text(), before)
        self.assertEqual(cr.record_ids(self.state / "calibration" / "records"), [1])


class Pull(unittest.TestCase):
    """What `pull` does with what it fetched: the repository copy, without journals (ADR-005 D3)."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.fetched = Path(self.tmp.name) / "fetched"
        self.dest = Path(self.tmp.name) / "repo"

    def tearDown(self):
        self.tmp.cleanup()

    def record(self, rid, name):
        d = self.fetched / f"{rid:04d}"
        (d / "journals" / "yaw-20261024").mkdir(parents=True)
        (d / "journals" / "yaw-20261024" / "journal.jsonl").write_text("{}\n")
        (d / "record.json").write_text(json.dumps({"name": name}))
        (d / "bore.yaml").write_text("mode: \"off\"\n")
        return d

    def test_each_record_lands_under_its_id_and_name_without_journals(self):
        self.record(1, "AS SHIPPED")
        self.record(2, "FRICTION 10-20")
        out = cr.place(self.fetched, self.dest)
        self.assertIn("#2 FRICTION 10-20", out)
        self.assertEqual(sorted(p.name for p in self.dest.iterdir()), ["0001-AS-SHIPPED", "0002-FRICTION-10-20"])
        self.assertEqual(sorted(p.name for p in (self.dest / "0002-FRICTION-10-20").iterdir()),
                         ["bore.yaml", "record.json"])

    def test_a_record_pulled_again_replaces_its_earlier_pull_and_others_are_kept(self):
        self.record(2, "FRICTION 10-20")
        self.record(3, "RANGE")
        cr.place(self.fetched, self.dest)
        (self.fetched / "0002" / "record.json").write_text(json.dumps({"name": "FRICTION B"}))
        cr.place(self.fetched, self.dest, ids=[2])
        self.assertEqual(sorted(p.name for p in self.dest.iterdir()), ["0002-FRICTION-B", "0003-RANGE"])


class Import(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.state = Path(self.tmp.name)
        cr.seed(self.state, CONFIG, FIRMWARE, "abc1234")

    def tearDown(self):
        self.tmp.cleanup()

    def staged(self):
        incoming = self.state / "calibration" / "incoming"
        return [p for p in incoming.iterdir() if not p.name.startswith(".")]

    def test_a_measured_asset_with_the_rest_inherited_is_staged_for_controld(self):
        yaw = FIRMWARE / "config" / "servo" / "yaw_servo.json"
        out = cr.import_record(self.state, "friction 10-20", 1, {"yaw": yaw})
        self.assertIn("adopts it at its next start", out)
        (staged,) = self.staged()
        rec = json.loads((staged / "record.json").read_text())
        self.assertEqual(rec["name"], "FRICTION 10-20")
        self.assertEqual(rec["parts"]["yaw"]["origin"], "measured")
        for part in ("pitch", "accuracy", "extrinsic", "bore"):
            self.assertEqual(rec["parts"][part], {"file": cr.PARTS[part], "origin": "inherited", "from": 1})
            self.assertTrue((staged / cr.PARTS[part]).is_file())
        # Nothing in records/ changes: controld is its only writer.
        self.assertEqual(cr.record_ids(self.state / "calibration" / "records"), [1])

    def test_missing_parts_without_a_source_are_refused(self):
        with self.assertRaises(SystemExit):
            cr.import_record(self.state, "PARTIAL", None, {"yaw": FIRMWARE / "config" / "servo" / "yaw_servo.json"})
        self.assertFalse((self.state / "calibration" / "incoming").exists() and self.staged())

    def test_a_name_outside_the_rule_is_refused(self):
        with self.assertRaises(SystemExit):
            cr.import_record(self.state, "a name that is far too long for a chip", 1, {})

    def test_the_listing_marks_the_record_in_force(self):
        cr.import_record(self.state, "COPY", 1, {})
        text = cr.listing(self.state)
        self.assertIn("* #1 AS SHIPPED", text)
        self.assertIn("incoming", text)


if __name__ == "__main__":
    unittest.main()
