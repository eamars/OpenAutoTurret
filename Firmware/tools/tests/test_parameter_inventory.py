"""The parameter inventory must stay true, and the checker that keeps it true must be provably armed.

Three claims, each with its own assertion:

1. The checked-in inventory is what the current binary says (regenerate-and-compare). A stale
   document is worse than none, because it reads like evidence.
2. Every entry carries the fields ADR-002.1 docs/02 requires, and the two claims that lie most
   easily — `mutability` and `readback_source` — are checked against the four/two names, not
   against each other's prose.
3. Coverage is real: every field a config or profile file can carry is either bound to an entry or
   excluded by a stated reason. And the checker is tested with a hole cut in it, because a coverage
   test that cannot fail is how a firmware field ends up unmentioned.
"""

from __future__ import annotations

import copy
import json
import os
import subprocess
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
FIRMWARE = os.path.dirname(TOOLS)
sys.path.insert(0, TOOLS)

import dump_parameter_inventory as tool  # noqa: E402

INVENTORY = tool.INVENTORY


def load() -> dict:
    with open(INVENTORY, encoding="utf-8") as handle:
        return json.load(handle)


class InventoryIsPresent(unittest.TestCase):
    def test_the_document_exists_and_records_where_it_came_from(self):
        document = load()
        self.assertEqual(document["schema"], 1)
        for key in ("config", "hardware_profile", "binary", "binary_sha256", "source_rev",
                    "generated_at_utc"):
            self.assertIn(key, document["generated_from"],
                          f"an inventory without {key} cannot be tied to the build that produced it")
        self.assertEqual(len(document["generated_from"]["binary_sha256"]), 64)


class EntriesAreComplete(unittest.TestCase):
    MUTABILITIES = {"experiment_writable", "fixed_in_campaign", "protected_read_only", "unsupported"}
    READBACKS = {"drive_register", "host_echo", "config_file", "none"}

    def setUp(self):
        self.entries = load()["entries"]
        self.assertTrue(self.entries, "an empty inventory proves nothing")

    def test_every_entry_states_the_contract_fields(self):
        required = ("name", "group", "type", "unit", "supported_range", "mode", "mutability",
                    "apply_condition", "source_binding", "readback_source", "encoding")
        for entry in self.entries:
            missing = [key for key in required if not entry.get(key)]
            self.assertEqual([], missing, f"{entry.get('name')} does not state {missing}")

    def test_mutability_and_readback_use_only_the_declared_vocabulary(self):
        for entry in self.entries:
            self.assertIn(entry["mutability"], self.MUTABILITIES, entry["name"])
            self.assertIn(entry["readback_source"], self.READBACKS, entry["name"])

    def test_a_parameter_that_cannot_be_searched_says_why(self):
        for entry in self.entries:
            if entry["mutability"] == "experiment_writable":
                continue
            self.assertTrue(entry.get("reason"),
                            f"{entry['name']} is {entry['mutability']} with no reason: an omission "
                            "has to be a decision, not an absence")

    def test_an_unsupported_field_claims_no_readback(self):
        for entry in self.entries:
            if entry["mutability"] == "unsupported":
                self.assertEqual("none", entry["readback_source"],
                                 f"{entry['name']} is unsupported yet names a readback source; that "
                                 "is how a write that goes nowhere gets reported as applied")

    def test_host_only_values_do_not_claim_a_register_readback(self):
        """The yaw current loop has no gain register; claiming one would repeat the kp2 mistake."""
        for entry in self.entries:
            if "host variable" in entry["encoding"]:
                self.assertEqual("host_echo", entry["readback_source"], entry["name"])

    def test_the_yaw_gains_are_bound_to_the_profile_the_trial_writes(self):
        by_name = {entry["name"]: entry for entry in self.entries}
        for name in ("yaw.current_kp_a_per_rad_s", "yaw.current_ki_a_per_rad_s"):
            self.assertIn(name, by_name, "the trial command tunes these; an inventory without them "
                                         "is not an inventory of what can be tuned")
            self.assertIn("Profile::Yaw", by_name[name]["source_binding"])
        cap = by_name["yaw.host_current_limit_a"]
        self.assertEqual("protected_read_only", cap["mutability"],
                         "the approved current envelope may not be widened by the optimizer")


class CheckerIsArmed(unittest.TestCase):
    """A coverage test that cannot fail is not a test."""

    def test_a_field_with_no_entry_and_no_rule_is_reported(self):
        document = load()
        hole = copy.deepcopy(document)
        hole["entries"] = [entry for entry in hole["entries"]
                           if not entry["name"].startswith("yaw.friction.positive")]
        problems = [line for line in tool.check(hole) if "positive_breakaway_a" in line]
        self.assertTrue(problems,
                        "removing every entry that binds an amplitude must leave that amplitude "
                        "unaccounted for; if it does not, coverage is not being measured")

    def test_a_rule_naming_nothing_is_reported(self):
        document = load()
        broken = copy.deepcopy(document)
        broken["exclusion_rules"].append({"config_struct": "NoSuchConfig", "reason": "x"})
        self.assertTrue(any("NoSuchConfig" in line for line in tool.check(broken)),
                        "a rule that matches nothing blocks nothing once the code moves on")

    def test_the_shipped_document_passes_the_same_check(self):
        self.assertEqual([], tool.check(load()))


class DocumentIsCurrent(unittest.TestCase):
    def test_regenerating_from_the_built_binary_changes_nothing(self):
        binary = tool.find_binary()
        import tempfile
        with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as handle:
            path = handle.name
        try:
            result = subprocess.run(
                [sys.executable, os.path.join(TOOLS, "dump_parameter_inventory.py"),
                 "--binary", binary, "--check"],
                capture_output=True, text=True, cwd=FIRMWARE)
            self.assertEqual(0, result.returncode,
                             "parameter_inventory.json is stale: " + result.stdout + result.stderr)
        finally:
            os.unlink(path)


if __name__ == "__main__":
    unittest.main(verbosity=2)
