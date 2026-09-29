"""The scorer's promises: it decides only on measured things, and it cannot be quietly renamed away."""

import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import adr0021_scorer as scorer  # noqa: E402

# The trace record's field names are emitted where the trace frame is built, not in the ring-buffer
# header: measured, `output_requested`, `rx_velocity_20` and the rest live in web_server.hpp. Pointing
# this guard at the wrong file would have failed on correct code, which is worse than no guard at all.
FIRMWARE_RECORD = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))), "control", "src", "web", "web_server.hpp")


def test_the_rules_the_scorer_must_never_break():
    assert scorer.selftest() == 0


def test_a_metric_cannot_lose_its_fields_by_renaming_them_in_the_firmware():
    """If controld renames a field, the scorer would report NOT_RUN forever and look honest doing it.

    That is the failure mode this file can catch cheaply, so it does: every name a metric needs must
    still be emitted by the record, or this test fails where someone will notice.
    """
    emitted = open(FIRMWARE_RECORD, encoding="utf-8").read()
    for name, spec in scorer.METRICS.items():
        for field in spec["fields"]:
            assert '"' + field + '"' in emitted, (name, field)


def test_the_frozen_table_hashes_to_something_the_lock_can_carry():
    frozen = scorer.metrics_sha256()
    assert len(frozen) == 64 and frozen == scorer.metrics_sha256()
    assert json.loads(json.dumps({"metrics_sha256": frozen}))["metrics_sha256"] == frozen


def test_a_window_with_nothing_measurable_is_not_run_and_says_which_field():
    verdict = scorer.score_window([{"phase": "hold"}])
    assert verdict["classification"] == "NOT_RUN"
    reasons = [row["reason"] for row in verdict["metrics"].values()]
    assert any("do not carry" in reason for reason in reasons), verdict
    assert any("raw byte" in reason for reason in reasons), verdict


def test_jitter_quiet_alone_cannot_carry_a_candidate_over_the_line():
    """D5: a perfectly still record set that never followed anything is not evidence of quality."""
    rows = [{"ref": 0.0, "rx_velocity_20": 0.0, "output_requested": 0.0, "output_reason": "idle",
             "safety": "ALLOW", "encoder_raw": 0.0, "pi_integral": 0.0, "phase": "hold",
             "enabled_state": 1, "rx_seq": index, "wall_t_ns": index * 5_000_000}
            for index in range(10)]
    verdict = scorer.score_window(rows)
    assert verdict["classification"] in ("PASS_SCOPE", "NOT_RUN"), verdict["classification"]
    assert verdict["metrics"]["thermal_electrical"]["status"] == "NOT_RUN"
