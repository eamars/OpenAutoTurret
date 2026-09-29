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


def test_the_frame_the_scorer_reads_still_carries_the_keys_it_is_told_to_read():
    """What is proven about the record, and only that, until a captured record is filed beside the module.

    Measured on the station: the trace frame emits `rows`, names the axes, and carries `safety` at frame
    level. The per-field names *inside* a row are not yet verified against a capture, so the metrics
    declare no fields and say why — the alternative is a scorer that quietly reads a field that does not
    exist and reports NOT_RUN forever while looking honest.
    """
    emitted = open(FIRMWARE_RECORD, encoding="utf-8").read()
    # The frame emits these keys as escaped quotes inside a C++ string literal, so the escaped form is
    # what the source contains — asserting the plain form would have failed on correct code.
    for key in ("rows", "axes", "safety", "control_trace"):
        assert '\\"' + key + '\\"' in emitted, key


def test_the_frozen_table_hashes_to_something_the_lock_can_carry():
    frozen = scorer.metrics_sha256()
    assert len(frozen) == 64 and frozen == scorer.metrics_sha256()
    assert json.loads(json.dumps({"metrics_sha256": frozen}))["metrics_sha256"] == frozen


def test_a_window_with_nothing_measurable_is_not_run_and_says_which_field():
    verdict = scorer.score_window([{"phase": "hold"}])
    assert verdict["classification"] == "NOT_RUN"
    reasons = [row["reason"] for row in verdict["metrics"].values()]
    assert any("do not carry" in reason or "marked RUN window" in reason for reason in reasons), verdict


def test_jitter_quiet_alone_cannot_carry_a_candidate_over_the_line():
    """D5: a perfectly still record set that never followed anything is not evidence of quality."""
    rows = [{"ref": 0.0, "rx_velocity_20": 0.0, "output_requested": 0.0, "output_reason": "idle",
             "safety": "ALLOW", "encoder_raw": 0.0, "pi_integral": 0.0, "phase": "hold",
             "enabled_state": 1, "rx_seq": index, "wall_t_ns": index * 5_000_000}
            for index in range(10)]
    verdict = scorer.score_window(rows)
    assert verdict["classification"] in ("PASS_SCOPE", "NOT_RUN"), verdict["classification"]
    assert verdict["metrics"]["thermal_electrical"]["status"] == "NOT_RUN"
