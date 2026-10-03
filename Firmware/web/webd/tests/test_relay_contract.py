"""Relay contracts that outlived the /dashboard page (removed 2026-10-03).

Both guard the wire rather than a page: webd's dataclass must declare every field a page reads, and
every §79 event must have a name. They lived in the dashboard's test file and moved here intact.
"""
import re


def test_webd_declares_the_fields_the_page_reads():
    """The daemon re-serialises telemetry through its own dataclass, so any field the
    page reads that the dataclass does not declare simply does not arrive — with no error
    anywhere. `selected_track_id` once looked like a vision bug for exactly this reason.
    """
    from pathlib import Path

    proto = (Path(__file__).resolve().parents[1] / "protocol.py").read_text(
        encoding="utf-8"
    )
    for field_name in ("tracks", "track_count", "track_list_age_ms", "selected_display_index",
                       "selection_visibility", "manual_lease_active", "roam_sweep_direction",
                       "confidence_band", "selection_last_seen_age_ms", "prediction_age_ms",
                       "roam_pattern", "roam_progress", "blackbox_capture_id",
                       "blackbox", "intent_has_joint_target", "intent_q_pitch_rad",
                       "intent_q_yaw_rad", "aim_point_valid", "aim_point_x",
                       "aim_point_y", "selected_uuid_valid", "selected_uuid",
                       "soft_limits_valid", "q_soft_min_pitch_rad",
                       "q_soft_max_pitch_rad", "q_soft_min_yaw_rad",
                       "q_soft_max_yaw_rad", "soft_limit_distance_pitch_rad",
                       "soft_limit_distance_yaw_rad"):
        assert re.search(rf"^\s*{field_name}\s*:", proto, re.M), (
            f"webd does not declare {field_name}: the page would read undefined "
            "and its own fallback text would hide it"
        )


def test_every_event_the_document_asks_for_has_a_name():
    """§79 lists the events by name. The name table is checked against that list.

    The list is read out of the architecture document rather than typed in here, for the
    reason every other guard in this file parses its source: a copied list keeps passing
    after the code has moved on, and it is the copy that made the old "dead buttons"
    possible in the first place. A missing name is not cosmetic — the fallback renders as
    UNKNOWN, and an operator looking at UNKNOWN learns only that something was forgotten.
    """
    from pathlib import Path

    root = Path(__file__).resolve().parents[3]
    doc = (root / "docs" / "archive" / "implemented" / "architecture" / "open_auto_turret_v3_three_mode_target_tracking_architecture.md").read_text(
        encoding="utf-8"
    )
    section = doc.split("# 79. Event logging", 1)
    assert len(section) == 2, "§79 moved; this guard is now pointing at nothing"
    asked = re.findall(r"^[A-Z][A-Z_]{3,}$", section[1].split("---")[0], re.M)
    assert len(asked) >= 15, f"§79's list did not parse ({len(asked)} names)"

    header = (root / "control" / "src" / "telemetry" / "telemetry.hpp").read_text(
        encoding="utf-8"
    )
    named = set(re.findall(r'return "([A-Z][A-Z_]+)";', header))
    missing = sorted(set(asked) - named)
    assert not missing, (
        f"§79 asks for events with no entry in event_name(): {missing}"
    )
