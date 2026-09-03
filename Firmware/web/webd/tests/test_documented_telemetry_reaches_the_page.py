"""§50 and §78: every documented telemetry field has to arrive somewhere an operator can read.

This guard exists because the same defect has now been found three times by hand, each time
differently. The bounds of travel were computed by controld and published nowhere; the selected
target's identifier was published as a display index and, where the real 128-bit value did
appear, only inside a capture taken after something had already gone wrong; a §50 field could be
added to the daemon and silently fail to reach the page because webd re-serialises a *declared*
model. None of those produced an error. They produced a page that looked complete.

So the two sections that describe what the operator is owed are read out of the architecture
document itself — not transcribed here, because a transcription drifts from the document and then
agrees with itself, which is the worst kind of passing test — and each documented field must
either:

  * be mapped to a real name that exists in the snapshot, in the wire JSON, in webd's declared
    model, and (where it is meant to be seen) on the page; or
  * appear in `ABSENT`, with a reason, and still be absent.

The second branch is the point. A gap that is written down and *checked* stays visible: the day
somebody implements it, this test fails and makes them update the map, which is the only way a
known hole stops being known.

Scope note: this is a presence check across process boundaries. It cannot see whether a number is
correct, whether the page renders it legibly, or whether it arrives often enough to be useful —
those are separate tests elsewhere, and several §110 items still need a person with a turret.
"""
from __future__ import annotations

import pathlib
import re

REPO = pathlib.Path(__file__).resolve().parents[4]
DOC = REPO / "Firmware" / "docs" / "open_auto_turret_v3_three_mode_target_tracking_architecture.md"
SNAPSHOT = REPO / "Firmware" / "control" / "src" / "telemetry" / "telemetry.hpp"
WIRE = REPO / "Firmware" / "control" / "src" / "web" / "web_server.hpp"
PROTOCOL = REPO / "Firmware" / "web" / "webd" / "protocol.py"
PAGE = REPO / "Firmware" / "web" / "webd" / "dashboard.py"


def _section(title: str) -> str:
    """The body of a top-level `# NN. Heading` section, up to the next one."""
    text = DOC.read_text(encoding="utf-8")
    m = re.search(r"^# \d+\. %s\b(.*?)(?=^# \d+\. |\Z)" % re.escape(title), text, re.M | re.S)
    assert m, "§%s has moved out of the architecture document" % title
    return m.group(1)


def _doc_items() -> list:
    """(group, field) for every field §50 lists, and every bullet §78 asks to add."""
    items = []
    body = _section("Runtime UI state schema")
    fence = re.search(r"```text\n(.*?)```", body, re.S)
    assert fence, "§50 no longer carries its field list in a text block"
    group = None
    for line in fence.group(1).splitlines():
        if not line.strip():
            continue
        if not line.startswith(" ") and line.rstrip().endswith(":"):
            group = line.strip()[:-1]
        elif group:
            items.append((group, line.strip()))

    body = _section("Diagnostics additions")
    group = None
    for line in body.splitlines():
        head = re.match(r"^## (.+)$", line.strip()) if line.startswith("## ") else None
        if head:
            group = head.group(1).strip()
            continue
        bullet = re.match(r"^- (.+?);?$", line.strip())
        if bullet and group:
            items.append((group, bullet.group(1).strip().rstrip(".").lower()))
    return items



# What each documented field is allowed to be called in the code, and how far it has to travel.
# `page=False` is not a shrug: it means the field is on the wire for a machine or a second view,
# and the reason says who reads it. Everything marked page=True is checked to be read by the
# dashboard, because "published" and "visible" have been confused here before.
def M(name, why, page=True, wire=None, proto=True, snap=None):  # noqa: N802 - terse on purpose
    """`name` is what the operator's side calls it (the wire key, and what protocol.py and the
    page read); `snap` is what controld's struct calls it, when the two differ — which happens
    when a field crosses as text but is stored as text-plus-a-validity-flag."""
    return dict(name=name, page=page, wire=wire or name, proto=proto, why=why, snap=snap or name)


MAPPED = {
    ("operating_mode", "MANUAL | AUTO_TRACK | AUTO_ROAM"): M(
        "operating_mode", "the three-mode label itself"),
    ("mode_phase", "..."): M(
        "mode_phase", "the ellipsis is the document's; the states are the §20 phases, "
                      "published as text"),

    ("target_selection", "selected"): M(
        "selected_track_id", "the page marks the selected row through the track list rather "
                             "than reading this, but the field is on the wire", page=False),
    ("target_selection", "track_uuid"): M(
        "selected_uuid", "as text: the high half is a 64-bit session nonce and a JSON number "
                         "would round it", snap="selected_uuid_text"),
    ("target_selection", "display_label"): M(
        "selected_descriptor", "'Person #2' (§10); the per-row label rides on each track",
        page=False),
    ("target_selection", "class_name"): M(
        "class_name", "carried per track in the list, which webd forwards whole", page=False,
        proto=False),
    ("target_selection", "visibility"): M("selection_visibility", ""),
    ("target_selection", "confidence"): M("selected_confidence", ""),
    ("target_selection", "last_seen_age_ms"): M("selection_last_seen_age_ms", ""),

    ("tracks", "visible candidate list"): M(
        "track_count", "the count beside the list; the page draws the rows, so the number "
                       "itself is for a machine reading the same frame", page=False),

    ("auto_track", "state"): M(
        "mode_phase", "AUTO_TRACK's §20 state *is* mode_phase in v3; the v1 `track_state` is "
                      "published beside it for the legacy view"),
    ("auto_track", "prediction_age_ms"): M(
        "prediction_age_ms", "how long the aim has been running on prediction rather than "
                             "measurement"),
    ("auto_track", "reacquire_score"): M(
        "reacquisition_score", "spelled out in full; the document's abbreviation does not cross "
                               "a process boundary"),

    ("auto_roam", "pattern"): M("roam_pattern", ""),
    ("auto_roam", "direction"): M("roam_sweep_direction", ""),
    ("auto_roam", "current_waypoint"): M(
        "roam_target_yaw_rad", "the waypoint as the joint goal the drive is actually given"),
    ("auto_roam", "progress"): M("roam_progress", ""),

    ("manual", "jog_lease_active"): M("manual_lease_active", ""),
    ("manual", "speed_profile"): M("manual_profile", "FINE/NORMAL/FAST"),
}

# §78's additions.
MAPPED.update({
    ("Track manager", "number of active tracks"): M("track_count", "", page=False),
    ("Track manager", "track ids"): M(
        "uuid_text", "both halves, as text, on each track in the list", page=False, wire="uuid",
        proto=False),
    ("Track manager", "missing-frame counts"): M(
        "vision_dropped", "counted where frames are lost — the vision link; per-track staleness "
                          "is `measurement_age_ms`"),
    ("Target selection", "selected uuid"): M("selected_uuid", "same field, §78's list",
                                             snap="selected_uuid_text"),
    ("Target selection", "selected display label"): M("selected_descriptor", "", page=False),
    ("Target selection", "reacquisition score"): M("reacquisition_score", ""),
    ("Target selection", "ambiguity margin"): M("ambiguity_margin", ""),
    ("AUTO_TRACK", "state"): M("mode_phase", ""),
    ("AUTO_TRACK", "target los"): M("target_az_world_rad", "az and el"),
    ("AUTO_TRACK", "confidence derating"): M(
        "intent_velocity_scale", "the scale the intent carries, with `confidence_band` beside it"),
    ("AUTO_ROAM", "pattern"): M("roam_pattern", ""),
    ("AUTO_ROAM", "direction"): M("roam_sweep_direction", ""),
    ("AUTO_ROAM", "waypoint"): M("roam_target_yaw_rad", ""),
    ("MANUAL", "active jog lease"): M("manual_lease_active", ""),
    ("MANUAL", "remaining lease time"): M(
        "manual_lease_remaining_ms", "the dead-man countdown — the number that tells a hand when "
                                     "to let go"),
    ("MANUAL", "speed profile"): M("manual_profile", ""),
})


# Documented, not implemented, and *checked to still be absent*. Each is a promise this station
# cannot keep yet; the reason says why, and closing one should break this test rather than pass it
# quietly — that is the only mechanism by which a known hole stays known.
ABSENT = {
    ("AUTO_TRACK", "predicted los"): (
        "predicted",
        "§78 asks for the predicted LOS beside the measured one. The estimator holds it "
        "internally and the loop uses it to project the aim point, but nothing publishes an "
        "az/el for it, so asking 'where did it think the target would be' still means reading a "
        "capture."),
    ("AUTO_ROAM", "boundary margin"): (
        "roam_boundary_margin",
        "§78 asks how far inside the roam region's edge the sweep is running. The planner knows "
        "(it clamps waypoints to stay inside) and nothing publishes it, so an operator cannot see "
        "the sweep working near its limit."),
    ("Track manager", "association timing"): (
        "association",
        "§78 asks how long association takes. Nothing measures it: the cycle cost is published, "
        "not the per-association cost, and a field that just renamed `track_list_age_ms` would "
        "be worse than admitting the gap."),
    ("manual", "frame"): (
        "manual_frame",
        "§50 lists the manual frame (Level / Joint). §110 MANUAL/7 makes the Level frame "
        "conditional on an IMU existing and this station has none, so there is no frame to "
        "publish — recorded absent rather than silently unmapped, so the day an IMU is fitted "
        "this test fails and somebody has to decide what the field says."),
}


def _field_names():
    """Every field name declared anywhere in telemetry.hpp — snapshot, capture, the per-track
    listing. A field the snapshot carries by embedding `TrackListing` counts, because that is what
    reaches the wire."""
    text = SNAPSHOT.read_text(encoding="utf-8")
    # The alternation is spelled out because a bare `int` does not match `int\\d*_t`, and a
    # guard that silently forgets every `int` field is the kind that reports a clean pass while
    # checking nothing at all.
    names = set(re.findall(
        r"^\s+(?:bool|u?int\d*_t|int|size_t|double|float|char|std::string)\s+([a-z_0-9]+)",
        text,
        re.M))
    return names


def _wire_keys():
    """Every JSON key controld emits, live telemetry and capture alike."""
    text = WIRE.read_text(encoding="utf-8")
    return set(re.findall(r'\\"([a-z_0-9]+)\\":', text))


def test_the_two_sections_still_have_their_lists():
    """Parse first, judge second: if either section stops looking like a list, this file's map
    is describing a document that no longer exists and every conclusion below is fiction."""
    items = _doc_items()
    groups = {g for g, _ in items}
    assert len(items) > 25, "only %d documented fields parsed" % len(items)
    for wanted in ("operating_mode", "mode_phase", "target_selection", "auto_track", "auto_roam",
                   "manual", "tracks"):
        assert wanted in groups, "§50's `%s` group was not parsed" % wanted
    for wanted in ("Track manager", "Target selection", "AUTO_TRACK", "AUTO_ROAM", "MANUAL"):
        assert wanted in groups, "§78's `%s` group was not parsed" % wanted


def test_every_mapped_field_exists_at_every_layer_it_is_promised():
    names, keys = _field_names(), _wire_keys()
    protocol = PROTOCOL.read_text(encoding="utf-8")
    page = PAGE.read_text(encoding="utf-8")
    broken = []
    for (group, field), spec in sorted(MAPPED.items()):
        name = spec["name"]
        if spec["snap"] not in names:
            broken.append("%s.%s -> `%s` is not a field controld publishes"
                          % (group, field, spec["snap"]))
            continue
        if spec["wire"] not in keys:
            broken.append("%s.%s -> `%s` is computed but never put on the wire"
                          % (group, field, spec["wire"]))
        if spec["proto"] and name not in protocol:
            broken.append("%s.%s -> `%s` is undeclared in webd's model: undeclared fields do not "
                          "error there, they stop arriving" % (group, field, name))
        if spec["page"] and name not in page:
            broken.append("%s.%s -> `%s` reaches webd but nothing on the page reads it"
                          % (group, field, name))
    assert not broken, ("documented telemetry that does not reach an operator:\n  "
                        + "\n  ".join(broken))


def test_documented_gaps_are_still_gaps_and_say_so():
    """The honest half. A checklist that only verifies the things that were built reports
    progress in exactly the direction the author wants to hear."""
    names = _field_names()
    for (group, field), absent in sorted(ABSENT.items()):
        token, why = absent
        assert (group, field) not in MAPPED, (
            "%s.%s is mapped *and* listed absent; pick one" % (group, field))
        near = sorted(n for n in names if token in n)
        assert not near, (
            "%s.%s looks implemented now (fields like %s exist). §50/§78 coverage changed: move "
            "it into MAPPED and say what it means.\n  reason on file: %s"
            % (group, field, ", ".join("`%s`" % n for n in near), why))


def test_nothing_in_the_document_is_unmapped_without_being_absent():
    """Anti-drift: a field added to §50 or §78 has to be dealt with here — mapped to a real
    name, or recorded as a gap with a reason. Neither answer is comfortable, which is the
    point; 'nobody noticed' is the only answer that is free."""
    unmapped = []
    for item in _doc_items():
        if item not in MAPPED and item not in ABSENT:
            unmapped.append("%s.%s" % item)
    assert not unmapped, (
        "documented telemetry fields with no verdict — mapped or explicitly absent:\n  "
        + "\n  ".join(unmapped)
        + "\n  (add it to MAPPED with the code name and why, or to ABSENT with the reason "
          "this station cannot publish it)")
