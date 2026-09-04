"""webd <-> controld JSON wire protocol (architecture §5.3, §6.1).

Transport is a ``SOCK_SEQPACKET`` Unix-domain socket, so each message is a
self-contained JSON object and message boundaries are preserved. Three message
types flow over it:

  * ``telemetry`` (controld -> webd): the §6.3 snapshot, downsampled to
    10-20 Hz. Read-only state for the dashboard.
  * ``command``   (webd -> controld): a high-level developer command
    (§42.2). ``{"type":"command","command":"<name>","arg":"<arg>"}``.
  * ``response``  (controld -> webd): the result of a command
    (``{"type":"response","command":"<name>","ok":..,"error":".."}``).

webd NEVER opens can0 and NEVER decides safety: it only submits commands, and
controld's ``validate_command`` gate decides whether they are legal. Video
frames do NOT traverse this socket (§42.3) — they are a separate low-priority
path.

The telemetry field names mirror ``format_telemetry`` in
``control/src/web/web_server.hpp`` exactly (same keys, same casing):
``track_state`` lowercase, ``installation_source`` lowercase, ``safety_action``
UPPERCASE.
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field, asdict
from typing import Any, Optional, Tuple


# Half a second. controld publishes telemetry at roughly 15 Hz, so this is about seven missing
# frames: long enough that a hiccup does not flash the banner, short enough that an operator is not
# watching frozen numbers for a second and a half without being told. The document names no number;
# this one is derived from the publish rate and is stated rather than hidden in a comparison. It
# lives here, beside the field it governs, so the server and the page cannot drift apart.
TELEMETRY_STALE_AFTER_S = 0.5


@dataclass
class Telemetry:
    """One §6.3 telemetry snapshot (controld -> webd)."""

    ts_ns: int = 0
    # Loop phase ("idle"|"homing"|"hold"|"parking"|"parked"|"fault"|
    # "payload_check") + the fault reason while phase == "fault" (§6.3).
    phase: str = ""
    fault: str = ""
    # Homed AND holding the ready pose (the P0 "homed + at ready pose" state).
    # `phase` alone cannot tell "waiting for a target" from "still homing".
    at_ready: bool = False
    track_state: str = "ready_hold"
    tracking_active: bool = False
    target_confidence: float = 0.0
    q_yaw_rad: float = 0.0
    v_yaw_rad_s: float = 0.0
    q_ref_yaw_rad: float = 0.0
    q_pitch_rad: float = 0.0
    v_pitch_rad_s: float = 0.0
    q_ref_pitch_rad: float = 0.0
    effort_yaw: float = 0.0
    effort_pitch: float = 0.0
    target_az_world_rad: float = 0.0
    target_el_world_rad: float = 0.0
    base_roll_rad: float = 0.0
    base_pitch_rad: float = 0.0
    base_yaw_rad: float = 0.0
    installation_calibrated: bool = False
    installation_source: str = "identity"
    safety_action: str = "ALLOW"
    feedback_age_ms: int = 0
    control_cycle_us: int = 0
    # v3 §50: who is driving, and why. Strings by design — this crosses a process
    # boundary into a browser, and an enum renumbered on one side becomes a wrong
    # label on the other with nothing to complain about it.
    operating_mode: str = ""             # MANUAL | AUTO_TRACK | AUTO_ROAM
    supervisory_state: str = ""          # READY | HOMING | PARKING | FAULT | ...
    mode_phase: str = ""                 # WAIT_TARGET | TRACK | COAST | SWEEP | ...
    intent_source: str = "none"          # who is asking for motion (§26)
    intent_type: str = "hold"            # what it asked for (§25)
    # §73's reticle, normalised against the detector's frame like `tracks`. controld owns the
    # gimbal angles, the camera extrinsic and the intrinsics, so the projection is done there
    # and shipped as a position; when `aim_point_valid` is false there is no aim point to
    # show — no estimate yet, or the geometry and the picture disagree — and the two numbers
    # beside it are not measurements.
    # §50/§78: the bounds of travel. controld refuses a step that leaves them and refuses a
    # reference beyond them, so the page should not be the one place that cannot say where the
    # end is. `soft_limits_valid` is false until homing has measured the range — the bounds are
    # then zero, and zero is the bound every axis appears to share before anything is measured.
    soft_limits_valid: bool = False
    q_soft_min_pitch_rad: float = 0.0
    q_soft_max_pitch_rad: float = 0.0
    q_soft_min_yaw_rad: float = 0.0
    q_soft_max_yaw_rad: float = 0.0
    soft_limit_distance_pitch_rad: float = 0.0
    soft_limit_distance_yaw_rad: float = 0.0
    aim_point_valid: bool = False
    aim_point_x: float = 0.0
    aim_point_y: float = 0.0
    intent_has_joint_target: bool = False  # §92: whether a pose was asked for at all
    intent_q_pitch_rad: float = 0.0      # what the mode wanted, before the envelope (§33)
    intent_q_yaw_rad: float = 0.0        # or the LOS solver (§67) changed it
    intent_reason: str = ""              # why
    intent_velocity_scale: float = 1.0   # confidence / derate applied to the ask
    # §52: the answer to the last command. -1 = none executed since controld
    # started; that is not the same claim as "the last one worked", and a client
    # that renders it as one is lying about a command nobody sent.
    cmd_ack_command: str = ""
    cmd_ack_accepted: int = -1           # -1 none | 0 rejected | 1 accepted
    cmd_ack_reason: str = ""
    cmd_ack_controller_state: str = ""
    cmd_ack_safety_state: str = ""
    # §61: v3 publisher generation, the two latency intervals (-1 = never), and which
    # candidate controld is actually following.
    vision_track_sets: int = 0
    vision_sensor_age_ms: int = -1
    vision_publish_to_receive_ms: int = -1
    selected_track_id: int = 0
    # §13/§78: the operator's selection as controld understands it.
    selected_display_index: int = 0
    selected_descriptor: str = ""
    selection_visibility: str = "NONE"
    selection_ambiguous: bool = False
    reacquisition_score: float = 0.0
    ambiguity_margin: float = 0.0
    # §11/§78: the candidate list. A list of plain dicts rather than a nested
    # dataclass, because telemetry_from_json copies values straight through and
    # asdict() on the way out would otherwise rebuild objects on every telemetry
    # frame for no reader's benefit. Undeclared fields here do not merely lose
    # their type — they disappear from the re-serialised frame entirely, which is
    # how selected_track_id once looked like a vision bug.
    # §79: the structured event window and the counter behind it. Undeclared fields do
    # not error here — they stop arriving, which is a much worse failure to diagnose.
    # §80: a preserved scene. The id changes only when the station stopped believing what
    # it was doing; the dict is that instant, and webd is what writes it to disk (see
    # controld_client). Absent when nothing has gone wrong since controld started.
    blackbox_capture_id: int = 0
    blackbox: dict = field(default_factory=dict)
    event_generation: int = 0
    events: list = field(default_factory=list)
    track_count: int = 0
    track_list_age_ms: int = -1
    # Each entry is controld's own listing: uuid, display_index, label, class_name, state,
    # confidence, anchor_x/anchor_y, selectable, selected, and `bbox` = [x_min, y_min,
    # x_max, y_max] normalised to the detector's frame. A missing or all-zero `bbox` means
    # no box was reported (an older controld), and the page must draw a marker in that case
    # rather than a rectangle at the origin.
    tracks: list = field(default_factory=list)
    roam_target_yaw_rad: float = 0.0
    roam_sweep_direction: int = 0
    manual_lease_active: bool = False
    manual_lease_remaining_ms: int = 0
    manual_profile: str = ""
    # §50's remaining fields. See the note on `tracks`: an undeclared field here does not
    # raise, it just stops arriving.
    # §50's `track_uuid`, live. Text, deliberately: the high half of a track identifier is a
    # 64-bit session nonce and JSON numbers are doubles here, so a uuid carried as a number
    # survives only while the nonce is small — it works in every test anyone thinks of and
    # rounds on a real session, at the precision where two tracks stop being the same track.
    selected_uuid_valid: bool = False
    selected_uuid: str = ""
    selection_last_seen_age_ms: int = -1
    prediction_age_ms: int = -1
    roam_pattern: str = ""
    roam_progress: float = 0.0
    confidence_band: str = "INVALID"
    selected_confidence: float = 0.0
    cmd_ack_seq: int = 0
    # Phase 9 payload verification (§28.5, §31.3, §42.1).
    payload_profile_name: str = ""          # active stored profile ("" = none)
    payload_profile_status: str = "no_profile"  # ok|no_profile|mismatch|error
    payload_derated: bool = False           # motion limits derated (mismatch)
    payload_check_active: bool = False      # in-loop verification running
    # Vision transport (§6.1/§6.2, Part 2 S1): distinguishes "no detector
    # output at all" from "the detector sees nothing".
    vision_connected: bool = False          # a visiond publisher is attached
    vision_frames: int = 0                  # decoded measurements since boot
    vision_dropped: int = 0                 # bad-size / undecodable datagrams
    vision_last_frame_sequence: int = 0
    vision_measurement_age_ms: int = -1     # since the last measurement
    # v3.2 section 20 camera geometry, which the HUD draws with. `camera_intrinsics` is the
    # calibration the DAEMON loaded, as an object {"valid","fx","fy","cx","cy","width","height"}.
    # When no calibration file existed it arrives valid=false and the page must say the reticle is
    # unclosed instead of assuming the image centre: section 7 makes the reticle the actual optical
    # axis, and the operator's centring acceptance is measured against that mark, so a guessed
    # principal point would quietly move the thing the acceptance rule is judged by.
    # effective_*_deg of 0.0 means NOT MEASURED, not "no field of view".
    camera_intrinsics: Dict[str, Any] = field(default_factory=dict)
    effective_hfov_deg: Optional[float] = None
    effective_vfov_deg: Optional[float] = None
    camera_fps: Optional[float] = None
    # Where inside the selected target the axis is aimed. NOT the same as aim_point_*, which is
    # where the axis currently points; this is the point inside the target the controller is trying
    # to put on it. The acceptance rule is the distance between the two, so the HUD needs both.
    target_aim_x_norm: float = 0.0
    target_aim_y_norm: float = 0.0
    target_aim_valid: bool = False
    target_aim_is_head: bool = False
    # The direction the AUTO_TRACK intent is built from, its horizon, and the rates that give it
    # value. Without these, lead cannot be measured: q_ref is post-slew-limiter, so it cannot tell
    # "no lead requested" from "lead requested and the reference could not slew" - opposite fixes.
    predicted_target_az_world_rad: Optional[float] = None
    predicted_target_el_world_rad: Optional[float] = None
    predicted_target_los_valid: bool = False
    # Declared because undeclared keys are dropped when a Telemetry is re-serialised: a field that
    # arrives from controld but is not named here simply never reaches the page.
    envelope_v_max_deg_s: Optional[float] = None
    prediction_horizon_ms: Optional[int] = None
    target_az_rate_world_rad_s: Optional[float] = None
    target_el_rate_world_rad_s: Optional[float] = None
    # The reference profile's own rate/accel: what "smooth" has to be judged on. The motion log's
    # v/a/j come from motor feedback and are too noisy to prove anything about the profile.
    q_ref_rate_yaw_rad_s: Optional[float] = None
    q_ref_rate_pitch_rad_s: Optional[float] = None
    q_ref_accel_yaw_rad_s2: Optional[float] = None
    q_ref_accel_pitch_rad_s2: Optional[float] = None
    q_ref_rate_valid: bool = False
    # §25: "stale telemetry stops visual interpolation and indicates stale/disconnected state".
    # Age is computed by webd when the snapshot is READ, not when it was received, because a cached
    # frame that is served for ten seconds is ten seconds old at the moment the operator sees it.
    # The threshold lives here so the server and the page cannot disagree about what "stale" means.
    telemetry_age_ms: Optional[int] = None
    telemetry_stale: bool = False
    # §20's nested prediction block, passed through as received. Declared as one dict rather than
    # field by field because telemetry_from_json silently drops keys it has never seen - twice now
    # this project has mistaken "webd never sent it" for "the daemon does not have it" - and a
    # container declaration means a new key inside the block arrives instead of vanishing.
    prediction: Optional[dict] = None
    # §20's field_of_regard block, passed through as received for the same reason as `prediction`:
    # telemetry_from_json discards keys it has not seen, so an undeclared field is indistinguishable
    # from one the daemon never sent. This exact shape of nothingness has already cost this project a
    # wrong conclusion twice; every container the contract names gets declared.
    field_of_regard: Optional[dict] = None
    # §20's camera and imu blocks, declared for the same reason as the two above them: an undeclared
    # field is indistinguishable from one the daemon never sent. `imu` in particular carries the
    # station's honest answer that there is no inertial sensor - operator-confirmed 2026-09-04 - and a
    # statement of absence is only useful if it survives the trip to the page.
    camera: Optional[dict] = None
    imu: Optional[dict] = None
    # CAN link health (§55 CAN family, §54.4 error states). The transport has
    # counted these from the start; they are here so a degrading link is visible
    # BEFORE feedback goes stale and the supervisor reacts to the symptom.
    # can_available=false means there is no CAN link at all (the simulated
    # backend), so the zeros below must never be read as "a healthy bus".
    can_available: bool = False
    can_kind: str = ""                      # socketcan | yousee
    can_device: str = ""                    # can0 | /dev/ttyUSB0
    can_up: bool = False
    can_state: int = -1                     # -1 unknown, 0 error-active,
                                            # 1 warning, 2 passive, 3 bus-off
    can_rx_frames: int = 0
    can_rx_error_frames: int = 0
    can_tx_frames: int = 0
    can_tx_failed: int = 0
    can_last_rx_age_ms: int = -1            # -1 = nothing received yet


@dataclass
class CommandMessage:
    """A high-level developer command (webd -> controld, §42.2)."""

    command: str
    arg: str = ""

    def to_json(self) -> str:
        return command_to_json(self.command, self.arg)


@dataclass
class ResponseMessage:
    """The result of a command (controld -> webd)."""

    command: str
    ok: bool
    error: str = ""
    # "submitted" once the command passes controld's gate and is queued, "rejected" when the gate
    # refused it. `ok` on its own reads as "the station did it", which is a different question: the
    # execution verdict arrives separately as cmd_ack_* on the telemetry stream.
    verdict: Optional[str] = None


def command_to_json(command: str, arg: str = "") -> str:
    """Serialize a command message to the wire JSON string."""
    return json.dumps(
        {"type": "command", "command": command, "arg": arg},
        separators=(",", ":"),
    )


def telemetry_to_json(t: Telemetry) -> str:
    """Serialize a telemetry snapshot to the wire JSON string."""
    d = asdict(t)
    d["type"] = "telemetry"
    return json.dumps(d, separators=(",", ":"))


def telemetry_from_json(obj: dict) -> Telemetry:
    """Build a Telemetry from a decoded telemetry JSON object.

    Unknown keys are ignored and missing keys fall back to the dataclass
    defaults, so the wire format can grow without breaking old clients.
    """
    fields = {f for f in Telemetry.__dataclass_fields__}  # type: ignore[attr-defined]
    kwargs = {k: v for k, v in obj.items() if k in fields and k != "type"}
    return Telemetry(**kwargs)


def parse_message(raw: str) -> Tuple[str, Any]:
    """Decode one wire message. Returns ``(type, payload)`` where payload is a
    dict for telemetry (the raw JSON object) and a dataclass for command /
    response."""
    obj = json.loads(raw)
    t = obj.get("type")
    if t == "telemetry":
        return "telemetry", obj
    if t == "command":
        return "command", CommandMessage(
            command=obj.get("command", ""), arg=obj.get("arg", "")
        )
    if t == "response":
        return "response", ResponseMessage(
            command=obj.get("command", ""),
            ok=bool(obj.get("ok", False)),
            error=obj.get("error", ""),
            verdict=obj.get("verdict"),
        )
    raise ValueError(f"unknown message type: {t!r}")
