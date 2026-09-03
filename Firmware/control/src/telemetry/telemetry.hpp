// OpenAutoTurret — telemetry and logging (architecture §6.3, §43).
//
// In-memory, allocation-bounded (ring buffers) telemetry that the control loop
// fills every cycle. No file I/O in the control loop (§46): the black-box ring
// and logs are in memory and can be persisted on fault or sampled to a log file
// at a low rate by a separate (non-real-time) thread.
//
//   §43.1  high-rate control log  — near control rate, both axes
//   §43.3  event log              — structured state/safety events
//   §43.4  black-box ring buffer  — rolling history for fault diagnosis
//   §6.3   telemetry snapshot     — the 10-20 Hz published snapshot
//
// Pure data structures — no CAN, no camera, no motor driver.
#pragma once

#include <array>
#include <cstddef>
#include <cstdio>
#include <mutex>
#include <string>
#include <vector>

#include "common/types.hpp"
#include "control/safety_supervisor.hpp"
#include "tracking/tracking_state_machine.hpp"

namespace ota {
namespace telemetry {

// §6.3 telemetry snapshot (published at 10-20 Hz).
// §78/§11: one candidate as the machine sees it, for the operator's list and the
// overlay. Fixed size and POD on purpose: this sits inside the snapshot the control
// thread fills at 200 Hz, so it must not allocate, and a bounded list is also the
// honest one — §58 caps what a TrackSet carries, and a telemetry list that could grow
// would be a second, unbounded buffer behind it.
// Long enough for "18446744073709551615:18446744073709551615" plus the NUL, which is the whole
// 128-bit identifier in decimal. One constant, so the listing, the snapshot and the capture all
// carry the same width and the formatter cannot be handed an array it would overflow.
inline constexpr std::size_t kUuidTextLen = 41;

// Both halves of a track identifier as text, in one place, so the live page and a post-incident
// capture cannot drift into describing the same track with two different strings.
inline void format_uuid_text(char (&out)[kUuidTextLen], uint64_t hi, uint64_t lo) {
  std::snprintf(out, kUuidTextLen, "%llu:%llu", static_cast<unsigned long long>(hi),
                static_cast<unsigned long long>(lo));
}

struct TrackListing {
  uint64_t uuid_lo = 0;
  // The other half of the identifier, and both of them as text. `uuid_lo` alone names a
  // *numbering*, not a track: the high half is the session nonce visiond draws when it starts
  // (vision/track_manager.py), which is what distinguishes this run's track 22 from the last
  // run's track 22. Text because that nonce spans the whole 64-bit range and a JavaScript
  // number is exact only to 2^53 — a browser handed the number would print an identifier that
  // never existed, and quoting it into an investigation would name a track no log contains.
  uint64_t uuid_hi = 0;
  char uuid_text[kUuidTextLen] = {};
  uint16_t display_index = 0;
  char label[24] = {};        // "Person #2" (§10) — what the operator reads
  char class_name[16] = {};   // as the detector named it
  char state[16] = {};        // CONFIRMED | TENTATIVE | OCCLUDED | LOST
  float confidence = 0.0f;
  float anchor_x = 0.0f;      // normalised, for the overlay's box
  float anchor_y = 0.0f;
  // The detector's box, normalised against the frame the detector used — so it scales
  // onto whatever the page is showing without knowing the sensor's size. All four at zero
  // means **there is no box**, not a box in the top-left corner: a page that drew this
  // unconditionally would paint a person-shaped lie at the origin of every frame the
  // moment it talked to a controld that does not fill them.
  float bbox[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  bool selectable = false;    // §8: only CONFIRMED can be chosen
  bool selected = false;      // the operator's choice, marked in the list
};

// §80's list, in one struct, fixed size for the reasons above. Nothing here is derived
// later: every field is what the control loop believed at that instant, which is the only
// version of the story that a replay can be checked against.
struct BlackBoxCapture {
  uint64_t id = 0;
  TimeNs t_ns = 0;
  char reason[64] = {};

  char operating_mode[16] = {};
  char mode_phase[24] = {};
  char phase[16] = {};
  char safety_action[16] = {};

  // The selection and everything the detector was offering at the time. "All candidate
  // tracks" is the phrase in §80, and it is the part that cannot be reconstructed: the
  // scene is gone, and a replay that shows only the chosen track proves nothing about
  // whether the choice was the only defensible one.
  // Was named `selected_uuid` while holding `selected_track_id`, which is a display index. A
  // capture that mislabels an identifier is worse than one missing it — an investigator reads
  // "uuid 11" as a fact about the world and searches for a track that never existed — so the
  // number is called what it is and the real identifier travels beside it as text (§50/§78).
  uint64_t selected_display_index = 0;
  char selected_uuid_text[kUuidTextLen] = {};
  char selected_label[24] = {};
  char selection_visibility[24] = {};
  int64_t selection_age_ms = -1;
  bool selection_ambiguous = false;
  float reacquisition_score = 0.0f;
  std::array<TrackListing, 8> candidates{};
  int candidate_count = 0;

  // What was asked for, as distinct from what the reference generator produced. The
  // `intent_has_joint_target` flag is load-bearing: an intent may be a LOS aim or a hold
  // rather than a joint pose, and publishing a zero pose for those would say "it was
  // told to go to zero", which is a different accident entirely. Index order matches
  // ix(AxisId::): 0 = pitch, 1 = yaw.
  bool intent_has_joint_target = false;
  char intent_type[16] = {};
  char intent_source[16] = {};
  double q_cmd[2] = {0.0, 0.0};
  double q_ref[2] = {0.0, 0.0};
  double q_actual[2] = {0.0, 0.0};
  double v_actual[2] = {0.0, 0.0};

  // The estimate and how old it was, plus the transport ages: a turret that swung at a
  // two-second-old measurement is not broken in the same place as one that swung at a
  // fresh one.
  double target_az_world_rad = 0.0;
  double target_el_world_rad = 0.0;
  bool estimator_ready = false;
  int64_t measurement_age_ms = -1;
  int64_t feedback_age_ms = -1;
};

struct TelemetrySnapshot {
  static constexpr int kMaxTrackList = 8;
  TimeNs timestamp_ns = 0;
  // Control-loop phase name ("idle"|"homing"|"hold"|"parking"|"parked"|
  // "fault"|"payload_check") + the fault reason when phase=fault. §6.3 lists
  // the loop state as snapshot content; the dashboard's System panel and the
  // station IPC client both need it (§42.1).
  std::string phase;
  std::string fault_reason;
  // "homed AND holding the safe ready pose" — the same condition the operator's
  // P0 log line reports ("homed + at ready pose") and the §27 auto payload check
  // gate. It distinguishes "waiting for a target" from "still homing / moving to
  // the ready pose", which `phase` alone cannot (§6.3/§38.1).
  bool at_ready = false;
  // Per-axis actuals.
  double q_yaw_rad = 0.0;
  double v_yaw_rad_s = 0.0;
  double effort_yaw = 0.0;
  double q_pitch_rad = 0.0;
  double v_pitch_rad_s = 0.0;
  double effort_pitch = 0.0;
  // Per-axis references.
  double q_ref_yaw_rad = 0.0;
  double q_ref_pitch_rad = 0.0;
  // Target / tracking.
  tracking::TrackState track_state = tracking::TrackState::ReadyHold;
  double target_confidence = 0.0;
  bool tracking_active = false;
  double target_az_world_rad = 0.0;   // world-frame LOS (valid when tracking)
  double target_el_world_rad = 0.0;
  // Vision transport (§6.1/§6.2, Part 2 S1): what the ingest thread actually
  // sees, so the dashboard can tell "no detector output" apart from "detector
  // sees nothing". Ages are host-monotonic.
  bool vision_connected = false;      // a visiond publisher is attached
  uint64_t vision_frames = 0;         // decoded measurements since boot
  uint64_t vision_dropped = 0;        // datagrams with a bad size / decode
  uint64_t vision_last_frame_sequence = 0;
  int64_t vision_measurement_age_ms = -1;

  // v3.2 §20 - camera geometry the operator-facing HUD has to have.
  //
  // The reticle represents the actual optical axis (§7), which IS the measured principal point;
  // the FOR inset and any "will it leave the frame" judgement need the field of view (§11). If
  // the page guessed the image centre instead of reading cx/cy, "the reticle overlaps the target"
  // would hold only by coincidence, and a hard-coded FOV is wrong the moment the stream size
  // changes - this sensor's field does change with it (79.3 / 81.2 / 68-79 deg per frame width at
  // 640x480 / 1280x720 / 1920x1080, measured). Zero means not measured, and stays zero.
  bool camera_intrinsics_valid = false;
  double camera_fx_px = 0.0;
  double camera_fy_px = 0.0;
  double camera_cx_px = 0.0;
  double camera_cy_px = 0.0;
  int32_t camera_width = 0;
  int32_t camera_height = 0;
  double effective_hfov_deg = 0.0;
  double effective_vfov_deg = 0.0;
  double camera_fps = 0.0;

  // Where inside the selected target the axis is aimed (v3.2 s20's target_selection aim cue).
  //
  // Do not confuse these with aim_point_x/y/valid, which mean the opposite thing: aim_point_* is
  // where the AXIS is pointing, projected into the image (a boresight mark). target_aim_* is the
  // point INSIDE THE TARGET we intend to put on the axis - the head, when the station asks for it.
  // The HUD needs both: one to show where the gun is, one to show where it is trying to land, and
  // the acceptance rule is the distance between them. head=false means the controller fell back to
  // the anchor, and the cue must not claim a head it is not tracking.
  double target_aim_x_norm = 0.0;
  double target_aim_y_norm = 0.0;
  bool target_aim_valid = false;
  bool target_aim_is_head = false;

  // The direction the AUTO_TRACK intent is actually built from, and what it is worth.
  //
  // §78 asks telemetry to show the PREDICTED LOS, and until now it did not: the only LOS published
  // was target_az/el_world_rad, which is the estimator's filtered state and therefore lags by
  // construction. That gap made the operator's lead requirement unverifiable in a specific way -
  // q_ref is post-slew-limiter, so it cannot distinguish "no lead requested" from "lead requested,
  // reference could not slew" - and the two have opposite fixes. Rates are published alongside
  // because lead is rate x horizon; without the rate the horizon is just a number.
  double predicted_target_az_world_rad = 0.0;
  double predicted_target_el_world_rad = 0.0;
  bool predicted_target_los_valid = false;
  int64_t prediction_horizon_ms = 0;

  // §20 prediction block. This is NOT a second prediction: it is the same ray that
  // predicted_target_*_world_rad and aim_point_* already describe - the line of sight the axis will
  // be pointing along once the loop's measured actuation delay has elapsed - published under the
  // names the data contract uses, in the frame the contract asks for (camera LOS rather than world
  // azimuth/elevation) and the units it asks for (degrees, normalised image coordinates). One
  // computation under several names; a second estimator would be free to disagree, which is the
  // only new failure mode the split would add.
  //
  // `prediction_valid` means the ray exists and points in front of the camera.
  // `prediction_anchor_*_norm` additionally means it lands inside the frame. The two are separate
  // because a prediction that has left the frame is still a real prediction - the edge cue needs it
  // - but it is not a point that can be painted on the picture.
  bool prediction_valid = false;
  double prediction_los_yaw_rad = 0.0;    // camera frame, + right, 0 = optical axis
  double prediction_los_pitch_rad = 0.0;  // camera frame, + up
  double prediction_anchor_x_norm = 0.0;
  double prediction_anchor_y_norm = 0.0;
  bool prediction_anchor_in_frame = false;

  // §20's `field_of_regard.safe_envelope_points[]` and §11's inset, in DEGREES of logical joint
  // travel (§11.3: "FOR coordinates use yaw/pitch degrees", not image coordinates). That choice is
  // what makes this publishable at all: the camera-to-axis boresight is not separable from the
  // principal point with the spans the theodolite probe has, so a FOR drawn over the picture would
  // inherit an offset nobody has measured. In joint degrees, every number comes from the encoders.
  //
  // Flat pairs, [yaw_deg, pitch_deg, yaw_deg, pitch_deg, ...], with `for_envelope_count` points.
  // Room for 32 points is reserved so a piecewise or coupled envelope (§19 anticipates one, and the
  // envelope header says the constant rectangle can be replaced without touching the controller) can
  // publish its real outline through these same fields instead of a new contract.
  //
  // `for_envelope_kind` is 0 for none, 1 for the constant quadrilateral in force today. It is
  // published rather than implied so the page draws what it is told rather than assuming the region
  // is always four corners - the assumption is exactly what stops being true when §19's polygon lands.
  bool for_envelope_valid = false;
  int for_envelope_kind = 0;
  int for_envelope_count = 0;
  double for_envelope_deg[64] = {0.0};

  // §20's imu block. There is no inertial sensor on this station - not in the CAN definition, not in
  // the calibration files, not in the control code (the only "imu" anywhere in the tree is inside the
  // word "simulation"). These fields exist so the absence is a stated fact on the wire rather than an
  // empty space that each reader fills with its own guess, and they are a `present` flag rather than a
  // constant so that adding hardware later is a change of value, not a change of contract.
  //
  // `imu_world_elevation_valid` gates the number, and that gate is the whole point. A world elevation
  // of 0.0 would claim the turret is level; with no sensor the honest value is "no value", and the
  // emitter sends JSON null rather than the field's initialiser. Anything that flattens the null to
  // zero turns an unknown into a safety claim.
  bool imu_present = false;
  bool imu_gravity_valid = false;
  bool imu_world_elevation_valid = false;
  double imu_world_elevation_deg = 0.0;
  double target_az_rate_world_rad_s = 0.0;
  double target_el_rate_world_rad_s = 0.0;

  // The REFERENCE profile's own rate and acceleration, derived from successive published
  // references. This exists because requirement (b) asks whether following is SMOOTH, and the
  // v/a/j fields already in the motion log are derived from motor feedback: differencing noisy
  // feedback velocities produced a peak "jerk" of 665 deg/s^3 on an axis whose configured jerk
  // limit is 300, on a run where the axis barely moved. That number cannot support or refute a
  // smoothness claim either way. What matters for smoothness is the trajectory the controller
  // ASKED FOR, and nothing published it. Derived here with the same timestamps the pairs came in
  // on, and labelled as derived rather than as a measured motor quantity.
  double q_ref_rate_yaw_rad_s = 0.0;
  double q_ref_rate_pitch_rad_s = 0.0;
  double q_ref_accel_yaw_rad_s2 = 0.0;
  double q_ref_accel_pitch_rad_s2 = 0.0;
  bool q_ref_rate_valid = false;
  // v3 §61: publisher generation, the two latency intervals, and which candidate is
  // being followed. Ages are -1 for "never", never 0 — same rule as the CAN family.
  uint64_t vision_track_sets = 0;
  int64_t vision_sensor_age_ms = -1;
  int64_t vision_publish_to_receive_ms = -1;
  uint64_t selected_track_id = 0;
  // §13/§78: the operator's own choice, as controld understands it. The label and the
  // uuid are both published because they answer different questions — "which one did I
  // pick" is the label, "is the machine talking about the same thing I am" is the uuid,
  // and after a label is reused (§10) only the uuid can answer the second.
  uint16_t selected_display_index = 0;  // 0 = nothing selected
  std::string selected_descriptor;      // "Person #2" (§10)
  std::string selection_visibility;     // NONE | VISIBLE | OCCLUDED | LOST_REACQUIRABLE | STALE
  bool selection_ambiguous = false;     // §21: two candidates, reselect — do not guess
  float reacquisition_score = 0.0f;   // §78: what the best candidate scored
  float ambiguity_margin = 0.0f;      // §78: and how far ahead of the next-best
  // §78 AUTO_TRACK: the derating band the controller granted itself, and the number it
  // was computed from. intent_velocity_scale already carries the resulting scale; the
  // band says why, which is the difference between "it slowed down" and "it slowed down
  // because it was only half sure".
  // §78 MANUAL / §50 ui_state: is a jog lease live, how long is left on it, and which
  // profile it is running at. The remaining time is published rather than inferred
  // because the operator's question is "if the tab dies now, when does it stop" — and
  // the answer must come from the machine that would do the stopping.
  // §11/§78: the candidates in the last TrackSet controld received, in display-index
  // order. Without this the UI can say *that* there are two people and cannot show
  // *which is which*, which is the whole of the candidate list.
  std::array<TrackListing, kMaxTrackList> tracks{};
  int track_count = 0;
  // §80: a preserved scene. When the station stops believing what it was doing — a
  // safety brake, a fault, motion that did not match the request — the question is
  // "what did it think it was looking at", and by the time anyone reads the log the
  // answer is gone: the estimator moved on, the candidates changed, the selection aged
  // out. So the relevant state is *copied at that instant* into fixed storage and
  // published until someone takes it.
  //
  // It rides the snapshot rather than being written from the control thread because
  // writing a file is an unbounded-time operation, and the thread that has a 5 ms
  // deadline must not do the disk's work. The web thread publishes at 15 Hz and does the
  // writing; the id tells it when there is something new and tells a reader that what it
  // is looking at is a scene, not a live value.
  uint64_t blackbox_capture_id = 0;
  BlackBoxCapture blackbox{};
  // How long ago the newest frame in that list arrived, measured on controld's own clock
  // (0..n; -1 when no set has ever been received). Published because controld must not
  // invent track state between frames — association is visiond's per-frame job (§58), so
  // with vision silent the last set stays exactly as it was received, and the *only*
  // honest way to show that is to say how old it is. A page that greys the list on this
  // number is truthful; a controld that rewrote CONFIRMED to LOST on a timer would be
  // reporting an inference as an observation.
  int64_t track_list_age_ms = -1;
  bool manual_lease_active = false;
  int64_t manual_lease_remaining_ms = 0;
  std::string manual_profile;
  // §79: the newest events, oldest-first, for the operator's feed and the diagnostics
  // panel. Fixed-size POD, because the control thread fills this snapshot at 200 Hz and
  // an event feed must not become a reason to allocate on that thread. Detail is
  // truncated rather than grown: a diagnostic that overruns its neighbour is worse than
  // a short one. The durable, uncapped history is the ring behind it and the log file.
  static constexpr int kEventTail = 8;
  struct EventTail {
    uint64_t t_ns = 0;
    char name[32] = {};
    char detail[88] = {};
  };
  std::array<EventTail, kEventTail> event_tail{};
  int event_tail_count = 0;
  uint64_t event_generation = 0;  // total events ever pushed, not the window size
  // §78 AUTO_ROAM: where the sweep is going next and which way it is running. Both are
  // published because during a sweep they are the only way an operator can tell the
  // turret is still executing the plan rather than stuck at an end — and "which way" is
  // what a person needs in order to step out of its path with confidence.
  double roam_target_yaw_rad = 0.0;
  int roam_sweep_direction = 0;
  std::string confidence_band;
  // §50's remaining fields. Each is the answer to a question the operator asks out loud,
  // and each is computed from what is measured rather than from what was commanded —
  // commanded values are how a dashboard keeps saying "72% through the sweep" while the
  // turret is stuck against something.
  //
  // How long ago the selected person was last seen, on controld's clock. -1 when no
  // selection has ever resolved. Distinct from the candidate list's age: the list can be
  // fresh while *this* person is not in it, which is precisely the occlusion case.
  int64_t selection_last_seen_age_ms = -1;
  // AUTO_TRACK: how stale the estimate the controller is steering from is. Equal to the
  // measurement age while a track is being followed; during coast it keeps growing,
  // which is the number that explains why the turret slowed before it stopped (§20).
  int64_t prediction_age_ms = -1;
  // AUTO_ROAM: which sweep is running, and how much of the current leg is behind it.
  // "NONE" when the planner is not sweeping, rather than an empty string, so a reader can
  // tell "not roaming" from "roaming with an unnamed pattern".
  std::string roam_pattern;
  double roam_progress = 0.0;
  float selected_confidence = 0.0f;  // since the last measurement (-1 = none)
  // Installation orientation (§29/§30, Phase 7): base tilt relative to level,
  // from the active R_W_B. installation_source is the PoseSource enum value
  // (kept as an int to keep this header decoupled from the calibration module).
  double base_roll_rad = 0.0;
  double base_pitch_rad = 0.0;
  double base_yaw_rad = 0.0;
  bool installation_calibrated = false;  // true iff a real (non-identity) pose
  int8_t installation_source = 0;        // PoseSource::Unknown default
  // CAN link health (§55 CAN family, §54.4 error states). Counted by the
  // transport from the first commit, published here so the dashboard and the
  // acceptance report can see them: a link degrading into error-passive, or a
  // TX path failing every send, is otherwise only noticed when feedback goes
  // stale — which is also what the supervisor reacts to, so the operator loses
  // the chance to see the cause. can_available=false means the backend has no
  // CAN link at all (a simulated run): never read those zeros as a healthy bus.
  bool can_available = false;
  std::string can_kind;                 // "socketcan" | "yousee"
  std::string can_device;               // "can0" | "/dev/ttyUSB0"
  bool can_up = false;
  int8_t can_state = -1;                // CanIfState (-1 unknown, 0 active,
                                        // 1 warning, 2 passive, 3 bus-off)
  uint64_t can_rx_frames = 0;
  uint64_t can_rx_error_frames = 0;
  uint64_t can_tx_frames = 0;
  uint64_t can_tx_failed = 0;
  int64_t can_last_rx_age_ms = -1;      // -1 = nothing received yet
  // Safety / timing.
  SafetyAction safety_action = SafetyAction::Allow;
  int64_t feedback_age_ms = 0;
  int64_t control_cycle_us = 0;
  // Phase 9: payload profiling status (§42.1, §31.3).
  std::string payload_profile_name;    // active profile ("" = none)
  // --- v3 §50/§52: the mode, and what the last command actually did --------
  // Names, not enums: these cross a process boundary into a browser, and a
  // renumbered enum on one side is a silent wrong label on the other.
  std::string operating_mode;       // MANUAL | AUTO_TRACK | AUTO_ROAM
  std::string supervisory_state;    // READY | HOMING | PARKING | FAULT | ... (§2)
  std::string mode_phase;           // the mode's substate, e.g. WAIT_TARGET
  std::string intent_source;        // who is asking for motion (§26)
  std::string intent_type;          // and what it asked for (§25)
  // §92's three positions, published so they can be told apart. `q_ref_*` is the reference
  // — the position the drive is being told to reach, with LimitSpd shaping the move (v3 has
  // no host-side interpolated reference: the drive's own position loop does the ramping, and
  // no amount of telemetry will make controld publish a curve it never computed). These are
  // what the *mode* asked for, before the envelope clamped it and before the reference
  // manager resolved a line of sight into joints. Where the two differ, something between
  // the wish and the command intervened — which is the one diagnostic an operator cannot
  // infer from the axis positions alone.
  //
  // `intent_has_joint_target` is not decoration. A mode may ask for a line of sight, or for
  // nothing at all, and a pose of 0.0 is a legitimate position on this station: without the
  // flag, "no pose was requested" and "the requested pose is zero" are the same number, and
  // the page would show the turret being told to go to zero.
  bool intent_has_joint_target = false;
  double intent_q_pitch_rad = 0.0;
  double intent_q_yaw_rad = 0.0;
  // §73's aim point: where the line the turret is pointing along lands in the picture,
  // normalised the same way the track anchors are. Only controld can compute it — the
  // estimate is a base-frame line of sight, the picture is camera pixels, and between them
  // sit the gimbal angles, the §10.3 camera extrinsic and the §28.2 intrinsics. The four
  // conditions that must hold are in the fill in `control_loop.cpp`; when any of them fails
  // the flag is false and the two numbers are not measurements. The last aim point of a
  // session that ended is not where the turret is pointing now.
  // §50 `target_selection.track_uuid`, live. Until now the selected target's identity existed
  // only inside a capture taken after something went wrong: the running page could say "person
  // #12" — which is a label, and re-numbered — but not which track the machine had actually
  // committed to. Empty text with `selected_uuid_valid` false means nobody is selected; it must
  // never be drawn as a zero-filled identifier (§72 again).
  bool selected_uuid_valid = false;
  char selected_uuid_text[kUuidTextLen] = {};
  bool aim_point_valid = false;
  double aim_point_x = 0.0;
  double aim_point_y = 0.0;
  // §50/§78: where the end of travel *is*, not merely how far away it is. The distance to a
  // soft limit — which the daemon had been computing for ages and never sending — is a
  // half-truth: "2.6 degrees to the limit" does not say whether the axis is near the front of
  // its travel or the back, and that is exactly what decides whether to keep jogging.
  // `soft_limits_valid` is false until homing has measured the range; the zeros beside it are
  // the bounds every axis appears to share before anything has been measured, so they must
  // never be drawn as if they were a measured limit (§72's rule again).
  bool soft_limits_valid = false;
  double q_soft_min_pitch_rad = 0.0;
  double q_soft_max_pitch_rad = 0.0;
  double q_soft_min_yaw_rad = 0.0;
  double q_soft_max_yaw_rad = 0.0;
  // Distance to the nearer soft limit on each axis, in radians — the same expression the
  // black-box capture records, so the number on the page and the number in an investigation
  // artifact cannot disagree about how close the turret came to the end of its travel.
  double soft_limit_distance_pitch_rad = 0.0;
  double soft_limit_distance_yaw_rad = 0.0;
  std::string intent_reason;        // why — the operator's question, answered
  double intent_velocity_scale = 1.0;
  // §52 CommandAck: every command answers, and "accepted" has to be earned.
  // cmd_ack_accepted = -1 means "no command since the daemon started", which is
  // different from a command that was refused — the dashboard must not show a
  // refusal that never happened, nor a success that never happened.
  std::string cmd_ack_command;
  int8_t cmd_ack_accepted = -1;
  std::string cmd_ack_reason;
  std::string cmd_ack_controller_state;
  std::string cmd_ack_safety_state;
  uint64_t cmd_ack_seq = 0;

  std::string payload_profile_status;  // "no_profile"|"ok"|"mismatch"|"error"
  bool payload_derated = false;        // motion limits derated after mismatch
  bool payload_check_active = false;   // verification motion in progress
};

// §43.1 high-rate control log record (one per cycle, both axes).
struct ControlLogRecord {
  TimeNs timestamp_ns = 0;
  double q_actual[kAxisCount] = {0.0, 0.0};
  double v_actual[kAxisCount] = {0.0, 0.0};
  double a_actual[kAxisCount] = {0.0, 0.0};    // position-derived accel (rad/s^2)
  double jerk_actual[kAxisCount] = {0.0, 0.0}; // position-derived jerk (rad/s^3)
  double effort[kAxisCount] = {0.0, 0.0};
  double q_ref[kAxisCount] = {0.0, 0.0};
  double soft_limit_distance[kAxisCount] = {0.0, 0.0};
  SafetyAction safety_action = SafetyAction::Allow;
  int64_t feedback_age_ms = 0;
  int64_t cycle_duration_us = 0;
  tracking::TrackState track_state = tracking::TrackState::ReadyHold;
};

// §43.3 structured event.
enum class Event : uint8_t {
  TargetAcquired,
  TargetLost,
  CoastingStarted,
  BrakingStarted,
  SearchStarted,
  SearchStopped,
  HoldEntered,
  SafetyBrake,
  LimitProximity,
  CanFault,
  MotorFault,
  LoopOverrun,
  HomingStarted,
  HomingComplete,
  CalibrationCommit,
  Shutdown,
  PayloadVerifyStarted,
  PayloadVerifyOk,
  PayloadVerifyMismatch,
  PayloadVerifyError,
  // §79: the v3 three-mode vocabulary. Appended rather than interleaved — a log line
  // written by an older build and read back by a newer one must still mean what it said.
  ModeChanged,
  // TARGET_ACQUIRED and TARGET_LOST already existed from §43.3 and are reused rather
  // than duplicated; TARGET_TRACKING is the one §79 kind with no v1 counterpart in the
  // tracking states, so it is the one added.
  TargetTracking,
  TargetSelected,
  TargetCleared,
  TargetOccluded,
  TargetReacquired,
  TargetReacquireAmbiguous,
  TargetUnreachable,
  RoamStarted,
  RoamBoundary,
  RoamDirectionReversed,
  RoamStopped,
  ManualJogStarted,
  ManualJogExpired,
  ManualJogStopped,
  ManualStep,
  StopMotion,
};

// The name is the wire format. Numbers are for storage; a log, a dashboard and a person
// reading a fault all need the same word, and the word must not be whatever the dashboard
// author guessed. Every value is covered: an event that renders as "UNKNOWN" on the
// operator's screen is an event nobody can act on.
inline const char* event_name(Event e) {
  switch (e) {
    case Event::TargetAcquired: return "TARGET_ACQUIRED";
    case Event::TargetLost: return "TARGET_LOST";
    case Event::CoastingStarted: return "COASTING_STARTED";
    case Event::BrakingStarted: return "BRAKING_STARTED";
    case Event::SearchStarted: return "SEARCH_STARTED";
    case Event::SearchStopped: return "SEARCH_STOPPED";
    case Event::HoldEntered: return "HOLD_ENTERED";
    case Event::SafetyBrake: return "SAFETY_BRAKE";
    case Event::LimitProximity: return "LIMIT_PROXIMITY";
    case Event::CanFault: return "CAN_FAULT";
    case Event::MotorFault: return "MOTOR_FAULT";
    case Event::LoopOverrun: return "LOOP_OVERRUN";
    case Event::HomingStarted: return "HOMING_STARTED";
    case Event::HomingComplete: return "HOMING_COMPLETE";
    case Event::CalibrationCommit: return "CALIBRATION_COMMIT";
    case Event::Shutdown: return "SHUTDOWN";
    case Event::PayloadVerifyStarted: return "PAYLOAD_VERIFY_STARTED";
    case Event::PayloadVerifyOk: return "PAYLOAD_VERIFY_OK";
    case Event::PayloadVerifyMismatch: return "PAYLOAD_VERIFY_MISMATCH";
    case Event::PayloadVerifyError: return "PAYLOAD_VERIFY_ERROR";
    case Event::ModeChanged: return "MODE_CHANGED";
    case Event::TargetTracking: return "TARGET_TRACKING";
    case Event::TargetSelected: return "TARGET_SELECTED";
    case Event::TargetCleared: return "TARGET_CLEARED";
    case Event::TargetOccluded: return "TARGET_OCCLUDED";
    case Event::TargetReacquired: return "TARGET_REACQUIRED";
    case Event::TargetReacquireAmbiguous: return "TARGET_REACQUIRE_AMBIGUOUS";
    case Event::TargetUnreachable: return "TARGET_UNREACHABLE";
    case Event::RoamStarted: return "ROAM_STARTED";
    case Event::RoamBoundary: return "ROAM_BOUNDARY";
    case Event::RoamDirectionReversed: return "ROAM_DIRECTION_REVERSED";
    case Event::RoamStopped: return "ROAM_STOPPED";
    case Event::ManualJogStarted: return "MANUAL_JOG_STARTED";
    case Event::ManualJogExpired: return "MANUAL_JOG_EXPIRED";
    case Event::ManualJogStopped: return "MANUAL_JOG_STOPPED";
    case Event::ManualStep: return "MANUAL_STEP";
    case Event::StopMotion: return "STOP_MOTION";
  }
  return "UNKNOWN";
}

struct EventRecord {
  TimeNs timestamp_ns = 0;
  Event event = Event::TargetAcquired;
  std::string detail;
};

// Fixed-capacity ring buffer (allocation-bounded, §46 "no unbounded
// allocation").
template <typename T, std::size_t N>
class RingBuffer {
 public:
  void push(const T& item) {
    buf_[write_ % N] = item;
    ++write_;
    if (count_ < N) ++count_;
  }
  // Oldest -> newest.
  std::vector<T> all() const {
    std::vector<T> out;
    out.reserve(count_);
    if (count_ < N) {
      for (std::size_t i = 0; i < count_; ++i) out.push_back(buf_[i]);
    } else {
      const std::size_t start = write_ % N;
      for (std::size_t i = 0; i < N; ++i)
        out.push_back(buf_[(start + i) % N]);
    }
    return out;
  }
  const T& newest() const { return buf_[(write_ - 1) % N]; }
  // The newest `n` records, oldest-first, copied into caller storage. all() returns a
  // vector, which is right for a web thread and wrong for the control thread that
  // publishes an event tail: allocating on someone's 5 ms deadline because a *reader*
  // wanted a window is not a trade anyone agreed to.
  int latest(T out[], int n) const {
    const std::size_t have = count_ < static_cast<std::size_t>(n) ? count_
                                             : static_cast<std::size_t>(n);
    for (std::size_t k = 0; k < have; ++k) out[k] = buf_[(write_ - have + k) % N];
    return static_cast<int>(have);
  }
  std::size_t size() const { return count_; }
  bool empty() const { return count_ == 0; }
  void clear() { write_ = 0; count_ = 0; }

 private:
  std::array<T, N> buf_{};
  std::size_t write_ = 0;
  std::size_t count_ = 0;
};

// The telemetry store, filled by the control loop.
class Telemetry {
 public:
  static constexpr std::size_t kControlLogCap = 4096;   // ~20 s at 200 Hz
  static constexpr std::size_t kEventCap = 512;
  static constexpr std::size_t kBlackBoxCap = 8192;     // ~40 s at 200 Hz

  // §6.3 the current-cycle snapshot (overwritten each cycle). The web/log
  // processes read this from a non-real-time thread, so the snapshot is
  // guarded by a lightweight mutex (uncontended in the hot path, ~tens of ns).
  void set_snapshot(const TelemetrySnapshot& s) {
    std::lock_guard<std::mutex> lk(snapshot_mu_);
    snapshot_ = s;
  }
  TelemetrySnapshot snapshot() const {
    std::lock_guard<std::mutex> lk(snapshot_mu_);
    return snapshot_;
  }

  // §43.1 high-rate control log.
  void push_control(const ControlLogRecord& r) { control_log_.push(r); }
  const RingBuffer<ControlLogRecord, kControlLogCap>& control_log() const {
    return control_log_;
  }

  // §43.3 event log.
  void push_event(TimeNs t, Event e, std::string detail = "") {
    event_log_.push(EventRecord{t, e, std::move(detail)});
    // size() stops counting at the capacity, so anything that watches the ring for
    // *changes* needs a number that does not saturate. Without it the published event
    // tail would freeze at the 512th event of a long session, which is exactly the sort
    // of thing that only shows up on the operator's third hour of running.
    ++event_pushes_;
  }
  uint64_t event_push_count() const { return event_pushes_; }
  const RingBuffer<EventRecord, kEventCap>& event_log() const {
    return event_log_;
  }

  // §43.4 black-box ring (rolling, for fault diagnosis).
  void push_blackbox(const ControlLogRecord& r) { blackbox_.push(r); }
  const RingBuffer<ControlLogRecord, kBlackBoxCap>& blackbox() const {
    return blackbox_;
  }

  // Clear everything (e.g. on reset).
  void clear() {
    control_log_.clear();
    event_log_.clear();
    blackbox_.clear();
    snapshot_ = TelemetrySnapshot{};
  }

 private:
  mutable std::mutex snapshot_mu_;
  TelemetrySnapshot snapshot_;
  RingBuffer<ControlLogRecord, kControlLogCap> control_log_;
  RingBuffer<EventRecord, kEventCap> event_log_;
  uint64_t event_pushes_ = 0;  // does not saturate where size() does
  RingBuffer<ControlLogRecord, kBlackBoxCap> blackbox_;
};

}  // namespace telemetry
}  // namespace ota
