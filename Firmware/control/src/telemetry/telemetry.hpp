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
struct TrackListing {
  uint64_t uuid_lo = 0;
  uint16_t display_index = 0;
  char label[24] = {};        // "Person #2" (§10) — what the operator reads
  char class_name[16] = {};   // as the detector named it
  char state[16] = {};        // CONFIRMED | TENTATIVE | OCCLUDED | LOST
  float confidence = 0.0f;
  float anchor_x = 0.0f;      // normalised, for the overlay's box
  float anchor_y = 0.0f;
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
  uint64_t selected_uuid = 0;
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
