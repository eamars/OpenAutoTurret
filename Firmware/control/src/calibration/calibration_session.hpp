#pragma once
// ADR-005 D7: one commissioning session, run by controld. These are commissiond's executors without
// their own I/O. The backend's servo thread for the axis under test steps the session; the other axis
// keeps production's hold, so runs do not vary (owner, 2026-10-24).
//
// A session is built from the manifest commission.py writes for commissiond, and it writes the
// journal commissiond writes. So identify.py, design.py and score.py read the result unchanged
// (tools/servo_commission/journal.py).
//
// Threads:
// - the web thread makes a session (parses the manifest, opens the journal);
// - the backend's servo thread steps it while Running;
// - the control thread watches state() and, once the session is Holding, re-anchors production's
//   hold and releases the axis;
// - the calibration writer thread writes the footer (finish()).
#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "capture/journal.hpp"
#include "common/types.hpp"
#include "position_loop.hpp"
#include "servo.hpp"
#include "session_parts.hpp"

namespace ota::calibration {

enum class SessionState : int {
  Running,   // following the script
  Holding,   // done or stopped: holding where it ended, until the control thread takes the axis back
};

class Session {
 public:
  virtual ~Session() = default;
  Session(const Session&) = delete;
  Session& operator=(const Session&) = delete;

  AxisId axis() const { return axis_; }
  const std::string& label() const { return label_; }
  const std::string& journal_path() const { return journal_path_; }
  SessionState state() const { return state_.load(std::memory_order_acquire); }
  // Valid once state() is Holding: whether the script ran to its end, and why not.
  bool complete() const { return complete_; }
  const std::string& detail() const { return detail_; }
  // The script's own length, approach and hold included (the run view's expected time).
  double expected_s() const { return expected_s_.load(std::memory_order_relaxed); }

  // Ends the session at its next step, holding where the axis is (STOP, a fault). Any thread.
  void stop(const char* why) { stop_request_.store(why, std::memory_order_release); }
  // Ends it now, when nothing will step it again (the backend took the axis away: a hazard, a
  // de-energise). The caller holds whatever lock its stepping thread takes.
  void abandon(const char* why) {
    if (state() == SessionState::Running) end(false, std::string("stopped: ") + why);
  }

  // Control thread or writer, once state() is Holding and nothing steps it any more: the idle axis's
  // record, then the footer. Blocks on the disk; never on the control thread.
  bool finish(const std::string& idle_axis_row);

 protected:
  Session(AxisId axis, std::string label, std::string journal_path)
      : axis_(axis), label_(std::move(label)), journal_path_(std::move(journal_path)) {}
  // Servo thread only.
  void record(const char* row, int length);
  void end(bool complete, std::string detail);
  const char* stop_requested() const { return stop_request_.load(std::memory_order_acquire); }

  AxisId axis_;
  std::string label_, journal_path_;
  std::unique_ptr<capture::Journal> journal_;
  std::atomic<SessionState> state_{SessionState::Running};
  std::atomic<const char*> stop_request_{nullptr};
  bool complete_ = false;
  std::string detail_;
  std::atomic<double> expected_s_{0};   // the approach is added when the session begins
  uint64_t steps_ = 0;
};

// The pitch working window: production's soft envelope from this power-up's homing (D7.5), not the
// window hand-copied into the manifest.
struct PitchWindow {
  double min_rad = 0, max_rad = 0;
  bool valid() const { return max_rad > min_rad; }
};

// commissiond's pitch servo trial (homing_session.cpp) after production's homing: the host position
// loop around the drive's speed loop, its reference relative to the window centre, reached by a
// smoothstep approach, with the manifest's gain schedule and speed excitation.
class PitchSession final : public Session {
 public:
  // From a commissiond pitch manifest (`adr0022.sensorless-homing/1` with `servo_trial`). The journal
  // is created at `journal_path`. Null, with `err`, when the manifest is not one controld runs.
  static std::shared_ptr<PitchSession> make(const std::string& manifest_json, const std::string& journal_path,
                                            std::string label, std::string& err);

  // Servo thread, every millisecond: the speed reference for the drive (rad/s). `window` is fixed by
  // the first step; `status_ns` and `torque_nm` are the feedback frame's, for the journal.
  double step(TimeNs now, double pose_rad, TimeNs status_ns, double torque_nm, const PitchWindow& window);

 private:
  PitchSession(std::string label, std::string journal_path) : Session(AxisId::Pitch, std::move(label), std::move(journal_path)) {}
  void begin(TimeNs now, double pose, const PitchWindow& window);
  double hold(TimeNs now, double pose);

  axis::PositionLoop loop_;
  axis::ReferenceTable reference_;
  axis::LogSweep excitation_;
  std::vector<std::array<double, 3>> schedule_;   // begin_s, kp, ki
  std::size_t next_gain_ = 0;
  TimeNs period_ns_ = 1'000'000, hold_after_ns_ = 0;
  double following_error_ = 0, approach_speed_ = 0, margin_ = 0;
  bool begun_ = false;
  double min_ = 0, max_ = 0, centre_ = 0, slack_ = 0, origin_ = 0, approach_s_ = 0;
  TimeNs begin_ns_ = 0, next_ns_ = 0, last_ns_ = 0;
  double command_ = 0, hold_q_ = 0;
};

// What one GM6020 feedback frame gives a yaw session.
struct YawFrame {
  TimeNs rx_ns = 0, rx_realtime_ns = 0, dequeue_ns = 0;   // the kernel's receipt, and the dequeue
  uint16_t angle_count = 0;
  int16_t speed_rpm = 0, current_raw = 0;
  uint8_t temperature_raw = 0;
  double current_a = 0;
};

// commissiond's yaw servo session (yaw_control_session.cpp): its own axis::Servo with the manifest's
// parameters, stepped on every GM6020 frame against the manifest's reference, relative to where
// control began, with its gain schedule, excitation and trips. It differs in what holds: commissiond
// left pitch unpowered and ended with yaw at zero current; here pitch is held by production, and
// yaw is handed back to production's servo as soon as the session ends (zero current until then,
// one control cycle at most).
class YawSession final : public Session {
 public:
  static std::shared_ptr<YawSession> make(const std::string& manifest_json, const std::string& journal_path,
                                          std::string label, std::string& err);
  // The current production was applying when the session took the axis: the servo starts there.
  void set_entry_current(double amps) { entry_current_ = amps; }
  double current_bound() const { return current_bound_; }

  // Yaw receive thread, every frame: the current to send (A), or NaN to send nothing this frame.
  double control(TimeNs now, const YawFrame& frame);
  // After the send: what went out (the journal's yaw_current_tx, the servo's acknowledgement).
  void sent(bool ok, double actual_a, int raw, TimeNs begin_ns, TimeNs accepted_ns);

 private:
  YawSession(std::string label, std::string journal_path) : Session(AxisId::Yaw, std::move(label), std::move(journal_path)) {}
  double position() const;
  void stop_here(const std::string& detail);

  axis::ServoParameters parameters_{};
  axis::Servo servo_;
  axis::ReferenceTable reference_;
  axis::LogSweep excitation_;
  axis::OscillationMonitor oscillation_;
  std::vector<std::array<double, 4>> schedule_;   // begin_s, kq, kv, ki
  std::size_t next_gain_ = 0;
  double current_bound_ = 0, speed_limit_ = 0, hold_after_s_ = 0, temperature_limit_ = 0;
  double entry_current_ = 0;
  // The encoder, unwrapped from the first frame (the servo's coordinates: the absolute GM6020 angle).
  bool have_encoder_ = false;
  uint16_t previous_count_ = 0;
  int64_t counts_ = 0;
  double offset_rad_ = 0, initial_position_ = 0;
  TimeNs epoch_ns_ = 0, begin_ns_ = 0, last_step_ns_ = 0, last_rock_ns_ = 0;
  std::array<std::pair<TimeNs, double>, 64> speed_ring_{};   // (time, position) for the 20 ms speed check
  std::size_t speed_head_ = 0, speed_count_ = 0;
  // The step whose current is being sent.
  double requested_ = 0, limited_ = 0;
  const char* phase_ = "control";
  bool pending_ = false;
};

}  // namespace ota::calibration
