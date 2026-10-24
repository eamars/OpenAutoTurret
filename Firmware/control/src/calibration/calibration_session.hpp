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
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "capture/journal.hpp"
#include "common/types.hpp"
#include "position_loop.hpp"
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

}  // namespace ota::calibration
