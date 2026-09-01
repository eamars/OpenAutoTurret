// OpenAutoTurret — the in-loop payload response check (§27, §31.3, §42.2).
//
// A NON-BLOCKING stepper (mirrors HomingPlan / ParkController: it is a pure
// reference computer that consumes feedback and returns a DesiredState; the
// ControlLoop does the actual commanding so the SafetySupervisor keeps
// authority). It runs the small verification motion §31.3 describes on ONE
// axis: a small + step and a small - step, in a safe central region, at
// conservative limits, capturing the per-step response. On completion it has
// compared both steps against the stored payload profile baseline and exposes
// the per-axis AxisVerifyResult.
//
// Transport-agnostic: it only consumes AxisSnapshot, so it is unit-testable
// against the SimMotorBackend with no CAN.
#pragma once

#include <string>
#include <vector>

#include "common/types.hpp"
#include "control/motor_backend.hpp"
#include "payload/payload_profile.hpp"
#include "payload/payload_verifier.hpp"
#include "payload/response_metrics.hpp"

namespace ota {
namespace payload {

// The small verification moves, in order.
enum class CheckMove : uint8_t { PosStep, PosReturn, NegStep, NegReturn, Done };

struct PayloadCheckConfig {
  // Conservative verification limits (§31.3 "conservative limits").
  double step_amplitude_rad = 2.0 * kDeg2Rad;  // small move
  double speed_rad_s = 10.0 * kDeg2Rad;        // conservative speed
  double settle_band_frac = 0.02;              // ±2 % of amplitude
  double at_rest_vel_rad_s = 0.05;
  double settle_time_s = 0.3;                  // dwell to confirm settled
  double max_move_s = 8.0;                     // hard bound per move (timeout)
  // The safe central region the check is clamped to (§44 "safe central
  // region"). The start pose must lie inside it.
  double region_center_rad = 0.0;
  double region_half_span_rad = 20.0 * kDeg2Rad;
  VerifyTolerances tol;
};

class PayloadCheck {
 public:
  // `profile` may be null (no stored profile -> status NoProfile).
  PayloadCheck() = default;
  PayloadCheck(const PayloadCheckConfig& cfg, const PayloadProfile* profile,
               AxisId axis)
      : cfg_(cfg), profile_(profile), axis_(axis) {}

  // Begin the check from `q_start` (the current axis position, which must lie
  // inside the safe central region). Returns false (fail_reason set) if the
  // start pose is outside the region or feedback was missing at start.
  bool begin(TimeNs now_ns, double q_start_rad, bool has_feedback);

  // One control cycle: consume the current snapshot, record a sample, advance
  // the move sequence if the current move has settled, and return the
  // reference + speed limit the loop should command THIS cycle.
  struct StepOut {
    double q_ref_rad = 0.0;
    double limit_spd_rad_s = 0.0;
    bool hold = false;  // the whole check is finished (at rest at start)
  };
  StepOut step(TimeNs now_ns, const AxisSnapshot& snap);

  bool active() const { return active_; }
  bool complete() const { return !active_; }
  bool failed() const { return failed_; }
  const std::string& fail_reason() const { return fail_reason_; }
  const AxisVerifyResult& axis_result() const { return axis_result_; }
  double amplitude_rad() const { return amp_; }

 private:
  double clamp_to_region(double q) const;
  void fail(const std::string& reason) {
    if (!failed_) {
      failed_ = true;
      active_ = false;
      fail_reason_ = reason;
    }
  }
  void advance_move(TimeNs now_ns);
  bool move_settled(const AxisSnapshot& snap) const;

  PayloadCheckConfig cfg_;
  const PayloadProfile* profile_ = nullptr;
  AxisId axis_ = AxisId::Pitch;

  bool active_ = false;
  bool failed_ = false;
  std::string fail_reason_;

  CheckMove move_ = CheckMove::Done;
  double q_start_ = 0.0;
  double amp_ = 0.0;      // clamped step amplitude (rad)
  double q_target_ = 0.0;
  TimeNs move_start_ns_ = 0;
  TimeNs settled_since_ns_ = -1;

  std::vector<ResponseSample> samples_;
  StepMetrics step_pos_;
  StepMetrics step_neg_;
  AxisVerifyResult axis_result_;
};

}  // namespace payload
}  // namespace ota
