// OpenAutoTurret — executable multi-axis homing plan (architecture §25).
//
// A homing plan is an ordered list of transport-agnostic actions, not a
// hard-coded "home pitch; home yaw". This lets the plan express payload-clearance
// poses between axes ("home twice at different angle") without redesign (§25.1):
//
//   home_endpoint(axis, endpoint, precision)  — coarse = one known stop (a
//       reference enough for the next safe move); fine = full-precision home of
//       that endpoint (coarse + fine + repeatability).
//   move(axis, position_deg)                  — drive a (homed) axis to a logical
//       clearance pose and hold.
//   home_full_range(axis, precision)          — full home: measure/validate both
//       endpoints (always full precision).
//
// The plan is a step-based sequencer: step(fb) advances the ACTIVE action with
// feedback for the active axis and returns a DesiredState. When an action
// reaches a terminal state the result is recorded (per-axis endpoints + logical
// model) and the plan advances. Any failed action fails the whole plan.
//
// Transport-agnostic: it takes HomingFeedback and returns DesiredState, so it is
// unit-testable against a simulated plant with no CAN (§54).
#pragma once

#include <array>
#include <cmath>
#include <optional>
#include <string>
#include <vector>

#include "calibration/contact_detector.hpp"
#include "calibration/full_axis_homing.hpp"
#include "calibration/homing_controller.hpp"
#include "calibration/move_to.hpp"
#include "common/logical_coordinates.hpp"
#include "common/types.hpp"

namespace ota {

enum class HomingActionType { HomeEndpoint, Move, HomeFullRange };
enum class Endpoint { Lower, Upper };
enum class Precision { Coarse, Fine };

// One step of the plan (mirrors HomingPlanActionConfig in the loader).
struct HomingAction {
  HomingActionType type = HomingActionType::HomeFullRange;
  AxisId axis = AxisId::Pitch;
  Endpoint endpoint = Endpoint::Lower;   // HomeEndpoint
  Precision precision = Precision::Fine;  // HomeEndpoint / HomeFullRange
  double position_deg = 0.0;             // Move (logical frame)
};

// Expected LOGICAL position band (deg) for one axis; the span (max-min) is the
// expected travel used to validate the measured endpoints.
struct TravelBand {
  double min_deg = 0.0;
  double max_deg = 0.0;
};

struct HomingPlanConfig {
  HomingParams homing;                                // shared homing params
  std::array<TravelBand, kAxisCount> travel_bands{};  // per-axis expected travel
  double move_speed_rad_s = 10.0 * kDeg2Rad;          // speed for Move actions
  double move_pos_tol_rad = 0.01;                     // "arrived" position tol
  double move_vel_tol_rad_s = 0.1 * kDeg2Rad;         // "arrived" velocity tol
  double move_timeout_s = 30.0;                       // max time for one Move
};

// Coarse endpoint home: drive toward the stop at coarse speed until the contact
// detector triggers, then settle. Yields ONE known stop position (§25.1) — enough
// to reference the axis for the next safe move.
class CoarseEndpointHome {
 public:
  CoarseEndpointHome(AxisId axis, int approach_dir, HomingParams p);
  DesiredState step(const HomingFeedback& fb);
  bool terminal() const { return complete_ || failed_; }
  bool valid() const { return complete_; }
  double contact_rad() const { return contact_rad_; }
  const std::string& fail_reason() const { return fail_reason_; }

 private:
  void fail(const std::string& reason);
  enum class St { Unknown, Approach, Settling, Complete, Failed };
  AxisId axis_;
  int dir_;
  HomingParams p_;
  ContactDetector detector_;
  St st_ = St::Unknown;
  TimeNs start_ns_ = 0;
  double start_pos_rad_ = 0.0;
  double contact_rad_ = 0.0;
  bool complete_ = false;
  bool failed_ = false;
  std::string fail_reason_;
};

// Per-axis homing progress + result.
struct AxisPlanState {
  bool have_low = false;
  bool have_high = false;
  bool finalized = false;  // both endpoints known, model computed
  double raw_low_rad = 0.0;
  double raw_high_rad = 0.0;
  double repeatability_rad = 0.0;
  bool homed = false;  // finalized && span within the expected band
  std::string span_issue;  // non-empty if the measured span is invalid
  AxisLogicalModel model;
};

class HomingPlan {
 public:
  HomingPlan(std::vector<HomingAction> actions, HomingPlanConfig cfg);

  // Advance the active action with feedback for the active axis. Safe to call
  // repeatedly; returns the DesiredState to execute this cycle.
  DesiredState step(const HomingFeedback& fb);

  AxisId active_axis() const;
  bool complete() const { return complete_; }
  bool failed() const { return failed_; }
  const std::string& fail_reason() const { return fail_reason_; }
  size_t action_index() const { return static_cast<size_t>(idx_); }
  size_t action_count() const { return actions_.size(); }

  // Per-axis results (valid once that axis is homed).
  bool axis_homed(AxisId a) const { return states_[ix(a)].homed; }
  const AxisLogicalModel& model(AxisId a) const { return states_[ix(a)].model; }
  double raw_low(AxisId a) const { return states_[ix(a)].raw_low_rad; }
  double raw_high(AxisId a) const { return states_[ix(a)].raw_high_rad; }
  double repeatability(AxisId a) const { return states_[ix(a)].repeatability_rad; }

 private:
  static size_t ix(AxisId a) { return static_cast<size_t>(a); }
  void start_action();
  void advance();
  void fail(const std::string& reason);
  void record_action_result();
  void record_endpoint(AxisId a, Endpoint e, double raw_rad, double rep_rad);
  void finalize_axis(AxisId a);  // compute model + validate span when both known
  std::string describe_action() const;

  std::vector<HomingAction> actions_;
  HomingPlanConfig cfg_;
  int idx_ = 0;
  bool complete_ = false;
  bool failed_ = false;
  std::string fail_reason_;

  std::array<AxisPlanState, kAxisCount> states_{};

  HomingActionType active_type_ = HomingActionType::HomeFullRange;
  Precision active_precision_ = Precision::Fine;
  std::optional<CoarseEndpointHome> coarse_;
  std::optional<HomingController> home_;
  std::optional<FullAxisHoming> full_;
  std::optional<MoveTo> move_;
};

}  // namespace ota
