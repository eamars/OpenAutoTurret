// OpenAutoTurret — full-axis homing sequence (architecture §23).
//
// Learns BOTH mechanical limits of one axis by precision-homing each endpoint
// in turn, then validates the measured travel against the commissioning bounds:
//
//   precision-home endpoint A
//       -> (retreat is the back-off inside the endpoint home)
//       -> traverse safely (the endpoint-B approach moves across the axis)
//       -> precision-home endpoint B
//       -> compute measured travel
//
//   expected_travel_min_deg <= measured_travel_deg <= expected_travel_max_deg
//
// If the travel is out of range the axis is HOMING_FAILED with
// position_valid = false (§23): the station must not enter tracking mode.
//
// This is a thin transport-agnostic wrapper around two single-endpoint
// HomingController instances (§22). step() takes the same HomingFeedback and
// returns the same DesiredState, so a move executor drives it exactly as it
// drives a single-endpoint homing. When it completes, the measured raw
// endpoints are used to set up the host logical model (§24) via
// setup_model_from_endpoints.
#pragma once

#include <string>

#include "calibration/homing_controller.hpp"
#include "common/logical_coordinates.hpp"
#include "common/types.hpp"

namespace ota {

// Full-axis homing parameters.
struct FullAxisHomingParams {
  HomingParams homing;            // per-endpoint homing parameters
  int dir_endpoint_a = +1;        // travel direction to reach endpoint A
  int dir_endpoint_b = -1;        // travel direction to reach endpoint B
  // Commissioning bounds on the measured travel (deg) (§23). The measured
  // travel is the span between the two validated endpoints. These must match
  // the current mechanism; out-of-range travel fails the homing.
  double expected_travel_min_deg = 0.0;
  double expected_travel_max_deg = 0.0;
};

// The outcome of a full-axis homing run.
struct FullAxisHomingResult {
  bool complete = false;          // the sequence ran to a terminal state
  bool valid = false;             // both endpoints homed and travel in range
  double endpoint_a_rad = 0.0;    // validated raw position at endpoint A
  double endpoint_b_rad = 0.0;    // validated raw position at endpoint B
  double measured_travel_deg = 0.0;  // span between the endpoints (deg)
  double repeatability_rad = 0.0;    // worst endpoint repeatability (rad)
  std::string fail_reason;        // non-empty if !valid
  AxisLogicalModel model;         // logical model (valid if result.valid)
};

// Sequence phase, exposed for logging/telemetry.
enum class FullAxisPhase { HomeA, HomeB, Complete, Failed };

class FullAxisHoming {
 public:
  FullAxisHoming(AxisId axis, FullAxisHomingParams p);

  // Drive one sequence step. `fb` is the current axis state; returns the
  // DesiredState to execute this cycle.
  DesiredState step(const HomingFeedback& fb);

  FullAxisPhase phase() const { return phase_; }
  const HomingController& home_a() const { return home_a_; }
  const HomingController& home_b() const { return home_b_; }
  const FullAxisHomingResult& result() const { return result_; }
  bool terminal() const {
    return phase_ == FullAxisPhase::Complete || phase_ == FullAxisPhase::Failed;
  }

 private:
  void validate();
  void fail(const std::string& reason);

  AxisId axis_;
  FullAxisHomingParams p_;
  HomingController home_a_;
  HomingController home_b_;

  FullAxisPhase phase_ = FullAxisPhase::HomeA;
  FullAxisHomingResult result_;
};

}  // namespace ota
