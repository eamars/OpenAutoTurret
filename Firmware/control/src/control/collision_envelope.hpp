// OpenAutoTurret — coupled collision envelope (architecture §19).
//
// The physical payload may restrict yaw differently at different pitch angles,
// so the safe region is a joint-space region, not just independent per-axis
// limits:
//
//   q_yaw_min = f(q_pitch)
//   q_yaw_max = g(q_pitch)
//
// or a more general 2D polygon / piecewise envelope. The controller and the
// safety checker depend only on the CollisionEnvelope interface, so the v1
// constant (rectangular) envelope can be replaced later by a piecewise table or
// a joint-space polygon WITHOUT rewriting the controller (§19).
//
// The safety checker must validate the PATH (the whole planned sequence of
// poses), not only the endpoint (§19): a trajectory can leave the safe region
// between two safe endpoints.
#pragma once

#include <vector>

namespace ota {

// A single (pitch, yaw) joint pose, in raw radians (feedback-angle space).
struct Pose2D {
  double pitch_rad = 0.0;
  double yaw_rad = 0.0;
};

// The coupled safe-region interface. Implementations answer "is this pose in
// the safe region?". The controller and the trajectory safety checker depend
// only on this abstract type, so the concrete envelope (constant now, piecewise
// table / polygon later) can be swapped without touching the control code.
class CollisionEnvelope {
 public:
  virtual ~CollisionEnvelope() = default;

  // Is a single (pitch, yaw) pose inside the safe region?
  virtual bool is_safe(double pitch_rad, double yaw_rad) const = 0;

  // Is the whole path (the planned sequence of poses) safe? Every point must be
  // inside the safe region — the endpoint alone is not enough (§19). An empty
  // path is trivially safe.
  bool is_path_safe(const std::vector<Pose2D>& path) const {
    for (const auto& p : path)
      if (!is_safe(p.pitch_rad, p.yaw_rad)) return false;
    return true;
  }
};

// Configuration for the constant (rectangular) envelope: independent pitch and
// yaw ranges. This is the v1 representation ("constant yaw and pitch limits can
// be configured", §19). Ranges are raw radians and inclusive.
struct RectangularEnvelopeConfig {
  double pitch_min_rad = -1.0;
  double pitch_max_rad = 1.0;
  double yaw_min_rad = -1.0;
  double yaw_max_rad = 1.0;
};

// The v1 collision envelope: an axis-aligned rectangle in the (pitch, yaw)
// joint space. Constant limits; the CollisionEnvelope interface admits a
// piecewise table or a polygon later without touching the controller.
class RectangularCollisionEnvelope final : public CollisionEnvelope {
 public:
  explicit RectangularCollisionEnvelope(RectangularEnvelopeConfig cfg) : cfg_(cfg) {}

  bool is_safe(double pitch_rad, double yaw_rad) const override {
    return pitch_rad >= cfg_.pitch_min_rad && pitch_rad <= cfg_.pitch_max_rad &&
           yaw_rad >= cfg_.yaw_min_rad && yaw_rad <= cfg_.yaw_max_rad;
  }

  const RectangularEnvelopeConfig& config() const { return cfg_; }

 private:
  RectangularEnvelopeConfig cfg_;
};

}  // namespace ota
