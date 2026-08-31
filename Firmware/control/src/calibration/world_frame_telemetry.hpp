#pragma once
// OpenAutoTurret — world-frame telemetry fill (architecture §29/§30, Phase 7;
// §42.1 Calibration panel).
//
// Given the active base->world orientation and the current base-frame line of
// sight, fill the world-frame portion of the telemetry snapshot: the base tilt
// (roll/pitch/yaw relative to level) and the world-frame target LOS. This is the
// bridge that makes tracking "world-correct" for a tilted base.
//
// Pure data — no CAN, no camera, no motor driver.
#include "calibration/installation_pose.hpp"
#include "telemetry/telemetry.hpp"

namespace ota {

inline void fill_world_frame_telemetry(const BaseOrientation& o,
                                       double az_base_rad, double el_base_rad,
                                       telemetry::TelemetrySnapshot& s) {
  R_W_B_to_euler(o, s.base_roll_rad, s.base_pitch_rad, s.base_yaw_rad);
  s.installation_calibrated = (o.valid && o.source != PoseSource::Identity);
  s.installation_source = static_cast<int8_t>(o.source);
  base_los_to_world(o, az_base_rad, el_base_rad, s.target_az_world_rad,
                    s.target_el_world_rad);
}

}  // namespace ota
