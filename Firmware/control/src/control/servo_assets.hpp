#pragma once
// The ADR-002.2 servo assets (config/servo/*_servo.json, or a calibration record's copies, ADR-005)
// parsed into what the backend configures. Parsing is file I/O and YAML, so it happens off the 1 kHz
// threads -- at backend open, or on the web thread when an operator chooses a calibration record --
// and the backend only applies the result under its locks.
#include <array>
#include <string>

#include "position_loop.hpp"
#include "servo.hpp"

namespace ota {

struct ServoAssets {
  std::string yaw_path, pitch_path;
  axis::ServoParameters yaw;
  // Optional: without the identified table the tracking history keeps the raw yaw reading.
  std::array<double, axis::ServoParameters::kCrosstalkBins> yaw_crosstalk{};
  bool yaw_crosstalk_valid = false;
  // speed_limit is the profile's (owner ruling: 100 RPM), filled in by the backend.
  axis::PositionLoopParameters pitch{0, 0, 0, 0};
  double pitch_following_error_rad = 0;
};

// False with `err` naming the file when an asset is missing, does not parse, or does not make a
// servo or a position loop.
bool parse_servo_assets(const std::string& yaw_path, const std::string& pitch_path, ServoAssets& out,
                        std::string& err);

}  // namespace ota
