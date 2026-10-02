#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>
#include "can/gm6020_friction.hpp"

namespace ota::config::mixed {

enum class Protocol { Gm6020, CyberGear };
enum class Topology { Continuous, Bounded };
// Current is the GM6020 torque-current mode: the drive closes its own current loop and the host
// commands amperes. It is only valid when the operator has recorded that the firmware and the
// Current Ring setting were verified -- the fields below fail closed until then.
enum class ControlMode { Voltage, Current, Position, Speed };

struct CanBus {
  std::string interface;
  std::string spi_parent;
  uint32_t bitrate = 0;
};

struct Axis {
  Protocol protocol = Protocol::CyberGear;
  std::string bus_name;
  uint8_t motor_id = 0;
  Topology topology = Topology::Bounded;
  ControlMode control_mode = ControlMode::Position;
  bool current_ring_verified = false;   // external precondition: firmware >= v1.0.11.2 and Current Ring enabled
  double host_current_limit_a = 0.0;      // host-side command clamp, amperes; 0 = unset, which current mode rejects
  // The yaw velocity loop's OUTPUT gains while in current mode, in amperes. They are separate
  // fields from anything voltage-shaped on purpose: a voltage ceiling in counts says nothing about
  // amperes, and reusing the voltage number silently re-tunes the axis. 0 = unset, refused.
  double current_kp_a_per_rad_s = 0.0;
  double current_ki_a_per_rad_s = 0.0; // legacy key spelling; physical unit A/rad
  int velocity_rx_window_ms = 0; // 0: legacy 50 ms filter; 20/30/40: fresh-RX window
  gm6020::FrictionConfig friction;
  std::optional<uint64_t> expected_unique_id;
  std::optional<uint32_t> feedback_frame_id;
  std::optional<uint32_t> command_frame_id;
  std::optional<double> current_limit_a;
  // Gate on the unitless feedback temperature byte, 0 = no gate. The official
  // guide gives byte 6 no scale, so a nonzero ceiling is an owner's operating
  // decision (enclosure, ambient, duty), never a manufacturer limit.
  int yaw_guard_temp_raw_ceiling = 0;
};

// ADR-003 3b: the ADR-002.2 servos (assets from tools/servo_commission/commission.py) own both
// axes whenever the control loop publishes a reference segment. Absent: the legacy speed paths.
struct Servo {
  std::string yaw_asset, pitch_asset;   // resolved paths of config/servo/*_servo.json
  double yaw_current_limit_a = 0;       // peak authority (owner ruling 2026-10-02: 3 A)
  double yaw_rms_limit_a = 0;           // continuous budget (1.62 A rated)
  int yaw_temperature_limit_raw = 0;    // the commissioning sessions' thermal trip (raw byte, ~deg C)
  double oscillation_limit_a = 0;       // limit-cycle guard: fast current RMS (commissioning rule)
  double speed_limit_rad_s = 0;         // owner ruling 2026-10-02: 100 RPM is the safety cap (both axes)
  // Pitch end-stop protection (owner: never drive the pitch into its mechanical end stop): the
  // guard lies this far beyond each soft limit (which homing places inside the measured ends),
  // and toward it the commanded speed always allows a stop at stop_acceleration.
  double pitch_guard_rad = 0;
  double pitch_stop_acceleration_rad_s2 = 0;
};

struct Profile {
  int schema_version = 0;
  CanBus yaw_bus;
  CanBus pitch_bus;
  Axis yaw;
  Axis pitch;
  std::optional<Servo> servo;
};

struct LoadResult {
  bool ok = false;
  Profile profile;
  std::vector<std::string> errors;
};

// Loads the dedicated production mixed-drive schema. This deliberately does
// not migrate or fall back to the legacy single-bus turret.yaml format.
LoadResult load_mixed_hardware_profile(const std::string& path);

}  // namespace ota::config::mixed
