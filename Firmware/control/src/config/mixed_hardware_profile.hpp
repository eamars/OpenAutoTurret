#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

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
  std::optional<uint64_t> expected_unique_id;
  std::optional<uint32_t> feedback_frame_id;
  std::optional<uint32_t> command_frame_id;
  std::optional<double> current_limit_a;
  // Gate on the unitless feedback temperature byte, 0 = no gate. The official
  // guide gives byte 6 no scale, so a nonzero ceiling is an owner's operating
  // decision (enclosure, ambient, duty), never a manufacturer limit.
  int yaw_guard_temp_raw_ceiling = 0;
};

struct Profile {
  int schema_version = 0;
  CanBus yaw_bus;
  CanBus pitch_bus;
  Axis yaw;
  Axis pitch;
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
