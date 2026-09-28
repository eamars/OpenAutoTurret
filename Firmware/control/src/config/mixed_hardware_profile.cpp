#include "config/mixed_hardware_profile.hpp"

#include <charconv>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <string_view>
#include <system_error>

#include <yaml-cpp/yaml.h>

namespace ota::config::mixed {
namespace {

void error(LoadResult& result, const std::string& message) {
  result.errors.push_back(message);
}

bool check_keys(const YAML::Node& node, const std::string& path,
                std::initializer_list<std::string_view> allowed,
                LoadResult& result,
                // Keys that are legal but may be absent: presence is still enforced for `allowed`,
                // so a new optional key cannot quietly turn the shipped station file into a file
                // with missing required keys.
                std::initializer_list<std::string_view> optional = {}) {
  if (!node.IsMap()) {
    error(result, path + " must be a mapping");
    return false;
  }
  bool valid = true;
  for (const auto& entry : node) {
    if (!entry.first.IsScalar()) {
      error(result, path + " contains a non-scalar key");
      valid = false;
      continue;
    }
    const auto key = entry.first.as<std::string>();
    bool known = false;
    for (const auto candidate : allowed) known |= key == candidate;
    for (const auto candidate : optional) known |= key == candidate;
    if (!known) {
      error(result, path + " has unknown key '" + key + "'");
      valid = false;
    }
  }
  for (const auto key : allowed) {
    if (!node[std::string(key)]) {
      error(result, path + "." + std::string(key) + " is required");
      valid = false;
    }
  }
  return valid;
}

std::string string_value(const YAML::Node& node, const std::string& path,
                         LoadResult& result) {
  try {
    if (!node.IsScalar()) throw YAML::BadConversion(node.Mark());
    return node.as<std::string>();
  } catch (const YAML::Exception&) {
    error(result, path + " must be a string");
    return {};
  }
}

uint64_t unsigned_value(const YAML::Node& node, const std::string& path,
                        LoadResult& result) {
  try {
    if (!node.IsScalar()) throw YAML::BadConversion(node.Mark());
    const auto value = node.as<int64_t>();
    if (value < 0) throw YAML::BadConversion(node.Mark());
    return static_cast<uint64_t>(value);
  } catch (const YAML::Exception&) {
    error(result, path + " must be a non-negative integer");
    return 0;
  }
}

double double_value(const YAML::Node& node, const std::string& path,
                    LoadResult& result) {
  try {
    if (!node.IsScalar()) throw YAML::BadConversion(node.Mark());
    const double value = node.as<double>();
    if (!std::isfinite(value)) throw YAML::BadConversion(node.Mark());
    return value;
  } catch (const YAML::Exception&) {
    error(result, path + " must be a finite number");
    return 0.0;
  }
}

uint64_t hex_value(const YAML::Node& node, const std::string& path,
                   LoadResult& result) {
  const auto text = string_value(node, path, result);
  uint64_t value = 0;
  const auto parsed = std::from_chars(text.data(), text.data() + text.size(), value, 16);
  if (text.empty() || parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size()) {
    error(result, path + " must be a hexadecimal string without a prefix");
    return 0;
  }
  return value;
}

CanBus read_bus(const YAML::Node& node, const std::string& path,
                LoadResult& result) {
  check_keys(node, path, {"interface", "spi_parent", "bitrate"}, result);
  CanBus bus;
  bus.interface = string_value(node["interface"], path + ".interface", result);
  bus.spi_parent = string_value(node["spi_parent"], path + ".spi_parent", result);
  const auto bitrate = unsigned_value(node["bitrate"], path + ".bitrate", result);
  if (bitrate > std::numeric_limits<uint32_t>::max()) {
    error(result, path + ".bitrate exceeds uint32 range");
  } else {
    bus.bitrate = static_cast<uint32_t>(bitrate);
  }
  return bus;
}

void check_bus(const CanBus& bus, const std::string& path,
               const char* interface, const char* parent, LoadResult& result) {
  if (bus.interface != interface)
    error(result, path + ".interface must be '" + interface + "'");
  if (bus.spi_parent != parent)
    error(result, path + ".spi_parent must be '" + parent + "'");
  if (bus.bitrate != 1000000)
    error(result, path + ".bitrate must be 1000000");
}

void check_axis_string(const YAML::Node& node, const char* expected,
                       const std::string& path, const char* field,
                       LoadResult& result) {
  if (string_value(node, path + "." + field, result) != expected)
    error(result, path + "." + field + " must be '" + expected + "'");
}

}  // namespace

LoadResult load_mixed_hardware_profile(const std::string& path) {
  LoadResult result;
  try {
    const YAML::Node root = YAML::LoadFile(path);
    if (!check_keys(root, "profile", {"schema_version", "buses", "axes"}, result))
      return result;
    const auto version = unsigned_value(root["schema_version"], "schema_version", result);
    if (version != 1) error(result, "schema_version must be 1");
    result.profile.schema_version = version <= INT32_MAX ? static_cast<int>(version) : 0;

    const auto buses = root["buses"];
    check_keys(buses, "buses", {"yaw", "pitch"}, result);
    result.profile.yaw_bus = read_bus(buses["yaw"], "buses.yaw", result);
    result.profile.pitch_bus = read_bus(buses["pitch"], "buses.pitch", result);
    check_bus(result.profile.yaw_bus, "buses.yaw", "can0", "spi0.0", result);
    check_bus(result.profile.pitch_bus, "buses.pitch", "can1", "spi1.0", result);
    if (result.profile.yaw_bus.interface == result.profile.pitch_bus.interface)
      error(result, "yaw and pitch must use separate CAN interfaces");

    const auto axes = root["axes"];
    check_keys(axes, "axes", {"yaw", "pitch"}, result);
    const auto yaw = axes["yaw"];
    check_keys(yaw, "axes.yaw", {"protocol", "bus", "motor_id", "topology",
                                  "control_mode", "feedback_frame_id", "command_frame_id",
                                  "guard_temp_raw_ceiling"}, result, {"current_ring_verified", "host_current_limit_a"});
    check_axis_string(yaw["protocol"], "gm6020", "axes.yaw", "protocol", result);
    check_axis_string(yaw["bus"], "yaw", "axes.yaw", "bus", result);
    check_axis_string(yaw["topology"], "continuous", "axes.yaw", "topology", result);
    auto& yaw_axis = result.profile.yaw;
    yaw_axis.protocol = Protocol::Gm6020;
    yaw_axis.bus_name = "yaw";
    yaw_axis.topology = Topology::Continuous;
    // Voltage is still the only mode that ships. `current` parses so the profile can be staged,
    // but validate_profile refuses it until the operator has recorded the external preconditions.
    // Presence is enforced here by hand rather than via string_value, whose missing-node behaviour
    // I have not verified -- an absent key must not become a silent default.
    const auto yaw_mode = yaw["control_mode"];
    if (!yaw_mode.IsScalar()) {
      error(result, "axes.yaw.control_mode is required ('voltage' or 'current')");
    } else if (yaw_mode.as<std::string>() == "voltage") {
      yaw_axis.control_mode = ControlMode::Voltage;
    } else if (yaw_mode.as<std::string>() == "current") {
      yaw_axis.control_mode = ControlMode::Current;
    } else {
      error(result, "axes.yaw.control_mode must be 'voltage' or 'current'");
    }

    // The two preconditions current mode depends on are configuration facts, not runtime facts:
    // they are the operator's record of the drive. Checked here, where the test surface already
    // exists offline, rather than behind a CAN socket. Absent key => limit stays 0.0 => refused.
    if (yaw_axis.control_mode == ControlMode::Current) {
      // Recorded, not merely checked: a field that stays false while the file says true would make
      // every consumer read a lie. The first red caught by these very tests.
      yaw_axis.current_ring_verified = false;
      yaw_axis.host_current_limit_a = 0.0;
      bool acknowledged = false;
      try {
        const auto ack = yaw["current_ring_verified"];
        acknowledged = ack.IsScalar() && ack.as<bool>();
      } catch (const std::exception&) { acknowledged = false; }
      if (!acknowledged)
        error(result, "axes.yaw: current mode requires current_ring_verified: true "
                      "(firmware >= v1.0.11.2 and Current Ring enabled in "
                      "RoboMaster Assistant v2.7+)");
      double limit = 0.0;
      try {
        const auto lim = yaw["host_current_limit_a"];
        if (lim.IsScalar()) limit = lim.as<double>();
      } catch (const std::exception&) { limit = 0.0; }
      // Negated compare so a non-finite limit fails closed instead of slipping both bounds.
      if (!(limit > 0.0) || limit > 1.62)
        error(result, "axes.yaw: current mode needs host_current_limit_a in (0, 1.62] A "
                      "-- 1.62 A is the motor's maximum continuous rating");
      yaw_axis.current_ring_verified = acknowledged;
      yaw_axis.host_current_limit_a = limit;
    }
    const auto yaw_id = unsigned_value(yaw["motor_id"], "axes.yaw.motor_id", result);
    if (yaw_id != 1) error(result, "axes.yaw.motor_id must be 1");
    if (yaw_id <= UINT8_MAX) yaw_axis.motor_id = static_cast<uint8_t>(yaw_id);
    const auto yaw_feedback = unsigned_value(yaw["feedback_frame_id"], "axes.yaw.feedback_frame_id", result);
    if (yaw_feedback != 0x205) error(result, "axes.yaw.feedback_frame_id must be 0x205");
    yaw_axis.feedback_frame_id = static_cast<uint32_t>(yaw_feedback);
    const auto yaw_command = unsigned_value(yaw["command_frame_id"], "axes.yaw.command_frame_id", result);
    if (yaw_command != 0x1ff) error(result, "axes.yaw.command_frame_id must be 0x1FF");
    yaw_axis.command_frame_id = static_cast<uint32_t>(yaw_command);
    if (yaw["guard_temp_raw_ceiling"]) {
      const int ceiling = yaw["guard_temp_raw_ceiling"].as<int>();
      if (ceiling < 0 || ceiling > 255)
        error(result, "axes.yaw.guard_temp_raw_ceiling must be 0..255 (0 disables the gate)");
      else
        yaw_axis.yaw_guard_temp_raw_ceiling = ceiling;
    }

    const auto pitch = axes["pitch"];
    check_keys(pitch, "axes.pitch", {"protocol", "bus", "motor_id", "topology",
        "control_mode", "expected_unique_id_hex", "current_limit_a"}, result);
    check_axis_string(pitch["protocol"], "cybergear", "axes.pitch", "protocol", result);
    check_axis_string(pitch["bus"], "pitch", "axes.pitch", "bus", result);
    check_axis_string(pitch["topology"], "bounded", "axes.pitch", "topology", result);
    auto& pitch_axis = result.profile.pitch;
    pitch_axis.protocol = Protocol::CyberGear;
    pitch_axis.bus_name = "pitch";
    pitch_axis.topology = Topology::Bounded;
    const auto mode = string_value(pitch["control_mode"], "axes.pitch.control_mode", result);
    if (mode == "position") pitch_axis.control_mode = ControlMode::Position;
    else if (mode == "speed") pitch_axis.control_mode = ControlMode::Speed;
    else error(result, "axes.pitch.control_mode must be 'position' or 'speed'");
    const auto pitch_id = unsigned_value(pitch["motor_id"], "axes.pitch.motor_id", result);
    if (pitch_id != 127) error(result, "axes.pitch.motor_id must be 127");
    if (pitch_id <= UINT8_MAX) pitch_axis.motor_id = static_cast<uint8_t>(pitch_id);
    const auto uid = hex_value(pitch["expected_unique_id_hex"],
                               "axes.pitch.expected_unique_id_hex", result);
    if (uid != 0x7216313130333105ULL)
      error(result, "axes.pitch.expected_unique_id_hex must identify the installed pitch motor");
    pitch_axis.expected_unique_id = uid;
    const double limit = double_value(pitch["current_limit_a"],
                                      "axes.pitch.current_limit_a", result);
    if (!(limit > 0.0 && limit <= 5.0))
      error(result, "axes.pitch.current_limit_a must be in (0, 5] A");
    pitch_axis.current_limit_a = limit;

    result.ok = result.errors.empty();
  } catch (const YAML::Exception& e) {
    error(result, "cannot parse mixed hardware profile: " + std::string(e.what()));
  } catch (const std::exception& e) {
    error(result, "cannot load mixed hardware profile: " + std::string(e.what()));
  }
  return result;
}

}  // namespace ota::config::mixed
