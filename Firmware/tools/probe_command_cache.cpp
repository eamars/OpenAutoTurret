// Exercise production backend -> CyberGearSystem -> failing transport, without motors.
#include <iostream>
#include <memory>
#include <vector>
#include "control/can_motor_backend.hpp"

class TraceTransport : public ota::can::CanTransport {
 public:
  bool fail_next = false;
  std::vector<uint16_t> registers;
  bool start(std::string&) override { return true; }
  void stop() override {}
  void set_frame_callback(FrameCallback) override {}
  bool send(uint32_t, const uint8_t data[8], std::string* error) override {
    registers.push_back(uint16_t(data[0]) | uint16_t(data[1]) << 8);
    if (!fail_next) return true;
    fail_next = false;
    if (error) *error = "injected transport failure";
    return false;
  }
  ota::can::BusStats stats() const override { return {}; }
  bool is_up() const override { return true; }
  ota::can::CanIfState can_state() const override { return ota::can::CanIfState::Unknown; }
  const char* kind() const override { return "offline-trace"; }
  std::string device() const override { return "no-hardware"; }
  int count(ota::cybergear::Reg reg) const {
    int n = 0;
    for (auto value : registers) if (value == static_cast<uint16_t>(reg)) ++n;
    return n;
  }
};

int main() {
  auto transport = std::make_unique<TraceTransport>();
  auto* trace = transport.get();
  ota::can::CyberGearSystem system;
  std::string error;
  if (!system.open({}, error, std::move(transport))) return 2;
  ota::CanMotorBackend first(system);
  first.keepalive(ota::AxisId::Yaw);
  const bool unknown_current_not_written = trace->count(ota::cybergear::Reg::LimitCur) == 0;
  // Seed position mode's command cache, then change only the reference and fail its send.
  first.command(ota::AxisId::Yaw, 0.2, 0.1);
  trace->fail_next = true;
  first.command(ota::AxisId::Yaw, 0.3, 0.1);
  first.command(ota::AxisId::Yaw, 0.3, 0.1);
  const bool retried_position = trace->count(ota::cybergear::Reg::LocRef) == 3;
  // A new backend must transmit its own initial reference, even at the same pose.
  ota::CanMotorBackend second(system);
  second.command(ota::AxisId::Yaw, 0.3, 0.1);
  const bool independent = trace->count(ota::cybergear::Reg::LocRef) == 4;
  trace->fail_next = true;
  second.command_velocity(ota::AxisId::Pitch, 0.05);
  second.command_velocity(ota::AxisId::Pitch, 0.05);
  const bool retried_speed = trace->count(ota::cybergear::Reg::SpdRef) == 2;
  // Failure to lower the speed limit must prevent publishing a new destination.
  trace->fail_next = true;
  second.command(ota::AxisId::Yaw, 0.4, 0.02);
  const bool ordered = trace->count(ota::cybergear::Reg::LocRef) == 4;
  std::cout << "{\"position_retry\":" << retried_position
            << ",\"instance_isolation\":" << independent
            << ",\"speed_retry\":" << retried_speed
            << ",\"limit_before_reference\":" << ordered
            << ",\"unknown_current_not_written\":" << unknown_current_not_written << "}\n";
  system.close();
  return retried_position && independent && retried_speed && ordered && unknown_current_not_written ? 0 : 1;
}
