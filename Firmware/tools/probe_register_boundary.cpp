// Immediate and unrelated replies through the production request/response path.
#include <cstring>
#include <iostream>
#include "can/cybergear_system.hpp"

class ReplyTransport : public ota::can::CanTransport {
 public:
  FrameCallback callback;
  bool wrong_motor = false, wrong_register = false;
  bool start(std::string&) override { return true; }
  void stop() override {}
  void set_frame_callback(FrameCallback cb) override { callback = std::move(cb); }
  bool send(uint32_t id, const uint8_t data[8], std::string*) override {
    auto request = ota::cybergear::unpack_ext_id(id);
    ota::can::RawFrame reply{};
    reply.id = ota::cybergear::pack_ext_id(17, request.target + (wrong_motor ? 1 : 0), 0);
    reply.dlc = 8;
    std::memcpy(reply.data, data, 8);
    if (wrong_register) ++reply.data[0];
    const float value = 12.5f;
    std::memcpy(reply.data + 4, &value, 4);
    callback(reply);  // Can arrive before send() returns.
    return true;
  }
  ota::can::BusStats stats() const override { return {}; }
  bool is_up() const override { return true; }
  ota::can::CanIfState can_state() const override { return ota::can::CanIfState::Unknown; }
  const char* kind() const override { return "offline-reply"; }
  std::string device() const override { return "no-hardware"; }
};

int main() {
  auto transport = std::make_unique<ReplyTransport>();
  auto* replies = transport.get();
  ota::can::CyberGearSystem system;
  std::string error;
  if (!system.open({}, error, std::move(transport))) return 2;
  double value = -1;
  auto read = [&] { return system.read_register(ota::AxisId::Pitch,
      ota::cybergear::Reg::LocKp, value, 20, &error); };
  const bool immediate = read() && value == 12.5;
  value = -1;
  replies->wrong_motor = true;
  const bool motor_matched = !read() && value == -1;
  replies->wrong_motor = false;
  replies->wrong_register = true;
  const bool register_matched = !read() && value == -1;
  std::cout << "{\"immediate_reply\":" << immediate
            << ",\"motor_matched\":" << motor_matched
            << ",\"register_matched\":" << register_matched << "}\n";
  system.close();
  return immediate && motor_matched && register_matched ? 0 : 1;
}
