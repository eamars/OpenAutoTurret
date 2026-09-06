// Exercise the real CAN request waiter while the independent watchdog runs.
// No physical device: the transport withholds replies and provides feedback.
#include <atomic>
#include <cstring>
#include <iostream>
#include "commissioning_watchdog.hpp"
#include "control/control_loop.hpp"
#include "config/station_wiring.hpp"
#include "sim/sim_motor_backend.hpp"
using namespace ota;
class WithheldReplyTransport : public can::CanTransport {
 public:
  FrameCallback callback;
  std::atomic<bool> running{false}, publish_feedback{true};
  std::atomic<bool> enabled[2]{};
  std::atomic<int> stops{0};
  std::thread feedback;
  bool start(std::string&) override {
    running = true;
    feedback = std::thread([this] {
      while (running) {
        if (publish_feedback) for (int i=0; i<2; ++i) {
          can::RawFrame f{};
          f.id = cybergear::pack_ext_id(2, (enabled[i] ? 0x8000 : 0) | (100+i), 0);
          f.rx_ns = now_monotonic_ns();
          // Center of feedback angle/speed/torque ranges, 25 C.
          f.data[0] = f.data[2] = f.data[4] = 0x80;
          f.data[6] = 0; f.data[7] = 250;
          callback(f);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
    });
    return true;
  }
  void stop() override { running = false; if (feedback.joinable()) feedback.join(); }
  ~WithheldReplyTransport() override { stop(); }
  void set_frame_callback(FrameCallback cb) override { callback = std::move(cb); }
  bool send(uint32_t id, const uint8_t[8], std::string*) override {
    const auto request = cybergear::unpack_ext_id(id);
    const auto axis = request.target - 100;
    if (axis > 1) return false;
    if (request.comm_type == 3) enabled[axis] = true;
    if (request.comm_type == 4) { enabled[axis] = false; ++stops; }
    // ReadReg deliberately never answers: production transact() must really wait.
    return true;
  }
  can::BusStats stats() const override { return {}; }
  bool is_up() const override { return running; }
  can::CanIfState can_state() const override { return can::CanIfState::Unknown; }
  const char* kind() const override { return "withheld-reply"; }
  std::string device() const override { return "no-hardware"; }
};

class ObserveHomingBackend : public sim::SimMotorBackend {
 public:
  double entry_current[2]{};
  bool fail_yaw = false;
  bool enter_speed_mode(AxisId a, double current, std::string& error) override {
    entry_current[static_cast<int>(a)] = current;
    if (fail_yaw && a == AxisId::Yaw) { error = "injected mode failure"; return false; }
    return SimMotorBackend::enter_speed_mode(a, current, error);
  }
};

int main(int argc, char** argv) {
  auto transport = std::make_unique<WithheldReplyTransport>();
  auto* io = transport.get();
  can::CyberGearSystem system;
  std::string error;
  if (!system.open({}, error, std::move(transport))) return 2;
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  using Watchdog = tools::CommissioningWatchdog;
  double trip_ms = 0;
  bool stalled_read_stopped = false, late_enable_denied = false;
  {
    Watchdog guard(system, {AxisId::Pitch, -1, 1, 50, 2'000'000'000});
    guard.command([&] { system.send_enable(AxisId::Pitch); });
    const auto start = now_monotonic_ns();
    double value;
    const bool read = system.read_register(AxisId::Pitch, cybergear::Reg::MechPos, value, 250, &error);
    trip_ms = (guard.trip_ns()-start)*1e-6;
    stalled_read_stopped = !read && guard.reason() == Watchdog::Reason::Heartbeat &&
        trip_ms >= 80 && trip_ms < 150 && !io->enabled[0] && io->stops >= 2;
    try { guard.command([&] { system.send_enable(AxisId::Pitch); }); }
    catch (const std::runtime_error&) { late_enable_denied = true; }
  }
  bool feedback_stopped = false;
  {
    Watchdog guard(system, {AxisId::Pitch, -1, 1, 50, 2'000'000'000});
    guard.command([&] { system.send_enable(AxisId::Pitch); });
    io->publish_feedback = false;
    for (int i=0; i<40 && guard.reason() == Watchdog::Reason::None; ++i) {
      guard.heartbeat();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    feedback_stopped = guard.reason() == Watchdog::Reason::Feedback && !io->enabled[0];
  }
  io->publish_feedback = true;
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  bool deadline_stopped = false;
  {
    Watchdog guard(system, {AxisId::Pitch, -1, 1, 50, 60'000'000});
    guard.command([&] { system.send_enable(AxisId::Pitch); });
    for (int i=0; i<30 && guard.reason() == Watchdog::Reason::None; ++i) {
      guard.heartbeat();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    deadline_stopped = guard.reason() == Watchdog::Reason::Deadline && !io->enabled[0];
  }
  system.close();
  const auto config = config::load_turret_config(argc > 1 ? argv[1] : "config/turret.yaml");
  if (!config.ok) return 2;
  auto backend = std::make_unique<ObserveHomingBackend>();
  auto* observe = backend.get();
  ControlLoop loop({}, std::move(backend));
  const bool homing_started = loop.start_homing(wire::make_homing_plan(config.config, error), error);
  const auto t = now_monotonic_ns();
  loop.step(t, 5000000LL);
  loop.step(t + 5000000LL, 5000000LL);
  const bool per_axis_limits = homing_started && observe->entry_current[0] == config.config.axes[0].limit_cur_a &&
      observe->entry_current[1] == config.config.axes[1].limit_cur_a;
  observe->fail_yaw = true;
  loop.start_homing(wire::make_homing_plan(config.config, error), error);
  loop.step(t + 10000000LL, 5000000LL);
  loop.step(t + 15000000LL, 5000000LL);
  const bool rejected = loop.phase() == Phase::Fault;
  const bool partial_failure_disabled = rejected &&
      !observe->snapshot(AxisId::Pitch, 0).in_speed_mode &&
      !observe->snapshot(AxisId::Yaw, 0).in_speed_mode;
  std::cout << "{\"stalled_read_stopped\":" << stalled_read_stopped
            << ",\"trip_ms\":" << trip_ms
            << ",\"late_enable_denied\":" << late_enable_denied
            << ",\"feedback_stopped\":" << feedback_stopped
            << ",\"deadline_stopped\":" << deadline_stopped
            << ",\"homing_per_axis_limits\":" << per_axis_limits
            << ",\"partial_failure_disabled\":" << partial_failure_disabled << "}\n";
  return stalled_read_stopped && late_enable_denied && feedback_stopped && deadline_stopped &&
      per_axis_limits && partial_failure_disabled ? 0 : 1;
}
