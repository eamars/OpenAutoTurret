// Exercise the real backend, watchdog, UART transport and framing through a PTY.
// The far end emulates only disabled motor feedback. No physical device opens.
#include <atomic>
#include <chrono>
#include <fcntl.h>
#include <iostream>
#include <poll.h>
#include <pty.h>
#include <thread>
#include <unistd.h>
#include "control/can_motor_backend.hpp"
#include "can/yousee_transport.hpp"

using namespace ota;
int main() {
  int master = -1, slave = -1;
  char path[128]{};
  if (openpty(&master, &slave, path, nullptr, nullptr)) return 2;
  std::atomic<bool> quit{false}, pitch_silent{false};
  std::atomic<int> clears{0}, enables{0};
  std::array<unsigned, 2> motor_faults{1, 1};  // emulator thread owns drive state
  std::thread motor([&] {
    can::YouseeCodec codec([&](const can::RawFrame& f) {
      const auto e = cybergear::unpack_ext_id(f.id);
      if (e.comm_type == 3) ++enables;
      if (e.comm_type != 4) return;
      const auto axis = e.target == 100 ? 0 : 1;
      if (f.data[0] == 1) { ++clears; motor_faults[axis] = 0; }
      if (e.target == 100 && pitch_silent.load()) return;
      // COMM_TYPE_2, disabled, fault-free, q/v/tq midpoint, 25 C.
      const uint8_t data[8] = {0x80, 0, 0x80, 0, 0x80, 0, 0, 250};
      std::vector<uint8_t> bytes;
      can::YouseeCodec::encode((2u << 24) | (motor_faults[axis] << 16) |
          (uint32_t(e.target) << 8), data, 8, bytes);
      if (::write(master, bytes.data(), bytes.size()) != static_cast<ssize_t>(bytes.size()))
        quit.store(true);
    });
    while (!quit.load()) {
      pollfd p{master, POLLIN, 0};
      if (::poll(&p, 1, 10) <= 0) continue;
      uint8_t buf[512];
      const auto n = ::read(master, buf, sizeof(buf));
      if (n > 0) codec.feed(buf, n, now_monotonic_ns());
    }
  });
  int result = 0;
  {
    can::CyberGearSystem system;
    can::YouseeTransport::Options options;
    options.port = path; options.skip_at_init = true;
    std::string error;
    if (!system.open({}, error, std::make_unique<can::YouseeTransport>(options))) {
      std::cerr << error << '\n'; result = 1;
    } else {
      system.start_watchdog();
      CanMotorBackend backend(system);
      auto recover = [&](bool expect_success) {
        system.heartbeat();
        if (!backend.begin_motor_recovery(error)) return false;
        auto status = MotorBackend::Transition::Pending;
        const auto began = now_monotonic_ns();
        while (status == MotorBackend::Transition::Pending && now_monotonic_ns() - began < 6'000'000'000) {
          system.heartbeat();
          if (!system.motion_inhibited() || system.send_enable(AxisId::Yaw)) return false;
          status = backend.poll_motor_recovery(now_monotonic_ns(), 75, error);
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        const bool success = status == MotorBackend::Transition::Complete;
        std::cout << "expect_success=" << expect_success << " success=" << success
                  << " inhibited=" << system.motion_inhibited() << " detail=" << error << '\n';
        return success == expect_success && system.motion_inhibited() != expect_success;
      };
      // Existing watchdog latch: a stalled caller, no energized motors.
      std::this_thread::sleep_for(std::chrono::milliseconds(140));
      if (!system.motion_inhibited() || !recover(true)) result = 1;
      pitch_silent.store(true);
      if (!recover(false)) result = 1;
      pitch_silent.store(false);
      if (!recover(true)) result = 1;
      if (clears.load() != 6 || enables.load() != 0) result = 1;
      std::cout << "clear_frames=" << clears << " enable_frames=" << enables
                << " result=" << (result ? "FAIL" : "PASS") << '\n';
    }
  }
  quit.store(true); motor.join();
  ::close(master); ::close(slave);
  return result;
}
