// Bounded, neutral-reference probe of the production asynchronous mode recipe.
// Requires explicit --hardware; never registered as an unattended CTest.
#include <algorithm>
#include <chrono>
#include <iostream>
#include <thread>
#include "control/can_motor_backend.hpp"
#include "commissioning_watchdog.hpp"
using namespace ota;
int main(int argc, char** argv) {
  if (argc != 2 || std::string(argv[1]) != "--hardware") return 2;
  can::CyberGearSystem sys;
  can::CyberGearSystemConfig cfg;
  cfg.transport = "yousee"; cfg.iface = "/dev/ttyUSB0";
  std::string err;
  if (!sys.open(cfg, err)) { std::cerr << err << '\n'; return 1; }
  CanMotorBackend backend(sys);
  struct Stop { CanMotorBackend& b; ~Stop() { b.deenergize(AxisId::Pitch); b.deenergize(AxisId::Yaw); } } stop{backend};
  backend.deenergize(AxisId::Pitch); backend.deenergize(AxisId::Yaw);
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  try {
    for (auto axis : {AxisId::Yaw, AxisId::Pitch}) {
      sys.send_stop(axis);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      const auto initial = backend.snapshot(axis, now_monotonic_ns());
      if (!initial.has_feedback || initial.faults) throw std::runtime_error("initial feedback invalid");
      tools::CommissioningWatchdog watchdog(sys, {axis, initial.q_rad - .5*kDeg2Rad,
          initial.q_rad + .5*kDeg2Rad, 55.0, 8'000'000'000LL});
      for (const bool position : {false, true, false}) {
        const auto started = now_monotonic_ns();
        TimeNs worst = 0;
        MotorBackend::Transition result;
        do {
          watchdog.heartbeat();
          const auto before = now_monotonic_ns();
          watchdog.command([&] { result = backend.transition_mode(axis, position,
              position ? 2*kDeg2Rad : (axis == AxisId::Pitch ? 3.0 : 1.0), before, err); });
          worst = std::max(worst, now_monotonic_ns() - before);
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
        } while (result == MotorBackend::Transition::Pending);
        std::cout << axis_name(axis) << " position=" << position << " duration_ms="
                  << (now_monotonic_ns()-started)/1e6 << " worst_call_ms=" << worst/1e6
                  << " ok=" << (result == MotorBackend::Transition::Complete) << " error=" << err << std::endl;
        if (result != MotorBackend::Transition::Complete || worst > 10000000LL)
          throw std::runtime_error("mode recipe probe failed");
      }
      // Cancel with a read/enable sequence still pending. No poll can run after
      // cancellation in the control loop because STOP also changes its phase.
      for (int i=0; i<8; ++i) {
        watchdog.heartbeat();
        watchdog.command([&] { backend.transition_mode(axis, true, 2*kDeg2Rad, now_monotonic_ns(), err); });
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
      watchdog.command([&] { backend.deenergize(axis); });
      watchdog.finish();
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      can::AxisLatest latest;
      sys.axis(axis).latest(latest);
      if (latest.mode == 2) throw std::runtime_error("motor enabled after cancellation");
      std::cout << axis_name(axis) << " cancelled_and_disabled=true" << std::endl;
    }
    // Exercise the production watchdog independently of the diagnostic guard.
    // Only a neutral speed reference is enabled, then the control caller stalls.
    sys.start_watchdog();
    MotorBackend::Transition status;
    do {
      sys.heartbeat();
      status=backend.transition_mode(AxisId::Yaw, false, 1.0, now_monotonic_ns(), err);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    } while (status == MotorBackend::Transition::Pending);
    if (status != MotorBackend::Transition::Complete) throw std::runtime_error(err);
    std::this_thread::sleep_for(std::chrono::milliseconds(140));
    can::AxisLatest stopped;
    sys.axis(AxisId::Yaw).latest(stopped);
    if (!sys.motion_inhibited() || stopped.mode == 2 || sys.send_enable(AxisId::Yaw))
      throw std::runtime_error("production watchdog did not stop/inhibit late enable");
    std::cout << "production_watchdog_stall_disabled=true late_enable_denied=true" << std::endl;
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
