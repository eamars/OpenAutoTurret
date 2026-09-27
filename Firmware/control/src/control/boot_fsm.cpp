// BootFsm — see boot_fsm.hpp.
#include "control/boot_fsm.hpp"
#include "common/time.hpp"

#include <cmath>

namespace ota {

BootFsm::BootFsm(MotorBackend& backend, BootConfig cfg)
    : backend_(backend), cfg_(cfg) {}

void BootFsm::fail(const std::string& reason) {
  state_ = BootState::FaultLocked;
  error_ = reason;
}

BootState BootFsm::step() {
  switch (state_) {
    case BootState::PowerOn:
      // Nothing to do; the process is up.
      state_ = BootState::ProcessInit;
      break;

    case BootState::ProcessInit:
      // Config is already loaded/validated (a hard error there prevents boot).
      state_ = BootState::CanInit;
      break;

    case BootState::CanInit:
      // The CAN bus is opened by the backend (done by the caller before this
      // FSM runs). Mark the transition and start discovery.
      state_ = BootState::DiscoverPitch;
      break;

    case BootState::DiscoverPitch: {
      std::string err;
      if (!backend_.discover(AxisId::Pitch, unique_ids_[0], err)) {
        fail("discovery failed (pitch): " + err);
        break;
      }
      state_ = BootState::DiscoverYaw;
      break;
    }

    case BootState::DiscoverYaw: {
      if (backend_.yaw_feedback_registerless()) {
        // GM6020 has no CyberGear discovery/UID transaction. Keep the ID empty
        // instead of manufacturing a device identity for retained calibration.
        unique_ids_[static_cast<size_t>(AxisId::Yaw)] = 0;
        state_ = BootState::SelfTestPitch;
        break;
      }
      std::string err;
      if (!backend_.discover(AxisId::Yaw, unique_ids_[1], err)) {
        fail("discovery failed (yaw): " + err);
        break;
      }
      state_ = BootState::SelfTestPitch;
      break;
    }

    case BootState::SelfTestPitch: {
      double v = 0;
      std::string err;
      if (!backend_.read_register(AxisId::Pitch, cfg_.self_test_register, v,
                                  cfg_.register_timeout_ms, err)) {
        fail("motor self-test failed (pitch): " + err);
        break;
      }
      state_ = BootState::SelfTestYaw;
      break;
    }

    case BootState::SelfTestYaw: {
      if (backend_.supports_continuous_yaw() || backend_.yaw_feedback_registerless()) {
        const TimeNs now = now_monotonic_ns();
        const AxisSnapshot feedback = backend_.snapshot(AxisId::Yaw, now);
        const TimeNs max_age = static_cast<TimeNs>(cfg_.feedback_max_age_ms) * 1000000LL;
        if (feedback.faults_known && feedback.faults != 0) {
          fail("motor self-test failed (yaw): backend reports a motor fault");
          break;
        }
        if (!feedback.has_feedback || feedback.rx_ns <= 0 || feedback.rx_ns > now ||
            max_age <= 0 || now - feedback.rx_ns > max_age ||
            !std::isfinite(feedback.q_rad) || !std::isfinite(feedback.v_rad_s)) {
          fail("motor self-test failed (yaw): no fresh finite session-relative feedback");
          break;
        }
        state_ = BootState::Unhomed;
        break;
      }
      double v = 0;
      std::string err;
      if (!backend_.read_register(AxisId::Yaw, cfg_.self_test_register, v,
                                  cfg_.register_timeout_ms, err)) {
        fail("motor self-test failed (yaw): " + err);
        break;
      }
      state_ = BootState::Unhomed;  // ready to start the homing plan
      break;
    }

    case BootState::Unhomed:
    case BootState::FaultLocked:
      // Terminal states; no further boot work.
      break;
  }
  return state_;
}

}  // namespace ota
