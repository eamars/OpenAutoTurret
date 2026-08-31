// OpenAutoTurret — ControlLoop implementation (§46 per-cycle engine).
#include "control/control_loop.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace ota {

namespace {
// At-rest threshold for the fault-stop "keep stopping, then hold" logic.
constexpr double kAtRestVelRadS = 0.05;
// Position tolerance for "at the ready pose".
constexpr double kReadyPosTolRad = 0.01;
}  // namespace

ControlLoop::ControlLoop(Config cfg, std::unique_ptr<MotorBackend> backend)
    : cfg_(std::move(cfg)), backend_(std::move(backend)) {
  SupervisorParams sp;
  sp.feedback_max_age_ms = cfg_.feedback_max_age_ms;
  sp.deadline_max_us = cfg_.deadline_max_us;
  sp.deadline_miss_threshold = cfg_.deadline_miss_threshold;
  sp.motor_overtemp_c = cfg_.motor_overtemp_c;
  sp.a_brake_rad_s2 = cfg_.a_brake_rad_s2;
  sp.j_brake_rad_s3 = cfg_.j_brake_rad_s3;
  sp.stop_margin_rad = cfg_.stop_margin_rad;
  supervisor_ = SafetySupervisor(sp);

  SafetyEnvelopeParams ep;
  ep.a_brake_rad_s2 = cfg_.a_brake_rad_s2;
  ep.j_brake_rad_s3 = cfg_.j_brake_rad_s3;
  ep.margin_rad = cfg_.stop_margin_rad;
  ep.v_max_rad_s = cfg_.hold_speed_rad_s;
  env_ = SafetyEnvelope(ep);

  deadline_ns_ = 1000000000LL / std::max(1, cfg_.control_hz);
}

bool ControlLoop::enter_position_mode_all(double limit_spd, std::string& err) {
  for (int i = 0; i < kAxisCount; ++i) {
    const AxisId a = static_cast<AxisId>(i);
    std::string e;
    if (!backend_->enter_position_mode(a, limit_spd, e)) {
      err = std::string(axis_name(a)) + ": " + e;
      return false;
    }
  }
  return true;
}

bool ControlLoop::start_homing(HomingPlan plan, std::string& err) {
  if (phase_ == Phase::Fault) {
    err = "in fault; reset required";
    return false;
  }
  homing_.reset(new HomingPlan(std::move(plan)));
  std::string e;
  if (!enter_position_mode_all(cfg_.hold_speed_rad_s, e)) {
    err = e;
    return false;
  }
  homed_ = false;
  at_ready_ = false;
  phase_ = Phase::Homing;
  return true;
}

bool ControlLoop::start_hold(std::string& err) {
  if (!homed_) {
    err = "not homed";
    return false;
  }
  std::string e;
  if (!enter_position_mode_all(cfg_.hold_speed_rad_s, e)) {
    err = e;
    return false;
  }
  phase_ = Phase::Hold;
  return true;
}

bool ControlLoop::start_parking(std::string& err) {
  if (!homed_) {
    err = "cannot park: not homed (position validity unknown, §38.1)";
    return false;
  }
  park_.reset(new ParkController(cfg_.park, limits_, models_));
  if (park_->failed()) {
    err = park_->fail_reason();
    fault(err);
    return false;
  }
  std::string e;
  if (!enter_position_mode_all(cfg_.park.speed_deg_s * kDeg2Rad, e)) {
    err = e;
    return false;
  }
  phase_ = Phase::Parking;
  return true;
}

void ControlLoop::deenergize_all() {
  for (int i = 0; i < kAxisCount; ++i) backend_->deenergize(static_cast<AxisId>(i));
}

bool ControlLoop::finalize_homing() {
  for (int i = 0; i < kAxisCount; ++i) {
    const AxisId a = static_cast<AxisId>(i);
    if (!homing_->axis_homed(a)) return false;
    models_[i] = homing_->model(a);
    limits_[i].set_from_endpoints(homing_->raw_low(a), homing_->raw_high(a),
                                  cfg_.soft_margin_rad);
    // Safe ready pose = midpoint of the measured travel (never at a stop).
    const double ll = models_[i].raw_to_logical_deg(homing_->raw_low(a));
    const double lh = models_[i].raw_to_logical_deg(homing_->raw_high(a));
    ready_raw_[i] = models_[i].logical_to_raw_rad(0.5 * (ll + lh));
  }
  homed_ = true;
  at_ready_ = false;
  return true;
}

HomingFeedback ControlLoop::to_feedback(const AxisSnapshot& s) {
  HomingFeedback fb;
  fb.t_ns = s.rx_ns;
  fb.pos_rad = s.q_rad;
  fb.vel_rad_s = s.v_rad_s;
  fb.torque_nm = s.torque_nm;
  fb.motor_fault = (s.faults != 0);
  return fb;
}

Phase ControlLoop::step(TimeNs now_ns, TimeNs period_ns) {
  // 1. Non-blocking snapshots.
  AxisSnapshot sp[kAxisCount];
  for (int i = 0; i < kAxisCount; ++i) {
    sp[i] = backend_->snapshot(static_cast<AxisId>(i), now_ns);
    last_q_[i] = sp[i].q_rad;  // for telemetry
  }

  // 2. §39.3 deadline watchdog: a cycle longer than the control period is a
  //    miss; consecutive misses feed the supervisor.
  int64_t overrun_us = 0;
  if (period_ns > deadline_ns_) {
    overrun_us = (period_ns - deadline_ns_) / 1000;
    ++deadline_miss_count_;
  } else {
    deadline_miss_count_ = 0;
  }

  // 3. Build the supervisor input.
  SupervisorInput in;
  for (int i = 0; i < kAxisCount; ++i) {
    in.axes[i].q_raw_rad = sp[i].q_rad;
    in.axes[i].v_rad_s = sp[i].v_rad_s;
    in.axes[i].has_feedback = sp[i].has_feedback;
    in.axes[i].feedback_age_ms =
        sp[i].has_feedback ? (now_ns - sp[i].rx_ns) / 1000000 : INT64_MAX;
    in.axes[i].temp_c = sp[i].temp_c;
    in.axes[i].motor_faults = sp[i].faults;
    in.axes[i].limits = limits_[i];
  }
  in.homing_valid = homed_;
  in.tracking_enabled = false;  // Phase 2: the station never tracks.
  in.cycle_overrun_us = overrun_us;
  in.deadline_miss_count = deadline_miss_count_;

  // 4. Safety decision (authoritative).
  last_decision_ = supervisor_.evaluate(in);

  // 5. Phase reference (per-axis q_ref, limit_spd). Default: hold in place.
  double q_ref[kAxisCount], lim[kAxisCount];
  for (int i = 0; i < kAxisCount; ++i) {
    q_ref[i] = sp[i].q_rad;
    lim[i] = 0.0;
  }
  switch (phase_) {
    case Phase::Homing: {
      const AxisId a = homing_->active_axis();
      DesiredState ds = homing_->step(to_feedback(sp[ix(a)]));
      q_ref[ix(a)] = ds.target_rad;
      lim[ix(a)] = ds.hold ? 0.0 : ds.speed_rad_s;
      if (homing_->failed()) {
        fault("homing failed: " + homing_->fail_reason());
      } else if (homing_->complete()) {
        if (finalize_homing()) {
          std::string e;
          if (enter_position_mode_all(cfg_.hold_speed_rad_s, e))
            phase_ = Phase::Hold;  // one-time re-pin, then ready-hold
          else
            fault(e);
        } else {
          fault("homing plan incomplete");
        }
      }
      break;
    }
    case Phase::Hold: {
      // Move to the safe ready pose (if not already there), then hold. Never
      // press against a stop.
      at_ready_ = true;
      for (int i = 0; i < kAxisCount; ++i) {
        if (std::fabs(sp[i].q_rad - ready_raw_[i]) > kReadyPosTolRad) {
          q_ref[i] = ready_raw_[i];
          lim[i] = cfg_.hold_speed_rad_s;
          at_ready_ = false;
        } else {
          q_ref[i] = sp[i].q_rad;
          lim[i] = 0.0;
        }
      }
      break;
    }
    case Phase::Parking: {
      ParkOutput po = park_->step(to_feedback(sp[ix(AxisId::Pitch)]),
                                  to_feedback(sp[ix(AxisId::Yaw)]));
      q_ref[ix(AxisId::Pitch)] = po.pitch.target_rad;
      lim[ix(AxisId::Pitch)] = po.pitch.hold ? 0.0 : po.pitch.speed_rad_s;
      q_ref[ix(AxisId::Yaw)] = po.yaw.target_rad;
      lim[ix(AxisId::Yaw)] = po.yaw.hold ? 0.0 : po.yaw.speed_rad_s;
      if (po.disable_pitch) backend_->deenergize(AxisId::Pitch);
      if (po.disable_yaw) backend_->deenergize(AxisId::Yaw);
      if (po.failed) fault("park failed: " + po.message);
      if (po.complete) phase_ = Phase::Parked;
      break;
    }
    case Phase::Fault: {
      // Controlled stop: keep braking while moving, then hold in place.
      for (int i = 0; i < kAxisCount; ++i) {
        if (std::fabs(sp[i].v_rad_s) > kAtRestVelRadS) {
          q_ref[i] = env_.emergency_stop_target(sp[i].q_rad, sp[i].v_rad_s,
                                                limits_[i]);
          lim[i] = cfg_.emergency_speed_rad_s;
        } else {
          q_ref[i] = sp[i].q_rad;
          lim[i] = 0.0;
        }
      }
      break;
    }
    case Phase::Idle:
    case Phase::Parked:
    default:
      break;  // hold (no motion)
  }

  // 6. Apply the safety action (overrides the phase reference), then command.
  bool any_disable = false;
  for (int i = 0; i < kAxisCount; ++i) {
    const AxisId a = static_cast<AxisId>(i);
    double qr = q_ref[i], ls = lim[i];
    bool do_command = true;
    switch (last_decision_.action) {
      case SafetyAction::Allow:
        break;
      case SafetyAction::Derate:
        ls *= cfg_.derate_factor;
        break;
      case SafetyAction::Hold:
        qr = sp[i].q_rad;
        ls = 0.0;
        break;
      case SafetyAction::Brake:
      case SafetyAction::FaultStop:
        qr = env_.emergency_stop_target(sp[i].q_rad, sp[i].v_rad_s, limits_[i]);
        ls = cfg_.emergency_speed_rad_s;
        break;
      case SafetyAction::Disable:
        backend_->deenergize(a);
        any_disable = true;
        do_command = false;
        break;
    }
    if (do_command) backend_->command(a, qr, ls);
  }

  // 7. Fault transitions (a fault is sticky; it needs a manual reset).
  if (any_disable) {
    fault(last_decision_.reason);
  } else if (last_decision_.action == SafetyAction::FaultStop &&
             phase_ != Phase::Fault) {
    fault(last_decision_.reason);
  }

  return phase_;
}

}  // namespace ota
