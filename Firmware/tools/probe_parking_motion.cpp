// Production control-loop probe with injected measured motion. No hardware or
// camera is opened. This checks supervision, not the physical braking response.
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <spdlog/spdlog.h>
#include "control/control_loop.hpp"
#include "sim/sim_motor_backend.hpp"

using namespace ota;
namespace {
class ObservedPlant : public sim::SimMotorBackend {
 public:
  double pitch_bias = 0;
  double gain_kp = 0, gain_ki = 0;
  int transitions = 0;
  bool replay_only = false;
  bool delay_setup = false;
  std::array<double, 2> command_rate{};
  bool adopt_running_mode(AxisId a, bool position, std::string& err,
                          double = -1, double = 1) override {
    return position ? enter_position_mode(a, .1, err) : enter_speed_mode(a, 1, err);
  }
  Transition transition_mode(AxisId a, bool position, double limit, TimeNs now,
      std::string& err, double ki = -1, double kp = 1, bool check_displacement = true) override {
    ++transitions; gain_kp = kp; gain_ki = ki;
    if (delay_setup) return Transition::Pending;
    return SimMotorBackend::transition_mode(a, position, limit, now, err, ki, kp,check_displacement);
  }
  void command_velocity(AxisId a, double velocity) override {
    command_rate[static_cast<int>(a)] = velocity;
    if (replay_only) return;
    SimMotorBackend::command_velocity(a, velocity);
    // Deliberate disturbance absent from the ideal speed-mode simulator.
    if (a == AxisId::Pitch && in_speed_mode(a))
      set_position(a, position(a) + pitch_bias * .005);
  }
};
struct Fixture {
  ObservedPlant* plant;
  ControlLoop loop;
  TimeNs now = 1'000'000'000;
  Fixture(bool recorded = false) : plant(new ObservedPlant), loop(config(recorded), std::unique_ptr<MotorBackend>(plant)) {
    std::array<AxisLogicalModel, 2> models;
    std::array<AxisLimits, 2> limits;
    for (int i=0; i<2; ++i) {
      models[i].set_reference(0, 0);
      limits[i].set_from_endpoints(-90*kDeg2Rad, 90*kDeg2Rad, 5*kDeg2Rad);
      plant->set_stops(static_cast<AxisId>(i), -90*kDeg2Rad, 90*kDeg2Rad);
      plant->set_position(static_cast<AxisId>(i), 0);
    }
    if (recorded) {
      limits[0].set_from_endpoints(-1.39105, .002861, 5*kDeg2Rad);
      limits[1].set_from_endpoints(-2.915427, 3.244449, 5*kDeg2Rad);
      plant->set_position(AxisId::Pitch, -.701343);
      plant->set_position(AxisId::Yaw, .300793);
      plant->replay_only = true;
    }
    std::string err;
    if (!loop.restore_retained_homing(models, limits, err)) throw std::runtime_error(err);
    tick();
    if (!loop.start_parking(err)) throw std::runtime_error(err);
  }
  static ControlLoop::Config config(bool recorded) {
    ControlLoop::Config c;
    c.service_speed_control = true;
    c.service_speed_kp = 4; c.service_speed_ki = .05;
    c.park.park_logical_deg = {-20, 30};
    if (recorded) c.park.park_logical_deg = {-1.216517*kRad2Deg, .164511*kRad2Deg};
    c.park.speed_deg_s = 3;
    return c;
  }
  void tick() { loop.step(now, 5'000'000); now += 5'000'000; }
  bool stopped_and_latched() {
    plant->pitch_bias = 0;
    const int before = plant->transitions;
    for (int i=0;i<200;++i) tick();
    std::string err;
    const bool cannot_home = !loop.start_homing(HomingPlan({}, {}), err);
    return loop.phase() == Phase::Fault && cannot_home &&
        before == plant->transitions && plant->command_rate[0] == 0 && plant->command_rate[1] == 0;
  }
};
}
int main(int argc, char** argv) {
  spdlog::set_level(spdlog::level::err);
  bool pass = true;
  {
    Fixture f;
    for (int i=0;i<10;++i) f.tick();
    bool ok = f.plant->gain_kp == 4 && f.plant->gain_ki == .05;
    std::cout << "parking_drive_gains=" << f.plant->gain_kp << ',' << f.plant->gain_ki
              << " configured_gains_preserved=" << ok << '\n';
    pass &= ok;
    // A half-degree/second disturbance must not be allowed to accumulate
    // while the other axis traverses its parking path.
    f.plant->pitch_bias = .5*kDeg2Rad;
    int cycles=0;
    while (f.loop.phase()==Phase::Parking && cycles++<2000) f.tick();
    const double excursion = std::abs(f.plant->position(AxisId::Pitch))*kRad2Deg;
    const bool stopped = f.stopped_and_latched();
    ok = stopped && excursion < 1.1;
    std::cout << "inactive_drift_deg=" << excursion << " latched=" << stopped << " pass=" << ok << '\n';
    pass &= ok;
  }
  if (argc == 2) {
    // Optional numeric replay: seconds, pitch radians, yaw radians, separated
    // by whitespace. Interpolate the 2 Hz parking log onto controller ticks.
    // Detection timing is approximate at that source resolution. The plant
    // deliberately ignores commands; this cannot prove physical stopping.
    std::ifstream input(argv[1]);
    double t, pitch, yaw;
    if (!(input >> t >> pitch >> yaw)) return 2;
    Fixture f(true);
    const auto began=f.now;
    double previous_t=t, previous_pitch=pitch, previous_yaw=yaw;
    while (f.loop.phase()==Phase::Parking && (input >> t >> pitch >> yaw)) {
      while (f.loop.phase()==Phase::Parking && (f.now-began)/1e9<=t) {
        const double elapsed=(f.now-began)/1e9;
        const double alpha=t>previous_t ? std::clamp((elapsed-previous_t)/(t-previous_t),0.0,1.0) : 1;
        f.plant->set_position(AxisId::Pitch, previous_pitch+alpha*(pitch-previous_pitch));
        f.plant->set_position(AxisId::Yaw, previous_yaw+alpha*(yaw-previous_yaw));
        f.tick();
      }
      previous_t=t; previous_pitch=pitch; previous_yaw=yaw;
    }
    const double detected=(f.now-began)/1e9;
    const auto reason=f.loop.fault_reason();
    const bool ok=f.stopped_and_latched() && detected<87.0;
    std::cout << "recorded_motion_detection_s=" << detected << " reason='" << reason
              << "' before_first_recorded_pitch_boundary_crossing=" << ok << '\n';
    pass &= ok;
  }
  {
    Fixture f;
    for (int i=0;i<10;++i) f.tick();
    int cycles=0;
    // Position evidence simulates a drive outrunning its 3 deg/s request.
    while (f.loop.phase()==Phase::Parking && cycles++<100) {
      f.plant->set_position(AxisId::Yaw, f.plant->position(AxisId::Yaw)+12*kDeg2Rad*.005);
      f.tick();
    }
    const bool ok=f.stopped_and_latched() && cycles<=40;
    std::cout << "overspeed_detection_ms=" << cycles*5 << " pass=" << ok << '\n';
    pass &= ok;
  }
  {
    Fixture f;
    for (int i=0;i<10;++i) f.tick();
    f.loop.submit_command("stop_motion", ""); f.tick();
    const bool ok=f.stopped_and_latched();
    std::cout << "operator_stop_cancels_parking=" << ok << '\n';
    pass &= ok;
  }
  {
    Fixture f;
    f.plant->delay_setup = true;
    f.tick();
    f.loop.submit_command("stop_motion", ""); f.tick();
    const bool ok = f.stopped_and_latched() &&
        f.plant->snapshot(AxisId::Pitch,f.now).disabled &&
        f.plant->snapshot(AxisId::Yaw,f.now).disabled;
    std::cout << "operator_stop_during_mode_setup_disables_and_latches=" << ok << '\n';
    pass &= ok;
  }
  std::cout << "simulated_motion_supervision=" << (pass ? "PASS" : "FAIL") << '\n';
  return pass ? 0 : 1;
}
