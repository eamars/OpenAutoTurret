// Plant models and the session simulator: the properties the design relies on.
#include "plant.hpp"
#include "position_loop.hpp"
#include "session_parts.hpp"
#include "simulate.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace ota::axis;

namespace {
void expect(bool value,const char* message) {
  if(!value) throw std::runtime_error(message);
}
YawPlantParameters yaw_fixture() {
  YawPlantParameters p{};
  p.inertia=.03; p.coulomb_positive=p.coulomb_negative=.35; p.stribeck_positive=p.stribeck_negative=.38;
  p.stribeck_speed=.3; p.creep_drop=.45; p.creep_speed=.012; p.presliding_stiffness=2000; p.presliding_damping=5;
  p.actuation_delay_s=.001; p.current_tau_s=.0005; p.encoder_delay_s=.0003; p.counts_per_rev=8192;
  return p;
}
void sliding_current_equals_the_friction_curve() {
  // Constant current above friction: the speed settles where the LuGre steady level matches it.
  YawPlant plant(yaw_fixture(),0.);
  plant.command(0.,.5);
  plant.advance(3.);
  const double v=plant.velocity();
  expect(v>0 && std::abs(plant.friction(v,plant.position())-.5)<.01,"steady speed sits on the friction curve");
}
void creeps_below_the_peak_and_holds_below_the_creep_level() {
  YawPlant creep(yaw_fixture(),0.), hold(yaw_fixture(),0.);
  creep.command(0.,.45); hold.command(0.,.2);
  creep.advance(2.); hold.advance(2.);
  expect(creep.position()>1e-3,"creeps under a force between the creep level and the peak");
  expect(std::abs(hold.position())<1e-3,"holds (only elastic pre-sliding) below the creep level");
}
void reading_carries_the_crosstalk_of_the_delayed_command() {
  auto p=yaw_fixture();
  for (auto& g:p.crosstalk_map) g=.005;
  p.crosstalk_delay_s=.002;
  YawPlant plant(p,0.);
  plant.command(0.,1.);
  expect(std::abs(plant.reading(0.,.001))<1e-12,"no crosstalk before the delay");
  expect(std::abs(plant.reading(0.,.003)-std::round(.005/(2*M_PI/8192))*(2*M_PI/8192))<1e-12,"quantized crosstalk after it");
}
void position_loop_does_not_wind_into_the_clamp() {
  PositionLoop loop;
  expect(loop.configure({10.,100.,.1,.5}),"configure");
  double cmd=0.;
  for (int k=0;k<1000;++k) cmd=loop.step(.001,1.,0.,0.);  // far behind: clamped
  expect(cmd==.5 && loop.integral()==0.,"no integral growth while clamped");
}
void guard_trips_on_a_limit_cycle_not_on_a_rock() {
  OscillationMonitor cycle(.3), rock(.3);
  bool tripped=false;
  for (int k=0;k<1000 && !tripped;++k) tripped=cycle.update(.001,1.5*std::sin(2*M_PI*15*k*.001));
  expect(tripped,"a sustained 15 Hz, 1.5 A limit cycle trips");
  for (int k=0;k<1000;++k) {
    const double t=k*.001;
    const bool rocking=t>=.5 && t<.525;
    const double u=rocking?-.6:(t>=.525 && t<.6?1.3:.9);  // stuck at 0.9 A, rock, slam back
    expect(!rock.update(.001,u,t>=.5 && t<.725),"a stall-recovery rock is not an oscillation");
  }
}
void simulator_is_deterministic() {
  const auto request=YAML::Load(R"({axis: pitch,
    loop: {kp_per_s: 30, ki_per_s2: 20, integral_clamp_rad_s: 0.1, speed_limit_rad_s: 0.6},
    plant: {speed_delay_s: 0.002, speed_tau_s: 0.012, reply_delay_s: 0.0005},
    reference: {t: [0, 1, 2], q: [0, 0.05, 0.05], v: [0.05, 0, 0], a: [0, 0, 0]}})");
  const auto a=simulate(request), b=simulate(request);
  expect(a.status=="COMPLETE" && a.rows==b.rows,"identical requests give identical runs");
  expect(std::abs(a.rows.back()[4]-0.05)<1e-3,"pitch settles on the reference");
}
}

int main() {
  try {
    sliding_current_equals_the_friction_curve();
    creeps_below_the_peak_and_holds_below_the_creep_level();
    reading_carries_the_crosstalk_of_the_delayed_command();
    position_loop_does_not_wind_into_the_clamp();
    guard_trips_on_a_limit_cycle_not_on_a_rock();
    simulator_is_deterministic();
  } catch(const std::exception& e) { std::cerr<<"FAIL: "<<e.what()<<'\n'; return 1; }
  std::cout<<"plant tests passed\n"; return 0;
}
