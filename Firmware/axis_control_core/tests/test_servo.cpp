#include "servo.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace ota::axis;

namespace {
void expect(bool value,const char* message) {
  if(!value) throw std::runtime_error(message);
}
ServoParameters fixture() {
  ServoParameters p{};
  p.encoder_variance=4.9e-8; p.gyro_variance=1.7e-6; p.process_variance=25.; p.max_encoder_age_s=.02;
  p.max_gyro_age_s=.08; p.use_gyro=0;
  p.inertia=.03; p.coulomb_positive=p.coulomb_negative=.3; p.stribeck_speed=.1; p.friction_band=.005;
  p.friction_map_limit=.5; p.friction_learning_speed=.035;
  p.kq=90; p.kv=1.6; p.ki=135; p.integral_cap=.6; p.error_clamp=.0873; p.hold_speed=.002;
  p.current_cap=1.5; p.slew=200; p.rms_limit=.8; p.rms_tau_s=3; p.dt_min=.0003; p.dt_max=.02;
  p.following_error_limit=.2618;
  return p;
}
// One encoder sample per millisecond at a fixed true position, acknowledging what was sent.
ServoOutput run(Servo& s,double& t,int steps,double q_true,double q_ref,double v_ref=0.,double crosstalk=0.) {
  ServoOutput out{};
  double applied=0.;
  for(int k=0;k<steps;++k) {
    t+=.001;
    expect(s.observe_encoder(t,q_true+crosstalk*applied),"encoder sample rejected");
    out=s.step(t,q_ref,v_ref,0.);
    if(out.status!=int(ServoStatus::Ok)) return out;
    applied=out.limited; s.acknowledge(true,applied);
  }
  return out;
}

void crosstalk_is_removed_from_the_position() {
  auto p=fixture();
  for(auto& g:p.crosstalk_map) g=-.005;  // reading moves -5 mrad per amp
  Servo s; expect(s.configure(p) && s.reset(0.,0.,0.,0.),"configure");
  double t=0.;
  const auto out=run(s,t,200,0.,.002,0.,-.005);  // stuck axis, current pushing
  expect(std::abs(out.position)<2e-4,"compensated position follows the true (stuck) angle");
}
void stiff_gains_stay_quiet_with_compensated_crosstalk() {
  // Axis held still, a small reference offset, real crosstalk on the reading. The
  // uncompensated loop reads its own current as position error and winds up.
  auto compensated=fixture(), blind=fixture();
  for(auto& g:compensated.crosstalk_map) g=-.005;
  Servo a,b; expect(a.configure(compensated) && a.reset(0.,0.,0.,0.) && b.configure(blind) && b.reset(0.,0.,0.,0.),"configure");
  double ta=0.,tb=0.;
  const auto quiet=run(a,ta,400,0.,.0005,0.,-.005), wound=run(b,tb,400,0.,.0005,0.,-.005);
  expect(std::abs(quiet.limited)<std::abs(wound.limited)*.8,"compensation removes the self-induced position error");
}
void following_error_stops_the_servo() {
  Servo s; expect(s.configure(fixture()) && s.reset(0.,0.,0.,0.),"configure");
  double t=0.;
  const auto out=run(s,t,5,0.,.3);
  expect(out.status==int(ServoStatus::FollowingError),"following error reported");
  expect(!s.ready(),"servo latched not-ready");
}
void stale_encoder_is_data_invalid() {
  Servo s; expect(s.configure(fixture()) && s.reset(0.,0.,0.,0.),"configure");
  double t=0.;
  run(s,t,5,0.,0.);
  const auto out=s.step(t+.05,0.,0.,0.);
  expect(out.status==int(ServoStatus::DataInvalid),"stale encoder rejected");
}
void rms_budget_lowers_authority() {
  Servo s; expect(s.configure(fixture()) && s.reset(0.,0.,0.,0.),"configure");
  double t=0.;
  const auto out=run(s,t,6000,0.,.05);  // stuck 2.9 deg short for 6 s
  expect(out.cap==fixture().rms_limit && std::abs(out.limited)<=fixture().rms_limit+1e-9,"RMS budget caps authority");
}
void stall_rocks_against_the_push() {
  auto p=fixture();
  p.stall_error=.005; p.stall_speed=.009; p.stall_current=.6; p.stall_time_s=.3; p.rock_current=.4; p.rock_s=.025;
  Servo s; expect(s.configure(p) && s.reset(0.,0.,0.,0.),"configure");
  double t=0.;
  bool rocked=false,reversed=false;
  for(int k=0;k<700;++k) {
    const auto out=run(s,t,1,0.,.03);
    if(out.rocking) {
      rocked=true; expect(out.requested<0,"rock requests force against the push");
      reversed|=out.limited<0;  // the slew limit takes a few ms to swing the output
    }
  }
  expect(rocked && reversed,"stall recovery triggered and reversed the applied current");
}
void friction_feedforward_follows_the_reference_only() {
  Servo s; expect(s.configure(fixture()) && s.reset(0.,0.,0.,0.),"configure");
  expect(s.friction(0.)==0.,"no friction feedforward at rest");
  expect(s.friction(.1)>.29 && s.friction(-.1)<-.29,"directional friction feedforward when moving");
}
}

int main() {
  try {
    crosstalk_is_removed_from_the_position();
    stiff_gains_stay_quiet_with_compensated_crosstalk();
    following_error_stops_the_servo();
    stale_encoder_is_data_invalid();
    rms_budget_lowers_authority();
    stall_rocks_against_the_push();
    friction_feedforward_follows_the_reference_only();
  } catch(const std::exception& e) { std::cerr<<"FAIL: "<<e.what()<<'\n'; return 1; }
  std::cout<<"servo tests passed\n"; return 0;
}
