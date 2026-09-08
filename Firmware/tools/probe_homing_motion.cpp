// Exercise the production loop with deliberately uncommanded encoder motion.
// The injected plant does not establish physical stopping or load support.
#include <iostream>
#include <limits>
#include <spdlog/spdlog.h>
#include "control/control_loop.hpp"
#include "sim/sim_motor_backend.hpp"
using namespace ota;
namespace {
class DisturbedPlant : public sim::SimMotorBackend {
 public:
  bool pending = false;
  int transitions = 0;
  AxisId timestamp_axis=AxisId::Pitch;
  TimeNs timestamp_bias=0;
  AxisSnapshot snapshot(AxisId axis, TimeNs now) override {
    auto fb=SimMotorBackend::snapshot(axis,now);
    if(axis==timestamp_axis) fb.rx_ns+=timestamp_bias;
    return fb;
  }
  Transition transition_mode(AxisId a, bool pos, double limit, TimeNs now,
      std::string& err, double ki = -1, double kp = 1, bool check_displacement = true) override {
    ++transitions;
    if (pending) return Transition::Pending;
    return SimMotorBackend::transition_mode(a,pos,limit,now,err,ki,kp,check_displacement);
  }
};
struct Fixture {
  DisturbedPlant* plant = new DisturbedPlant;
  ControlLoop loop{ControlLoop::Config{}, std::unique_ptr<MotorBackend>(plant)};
  TimeNs now = 1'000'000'000;
  Fixture(AxisId active, int dir, bool pending) {
    plant->pending = pending;
    for (auto a : {AxisId::Pitch,AxisId::Yaw}) plant->set_stops(a,-2,2);
    HomingPlanConfig c;
    c.homing.coarse_speed_rad_s = 5*kDeg2Rad;
    c.homing.fine_speed_rad_s = c.homing.backoff_speed_rad_s = 3*kDeg2Rad;
    c.move_speed_rad_s = 3*kDeg2Rad;
    std::string err;
    HomingAction action{HomingActionType::HomeEndpoint,active,
        dir>0 ? Endpoint::Upper : Endpoint::Lower,Precision::Fine,0};
    if (!loop.start_homing(HomingPlan({action},c),err)) throw std::runtime_error(err);
    for (int i=0;i<8;++i) tick();
  }
  void tick() { loop.step(now,5'000'000); now+=5'000'000; }
  bool latched() {
    if (loop.phase()!=Phase::Fault) return false;
    const int before=plant->transitions;
    plant->pending=false;
    plant->timestamp_bias=0;
    for(auto axis:{AxisId::Pitch,AxisId::Yaw}) {
      plant->set_feedback_ok(axis,true); plant->set_faults(axis,0);
      plant->set_temp(axis,25); plant->set_position(axis,0);
    }
    for (int i=0;i<100;++i) tick();
    std::string err;
    return loop.phase()==Phase::Fault && plant->transitions==before &&
        plant->snapshot(AxisId::Pitch,now).disabled &&
        plant->snapshot(AxisId::Yaw,now).disabled &&
        !loop.start_homing(HomingPlan({},{}),err);
  }
};
bool run(AxisId active, int dir, const std::string& condition) {
  Fixture f(active,dir,condition=="pending drift");
  const AxisId other=active==AxisId::Yaw ? AxisId::Pitch : AxisId::Yaw;
  const AxisId disturbed=condition=="inactive drift" ? other : active;
  int cycles=0;
  while(f.loop.phase()==Phase::Homing && cycles++<500) {
    if(condition=="nonfinite") f.plant->set_position(disturbed,std::numeric_limits<double>::quiet_NaN());
    else if(condition=="stale") f.plant->set_feedback_ok(disturbed,false);
    else if(condition=="future" || condition=="regressing") {
      f.plant->timestamp_axis=disturbed;
      f.plant->timestamp_bias=condition=="future" ? 20'000'000 : -20'000'000;
    }
    else if(condition=="hot") f.plant->set_temp(disturbed,125);
    else if(condition=="fault") f.plant->set_faults(disturbed,1);
    else if(condition!="normal") {
      const double rate=condition=="overspeed" ? 20 : condition=="wrong direction" ? -12 : 1;
      f.plant->set_position(disturbed,f.plant->position(disturbed)+dir*rate*kDeg2Rad*.005);
    }
    f.tick();
  }
  const bool ok=condition=="normal" ? f.loop.phase()==Phase::Homing : f.latched();
  std::cout<<axis_name(active)<<" dir="<<dir<<" condition="<<condition
           <<" ms="<<cycles*5<<" pass="<<ok<<" reason='"<<f.loop.fault_reason()<<"'\n";
  return ok;
}
bool backoff_clearance(AxisId axis, int dir) {
  HomingParams p; p.contact.contact_dwell_ms=100;
  HomingController home(axis,dir,p);
  HomingFeedback fb; fb.t_ns=1'000'000'000; fb.torque_nm=5*dir;
  for(int i=0;i<400 && home.state()!=AxisHomeState::Backoff;++i) {
    home.step(fb); fb.t_ns+=5'000'000;
  }
  if(home.state()!=AxisHomeState::Backoff) return false;
  fb.pos_rad=-3*kDeg2Rad*dir; fb.torque_nm=0;
  bool rearmed=false;
  for(int i=0;i<300;++i) { rearmed |= home.step(fb).rearm_speed_mode; fb.t_ns+=5'000'000; }
  const bool ok=!rearmed && home.state()==AxisHomeState::Backoff;
  std::cout<<axis_name(axis)<<" dir="<<dir<<" shortened_5deg_backoff_rejected="<<ok<<'\n';
  return ok;
}
bool known_contact_bound(AxisId axis, int dir) {
  HomingParams p; p.contact.contact_dwell_ms=100;
  HomingController home(axis,dir,p);
  HomingFeedback fb; fb.t_ns=1'000'000'000; fb.torque_nm=5*dir;
  for(int i=0;i<400 && home.state()!=AxisHomeState::Backoff;++i) {
    home.step(fb); fb.t_ns+=5'000'000;
  }
  fb.pos_rad=-5*kDeg2Rad*dir; fb.torque_nm=0;
  for(int i=0;i<400 && home.state()!=AxisHomeState::ApproachFine;++i) {
    home.step(fb); fb.t_ns+=5'000'000;
  }
  if(home.state()!=AxisHomeState::ApproachFine) return false;
  // The previously observed contact has disappeared. Fine home must not
  // traverse another 150 degrees trying to find a replacement obstruction.
  fb.pos_rad=.75*kDeg2Rad*dir;
  const auto ds=home.step(fb);
  const bool ok=home.terminal() && !home.result().valid && ds.hold;
  std::cout<<axis_name(axis)<<" dir="<<dir<<" missing_known_contact_aborts="<<ok<<'\n';
  return ok;
}
}
int main() {
  spdlog::set_level(spdlog::level::off);
  bool ok=true;
  for(auto axis:{AxisId::Pitch,AxisId::Yaw}) for(int dir:{-1,1})
  {
    ok=backoff_clearance(axis,dir)&&ok;
    ok=known_contact_bound(axis,dir)&&ok;
    for(const auto* condition:{"normal","pending drift","inactive drift","overspeed","wrong direction","nonfinite","stale","future","regressing","hot","fault"})
      ok=run(axis,dir,condition)&&ok;
  }
  return ok ? 0 : 1;
}
