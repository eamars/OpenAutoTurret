#include "simulate.hpp"
#include "plant.hpp"
#include "position_loop.hpp"
#include "session_parts.hpp"
#include "servo.hpp"
#include "servo_config.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <stdexcept>

namespace ota::axis {
namespace {
ReferenceTable reference(const YAML::Node& r) {
  const auto t=r["t"].as<std::vector<double>>(),q=r["q"].as<std::vector<double>>(),
             v=r["v"].as<std::vector<double>>(),a=r["a"].as<std::vector<double>>();
  if (t.size()<2 || q.size()!=t.size() || v.size()!=t.size() || a.size()!=t.size())
    throw std::runtime_error("DATA_INVALID: simulation reference");
  ReferenceTable table;
  for (std::size_t k=0;k<t.size();++k) table.add({t[k],q[k],v[k],a[k]});
  return table;
}
LogSweep sweep(const YAML::Node& x,const char* amplitude_key) {
  LogSweep s;
  if (!x) return s;
  s.amplitude=x[amplitude_key].as<double>(); s.f0=x["f0_hz"].as<double>(); s.f1=x["f1_hz"].as<double>();
  s.begin_s=x["begin_s"].as<double>(); s.duration_s=x["duration_s"].as<double>();
  return s;
}

SimulationResult yaw(const YAML::Node& request) {
  auto servo_parameters=servo_from_yaml(request["servo"]);
  const auto plant_parameters=yaw_plant_from_yaml(request["plant"]);
  const auto table=reference(request["reference"]);
  const auto excitation=sweep(request["excitation"],"amplitude_A");
  std::vector<std::array<double,4>> schedule;
  if (const auto s=request["gain_schedule"])
    for (const auto& g:s) schedule.push_back({g["begin_s"].as<double>(),g["kq"].as<double>(),g["kv"].as<double>(),g["ki"].as<double>()});
  const double start=request["start_position_rad"].as<double>(0.);
  const double hold_after=request["hold_after_s"].as<double>(1.5);
  const double speed_limit=request["speed_limit_rad_s"].as<double>(1.75);
  const double processing=request["processing_delay_s"].as<double>(2e-4);
  const double period=request["encoder_period_s"].as<double>(1e-3);
  const double cap=servo_parameters.current_cap;
  OscillationMonitor oscillation(request["oscillation_limit_A"].as<double>(0.));
  double last_step=-1.,last_rock=-1.;

  YawPlant plant(plant_parameters,start);
  Servo servo;
  if (!servo.configure(servo_parameters)) throw std::runtime_error("DATA_INVALID: servo parameters");
  SimulationResult result;
  result.columns={"t","qr","vr","ar","q_true","v_true","q_meas","u","excitation","q_hat","v_hat","integral",
                  "friction","rocking","stalls","current","saturated","requested","rms","cap"};
  // Control begins on the first receipt; its reading is the reference origin.
  const double first_receipt=period+plant_parameters.encoder_delay_s;
  plant.advance(period);
  const double origin=plant.reading(plant.position(),first_receipt);
  plant.advance(first_receipt);
  if (!servo.reset(first_receipt,origin,0.,0.)) throw std::runtime_error("DATA_INVALID: servo reset");
  std::deque<std::pair<double,double>> speed_history;
  std::size_t next_gain=0;
  result.status="COMPLETE";
  const double end=table.duration()+hold_after;
  for (int k=1;;++k) {
    const double sampled=k*period, receipt=sampled+plant_parameters.encoder_delay_s;
    plant.advance(sampled);
    const double q_sampled=plant.position();
    plant.advance(receipt);
    const double q_meas=plant.reading(q_sampled,receipt);
    const double since=receipt-first_receipt;
    if (since>=end) break;
    if (!servo.observe_encoder(receipt,q_meas)) { result.status="MEASUREMENT_LIMITED: encoder update rejected"; break; }
    const double now=receipt+processing;
    plant.advance(now);
    while (next_gain<schedule.size() && since>=schedule[next_gain][0]) {
      const auto& g=schedule[next_gain++]; servo.set_gains(g[1],g[2],g[3]);
    }
    const auto r=table.at(since);
    const auto out=servo.step(now,origin+r.position,r.velocity,r.acceleration);
    if (out.status==int(ServoStatus::FollowingError)) { result.status="HARD_ABORT: servo following error limit"; break; }
    if (out.status!=int(ServoStatus::Ok)) { result.status="MEASUREMENT_LIMITED: servo sensor data stale or invalid"; break; }
    speed_history.push_back({now,q_meas});
    while (speed_history.size()>2 && now-speed_history[1].first>=0.02) speed_history.pop_front();
    const double encoder_speed=now-speed_history.front().first>=0.02?
      (q_meas-speed_history.front().second)/(now-speed_history.front().first):0.;
    if (out.rocking) last_rock=now;
    if (oscillation.update(last_step<0?0.:now-last_step,out.limited,last_rock>=0 && now-last_rock<0.2)) {
      result.status="HARD_ABORT: servo oscillation"; break;
    }
    last_step=now;
    const double x=excitation.at(since);
    const double sent=std::clamp(out.limited+x,-cap,cap);
    plant.command(now,sent); servo.acknowledge(true,sent);
    result.rows.push_back({since,origin+r.position,r.velocity,r.acceleration,plant.position(),plant.velocity(),q_meas,sent,x,
                           out.position,out.velocity,out.integral,out.friction,double(out.rocking),double(out.stall_events),
                           plant.current(),double(out.saturated),out.requested,out.rms,out.cap});
    if (std::abs(encoder_speed)>speed_limit) { result.status="HARD_ABORT: yaw speed limit"; break; }
  }
  result.learned=servo_to_json(servo.learned());
  return result;
}

SimulationResult pitch(const YAML::Node& request) {
  const auto l=request["loop"];
  PositionLoop loop;
  if (!loop.configure({l["kp_per_s"].as<double>(),l["ki_per_s2"].as<double>(),l["integral_clamp_rad_s"].as<double>(),
                       l["speed_limit_rad_s"].as<double>()})) throw std::runtime_error("DATA_INVALID: pitch loop");
  const auto plant_parameters=pitch_plant_from_yaml(request["plant"]);
  const auto table=reference(request["reference"]);
  const auto excitation=sweep(request["excitation"],"amplitude_rad_s");
  std::vector<std::array<double,3>> schedule;
  if (const auto s=request["gain_schedule"])
    for (const auto& g:s) schedule.push_back({g["begin_s"].as<double>(),g["kp"].as<double>(),g["ki"].as<double>()});
  const double centre=request["start_position_rad"].as<double>(0.);
  const double hold_after=request["hold_after_s"].as<double>(1.);
  const double period=l["command_period_s"].as<double>(1e-3);
  const double following=l["following_error_rad"].as<double>(0.1);
  const double limit=l["speed_limit_rad_s"].as<double>();
  PitchPlant plant(plant_parameters,centre);
  SimulationResult result;
  result.columns={"t","qr","vr","ar","q_true","v_true","q_meas","cmd","excitation","integral"};
  result.status="COMPLETE";
  double pose=plant.reading(), last=-1;
  std::deque<std::pair<double,double>> replies;  // (receipt time, position)
  std::size_t next_gain=0;
  const double end=table.duration()+hold_after;
  for (int k=0;;++k) {
    const double now=k*period;
    if (now>=end) break;
    plant.advance(now);
    while (!replies.empty() && replies.front().first<=now) { pose=replies.front().second; replies.pop_front(); }
    while (next_gain<schedule.size() && now>=schedule[next_gain][0]) {
      const auto& g=schedule[next_gain++]; loop.set_gains(g[1],g[2]);
    }
    const auto r=table.at(now,true);
    const double q_ref=centre+r.position;
    if (std::abs(q_ref-pose)>following) { result.status="HARD_ABORT: pitch servo following error"; break; }
    const double x=excitation.at(now);
    const double speed=std::clamp(loop.step(last<0?0.:now-last,q_ref,r.velocity,pose)+x,-limit,limit);
    last=now;
    plant.command(now,speed);
    // The drive answers each command; the reply carries the position when it is sent.
    plant.advance(now+plant_parameters.reply_delay_s);
    replies.push_back({now+plant_parameters.reply_delay_s,plant.reading()});
    result.rows.push_back({now,q_ref,r.velocity,r.acceleration,plant.position(),plant.velocity(),pose,speed,x,loop.integral()});
  }
  return result;
}
}

SimulationResult simulate(const YAML::Node& request) {
  const auto axis=request["axis"].as<std::string>();
  if (axis=="yaw") return yaw(request);
  if (axis=="pitch") return pitch(request);
  throw std::runtime_error("DATA_INVALID: simulation axis");
}
}
