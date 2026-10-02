#include "tracking_sim.hpp"
#include "plant.hpp"
#include "position_loop.hpp"
#include "servo.hpp"
#include "servo_config.hpp"
#include "tracker.hpp"
#include "tracking_config.hpp"
#include <algorithm>
#include <cmath>
#include <deque>
#include <memory>
#include <random>
#include <stdexcept>

namespace ota::track {
namespace {
constexpr int64_t kEpoch=1'000'000'000;  // simulation t=0 is 1 s on the tracker clock
int64_t clock_ns(double t) { return kEpoch+static_cast<int64_t>(std::llround(t*1e9)); }

std::array<double,2> pair(const YAML::Node& n,const char* key,std::array<double,2> fallback={0.,0.}) {
  if (!n || !n[key]) return fallback;
  return {n[key][0].as<double>(),n[key][1].as<double>()};
}

// Target truth: base-frame LOS with piecewise-constant angular acceleration. A segment may
// start with a jump (position), a new rate (a sudden stop or reversal) or a new identity.
class Truth {
 public:
  explicit Truth(const YAML::Node& n) {
    theta_={n["start"]["az"].as<double>(),n["start"]["el"].as<double>()};
    omega_=pair(n["start"],"rate");
    double t=0; auto th=theta_; auto om=omega_; uint64_t id=n["start"]["identity"].as<uint64_t>(1);
    for (const auto& s:n["segments"]) {
      Segment g;
      g.begin=t; g.end=t+s["duration"].as<double>();
      if (!(g.end>g.begin)) throw std::runtime_error("DATA_INVALID: truth segment duration");
      if (s["jump"]) { const auto j=pair(s,"jump"); th[0]+=j[0]; th[1]+=j[1]; }
      if (s["rate"]) om=pair(s,"rate");
      if (s["identity"]) id=s["identity"].as<uint64_t>();
      g.accel=pair(s,"accel"); g.theta=th; g.omega=om; g.identity=id;
      const double d=g.end-g.begin;
      for (int i=0;i<2;++i) { th[i]+=om[i]*d+g.accel[i]*d*d/2; om[i]+=g.accel[i]*d; }
      segments_.push_back(g); t=g.end;
    }
    if (segments_.empty()) throw std::runtime_error("DATA_INVALID: truth has no segments");
  }
  void at(double t,std::array<double,2>& theta,std::array<double,2>& omega,uint64_t& identity) const {
    const Segment* g=&segments_.back();
    for (const auto& s:segments_) if (t<s.end) { g=&s; break; }
    const double d=std::max(0.,std::min(t,g->end)-g->begin), extra=std::max(0.,t-g->end);
    for (int i=0;i<2;++i) {
      const double w=g->omega[i]+g->accel[i]*d;
      theta[i]=g->theta[i]+g->omega[i]*d+g->accel[i]*d*d/2+w*extra;
      omega[i]=w;
    }
    identity=g->identity;
  }
 private:
  struct Segment { double begin=0,end=0; std::array<double,2> accel{},theta{},omega{}; uint64_t identity=1; };
  std::array<double,2> theta_{},omega_{};
  std::vector<Segment> segments_;
};

// Time-stamped scalar history with linear interpolation (the pose at an observation time).
class History {
 public:
  void add(double t,double q) {
    if (!h_.empty() && t<=h_.back().first) return;
    h_.push_back({t,q}); while (h_.size()>4000) h_.pop_front();
  }
  bool at(double t,double& q) const {
    if (h_.size()<2 || t<h_.front().first || t>h_.back().first) return false;
    const auto it=std::lower_bound(h_.begin(),h_.end(),t,[](const auto& a,double x) { return a.first<x; });
    if (it==h_.begin()) { q=it->second; return true; }
    const auto& b=*it; const auto& a=*(it-1);
    q=a.second+(b.second-a.second)*(t-a.first)/(b.first-a.first); return true;
  }
  double last() const { return h_.empty()?0.:h_.back().second; }
 private:
  std::deque<std::pair<double,double>> h_;
};

bool inside(const YAML::Node& windows,double t,double* value=nullptr) {
  if (!windows) return false;
  for (const auto& w:windows)
    if (t>=w[0].as<double>() && t<w[1].as<double>()) { if (value && w.size()>2) *value=w[2].as<double>(); return true; }
  return false;
}

struct Frame {
  double start=0, mid=0, reported=0, exposure=0, arrival=0;
  double noise_u=0, noise_v=0;
  bool dropped=false, delivered=false, superseded=false, out_of_view=false, accepted=false, recorded=false;
  double u_true=0, v_true=0, err=0, blur=0, u=0, v=0;
  uint64_t identity=0;
};
}

TrackingSimulation simulate_tracking(const YAML::Node& request) {
  TrackingSimulation out;
  const auto params=tracker_from_yaml(request["tracker"]);
  out.parameters=tracker_to_json(params);
  const auto geo_node=request["geometry"];
  geo::CameraIntrinsics in;
  in.fx=geo_node["intrinsics"]["fx"].as<double>(); in.fy=geo_node["intrinsics"]["fy"].as<double>();
  in.cx=geo_node["intrinsics"]["cx"].as<double>(); in.cy=geo_node["intrinsics"]["cy"].as<double>();
  in.width=geo_node["intrinsics"]["width"].as<int>(); in.height=geo_node["intrinsics"]["height"].as<int>();
  geo::TurretKinematics kin;
  const auto r=geo_node["R_PC"].as<std::vector<double>>();
  if (r.size()!=9) throw std::runtime_error("DATA_INVALID: R_PC");
  kin.R_PC=geo::Mat3{r[0],r[1],r[2],r[3],r[4],r[5],r[6],r[7],r[8]};
  const auto sight_v=geo_node["sight"].as<std::vector<double>>(std::vector<double>{0.,0.,1.});
  const geo::Vec3 sight{sight_v[0],sight_v[1],sight_v[2]};
  Travel travel;
  if (geo_node["travel"]["yaw"]) { travel.yaw_low=geo_node["travel"]["yaw"][0].as<double>(); travel.yaw_high=geo_node["travel"]["yaw"][1].as<double>(); }
  travel.pitch_low=geo_node["travel"]["pitch"][0].as<double>(); travel.pitch_high=geo_node["travel"]["pitch"][1].as<double>();
  Tracker tracker(params,kin,in,sight,travel);
  if (!tracker.ok()) throw std::runtime_error("DATA_INVALID: tracker parameters or geometry");
  const geo::CameraModel camera(in);
  double frame_u=in.cx, frame_v=in.cy;  // the framing pixel: where the sight axis images
  camera.ray_to_pixel(sight.normalized(),frame_u,frame_v);

  const Truth truth(request["target"]);
  const auto cam=request["camera"];
  const double frame_period=cam["frame_period_s"].as<double>(), exposure=cam["exposure_s"].as<double>();
  const double latency=cam["latency_s"].as<double>(), latency_jitter=cam["latency_jitter_s"].as<double>(0.);
  const double frame_jitter=cam["frame_jitter_s"].as<double>(0.), noise=cam["pixel_noise_px"].as<double>(0.);
  const double timing_error=cam["true_timing_offset_s"].as<double>(0.);
  std::mt19937_64 random(cam["seed"].as<uint64_t>(1));
  std::uniform_real_distribution<double> uniform(-1.,1.);
  std::normal_distribution<double> normal(0.,1.);
  const double duration=request["duration_s"].as<double>();
  const double control_period=request["control_period_s"].as<double>(params.level1.period_s);
  const int tick_ms=static_cast<int>(std::lround(control_period*1e3));
  if (tick_ms<1 || std::abs(tick_ms*1e-3-control_period)>1e-9) throw std::runtime_error("DATA_INVALID: control period must be whole ms");

  std::vector<Frame> frames;
  for (double t=cam["first_frame_s"].as<double>(0.02);t<duration;t+=frame_period) {
    Frame f;
    f.start=t+frame_jitter*uniform(random); f.exposure=exposure; f.mid=f.start+exposure/2;
    f.reported=f.start-timing_error;
    double extra=0; inside(cam["extra_delay"],f.mid,&extra);
    f.arrival=f.mid+latency+latency_jitter*uniform(random)+extra;
    f.dropped=inside(cam["drops"],f.mid);
    f.noise_u=noise*normal(random); f.noise_v=noise*normal(random);
    frames.push_back(f);
  }

  const bool ideal=request["actuator"].as<std::string>("servo")=="ideal";
  const auto start=pair(request["turret"],"start");
  axis::Servo servo; axis::PositionLoop loop;
  std::unique_ptr<axis::YawPlant> yaw_plant; std::unique_ptr<axis::PitchPlant> pitch_plant;
  axis::YawPlantParameters yp{}; axis::PitchPlantParameters pp{};
  double yaw_cap=0, processing=2e-4, pitch_limit=0;
  if (!ideal) {
    const auto sp=axis::servo_from_yaml(request["yaw"]["servo"]);
    if (!servo.configure(sp)) throw std::runtime_error("DATA_INVALID: yaw servo");
    yaw_cap=sp.current_cap;
    yp=axis::yaw_plant_from_yaml(request["yaw"]["plant"]);
    yaw_plant=std::make_unique<axis::YawPlant>(yp,start[0]);
    const auto l=request["pitch"]["loop"];
    pitch_limit=l["speed_limit_rad_s"].as<double>();
    if (!loop.configure({l["kp_per_s"].as<double>(),l["ki_per_s2"].as<double>(),l["integral_clamp_rad_s"].as<double>(),pitch_limit}))
      throw std::runtime_error("DATA_INVALID: pitch loop");
    pp=axis::pitch_plant_from_yaml(request["pitch"]["plant"]);
    pitch_plant=std::make_unique<axis::PitchPlant>(pp,start[1]);
  }
  History yaw_meas,pitch_meas,yaw_true,pitch_true;
  std::deque<std::pair<double,double>> replies;
  double pitch_pose=start[1], yaw_reading=start[0], sent_current=0, rms=0, pitch_cmd=0, last_pitch_step=-1;
  yaw_meas.add(-1e-3,start[0]); pitch_meas.add(-1e-3,start[1]); yaw_true.add(-1e-3,start[0]); pitch_true.add(-1e-3,start[1]);
  if (!ideal) {
    if (!servo.reset(0.,start[0],0.,0.)) throw std::runtime_error("DATA_INVALID: servo reset");
  }
  tracker.engage(clock_ns(0.),start,{0.,0.},{0.,0.});
  ReferenceSample sample=tracker.level1().last();
  const bool base_motion=params.target_motion;
  std::size_t next_record=0;
  out.tick_columns={"t","truth_az","truth_el","truth_waz","truth_wel","est_az","est_el","est_waz","est_wel","sig_waz","sig_wel",
    "goal_az","goal_el","goal_waz","goal_wel","ffw_az","ffw_el","fade","age","pos_valid","vel_valid",
    "qt_y","qt_p","vt_y","vt_p","goal_valid","qr_y","qr_p","vr_y","vr_p","ar_y","ar_p","jr_y","jr_p","flag_y","flag_p",
    "areq_y","areq_p","qm_y","qm_p","qtrue_y","qtrue_p","qT_y","qT_p","err_u","err_v","nis","weight","scale_az","scale_el",
    "accepted","rejected","downweighted","rate_limited","gap_resets","identity_changes","u_yaw","rms_yaw","cmd_pitch","motion","saturated"};
  out.frame_columns={"t_start","t_mid","t_reported","t_arrival","dropped","delivered","superseded","out_of_view",
    "u_true","v_true","err_px","blur_px","u_meas","v_meas","identity","accepted"};
  out.status="COMPLETE";

  // Pixel of the true target through the true camera pose at time t (false: not in front of the camera).
  const auto project=[&](double t,double& u,double& v) {
    std::array<double,2> th{},om{}; uint64_t id=0; truth.at(t,th,om,id);
    double qy,qp;
    if (!yaw_true.at(t,qy)) qy=yaw_true.last();
    if (!pitch_true.at(t,qp)) qp=pitch_true.last();
    const geo::Vec3 c=kin.base_to_ray(geo::TurretKinematics::los_to_base_ray(th[0],th[1]),qy,qp);
    if (c.z<=1e-6) { u=v=NAN; return false; }
    camera.ray_to_pixel(c,u,v); return true;
  };

  const int steps=static_cast<int>(std::floor(duration*1e3));
  for (int k=0;k<=steps;++k) {
    const double t=k*1e-3;
    const double saturation_now=[&]{ double x=0; return inside(request["saturate"],t,&x)?x:-1.; }();
    // ---- control tick: deliver the newest arrived frame (latest only), then Level 1
    if (k%tick_ms==0) {
      Frame* newest=nullptr;
      for (auto& f:frames)
        if (!f.dropped && !f.delivered && !f.superseded && f.arrival<=t) { if (newest) newest->superseded=true; newest=&f; }
      if (newest) {
        newest->delivered=true;
        double u,v; std::array<double,2> th{},om{}; uint64_t id=0; truth.at(newest->mid,th,om,id); newest->identity=id;
        if (project(newest->mid,u,v) && u>=0 && v>=0 && u<=in.width && v<=in.height) {
          PixelObservation z; z.sensor_ns=clock_ns(newest->reported); z.exposure_s=newest->exposure;
          z.u=std::clamp(u+newest->noise_u,0.,double(in.width)); z.v=std::clamp(v+newest->noise_v,0.,double(in.height));
          z.identity=id; newest->u=z.u; newest->v=z.v;
          const double t_o=(tracker.observation_time(z)-kEpoch)*1e-9;
          double qy,qp;
          if (yaw_meas.at(t_o,qy) && pitch_meas.at(t_o,qp)) newest->accepted=tracker.observe(z,qy,qp);
        } else newest->out_of_view=true;
      }
      tracker.parameters().target_motion=base_motion && !inside(request["velocity_unavailable"],t);
      const std::array<double,2> q_measured={yaw_meas.last(),pitch_meas.last()};
      const auto rec=tracker.tick(clock_ns(t),q_measured);
      sample=rec.reference;
      std::array<double,2> th{},om{}; uint64_t id=0; truth.at(t,th,om,id);
      LosGoal truth_goal; truth_goal.position_valid=truth_goal.velocity_valid=true; truth_goal.theta=th; truth_goal.omega=om;
      const auto ideal_goal=tracker.joint_goal(truth_goal,rec.reference.q);
      double u=NAN,v=NAN; project(t,u,v);
      const auto& x=tracker.estimator().state(); const auto& d=tracker.estimator().diagnostics();
      out.ticks.push_back({t,th[0],th[1],om[0],om[1],x[0].theta,x[1].theta,x[0].omega,x[1].omega,
        std::sqrt(std::max(0.,x[0].vv)),std::sqrt(std::max(0.,x[1].vv)),
        rec.los.theta[0],rec.los.theta[1],rec.los.omega[0],rec.los.omega[1],rec.los.ff_weight[0],rec.los.ff_weight[1],
        rec.los.fade,rec.los.age_s,double(rec.los.position_valid),double(rec.los.velocity_valid),
        rec.joint.q[0],rec.joint.q[1],rec.joint.v[0],rec.joint.v[1],double(rec.joint.valid),
        sample.q[0],sample.q[1],sample.v[0],sample.v[1],sample.a[0],sample.a[1],sample.j[0],sample.j[1],
        double(sample.flags[0]),double(sample.flags[1]),sample.a_request[0],sample.a_request[1],
        q_measured[0],q_measured[1],yaw_true.last(),pitch_true.last(),ideal_goal.q[0],ideal_goal.q[1],
        u-frame_u,v-frame_v,d.nis,d.weight,d.scale[0],d.scale[1],double(d.accepted),double(d.rejected),double(d.downweighted),
        double(d.rate_limited),double(d.gap_resets),double(tracker.identity_changes()),sent_current,rms,pitch_cmd,
        double(tracker.parameters().target_motion),saturation_now>=0?1.:0.});
    }
    // ---- actuators (1 kHz)
    if (ideal) {
      double q,v,a;
      if (sample.at(0,clock_ns(t),q,v,a)) { yaw_true.add(t,q); yaw_meas.add(t,q); }
      if (sample.at(1,clock_ns(t),q,v,a)) { pitch_true.add(t,q); pitch_meas.add(t,q); }
    } else {
      // Yaw: encoder sampled at t, received after its delay; the servo steps on the receipt.
      const double receipt=t+yp.encoder_delay_s, now=receipt+processing;
      yaw_plant->advance(t);
      const double q_sampled=yaw_plant->position();
      yaw_plant->advance(receipt);
      yaw_reading=yaw_plant->reading(q_sampled,receipt);
      if (!servo.observe_encoder(receipt,yaw_reading)) { out.status="MEASUREMENT_LIMITED: encoder update rejected"; break; }
      yaw_meas.add(receipt,yaw_reading);
      yaw_plant->advance(now);
      double q=sample.q[0],v=0,a=0;
      if (!sample.at(0,clock_ns(now),q,v,a)) { v=a=0; }
      const auto o=servo.step(now,q,v,a);
      if (o.status==int(axis::ServoStatus::FollowingError)) { out.status="HARD_ABORT: servo following error limit"; break; }
      double sent=std::clamp(o.limited,-yaw_cap,yaw_cap);
      if (saturation_now>=0) sent=std::clamp(sent,-saturation_now,saturation_now);
      yaw_plant->command(now,sent); servo.acknowledge(true,sent);
      sent_current=sent; rms=o.rms;
      yaw_true.add(now,yaw_plant->position());
      // Pitch: the host loop on the latest type-2 reply, SpdRef answered by a reply.
      pitch_plant->advance(t);
      while (!replies.empty() && replies.front().first<=t) { pitch_pose=replies.front().second; pitch_meas.add(replies.front().first,pitch_pose); replies.pop_front(); }
      if (!sample.at(1,clock_ns(t),q,v,a)) { q=sample.q[1]; v=a=0; }
      double speed=std::clamp(loop.step(last_pitch_step<0?0.:t-last_pitch_step,q,v,pitch_pose),-pitch_limit,pitch_limit);
      if (saturation_now>=0) speed=std::clamp(speed,-saturation_now,saturation_now);
      last_pitch_step=t; pitch_cmd=speed;
      pitch_plant->command(t,speed);
      pitch_plant->advance(t+pp.reply_delay_s);
      replies.push_back({t+pp.reply_delay_s,pitch_plant->reading()});
      pitch_true.add(t+pp.reply_delay_s,pitch_plant->position());
    }
    // ---- frames whose exposure has ended: the true framing error and blur during the exposure
    while (next_record<frames.size() && frames[next_record].start+frames[next_record].exposure<=t) {
      auto& f=frames[next_record++];
      double u,v; std::array<double,2> th{},om{}; truth.at(f.mid,th,om,f.identity);
      if (project(f.mid,f.u_true,f.v_true)) f.err=std::hypot(f.u_true-frame_u,f.v_true-frame_v); else f.err=NAN;
      double path=0, pu=NAN, pv=NAN;
      for (int s=0;s<=8;++s) {
        if (!project(f.start+f.exposure*s/8.,u,v)) { path=NAN; break; }
        if (s) path+=std::hypot(u-pu,v-pv);
        pu=u; pv=v;
      }
      f.blur=path; f.recorded=true;
    }
  }
  for (const auto& f:frames) {
    if (!f.recorded) continue;
    out.frames.push_back({f.start,f.mid,f.reported,f.arrival,double(f.dropped),double(f.delivered),double(f.superseded),
                          double(f.out_of_view),f.u_true,f.v_true,f.err,f.blur,f.u,f.v,double(f.identity),double(f.accepted)});
  }
  return out;
}
}
