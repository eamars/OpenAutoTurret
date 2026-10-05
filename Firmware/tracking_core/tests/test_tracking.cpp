// tracking_core unit tests: the estimator's time and noise semantics, the coast and FF
// weight laws, Level 1's single integral and its bounds, and the tracker's interface
// invariants (ego-motion removed once, one update per frame, identity resets motion,
// observation time not replaced by arrival). The 14 closed-loop scenarios are in
// tools/tracking (they run the simulator).
#include "estimator.hpp"
#include "level1.hpp"
#include "control/reference_limiter.hpp"
#include "tracker.hpp"
#include "tracking_config.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>

using namespace ota::track;

namespace {
int failures=0;
void run(const char* name,void (*test)()) { std::printf("%s\n",name); std::fflush(stdout); test(); }
void check(bool ok,const char* what) {
  if (!ok) { std::printf("FAIL: %s\n",what); ++failures; }
}
constexpr int64_t kS=1'000'000'000;
constexpr double kDeg=M_PI/180;
std::array<double,2> r_measured(const ReferenceSample& r) { return {r.q[0],r.q[1]}; }

EstimatorParameters estimator_parameters() {
  EstimatorParameters p;
  p.process_density={0.05,0.05}; p.measurement_floor={1e-7,1e-7}; p.initial_rate_sigma=0.5;
  p.scale_max={50.,50.}; p.scale_tau_s=0.066; p.rate_domain=2.0; p.fresh_s=0.1; p.horizon_s=0.5;
  p.position_sigma_limit=0;
  return p;
}

Level1Parameters level1_parameters() {
  Level1Parameters p;
  for (auto& a:p.axis) { a.lambda=6; a.v_max=20*kDeg; a.a_max=60*kDeg; a.j_max=300*kDeg; a.lead_limit=2*kDeg; a.q_min=a.q_max=0; }
  p.axis[1].q_min=-1.371; p.axis[1].q_max=-0.255;
  p.period_s=0.005; p.valid_s=0.02;
  return p;
}

void estimator_tracks_constant_rate_with_irregular_frames() {
  TargetEstimator e; check(e.configure(estimator_parameters()),"estimator configures");
  std::mt19937 g(3); std::uniform_real_distribution<double> gap(0.02,0.06); std::normal_distribution<double> n(0,0.0005);
  double t=1;
  for (int k=0;k<200;++k) {
    t+=gap(g);
    LosObservation z; z.t_ns=int64_t(t*kS); z.az=0.3*t+n(g); z.el=-0.1*t+n(g); z.var_az=z.var_el=0.0005*0.0005;
    check(e.update(z),"constant-rate frame accepted");
  }
  check(std::abs(e.state()[0].omega-0.3)<0.01 && std::abs(e.state()[1].omega+0.1)<0.01,"rate converges under irregular frame intervals");
  const auto goal=e.query(int64_t((t+0.05)*kS));
  check(goal.ff_weight[0]>0.99 && std::abs(goal.theta[0]-(e.state()[0].theta+0.3*0.05))<0.002,"confident rate: full FF and age prediction");
  check(!goal.acceleration_valid,"no acceleration estimate is ever claimed");
  // The query is a copy: it must not advance the committed state or its time.
  check(e.state_ns()==int64_t(t*kS),"query leaves the state at the observation time");
}

void estimator_rejects_repeated_and_old_frames() {
  TargetEstimator e; e.configure(estimator_parameters());
  LosObservation z; z.t_ns=2*kS; z.var_az=z.var_el=1e-6;
  check(e.update(z),"first frame");
  check(!e.update(z),"the same frame twice is not a second observation");
  z.t_ns=2*kS-1000; check(!e.update(z),"an older frame is rejected");
  z.t_ns=2*kS+33'000'000; z.az=NAN; check(!e.update(z),"invalid numbers rejected");
}

void estimator_follows_a_sudden_stop() {
  // 30 deg/s, then stopped: the robust update and manoeuvre scale bring the rate to zero
  // within a few frames instead of rejecting the new truth.
  TargetEstimator e; e.configure(estimator_parameters());
  double t=1, q=0;
  for (int k=0;k<60;++k) { t+=0.033; q+=30*kDeg*0.033; LosObservation z; z.t_ns=int64_t(t*kS); z.az=q; z.var_az=z.var_el=1e-7; e.update(z); }
  check(std::abs(e.state()[0].omega-30*kDeg)<0.01,"moving rate learned");
  int frames=0;
  while (std::abs(e.state()[0].omega)>3*kDeg && frames<30) {
    t+=0.033; ++frames; LosObservation z; z.t_ns=int64_t(t*kS); z.az=q; z.var_az=z.var_el=1e-7; e.update(z);
  }
  check(frames<=8,"stop recognised within 8 frames");
  check(e.diagnostics().rejected==0,"no stop frame rejected");
  check(e.diagnostics().downweighted>0,"the stop used the robust update");
}

void estimator_keeps_a_fast_subject_moving() {
  // Station 2026-10-02 21:14: a person at 60-80 deg/s against a 57 deg/s rate domain. The old
  // estimator zeroed the rate on every other frame, so the goal velocity alternated 0 / 70 deg/s
  // at 15 Hz, the reference fell 13 deg behind and overshot the stop. Beyond the domain the rate
  // is held at its edge: the subject is still moving that way.
  TargetEstimator e; e.configure(estimator_parameters());
  const double rate=1.5*estimator_parameters().rate_domain;
  double t=1, worst=1e9;
  bool weight_dropped=false;
  for (int k=0;k<90;++k) {
    t+=0.033; LosObservation z; z.t_ns=int64_t(t*kS); z.az=rate*(t-1); z.var_az=z.var_el=1e-7;
    check(e.update(z),"fast frame accepted");
    if (k>=15) {
      worst=std::min(worst,e.state()[0].omega);
      weight_dropped|=e.query(z.t_ns+int64_t(0.06*kS)).ff_weight[0]<0.99;
    }
  }
  check(worst>=0.99*estimator_parameters().rate_domain,"the rate stays at the domain's edge, never back to zero");
  check(!weight_dropped,"the velocity feedforward stays on");
  check(e.diagnostics().rate_limited>0,"the limitation is counted");
}

void velocity_weight_and_coast_laws() {
  check(velocity_weight(0.,1.)==0 && velocity_weight(1.,1.)==0 && std::abs(velocity_weight(3.,1.)-1)<1e-12,
        "w=0 below 1 sigma, 1 from 3 sigma");
  double last=0; bool monotone=true;
  for (double r=0;r<4;r+=0.01) { const double w=velocity_weight(r,1.); monotone&=w>=last-1e-15; last=w; }
  check(monotone,"weight is monotone and continuous");
  check(coast_fade(0.05,0.1,0.5)==1 && coast_fade(0.6,0.1,0.5)==0 && std::abs(coast_fade(0.3,0.1,0.5)-0.5)<1e-12,"fade law");
  // The integral of the fade is the travel: check against a fine numerical integral.
  double sum=0; for (double a=0;a<0.7;a+=1e-5) sum+=coast_fade(a+5e-6,0.1,0.5)*1e-5;
  check(std::abs(sum-coast_integral(0.7,0.1,0.5))<1e-6,"coast travel is the integral of the fade");
  check(coast_integral(0.2,0.1,0.1)==0.1,"zero fade span holds without dividing by zero");
}

void estimator_coast_is_bounded() {
  TargetEstimator e; e.configure(estimator_parameters());
  double t=1;
  for (int k=0;k<60;++k) { t+=0.033; LosObservation z; z.t_ns=int64_t(t*kS); z.az=0.5*(t-1); z.var_az=z.var_el=1e-7; e.update(z); }
  const auto at_h=e.query(int64_t((t+0.5)*kS)), beyond=e.query(int64_t((t+0.51)*kS));
  check(at_h.position_valid && at_h.omega[0]==0,"at the horizon the rate has faded to zero");
  check(std::abs(at_h.theta[0]-(e.state()[0].theta+e.state()[0].omega*0.3))<1e-3,"travel = rate x (F + (H-F)/2)");
  check(!beyond.position_valid,"beyond the horizon there is no target");
  const auto fb=e.query(int64_t((t+0.05)*kS),false);
  check(fb.omega[0]==0 && fb.theta[0]==e.state()[0].theta,"FB-only: no FF and no motion extrapolation");
}

void level1_is_one_integral() {
  Level1Generator g; check(g.configure(level1_parameters()),"level1 configures");
  g.reset(kS,{0.,-0.8},{0.,0.},{0.,0.});
  JointGoal goal; goal.valid=true; goal.velocity_valid={true,true};
  double max_v=0, max_a=0, max_j=0; bool consistent=true;
  ReferenceSample prev=g.last();
  for (int k=1;k<=1200;++k) {
    const int64_t t=kS+int64_t(k)*5'000'000;
    const double s=k*0.005;
    goal.q={0.5*std::sin(0.8*s),-0.8+0.2*std::sin(1.3*s)}; goal.v={0.4*std::cos(0.8*s),0.26*std::cos(1.3*s)};
    const auto r=g.step(t,goal,r_measured(prev));
    // The published state at this tick is the previous segment integrated to now.
    double q,v,a; prev.at(0,t,q,v,a);
    consistent&=std::abs(q-r.q[0])<1e-12 && std::abs(v-r.v[0])<1e-12 && std::abs(a-r.a[0])<1e-12;
    for (int i=0;i<2;++i) { max_v=std::max(max_v,std::abs(r.v[i])); max_a=std::max(max_a,std::abs(r.a[i])); max_j=std::max(max_j,std::abs(r.j[i])); }
    prev=r;
  }
  check(consistent,"q/v/a at each tick equal the previous segment's polynomial (one integral)");
  check(max_v<=20*kDeg+1e-12 && max_a<=60*kDeg+1e-9 && max_j<=300*kDeg+1e-9,"speed, acceleration and jerk bounds hold");
}

void level1_tracks_ramp_without_lag() {
  // With the target's own velocity fed forward the steady ramp error is zero (no acceleration term needed).
  Level1Generator g; g.configure(level1_parameters());
  g.reset(kS,{0.,-0.8},{0.,0.},{0.,0.});
  JointGoal goal; goal.valid=true; goal.velocity_valid={true,true};
  ReferenceSample r;
  for (int k=1;k<=1000;++k) { const double s=k*0.005; goal.q={0.1*s,-0.8}; goal.v={0.1,0.}; r=g.step(kS+int64_t(k)*5'000'000,goal,{r.q[0],r.q[1]}); }
  check(std::abs(r.q[0]-0.1*5)<1e-6 && std::abs(r.v[0]-0.1)<1e-6,"FF+FB: zero steady ramp lag");
}

void level1_respects_lead_limit_and_resumes() {
  Level1Generator g; g.configure(level1_parameters());
  g.reset(kS,{0.,-0.8},{0.,0.},{0.,0.});
  JointGoal goal; goal.valid=true; goal.velocity_valid={true,true};
  ReferenceSample r;
  // The axis is stuck at 0 while the target runs away at 10 deg/s for 3 s.
  for (int k=1;k<=600;++k) { const double s=k*0.005; goal.q={10*kDeg*s,-0.8}; goal.v={10*kDeg,0.}; r=g.step(kS+int64_t(k)*5'000'000,goal,{0.,-0.8}); }
  const double held=r.q[0];
  for (int k=601;k<=800;++k) { const double s=k*0.005; goal.q={10*kDeg*s,-0.8}; goal.v={10*kDeg,0.}; r=g.step(kS+int64_t(k)*5'000'000,goal,{0.,-0.8}); }
  // Bound: the lead limit plus the worst braking distance (full speed, full acceleration outward).
  const double braking=ota::control::stopping_distance_rad(20*kDeg,60*kDeg,60*kDeg,300*kDeg);
  check(std::abs(held)<=2*kDeg+braking && std::abs(r.q[0]-held)<1e-4 && std::abs(r.v[0])<1e-4,
        "a stuck axis: the reference stops within its braking distance past the lead limit and holds");
  check(r.flags[0]&kLeadLimited,"lead limitation is flagged");
  // Released: the axis follows again; the reference continues from where it is, toward the current target.
  double q_axis=r.q[0];
  for (int k=801;k<=1600;++k) { const double s=k*0.005; goal.q={10*kDeg*s,-0.8}; goal.v={10*kDeg,0.}; r=g.step(kS+int64_t(k)*5'000'000,goal,{q_axis,-0.8}); q_axis=r.q[0]; }
  check(std::abs(r.q[0]-10*kDeg*8)<0.01,"after release it converges on the current target (no replay of old waypoints)");
}

void level1_pitch_prefers_undershoot_and_ignores_small_motion() {
  // Owner, 2026-10-02 (sessions human-3/4): pitch sits on the frame's short edge (+-20 deg against
  // yaw's +-35), so running past a subject who stops or turns back loses them; undershoot is
  // preferred. Pitch has a dead band (head motion inside it moves nothing) and partial velocity
  // feedforward. Owner, 22:25 (human-5): "at certain height the aim never converges" -- pitch sat
  // 1.6 deg off a still subject for 20 s. A move still stops short; an offset that persists (its 1 s
  // average beyond the centre band) is then closed slowly, never passing; bob averages away.
  Level1Parameters p=level1_parameters();
  auto& a=p.axis[1];
  a.lambda=2.5; a.v_max=34*kDeg; a.a_max=60*kDeg; a.j_max=1500*kDeg; a.dead_band=3*kDeg; a.feedforward_gain=0.7;
  a.centre_band=0.5*kDeg; a.centre_tau_s=1.0; a.centre_speed=3*kDeg;
  a.q_min=-1.42; a.q_max=-0.2;
  // A head rising at 17 deg/s for 0.6 s, then stopping (the stop that overshot on the station).
  {
    Level1Generator g; check(g.configure(p),"pitch with a dead band configures");
    g.reset(kS,{0.,-0.85},{0.,0.},{0.,0.});
    JointGoal goal; goal.valid=true; goal.velocity_valid={true,true};
    ReferenceSample r=g.last(); double beyond=-1e9; const double rate=17*kDeg, stop=0.6;
    for (int k=1;k<=1200;++k) {
      const double s=k*0.005;
      goal.q={0.,-0.85+rate*std::min(s,stop)}; goal.v={0.,s<stop?rate:0.};
      r=g.step(kS+int64_t(k)*5'000'000,goal,{r.q[0],r.q[1]});
      beyond=std::max(beyond,r.q[1]-goal.q[1]);
    }
    check(beyond<=0.2*kDeg,"pitch does not run past a head that stops");
    check(std::abs(r.q[1]-goal.q[1])<=0.5*kDeg && std::abs(r.v[1])<1e-3,"and finishes centred on it (within the centre band)");
  }
  // Head bob: +-2 deg at 2 Hz inside the band moves pitch not at all.
  {
    Level1Generator g; g.configure(p);
    g.reset(kS,{0.,-0.85},{0.,0.},{0.,0.});
    JointGoal goal; goal.valid=true; goal.velocity_valid={true,true};
    ReferenceSample r=g.last(); double travel=0;
    for (int k=1;k<=800;++k) {
      const double s=k*0.005, w=2*M_PI*2;
      goal.q={0.,-0.85+2*kDeg*std::sin(w*s)}; goal.v={0.,2*kDeg*w*std::cos(w*s)};
      const double before=r.q[1];
      r=g.step(kS+int64_t(k)*5'000'000,goal,{r.q[0],r.q[1]});
      travel+=std::abs(r.q[1]-before);
    }
    check(travel<1e-9,"head bob inside the dead band does not move pitch");
  }
  // A still subject 1.6 deg off (inside the dead band): re-centred within a few seconds, without passing.
  {
    Level1Generator g; g.configure(p);
    g.reset(kS,{0.,-0.85},{0.,0.},{0.,0.});
    JointGoal goal; goal.valid=true; goal.velocity_valid={true,true}; goal.q={0.,-0.85+1.6*kDeg}; goal.v={0.,0.};
    ReferenceSample r=g.last(); double beyond=-1e9;
    for (int k=1;k<=800;++k) { r=g.step(kS+int64_t(k)*5'000'000,goal,{r.q[0],r.q[1]}); beyond=std::max(beyond,r.q[1]-goal.q[1]); }
    check(std::abs(r.q[1]-goal.q[1])<=0.5*kDeg,"a persistent offset inside the dead band is re-centred");
    check(beyond<=0.1*kDeg,"without passing the subject");
  }
  // Yaw keeps no band and full feedforward: zero steady ramp lag as before.
  check(p.axis[0].dead_band==0 && p.axis[0].feedforward_gain==1 && p.axis[0].centre_band==0,"defaults leave an axis unchanged");
}

void level1_holds_inside_pitch_travel() {
  Level1Generator g; g.configure(level1_parameters());
  g.reset(kS,{0.,-0.5},{0.,0.},{0.,0.});
  JointGoal goal; goal.valid=true; goal.velocity_valid={true,true}; goal.q={0.,0.2}; goal.v={0.,0.3};
  double max_q=-10; ReferenceSample r;
  for (int k=1;k<=800;++k) { r=g.step(kS+int64_t(k)*5'000'000,goal,{r.q[0],r.q[1]}); max_q=std::max(max_q,r.q[1]); }
  check(max_q<=-0.255+1e-6,"pitch reference never leaves its travel");
  goal.valid=false;
  for (int k=801;k<=1200;++k) r=g.step(kS+int64_t(k)*5'000'000,goal,{r.q[0],r.q[1]});
  check(std::abs(r.v[1])<1e-6 && (r.flags[1]&kGoalInvalid),"no goal: brakes to rest and holds");
}

void level1_slows_to_a_falling_speed_cap() {
  // The host's boundary governor lowers the speed allowed toward an end as the axis nears it; a
  // reference already faster than the new cap must slow to it within a_max and j_max, not merely
  // stop accelerating (station 2026-10-02: pitch rode Level 1's own braking curve outside the
  // supervisor's, which braked the station).
  Level1Generator g; g.configure(level1_parameters());
  g.reset(kS,{0.,-0.8},{0.,0.},{0.,0.});
  JointGoal goal; goal.valid=true; goal.velocity_valid={true,true}; goal.q={2.,-0.8}; goal.v={0.5,0.};
  ReferenceSample r;
  int k=1;
  for (;k<=400;++k) r=g.step(kS+int64_t(k)*5'000'000,goal,r_measured(r));
  check(std::abs(r.v[0]-20*kDeg)<1e-3,"running at v_max before the cap falls");
  g.set_speed_bounds(0,20*kDeg,5*kDeg);
  double worst_a=0, worst_j=0; ReferenceSample prev=r;
  for (;k<=600;++k) {
    r=g.step(kS+int64_t(k)*5'000'000,goal,r_measured(r));
    worst_a=std::max(worst_a,std::abs(r.a[0]));
    worst_j=std::max(worst_j,std::abs(r.a[0]-prev.a[0])/0.005);
    prev=r;
  }
  check(r.v[0]<=5*kDeg+1e-3,"slowed to the cap");
  check(worst_a<=60*kDeg+1e-9 && worst_j<=300*kDeg+1e-6,"within a_max and j_max while slowing");
  g.set_speed_bounds(0,20*kDeg,0.);
  for (;k<=900;++k) r=g.step(kS+int64_t(k)*5'000'000,goal,r_measured(r));
  check(std::abs(r.v[0])<1e-3,"a zero cap toward an end stops the reference");
}

TrackerParameters tracker_parameters() {
  TrackerParameters p;
  p.estimator=estimator_parameters(); p.level1=level1_parameters(); p.pixel_sigma=1.5;
  return p;
}

ota::geo::TurretKinematics station_kinematics() {
  ota::geo::TurretKinematics k;
  k.R_PC=ota::geo::Mat3{0,-0.6524374681640519,0.757842562895277,-1,0,0,0,-0.757842562895277,-0.6524374681640519};
  return k;
}

ota::geo::CameraIntrinsics station_camera() {
  ota::geo::CameraIntrinsics c; c.fx=1389; c.fy=1467; c.cx=960; c.cy=540; c.width=1920; c.height=1080; return c;
}

void tracker_removes_camera_rotation_once() {
  // A stationary subject seen while the turret turns at 15 deg/s: the base-frame LOS is fixed,
  // so the estimated rate must stay near zero (the camera's own motion is not target motion).
  const auto kin=station_kinematics(); const auto cam=station_camera();
  Tracker tr(tracker_parameters(),kin,cam,{0,0,1},{0,0,-1.371,-0.255});
  check(tr.ok(),"tracker constructs");
  const ota::geo::CameraModel model(cam);
  const auto target=tr.los(0.2,-0.8);
  for (int k=0;k<90;++k) {
    const double t=1+k*0.033, yaw=0.15*std::sin(1.2*k*0.033), pitch=-0.8+0.05*std::sin(0.7*k*0.033);
    const auto c=kin.base_to_ray(ota::geo::TurretKinematics::los_to_base_ray(target[0],target[1]),yaw,pitch);
    double u,v; model.ray_to_pixel(c,u,v);
    PixelObservation z; z.sensor_ns=int64_t(t*kS); z.u=u; z.v=v; z.identity=7;
    // exposure 0, so t_o = sensor time and the pose is the pose at t (exact).
    tr.observe(z,yaw,pitch);
    const auto& o=tr.last_observation();
    check(o.sensor_ns==z.sensor_ns && o.t_ns==tr.observation_time(z) && o.u==u && o.v==v && o.yaw==yaw &&
          o.pitch==pitch && o.accepted,"the trace sees the observation exactly as the estimator got it");
    check(std::abs(o.az-target[0])<1e-6 && std::abs(o.el-target[1])<1e-6,"and its LOS is the subject's");
  }
  const auto& x=tr.estimator().state();
  check(std::abs(x[0].omega)<0.002 && std::abs(x[1].omega)<0.002,"stationary subject: base rate ~0 while the camera turns");
  check(std::abs(x[0].theta-target[0])<1e-4 && std::abs(x[1].theta-target[1])<1e-4,"and its LOS is where it is");
}

void tracker_identity_change_drops_motion() {
  Tracker tr(tracker_parameters(),station_kinematics(),station_camera(),{0,0,1},{0,0,-1.371,-0.255});
  for (int k=0;k<40;++k) {
    PixelObservation z; z.sensor_ns=int64_t((1+k*0.033)*kS); z.u=500+k*10; z.v=540; z.identity=1;
    tr.observe(z,0.,-0.8);
  }
  check(std::abs(tr.estimator().state()[0].omega)>0.1,"first subject moving");
  PixelObservation z; z.sensor_ns=int64_t((1+40*0.033)*kS); z.u=1200; z.v=300; z.identity=2;
  tr.observe(z,0.,-0.8);
  check(tr.estimator().state()[0].omega==0 && tr.identity_changes()==1,"a new subject starts with no velocity");
}

void tracker_uses_optical_time() {
  TrackerParameters p=tracker_parameters(); p.timing.fixed_offset_s=0.004; p.timing.exposure_fraction=0.5;
  Tracker tr(p,station_kinematics(),station_camera(),{0,0,1},{0,0,-1.371,-0.255});
  PixelObservation z; z.sensor_ns=5*kS; z.exposure_s=0.01;
  check(tr.observation_time(z)==5*kS+9'000'000,"t_o = sensor + offset + exposure/2 (never the arrival time)");
}

void tracker_joint_rate_matches_geometry() {
  Tracker tr(tracker_parameters(),station_kinematics(),station_camera(),{0,0,1},{0,0,-1.371,-0.255});
  // Move the joints at a known rate; the LOS rate mapped back through J must return that rate.
  const double y=0.3, p=-0.7, vy=0.2, vp=-0.1, h=1e-4;
  const auto a=tr.los(y,p), b=tr.los(y+vy*h,p+vp*h);
  LosGoal g; g.position_valid=g.velocity_valid=true; g.theta=a; g.omega={(b[0]-a[0])/h,(b[1]-a[1])/h};
  const auto j=tr.joint_goal(g,{y,p});
  check(j.valid && std::abs(j.q[0]-y)<1e-6 && std::abs(j.q[1]-p)<1e-6,"LOS solves back to the joint pose");
  check(std::abs(j.v[0]-vy)<1e-3 && std::abs(j.v[1]-vp)<1e-3,"joint rate from J^-1 omega");
}

void config_round_trip() {
  const auto p=tracker_parameters();
  const auto text=tracker_to_json(p);
  const auto q=tracker_from_yaml(YAML::Load(text));
  check(tracker_to_json(q)==text,"parameters read back identically");
  auto node=YAML::Load(text); node["level1"]["yaw"].remove("lambda_rad_s");
  bool threw=false; try { tracker_from_yaml(node); } catch (const std::exception&) { threw=true; }
  check(threw,"a missing parameter fails the load");
}
}

int main() {
  run("estimator_tracks_constant_rate_with_irregular_frames",estimator_tracks_constant_rate_with_irregular_frames);
  run("estimator_rejects_repeated_and_old_frames",estimator_rejects_repeated_and_old_frames);
  run("estimator_follows_a_sudden_stop",estimator_follows_a_sudden_stop);
  run("estimator_keeps_a_fast_subject_moving",estimator_keeps_a_fast_subject_moving);
  run("velocity_weight_and_coast_laws",velocity_weight_and_coast_laws);
  run("estimator_coast_is_bounded",estimator_coast_is_bounded);
  run("level1_is_one_integral",level1_is_one_integral);
  run("level1_tracks_ramp_without_lag",level1_tracks_ramp_without_lag);
  run("level1_respects_lead_limit_and_resumes",level1_respects_lead_limit_and_resumes);
  run("level1_pitch_prefers_undershoot_and_ignores_small_motion",level1_pitch_prefers_undershoot_and_ignores_small_motion);
  run("level1_holds_inside_pitch_travel",level1_holds_inside_pitch_travel);
  run("level1_slows_to_a_falling_speed_cap",level1_slows_to_a_falling_speed_cap);
  run("tracker_removes_camera_rotation_once",tracker_removes_camera_rotation_once);
  run("tracker_identity_change_drops_motion",tracker_identity_change_drops_motion);
  run("tracker_uses_optical_time",tracker_uses_optical_time);
  run("tracker_joint_rate_matches_geometry",tracker_joint_rate_matches_geometry);
  run("config_round_trip",config_round_trip);
  if (failures) { std::printf("%d failure(s)\n",failures); return 1; }
  std::printf("tracking_core: all checks passed\n");
  return 0;
}
