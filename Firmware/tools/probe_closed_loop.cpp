// Camera -> production tracker -> guidance -> speed servo -> plant -> camera.
// The deterministic plant is an explicit substitute for motors; no CAN is constructed.
#include <deque>
#include <iostream>
#include <iomanip>
#include <vector>
#include "calibration/camera_calibration.hpp"
#include "config/turret_config.hpp"
#include "control/tracking_controller.hpp"
#include "control/reference_limiter.hpp"
#include "control/speed_servo.hpp"
#include "control/tracking_reference.hpp"

int main(int argc,char** argv) {
  using namespace ota;
  if(argc<2||argc>4) {std::cerr<<"usage: probe-closed-loop config.yaml [baseline|direct-trusted|direct-trusted-noff|direct-trusted-nolead|planned-trusted|damped|damped-trusted] [--verify]\n";return 2;}
  const bool verify=std::string(argv[argc-1])=="--verify";
  if (argc==4&&!verify) return 2;
  const std::string variant=argc>=3&&std::string(argv[2])!="--verify"?argv[2]:"baseline";
  if (variant!="baseline" && variant!="direct-trusted" && variant!="direct-trusted-noff" &&
      variant!="direct-trusted-nolead" && variant!="planned-trusted" &&
      variant!="damped" && variant!="damped-trusted") return 2;
  const bool trusted=variant.find("trusted")!=std::string::npos;
  const bool direct=variant.find("direct")!=std::string::npos;
  const bool planned=variant.find("planned")!=std::string::npos;
  const bool feedforward=variant.find("noff")==std::string::npos;
  bool passed=true;
  auto loaded=config::load_turret_config(argv[1]);if(!loaded.ok)return 2;
  const auto& c=loaded.config;
  auto in=load_camera_intrinsics(c.camera.intrinsics_file);std::string detail;
  auto kin=load_camera_extrinsics(c.camera.extrinsics_file,detail);
  if(!in.found||detail.find("loaded")==std::string::npos)return 2;
  std::cout<<std::setprecision(12);
  if(!verify)std::cout<<"case,time_s,yaw_deg,pitch_deg,error_u,error_v,desired_yaw_deg,desired_pitch_deg,reference_yaw_deg,reference_pitch_deg,az_rate,el_rate\n";
  for(int scenario=0;scenario<9;++scenario) {
    TrackingController::Config tc;tc.intrinsics=in.intrinsics;tc.kinematics=kin;
    tc.uncertainty_gated_motion=trusted;
    tc.estimator.measurement_sigma_rad=c.tracking.estimator_measurement_sigma_rad;
    tc.estimator.angular_accel_sigma_rad_s2=c.tracking.estimator_accel_sigma_rad_s2;
    tc.motor_response_ns=c.tracking.motor_response_ms*1e6;
    tc.control_delay_ns=c.tracking.control_delay_ms*1e6;
    if(variant.find("nolead")!=std::string::npos)tc.motor_response_ns=tc.control_delay_ns=0;
    tc.track_v_max_rad_s=15*kDeg2Rad;
    TrackingController tracker(tc);geo::LosJointSolver solver(kin);
    auto truth=kin;auto ti=in.intrinsics;
    if(scenario>=2) {
      truth.R_PC=geo::Mat3::rot_y(-.89144816316)*geo::Mat3::rot_z(-M_PI/2);
      ti.fx=1621.4685;ti.fy=1566.6908;
    }
    geo::CameraModel camera(ti);
    double q[2]={-.15,-.50},v[2]={0,0},desired[2]={q[0],q[1]};
    control::ReferenceLimiter reference[2];control::SpeedServo servo[2];
    for(int axis=0;axis<2;++axis)reference[axis].reset_at(q[axis]);
    struct Pending {int due;vision::TargetMeasurement m;};std::deque<Pending> pending;
    std::vector<double> late_error;
    double moving_squared_error=0;int moving_samples=0;
    double initial_overshoot_rad=0;
    constexpr double dt=.005;const int delay=scenario==4?32:16;
    for(int i=0;i<12000;++i) {
      const auto now=static_cast<TimeNs>(1000000000LL+i*5000000LL);
      const double target_yaw=(scenario==6||scenario==7)?(scenario==6?10:15)*kDeg2Rad*std::clamp(i*dt-5.,0.,4.):0;
      const auto target=truth.ray_to_base({0,0,1},target_yaw,-.35);
      tracker.update_snapshots(now,q[1],q[0]);
      double u,w;camera.ray_to_pixel(truth.base_to_ray(target,q[0],q[1]),u,w);
      if(i%8==0) {
        vision::TargetMeasurement m;m.valid=true;m.sensor_timestamp_ns=now;m.frame_sequence=i/8;
        m.anchor_u_px=u;m.anchor_v_px=w;
        if(scenario==1||scenario>=3) {
          m.anchor_u_px+=12*std::sin(i*dt*(scenario==8?2.5:5.1))+6*std::sin(i*dt*31);
          m.anchor_v_px+=12*std::cos(i*dt*(scenario==8?2.3:4.7))+6*std::cos(i*dt*27);
        }
        m.confidence=.8;m.association_quality=.95;m.identity_confidence=.99;
        m.authoritative_anchor=true;m.has_track_id=true;m.visual_track_id=1;
        m.bbox_x_min_norm=.3;m.bbox_x_max_norm=.7;m.bbox_y_min_norm=.1;m.bbox_y_max_norm=.9;
        pending.push_back({i+delay,m});
      }
      while(!pending.empty()&&pending.front().due<=i){tracker.set_measurement(pending.front().m);pending.pop_front();}
      tracker.compute_reference(now,q[1],q[0]);
      double rate[2]={tracker.target_motion_rate(0),tracker.target_motion_rate(1)};
      if(tracker.prediction_valid()) {
        double az,el;tracker.predicted_los_at_actuation(az,el);
        solver.solve_from_pose(az,el,q[0],q[1],desired[0],desired[1]);
        desired[0]=geo::wrap_near(desired[0],q[0]);
        const auto joint_rate=tracker.joint_motion_rates(desired[0],desired[1]);
        rate[0]=joint_rate[0];rate[1]=joint_rate[1];
      }
      for(int axis=0;axis<2;++axis) {
        if(variant.find("damped")!=std::string::npos)
          control::track_reference(reference[axis],desired[axis],rate[axis],dt,15*kDeg2Rad,15*kDeg2Rad,60*kDeg2Rad);
        else if(direct){reference[axis].q_rad=desired[axis];reference[axis].v_rad_s=feedforward?rate[axis]:0;}
        else control::limit_reference(reference[axis],desired[axis],dt,15*kDeg2Rad,15*kDeg2Rad,60*kDeg2Rad,
            planned?std::optional<double>{0}:std::nullopt);
        double cmd=servo[axis].step(reference[axis].q_rad,reference[axis].v_rad_s,q[axis],20*kDeg2Rad,dt,30*kDeg2Rad,120*kDeg2Rad);
        const double tau=scenario==5?.15:axis==0?.05:.075;
        v[axis]+=(cmd-v[axis])*(1-std::exp(-dt/tau));q[axis]+=v[axis]*dt;
        if (i*dt<5) initial_overshoot_rad=std::max(initial_overshoot_rad,
            q[axis]-(axis==0?0.0:-.35));
      }
      if(i%8==0) {
        if(i*dt>40)late_error.push_back(std::hypot(u-ti.cx,w-ti.cy));
        if(i*dt>7&&i*dt<9){moving_squared_error+=(u-ti.cx)*(u-ti.cx);++moving_samples;}
        if(!verify)std::cout<<scenario<<','<<i*dt<<','<<q[0]/kDeg2Rad<<','<<q[1]/kDeg2Rad<<','<<u-ti.cx<<','<<w-ti.cy<<','<<desired[0]/kDeg2Rad<<','<<desired[1]/kDeg2Rad<<','<<reference[0].q_rad/kDeg2Rad<<','<<reference[1].q_rad/kDeg2Rad<<','<<tracker.target_az_rate_rad_s()<<','<<tracker.target_el_rate_rad_s()<<'\n';
      }
    }
    std::sort(late_error.begin(),late_error.end());
    const double p95=late_error[static_cast<size_t>(.95*(late_error.size()-1))];
    const double moving_rms=std::sqrt(moving_squared_error/std::max(1,moving_samples));
    // A stationary object should not amplify the injected 12+6 px measurement
    // disturbance into a larger persistent orbit, including mismatch/delay.
    // Late settling alone missed the rejected follower's large initial excursion.
    // Check the return before the moving-target scenarios begin as well.
    passed &= p95<20 && initial_overshoot_rad<1.5*kDeg2Rad && (scenario!=6 || moving_rms<80);
    if(verify)std::cout<<"case "<<scenario<<" stationary p95="<<p95
        <<" px; initial overshoot="<<initial_overshoot_rad/kDeg2Rad
        <<" deg; moving RMS="<<moving_rms<<" px\n";
  }
  return verify&&!passed?1:0;
}
