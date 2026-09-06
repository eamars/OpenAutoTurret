// Offline isolation: production reference planner or production LOS estimator.
// No CAN, camera, motor backend, or live commands are constructed by this tool.
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include "calibration/camera_calibration.hpp"
#include "config/turret_config.hpp"
#include "control/reference_limiter.hpp"
#include "geometry/los_uncertainty.hpp"
#include "tracking/target_estimator.hpp"

int main(int argc, char** argv) {
  using namespace ota;
  std::cout << std::setprecision(12);
  if (argc == 2 && std::string(argv[1]) == "--clean-reference") {
    constexpr double deg=M_PI/180, dt=.005;
    std::cout << "target_speed_deg_s,time_s,target_deg,reference_deg,rate_deg_s,accel_deg_s2\n";
    for (double speed : {3., 10., 15.}) {
      control::ReferenceLimiter state;
      state.reset_at(0);
      for (int i=0; i<2400; ++i) {
        const double t=i*dt, target=speed*std::clamp(t-1.,0.,4.)*deg;
        const double q=control::limit_reference(state,target,dt,15*deg,15*deg,60*deg);
        std::cout << speed << ',' << t << ',' << target/deg << ',' << q/deg << ','
                  << state.v_rad_s/deg << ',' << state.a_rad_s2/deg << '\n';
      }
    }
    return 0;
  }
  if (argc != 3) {
    std::cerr << "usage: probe-tracking-boundaries --clean-reference | config.yaml measurements.csv\n";
    return 2;
  }
  const auto loaded=config::load_turret_config(argv[1]);
  if (!loaded.ok) { for (const auto& e:loaded.errors) std::cerr << e << '\n'; return 2; }
  const auto& cfg=loaded.config;
  const auto il=load_camera_intrinsics(cfg.camera.intrinsics_file);
  std::string detail;
  const auto kin=load_camera_extrinsics(cfg.camera.extrinsics_file,detail);
  if (!il.found || detail.find("loaded")==std::string::npos) {
    std::cerr << "calibration required: " << detail << '\n'; return 2;
  }
  const auto& in=il.intrinsics;
  geo::CameraModel camera(in);
  tracking::TargetEstimatorConfig ec;
  ec.measurement_sigma_rad=cfg.tracking.estimator_measurement_sigma_rad;
  ec.angular_accel_sigma_rad_s2=cfg.tracking.estimator_accel_sigma_rad_s2;
  ec.use_kalman=cfg.tracking.estimator_model=="constant_velocity";
  ec.alpha=cfg.tracking.estimator_alpha; ec.beta=cfg.tracking.estimator_beta;
  tracking::TargetEstimator estimator(ec);
  std::ifstream file(argv[2]);
  if (!file) return 2;
  std::string line; std::getline(file,line); // named CSV header
  uint64_t identity=0;
  std::cout << "capture_ns,arrival_ns,identity,valid,accepted,u_px,v_px,raw_az,raw_el,filtered_az,filtered_el,predicted_az,predicted_el,predicted_u,predicted_v,az_rate,el_rate,nis,q_scale,prediction_valid\n";
  while (std::getline(file,line)) {
    std::replace(line.begin(),line.end(),',',' ');
    std::istringstream row(line);
    uint64_t capture,arrival,id; int valid;
    double u,v,width,height,confidence,association,continuity,yaw,pitch;
    if (!(row >> capture >> arrival >> id >> valid >> u >> v >> width >> height
              >> confidence >> association >> continuity >> yaw >> pitch)) {
      std::cerr << "invalid input row\n"; return 2;
    }
    double az=0,el=0;
    geo::TurretKinematics::base_ray_to_los(kin.ray_to_base(camera.pixel_to_ray(u,v),yaw,pitch),az,el);
    bool accepted=false;
    if (valid) {
      if (id!=identity) { estimator.reset(); identity=id; }
      const auto variance=geo::pixel_los_variance(camera,kin,yaw,pitch,u,v,
          std::max({2.,ec.measurement_sigma_rad*in.fx,.02*width}),
          std::max({2.,ec.measurement_sigma_rad*in.fy,.02*height}));
      const double quality=std::clamp(confidence*association*continuity,.05,1.);
      accepted=estimator.update(az,el,capture,variance[0]/quality,variance[1]/quality);
    }
    double pa=0,pe=0,pu=0,pv=0;
    const auto predicted_at=arrival+static_cast<int64_t>((cfg.tracking.control_delay_ms+
        cfg.tracking.motor_response_ms)*1e6);
    estimator.predict(predicted_at,pa,pe);
    camera.ray_to_pixel(kin.base_to_ray(geo::TurretKinematics::los_to_base_ray(pa,pe),yaw,pitch),pu,pv);
    const auto& d=estimator.diagnostics();
    std::cout << capture << ',' << arrival << ',' << id << ',' << valid << ',' << accepted << ','
              << u << ',' << v << ',' << az << ',' << el << ',' << estimator.azimuth() << ','
              << estimator.elevation() << ',' << pa << ',' << pe << ',' << pu << ',' << pv << ','
              << estimator.azimuth_rate() << ',' << estimator.elevation_rate() << ','
              << d.mahalanobis << ',' << d.process_noise_scale << ','
              << estimator.prediction_valid(predicted_at) << '\n';
  }
}
