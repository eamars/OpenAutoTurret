#pragma once
#include "control/reference_limiter.hpp"
#include "control/safety_envelope.hpp"

namespace ota::control {

// Service-only, independent of the target and reference planner. Braking uses
// the service servo's actual acceleration/jerk bounds and reserves response
// time as well as distance. Outward acceleration tapers; braking is not derated.
struct BoundaryGovernor {
  double acceleration = 30*kDeg2Rad;
  double jerk = 120*kDeg2Rad;
  double response_s = .20;
  double margin = .05;
  struct Bounds {
    double negative_speed, positive_speed;
    double negative_acceleration_scale, positive_acceleration_scale;
  };
  double travel(double speed, double outward_acceleration) const {
    // Conservatively allow the existing outward acceleration to continue
    // during the entire response interval before applying the braking ramp.
    const double v=std::max(0.0,speed);
    const double a=std::clamp(outward_acceleration,0.0,acceleration);
    return v*response_s + .5*a*response_s*response_s +
        stopping_distance_rad(v+a*response_s,a,acceleration,jerk);
  }
  double speed_for(double clearance, double cap, double outward_acceleration=0) const {
    if (clearance<=0 || cap<=0) return 0;
    double lo=0,hi=cap;
    for(int i=0;i<24;++i) {
      const double mid=.5*(lo+hi);
      if(travel(mid,outward_acceleration)<=clearance) lo=mid; else hi=mid;
    }
    return lo;
  }
  Bounds at(double q, const AxisLimits& limits, double cap, double current_acceleration=0,
            double measured_velocity=0) const {
    if(!limits.valid) return {cap,cap,1,1};
    // Reserve the outstanding physical motion as well as the next command.
    // A falling command does not instantly remove motor response lag.
    const double negative=q-limits.q_soft_min_rad-margin-std::max(0.0,-measured_velocity)*response_s;
    const double positive=limits.q_soft_max_rad-q-margin-std::max(0.0,measured_velocity)*response_s;
    const double zone=std::max(travel(cap,acceleration),1e-9);
    const double ns=std::clamp(negative/zone,0.0,1.0), ps=std::clamp(positive/zone,0.0,1.0);
    return {speed_for(negative,cap,std::max(-current_acceleration,acceleration*ns)),
            speed_for(positive,cap,std::max(current_acceleration,acceleration*ps)),ns,ps};
  }
};
}  // namespace ota::control
