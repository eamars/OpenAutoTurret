#include "position_loop.hpp"
#include <algorithm>
#include <cmath>

namespace ota::axis {
bool PositionLoop::configure(const PositionLoopParameters& p) {
  for (double value:{p.kp,p.ki,p.integral_clamp,p.speed_limit})
    if (!std::isfinite(value) || value<0) return false;
  if (!(p.kp>0) || !(p.speed_limit>0)) return false;
  p_=p; integral_=0.; return true;
}
double travel_governor(double speed,double q,double low,double high,double stop_acceleration) {
  if (!(stop_acceleration>0) || !(high>low)) return 0.;
  const double up=std::sqrt(2*stop_acceleration*std::max(0.,high-q));
  const double down=std::sqrt(2*stop_acceleration*std::max(0.,q-low));
  return std::clamp(speed,-down,up);
}

double PositionLoop::step(double dt,double q_ref,double v_ref,double q) {
  const double error=q_ref-q;
  const double unclamped=v_ref+p_.kp*error+integral_;
  if (std::abs(unclamped)<p_.speed_limit || unclamped*error<0)
    integral_=std::clamp(integral_+p_.ki*error*dt,-p_.integral_clamp,p_.integral_clamp);
  return std::clamp(v_ref+p_.kp*error+integral_,-p_.speed_limit,p_.speed_limit);
}
}
