#include <iostream>
#include <cmath>
#include "control/reference_limiter.hpp"
int main() {
  constexpr double deg = M_PI/180;
  bool settled = true;
  double worst = 0;
  for (bool jitter : {false, true}) {
    ota::control::ReferenceLimiter reference;
    reference.reset_at(0);
    double t=0, error=0;
    for (int i=0; t<15; ++i) {
      const double dt = jitter && i%3==0 ? .022 : .005;
      t += dt;
      const auto q = ota::control::limit_reference(reference, .5*deg, dt, 2*deg, 60*deg, 300*deg);
      if (t>2) error = std::max(error,std::abs(q-.5*deg)/deg);
    }
    worst = std::max(worst,error);
    settled = settled && error < .01;
  }
  const auto braking = ota::control::stopping_distance_rad(2*deg,0,60*deg,300*deg)/deg;
  std::cout << "{\"small_step_settled\":" << settled << ",\"steady_error_max_deg\":" << worst
            << ",\"brake_distance_deg\":" << braking << "}\n";
  return settled && braking > .1 ? 0 : 1;
}
