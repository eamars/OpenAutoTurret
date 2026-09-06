// Capture-order and long-loss probe, using the production estimator, without motors.
#include <cmath>
#include <iostream>
#include "tracking/target_estimator.hpp"

int main() {
  ota::tracking::TargetEstimator estimator;
  for (int i = 0; i < 60; ++i)
    estimator.update(0.01 * i, 0.005 * i, 1000000000LL + i * 60000000LL);
  const double before = estimator.azimuth_rate();
  estimator.update(0.0, 0.0, 1000000000LL);  // delayed old metadata
  const bool rejected_old = estimator.azimuth_rate() == before;
  double az, el;
  estimator.predict(20000000000000LL, az, el);  // hours after vision loss
  const bool bounded = std::isfinite(el) && std::abs(el) <= M_PI / 2;
  const auto stamp = estimator.state_timestamp_ns();
  const bool outlier_rejected = !estimator.update(-2.4, -1.0, stamp + 60'000'000);
  std::cout << "{\"old_capture_rejected\":" << (rejected_old ? "true" : "false")
            << ",\"long_loss_elevation_rad\":" << el
            << ",\"prediction_bounded\":" << (bounded ? "true" : "false")
            << ",\"outlier_rejected\":" << outlier_rejected << "}\n";
  return rejected_old && bounded && outlier_rejected ? 0 : 1;
}
