#pragma once
#include <array>
#include <cmath>
#include "geometry/camera_model.hpp"
#include "geometry/turret_kinematics.hpp"

namespace ota::geo {
// Project pixel uncertainty through the same camera and joint geometry as the
// measurement. Azimuth becomes much less certain near a vertical optical axis;
// treating image-angle variance as world-azimuth variance overstates confidence.
inline std::array<double, 2> pixel_los_variance(
    const CameraModel& camera, const TurretKinematics& kin, double yaw, double pitch,
    double u, double v, double sigma_u, double sigma_v) {
  auto los = [&](double x, double y) {
    std::array<double, 2> a;
    TurretKinematics::base_ray_to_los(
        kin.ray_to_base(camera.pixel_to_ray(x, y), yaw, pitch), a[0], a[1]);
    return a;
  };
  auto difference = [](double a, double b) {
    return std::atan2(std::sin(a-b), std::cos(a-b));
  };
  const auto up = los(u+1,v), um = los(u-1,v);
  const auto vp = los(u,v+1), vm = los(u,v-1);
  const double az_u = .5*difference(up[0],um[0])*sigma_u;
  const double az_v = .5*difference(vp[0],vm[0])*sigma_v;
  const double el_u = .5*(up[1]-um[1])*sigma_u;
  const double el_v = .5*(vp[1]-vm[1])*sigma_v;
  const double cross = std::abs(az_u*el_u + az_v*el_v);
  // The estimator is diagonal. Adding |cross covariance| to both diagonal
  // terms conservatively bounds the full correlated covariance matrix.
  return {az_u*az_u + az_v*az_v + cross, el_u*el_u + el_v*el_v + cross};
}
}
