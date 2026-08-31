// Unit tests for the LOS->joint solver (architecture §14). Verifies the solver
// is the inverse of the kinematics (round-trip) for the aligned ideal gimbal and
// still converges for a slightly misaligned (calibrated) extrinsic.
#include <gtest/gtest.h>

#include <cmath>

#include "geometry/los_joint_solver.hpp"

namespace {
using ota::geo::LosJointSolver;
using ota::geo::Mat3;
using ota::geo::TurretKinematics;
using ota::geo::Vec3;

TEST(LosJointSolver, AlignedRoundTrip) {
  LosJointSolver solver(TurretKinematics::aligned());
  const double cases[][2] = {
      {0.0, 0.0},        {0.3, 0.1},     {-0.5, -0.2}, {1.2, 0.4},
      {-1.5, 0.0},       {0.1, -0.3},    {M_PI / 2, 0.0}, {-M_PI / 2, 0.1},
  };
  for (const auto& c : cases) {
    const double az = c[0], el = c[1];
    double qy, qp;
    ASSERT_TRUE(solver.solve(az, el, qy, qp));
    // Round-trip: solve -> optical axis -> base-frame LOS.
    const Vec3 r = solver.optical_axis(qy, qp).normalized();
    double az2, el2;
    TurretKinematics::base_ray_to_los(r, az2, el2);
    EXPECT_NEAR(az2, az, 1e-6) << "az=" << az;
    EXPECT_NEAR(el2, el, 1e-6) << "el=" << el;
  }
}

TEST(LosJointSolver, AlignedMatchesAngularDecomposition) {
  LosJointSolver solver(TurretKinematics::aligned());
  const double az = 0.4, el = -0.2;
  double qy, qp;
  ASSERT_TRUE(solver.solve(az, el, qy, qp));
  double gy, gp;
  LosJointSolver::angular_decomposition(az, el, gy, gp);
  EXPECT_NEAR(qy, gy, 1e-6);
  EXPECT_NEAR(qp, gp, 1e-6);
}

TEST(LosJointSolver, OpticalAxisAtZeroIsForward) {
  LosJointSolver solver(TurretKinematics::aligned());
  const Vec3 r = solver.optical_axis(0.0, 0.0);
  // Zero joints: optical axis along the base forward (+X).
  EXPECT_NEAR(r.x, 1.0, 1e-6);
  EXPECT_NEAR(r.y, 0.0, 1e-6);
  EXPECT_NEAR(r.z, 0.0, 1e-6);
}

// A calibrated extrinsic with a small roll offset: the angular-decomposition
// guess is slightly wrong, but the gradient refinement must still recover the
// correct joints (round-trip residual stays small).
TEST(LosJointSolver, MisalignedExtrinsicStillConverges) {
  TurretKinematics kin = TurretKinematics::aligned();
  // Apply a small (2 deg) roll to the extrinsic.
  const double roll = 2.0 * M_PI / 180.0;
  const Mat3 Rroll =
      Mat3{1.0, 0.0, 0.0,
           0.0, std::cos(roll), -std::sin(roll),
           0.0, std::sin(roll), std::cos(roll)};
  kin.R_PC = Rroll * kin.R_PC;

  LosJointSolver solver(kin);
  const double az = 0.35, el = -0.15;
  double qy, qp;
  ASSERT_TRUE(solver.solve(az, el, qy, qp));
  const Vec3 r = solver.optical_axis(qy, qp).normalized();
  double az2, el2;
  TurretKinematics::base_ray_to_los(r, az2, el2);
  // The solved joints point the (tilted) optical axis at the target.
  EXPECT_NEAR(az2, az, 2e-3);
  EXPECT_NEAR(el2, el, 2e-3);
}

}  // namespace
