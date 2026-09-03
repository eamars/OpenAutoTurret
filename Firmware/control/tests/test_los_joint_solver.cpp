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


// ---------------------------------------------------------------------------
// The seed decides the answer.
//
// Station case, 2026-09-04, with the measured mount roll (R_P_C = rot_z(-90 deg)) and
// the pitch the station is actually at (its soft limits are -74.7..-4.9 deg, so
// sin(q_pitch) < 0 everywhere). AUTO_TRACK diverged from a target 182 px off the
// reticle because the analytic seed for the direction the camera was ALREADY looking
// at pointed 180 deg away in azimuth - orthogonal in 3D - and the twelve gradient steps
// could
// not recover from a start that is orthogonal to the answer.
// ---------------------------------------------------------------------------
TEST(LosJointSolver, SeededFromTheCurrentPoseItReproducesThatPose) {
  ota::geo::TurretKinematics kin = ota::geo::TurretKinematics::aligned();
  ota::geo::Mat3 r_pc{};
  r_pc.m[0][0] = 0.0;  r_pc.m[0][1] = 1.0;  r_pc.m[0][2] = 0.0;
  r_pc.m[1][0] = -1.0; r_pc.m[1][1] = 0.0;  r_pc.m[1][2] = 0.0;
  r_pc.m[2][0] = 0.0;  r_pc.m[2][1] = 0.0;  r_pc.m[2][2] = 1.0;
  kin.R_PC = r_pc;
  const ota::geo::LosJointSolver solver(kin);

  const double qy = 1.7924, qp = -0.6678;                 // where the station stood
  const ota::geo::Vec3 ax = kin.ray_to_base({0.0, 0.0, 1.0}, qy, qp);
  double az = 0.0, el = 0.0;
  ota::geo::TurretKinematics::base_ray_to_los(ax, az, el);

  // First, the reason: the ideal-gimbal seed does not point at the direction it was given.
  double seed_qy = 0.0, seed_qp = 0.0;
  solver.angular_decomposition(az, el, seed_qy, seed_qp);
  const ota::geo::Vec3 seeded_axis = kin.ray_to_base({0.0, 0.0, 1.0}, seed_qy, seed_qp);
  double seed_az = 0.0, seed_el = 0.0;
  ota::geo::TurretKinematics::base_ray_to_los(seeded_axis, seed_az, seed_el);
  double daz = seed_az - az;
  while (daz > M_PI) daz -= 2.0 * M_PI;
  while (daz < -M_PI) daz += 2.0 * M_PI;
  EXPECT_NEAR(std::fabs(daz), M_PI, 1e-6)
      << "the seed's azimuth should be opposite the requested one here, got " << daz;
  double dot = seeded_axis.dot(ax);
  dot = dot > 1.0 ? 1.0 : (dot < -1.0 ? -1.0 : dot);
  EXPECT_NEAR(std::acos(dot), M_PI / 2.0, 1e-6)
      << "and because the seeded elevation is 90 deg - el, the seed is exactly orthogonal";

  // And the fix: refining from the pose we are at returns that same pose.
  double sy = 0.0, sp = 0.0;
  ASSERT_TRUE(solver.solve_from_pose(az, el, qy, qp, sy, sp));
  EXPECT_NEAR(sy, qy, 1e-4);
  EXPECT_NEAR(sp, qp, 1e-4);

  // A target 12 deg further round must ask for 12 deg of yaw, not for a 140 deg sweep.
  double ty = 0.0, tp = 0.0;
  ASSERT_TRUE(solver.solve_from_pose(az + 12.0 * M_PI / 180.0, el, qy, qp, ty, tp));
  EXPECT_NEAR(std::fabs(ty - qy), 12.0 * M_PI / 180.0, 2e-3)
      << "got a " << (ty - qy) << " rad yaw demand";
}
