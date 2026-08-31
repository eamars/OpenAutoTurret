// Unit tests for the coupled collision envelope (architecture §19).
#include <gtest/gtest.h>

#include <vector>

#include "control/collision_envelope.hpp"

namespace {

using ota::Pose2D;
using ota::RectangularCollisionEnvelope;
using ota::RectangularEnvelopeConfig;

TEST(CollisionEnvelope, IsSafeInsideRectangle) {
  RectangularCollisionEnvelope env(RectangularEnvelopeConfig{});  // ±1 rad box
  EXPECT_TRUE(env.is_safe(0.0, 0.0));
  EXPECT_TRUE(env.is_safe(0.5, -0.5));
  // Boundaries are inclusive.
  EXPECT_TRUE(env.is_safe(1.0, 1.0));
  EXPECT_TRUE(env.is_safe(-1.0, -1.0));
  EXPECT_TRUE(env.is_safe(-1.0, 0.7));
  EXPECT_TRUE(env.is_safe(0.7, -1.0));
}

TEST(CollisionEnvelope, IsSafeOutsideRectangle) {
  RectangularCollisionEnvelope env(RectangularEnvelopeConfig{});  // ±1 rad box
  EXPECT_FALSE(env.is_safe(1.001, 0.0));
  EXPECT_FALSE(env.is_safe(0.0, 1.001));
  EXPECT_FALSE(env.is_safe(-1.001, 0.0));
  EXPECT_FALSE(env.is_safe(0.0, -1.001));
  EXPECT_FALSE(env.is_safe(1.5, 1.5));
}

TEST(CollisionEnvelope, CustomRanges) {
  RectangularEnvelopeConfig cfg{-0.3, 0.7, -1.2, 1.2};
  RectangularCollisionEnvelope env(cfg);
  EXPECT_TRUE(env.is_safe(0.0, 0.0));
  EXPECT_TRUE(env.is_safe(0.7, 1.2));  // pitch max / yaw max corner
  EXPECT_TRUE(env.is_safe(-0.3, -1.2));  // pitch min / yaw min corner
  EXPECT_FALSE(env.is_safe(0.71, 0.0));  // past pitch max
  EXPECT_FALSE(env.is_safe(0.0, 1.21));  // past yaw max
  EXPECT_FALSE(env.is_safe(-0.31, 0.0));  // below pitch min
}

// §19: the path (not just the endpoint) must be validated. A path whose
// endpoints are inside but which leaves the region mid-flight is unsafe.
TEST(CollisionEnvelope, PathValidatesWholePathNotJustEndpoint) {
  RectangularCollisionEnvelope env(RectangularEnvelopeConfig{});  // ±1 rad box
  std::vector<Pose2D> path{{0.0, 0.0}, {0.0, 2.0}, {0.0, 0.0}};
  EXPECT_TRUE(env.is_safe(path.front().pitch_rad, path.front().yaw_rad));
  EXPECT_TRUE(env.is_safe(path.back().pitch_rad, path.back().yaw_rad));
  EXPECT_FALSE(env.is_path_safe(path));  // mid-path point is outside
}

TEST(CollisionEnvelope, PathSafeWhenAllInside) {
  RectangularCollisionEnvelope env(RectangularEnvelopeConfig{});
  std::vector<Pose2D> path{{-0.5, -0.5}, {0.0, 0.0}, {0.5, 0.5}, {0.2, -0.2}};
  EXPECT_TRUE(env.is_path_safe(path));
}

TEST(CollisionEnvelope, EmptyPathIsSafe) {
  RectangularCollisionEnvelope env(RectangularEnvelopeConfig{});
  EXPECT_TRUE(env.is_path_safe({}));
}

// §19: the controller depends only on the CollisionEnvelope interface, so a
// piecewise table or polygon envelope can be swapped in later without
// rewriting the controller. Verify the abstract interface is usable polymorphically.
TEST(CollisionEnvelope, InterfaceIsPolymorphicForFutureEnvelopes) {
  const ota::RectangularCollisionEnvelope concrete(RectangularEnvelopeConfig{});
  const ota::CollisionEnvelope* env = &concrete;  // upcast to the interface
  EXPECT_TRUE(env->is_safe(0.0, 0.0));
  EXPECT_FALSE(env->is_safe(2.0, 0.0));
  std::vector<Pose2D> path{{0.0, 0.0}, {0.5, 0.5}};
  EXPECT_TRUE(env->is_path_safe(path));
}

}  // namespace
