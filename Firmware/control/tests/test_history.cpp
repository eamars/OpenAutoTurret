// MotorStateHistory interpolation tests (architecture §11.1, §54.1).
#include <gtest/gtest.h>

#include "common/motor_state_history.hpp"

using ota::MotorStateHistory;

TEST(History, InterpolatesBetweenBrackets) {
  MotorStateHistory h(16);
  h.add(0, 0.0f, 1.0f);
  h.add(1000000000, 1.0f, 1.0f);  // 1 s, 1 rad/s
  ota::MotorSample s;
  ASSERT_TRUE(h.interpolate(500000000, s));
  EXPECT_NEAR(s.q, 0.5f, 1e-6);
  EXPECT_NEAR(s.v, 1.0f, 1e-6);
}

TEST(History, EndpointQueries) {
  MotorStateHistory h(16);
  h.add(10, 2.0f, 0.0f);
  h.add(20, 3.0f, 0.0f);
  ota::MotorSample s;
  ASSERT_TRUE(h.interpolate(10, s));
  EXPECT_FLOAT_EQ(s.q, 2.0f);
  ASSERT_TRUE(h.interpolate(20, s));
  EXPECT_FLOAT_EQ(s.q, 3.0f);
}

TEST(History, RejectsOutOfCoverage) {
  MotorStateHistory h(16);
  h.add(100, 0.0f, 0.0f);
  h.add(200, 1.0f, 0.0f);
  ota::MotorSample s;
  EXPECT_FALSE(h.interpolate(99, s));
  EXPECT_FALSE(h.interpolate(201, s));
}

TEST(History, TooFewSamples) {
  MotorStateHistory h(16);
  ota::MotorSample s;
  EXPECT_FALSE(h.interpolate(0, s));
  h.add(0, 0.0f, 0.0f);
  EXPECT_FALSE(h.interpolate(0, s));
}

TEST(History, RingWrapAndCoverage) {
  // Fill past capacity; only the last `capacity` samples are coverable.
  const std::size_t cap = 8;
  MotorStateHistory h(cap);
  for (int i = 0; i < 100; ++i) {
    h.add(static_cast<ota::TimeNs>(i * 10), static_cast<float>(i), 0.0f);
  }
  EXPECT_EQ(h.size(), cap);
  ota::MotorSample s;
  const int newest = 99;
  const int oldest = static_cast<int>(100 - cap);  // 92
  ASSERT_TRUE(h.interpolate(10 * oldest, s));
  EXPECT_FLOAT_EQ(s.q, static_cast<float>(oldest));
  ASSERT_TRUE(h.interpolate(10 * newest, s));
  EXPECT_FLOAT_EQ(s.q, static_cast<float>(newest));
  EXPECT_FALSE(h.interpolate(10 * (oldest - 1), s));  // fell out of the ring
}

TEST(History, DuplicateTimestamps) {
  MotorStateHistory h(16);
  h.add(5, 1.0f, 0.0f);
  h.add(5, 2.0f, 0.0f);
  ota::MotorSample s;
  ASSERT_TRUE(h.interpolate(5, s));
  EXPECT_FLOAT_EQ(s.q, 2.0f);
}

TEST(History, WrapIndexingCorrectness) {
  // Values equal to index make torn/wrong-slot reads visible.
  const std::size_t cap = 4;
  MotorStateHistory h(cap);
  for (int i = 0; i < 50; ++i) h.add(i, static_cast<float>(i * 7 % 97), 0.0f);
  ota::MotorSample s;
  for (int i = 46; i <= 49; ++i) {
    ASSERT_TRUE(h.interpolate(i, s));
    EXPECT_FLOAT_EQ(s.q, static_cast<float>(i * 7 % 97));
  }
}
