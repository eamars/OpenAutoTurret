// TimingStats tests (architecture §7.2, §55 control timing).
#include <gtest/gtest.h>

#include "common/timing_stats.hpp"

using ota::TimingStats;

TEST(TimingStats, EmptyReport) {
  TimingStats ts;
  auto r = ts.report();
  EXPECT_EQ(r.samples, 0u);
  EXPECT_EQ(r.deadline_misses, 0u);
}

TEST(TimingStats, PercentilesAndRate) {
  TimingStats ts(64);
  for (int i = 1; i <= 20; ++i) {
    ts.record_period(5000000 + i * 1000);  // ~200 Hz with small spread
  }
  ts.record_deadline_miss();
  ts.record_deadline_miss();
  auto r = ts.report();
  EXPECT_EQ(r.samples, 20u);
  EXPECT_EQ(r.deadline_misses, 2u);
  // mean of 5000000 + i*1000 for i = 1..20 is 5000000 + 10.5*1000.
  EXPECT_NEAR(r.mean_period_ns, 5010500.0, 1.0);
  EXPECT_NEAR(r.measured_hz, 1e9 / 5010500.0, 0.5);
  EXPECT_LE(r.p50_ns, r.p95_ns);
  EXPECT_LE(r.p95_ns, r.p99_ns);
  EXPECT_LE(r.p99_ns, r.worst_ns + 1.0);
}

TEST(TimingStats, RingWrapsWithoutCorruption) {
  TimingStats ts(8);
  for (int i = 0; i < 100; ++i) {
    ts.record_period(1000 * (i + 1));
  }
  auto r = ts.report();
  EXPECT_EQ(r.samples, 8u);
  EXPECT_NEAR(r.worst_ns, 100000.0, 1e-9);  // 100 * 1000
  // Mean of last 8: 93..100 (x1000 ns)
  EXPECT_NEAR(r.mean_period_ns, 1000.0 * 96.5, 1e-6);
}

TEST(TimingStats, Reset) {
  TimingStats ts(8);
  ts.record_period(1000);
  ts.record_deadline_miss();
  ts.reset();
  auto r = ts.report();
  EXPECT_EQ(r.samples, 0u);
  EXPECT_EQ(r.deadline_misses, 0u);
}
