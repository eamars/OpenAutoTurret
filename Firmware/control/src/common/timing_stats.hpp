#pragma once
// Control-loop timing instrumentation (architecture §7.2, §43.1).
//
// Collects cycle-period samples and reports p50/p95/p99/worst plus
// deadline-miss counts so the production loop rate can be chosen from
// measurement, not assumption.
#include <algorithm>
#include <cstdint>
#include <vector>

#include "types.hpp"

namespace ota {

struct TimingReport {
  uint64_t samples{0};
  double p50_ns{0.0};
  double p95_ns{0.0};
  double p99_ns{0.0};
  double worst_ns{0.0};
  uint64_t deadline_misses{0};
  double mean_period_ns{0.0};
  double measured_hz{0.0};
};

class TimingStats {
 public:
  explicit TimingStats(std::size_t capacity = 4096) : ring_(capacity) {}

  // Record the observed period of one control cycle.
  void record_period(TimeNs period_ns) {
    ring_[head_ % ring_.size()] = period_ns;
    ++head_;
  }

  // Record a missed deadline (cycle started after its absolute deadline).
  void record_deadline_miss() { ++deadline_misses_; }

  void reset() {
    head_ = 0;
    deadline_misses_ = 0;
  }

  TimingReport report() const {
    TimingReport r;
    const std::size_t n = count();
    r.samples = n;
    r.deadline_misses = deadline_misses_;
    if (n == 0) return r;

    // Snapshot the most recent `n` samples in chronological order and
    // derive all statistics from the window (not from all history).
    std::vector<TimeNs> v(n);
    const std::size_t start = (head_ > ring_.size()) ? (head_ - ring_.size()) : 0;
    double sum = 0.0;
    TimeNs worst = 0;
    for (std::size_t i = 0; i < n; ++i) {
      const TimeNs p = ring_[(start + i) % ring_.size()];
      v[i] = p;
      sum += static_cast<double>(p);
      if (p > worst) worst = p;
    }

    std::vector<TimeNs> s = v;
    std::sort(s.begin(), s.end());
    r.p50_ns = static_cast<double>(s[static_cast<std::size_t>(0.50 * (n - 1))]);
    r.p95_ns = static_cast<double>(s[static_cast<std::size_t>(0.95 * (n - 1))]);
    r.p99_ns = static_cast<double>(s[static_cast<std::size_t>(0.99 * (n - 1))]);
    r.worst_ns = static_cast<double>(worst);
    r.mean_period_ns = sum / static_cast<double>(n);
    if (r.mean_period_ns > 0.0) {
      r.measured_hz = 1e9 / r.mean_period_ns;
    }
    return r;
  }

 private:
  std::size_t count() const {
    return (head_ > ring_.size()) ? ring_.size() : head_;
  }

  std::vector<TimeNs> ring_;
  std::size_t head_{0};
  uint64_t deadline_misses_{0};
};

}  // namespace ota
