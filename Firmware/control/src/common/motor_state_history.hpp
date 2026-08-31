#pragma once
// High-rate axis state history ring (architecture §8.4, §11.1).
//
// Retains ~1 s of axis state at feedback rate. Purpose:
//  - interpolate motor pose at the camera capture timestamp (core req §11);
//  - latency diagnosis; settling/overshoot analysis.
//
// Concurrency model: single producer (CAN RX thread) appends samples;
// single consumer (control thread) interpolates. A monotonically increasing
// "published" commit index makes every read observe only fully-committed
// samples, so no mutex is needed and the reader never sees torn data.
#include <atomic>
#include <cstddef>
#include <vector>

#include "types.hpp"

namespace ota {

struct MotorSample {
  TimeNs t_ns{0};
  float q{0.0f};  // rad (raw/mechanical coordinate)
  float v{0.0f};  // rad/s
};

class MotorStateHistory {
 public:
  explicit MotorStateHistory(std::size_t capacity)
      : ring_(capacity), capacity_(capacity == 0 ? 2 : capacity) {}

  // Clear and resize. Must be called with no concurrent producer/reader
  // (i.e. before the RX thread starts or after it has stopped).
  void reset(std::size_t capacity) {
    ring_.assign(capacity == 0 ? 2 : capacity, MotorSample{});
    capacity_ = capacity == 0 ? 2 : capacity;
    published_.store(0, std::memory_order_relaxed);
  }

  // Producer (RX thread) only.
  void add(TimeNs t_ns, float q, float v) {
    const std::size_t pub = published_.load(std::memory_order_relaxed);
    ring_[pub % capacity_] = MotorSample{t_ns, q, v};
    published_.store(pub + 1, std::memory_order_release);
  }

  // Number of samples currently observable (<= capacity).
  std::size_t size() const {
    const std::size_t pub = published_.load(std::memory_order_acquire);
    return pub < capacity_ ? pub : capacity_;
  }

  // Linear interpolation at time t (architecture §11.1).
  // Returns false if the history cannot cover t — the caller must then flag
  // the measurement as timing-invalid, never silently use a newer pose.
  bool interpolate(TimeNs t, MotorSample& out) const {
    const std::size_t pub = published_.load(std::memory_order_acquire);
    if (pub < 2) return false;
    const std::size_t n = (pub < capacity_) ? pub : capacity_;
    auto at = [&](std::size_t i) -> const MotorSample& {
      return ring_[(pub - n + i) % capacity_];
    };
    const MotorSample& first = at(0);
    const MotorSample& last = at(n - 1);
    if (t < first.t_ns || t > last.t_ns) return false;

    // Largest i with at(i).t_ns <= t.
    std::size_t lo = 0, hi = n - 1;
    while (lo < hi) {
      const std::size_t mid = (lo + hi + 1) / 2;
      if (at(mid).t_ns <= t) {
        lo = mid;
      } else {
        hi = mid - 1;
      }
    }
    const std::size_t i1 = lo;
    // a is the newest sample at or before t; b the next sample after (or a
    // itself when t is exactly the newest sample).
    const MotorSample& a = at(i1);
    const MotorSample& b = (i1 + 1 < n) ? at(i1 + 1) : at(i1);
    if (b.t_ns == a.t_ns) {
      out = b;
      return true;
    }
    const double alpha = static_cast<double>(t - a.t_ns) /
                         static_cast<double>(b.t_ns - a.t_ns);
    out = MotorSample{t,
                      a.q + static_cast<float>(alpha) * (b.q - a.q),
                      a.v + static_cast<float>(alpha) * (b.v - a.v)};
    return true;
  }

 private:
  std::vector<MotorSample> ring_;
  std::atomic<std::size_t> published_{0};
  std::size_t capacity_;
};

}  // namespace ota
