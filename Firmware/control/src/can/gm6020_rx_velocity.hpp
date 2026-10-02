#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include "common/time.hpp"

namespace ota::gm6020 {
// Transport samples, not control ticks. Position must already be unwrapped
// and must not change origin when the session reference is established.
class RxVelocity {
 public:
  void reset() { count_ = next_ = 0; }
  bool observe(double position, TimeNs stamp) {
    if (!std::isfinite(position) || stamp <= 0) return false;
    if (count_ && stamp <= samples_[(next_ + samples_.size()-1)%samples_.size()].stamp)
      return false;
    samples_[next_] = {position, stamp};
    next_ = (next_+1)%samples_.size();
    count_ = std::min(count_+1, samples_.size());
    return true;
  }
  double estimate(int window_ms) const {
    if (count_ < 2 || window_ms < 1 || window_ms > 80) return NAN;
    const auto latest = samples_[(next_+samples_.size()-1)%samples_.size()];
    for (std::size_t ago=1; ago<count_; ++ago) {
      const auto older = samples_[(next_+samples_.size()-1-ago)%samples_.size()];
      const auto elapsed = latest.stamp-older.stamp;
      if (elapsed >= window_ms*1'000'000LL)
        return (latest.position-older.position)/(elapsed*1e-9);
    }
    return NAN;
  }
 private:
  struct Sample { double position = 0; TimeNs stamp = 0; };
  std::array<Sample,128> samples_{};
  std::size_t count_ = 0, next_ = 0;
};
}
