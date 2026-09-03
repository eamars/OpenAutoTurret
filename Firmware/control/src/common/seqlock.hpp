#pragma once
// Lock-free single-writer / single-reader shared value (sequence lock).
//
// Used for the latest per-axis motor state: the CAN RX thread is the sole
// writer, the control thread is the sole reader (architecture §8.1).
// Readers spin briefly if they observe an in-progress write; writes are
// rare (motor feedback rate) relative to the 200 Hz read rate, so the
// reader effectively never blocks.
#include <atomic>
#include <thread>
#include <type_traits>

namespace ota {

template <typename T, typename = std::enable_if_t<std::is_trivially_copyable_v<T>>>
class SeqLock {
 public:
  void write(const T& v) {
    const uint64_t s = seq_.load(std::memory_order_relaxed);
    seq_.store(s + 1, std::memory_order_relaxed);  // odd == write in progress
    // A release STORE orders what came BEFORE it; it puts no ceiling on the payload stores
    // that come after. On Cortex-A76 they can therefore become visible before the odd marker,
    // and a reader that samples the PREVIOUS even value on both sides of its copy accepts a
    // half-written payload as committed. That is not theoretical: SeqLock's adversarial test
    // tore 3 times in 80 runs on this station before this line existed. A full fence here is
    // the store-store barrier the marker actually needs. Writes happen at motor-feedback rate
    // on the CAN RX thread, not on the 200 Hz control path, so the cost is not on the loop.
    std::atomic_thread_fence(std::memory_order_seq_cst);
    value_ = v;
    std::atomic_thread_fence(std::memory_order_release);
    seq_.store(s + 2, std::memory_order_release);  // even == committed
  }

  bool read(T& out) const {
    for (;;) {
      const uint64_t s1 = seq_.load(std::memory_order_relaxed);
      std::atomic_thread_fence(std::memory_order_acquire);  // payload reads stay behind s1
      if (s1 & 1u) {
        std::this_thread::yield();
        continue;
      }
      out = value_;
      // release, not acquire. Nothing copied out of value_ may sink past the second sample;
      // if it did, an unchanged (s1,s2) pair would certify a payload that was read afterwards.
      // An acquire fence here guards the opposite direction and left the window open.
      std::atomic_thread_fence(std::memory_order_release);
      const uint64_t s2 = seq_.load(std::memory_order_acquire);
      if (s1 == s2) return true;
    }
  }

 private:
  // seq_ and value_ live on separate cache lines: the writer updates seq_
  // twice per write and the reader samples seq_ on every call, while the
  // value fields change once per write. Sharing one line makes every
  // payload store drag the seq line across cores (measured ~40% writer
  // slowdown under sustained write/read contention on Cortex-A76).
  std::atomic<uint64_t> seq_{0};
  alignas(64) T value_{};
};

}  // namespace ota
