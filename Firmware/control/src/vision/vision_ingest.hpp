#pragma once
// OpenAutoTurret — vision ingest (architecture §5.1, §6.1, Part-2 S1).
//
// controld BINDS a SOCK_SEQPACKET Unix socket; visiond is the client that
// publishes 58-byte fixed-size TargetMeasurement datagrams (§6.2) on it. Video
// frames never traverse this path (§6.1).
//
// Threading model (§46 loop discipline):
//   * This component owns its own NON-real-time thread(s) (accept + one reader
//     per publisher connection). It NEVER runs inside the 200 Hz control cycle.
//   * Hand-off to the control thread is a plain data copy through the handler
//     callback (ControlLoop::feed_measurement, which is thread-safe and
//     non-blocking) plus the lock-free VisionLink counters below, which the
//     control thread only READS for the §6.3 snapshot.
//
// This component is transport-only: it decodes and counts. It makes no safety
// decision and never touches CAN (the same rule visiond follows in reverse).
#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <poll.h>
#include <string>
#include <sys/socket.h>
#include <sys/un.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include "common/types.hpp"
#include "tracking/target_measurement.hpp"

namespace ota {
namespace vision {

// Lock-free view of the vision link. WRITTEN by the ingest thread(s), READ by
// the control thread (telemetry snapshot) and by main (the 1 Hz status log).
// Counters are monotonic for the process lifetime; `connected` reflects the
// number of live publisher connections (>0 == a visiond is publishing).
class VisionLink {
 public:
  struct Stats {
    bool connected = false;
    int32_t clients = 0;
    uint64_t frames = 0;          // datagrams decoded OK
    uint64_t dropped = 0;         // wrong-size / undecodable datagrams
    uint64_t last_frame_sequence = 0;
    TimeNs last_arrival_ns = 0;   // host monotonic time of the last good frame
  };

  void note_client_added() { clients_.fetch_add(1, std::memory_order_relaxed); }
  void note_client_removed() {
    const int32_t n = clients_.fetch_sub(1, std::memory_order_relaxed) - 1;
    if (n < 0) clients_.store(0, std::memory_order_relaxed);
  }
  void note_frame(uint64_t frame_sequence, TimeNs arrival_ns) {
    last_seq_.store(frame_sequence, std::memory_order_relaxed);
    last_arrival_ns_.store(arrival_ns, std::memory_order_relaxed);
    frames_.fetch_add(1, std::memory_order_relaxed);
  }
  void note_dropped() { dropped_.fetch_add(1, std::memory_order_relaxed); }

  Stats stats() const {
    Stats s;
    s.clients = clients_.load(std::memory_order_relaxed);
    s.connected = s.clients > 0;
    s.frames = frames_.load(std::memory_order_relaxed);
    s.dropped = dropped_.load(std::memory_order_relaxed);
    s.last_frame_sequence = last_seq_.load(std::memory_order_relaxed);
    s.last_arrival_ns = last_arrival_ns_.load(std::memory_order_relaxed);
    return s;
  }

 private:
  std::atomic<int32_t> clients_{0};
  std::atomic<uint64_t> frames_{0};
  std::atomic<uint64_t> dropped_{0};
  std::atomic<uint64_t> last_seq_{0};
  std::atomic<int64_t> last_arrival_ns_{0};
};

// The ingest server: bind + accept + decode + hand off.
class VisionIngest {
 public:
  struct Config {
    std::string socket_path = "/tmp/ota_vision.sock";
    // visiond is the only publisher (§5.1); extra connections are accepted and
    // dropped (a second publisher would interleave measurements with no
    // ordering guarantee, which the control loop must never see).
    int max_clients = 1;
  };
  using MeasurementHandler =
      std::function<void(const TargetMeasurement& m)>;

  VisionIngest(Config cfg, VisionLink* link, MeasurementHandler handler)
      : cfg_(std::move(cfg)), link_(link), handler_(std::move(handler)) {}
  ~VisionIngest() { stop(); }

  // Bind + listen + spawn the accept thread. Returns false + err on failure.
  bool start(std::string& err);
  // Stop the threads, close + unlink the socket. Idempotent.
  void stop();

  bool running() const { return running_.load(); }
  const Config& config() const { return cfg_; }

 private:
  void accept_loop();
  void client_loop(int cfd);

  Config cfg_;
  VisionLink* link_ = nullptr;
  MeasurementHandler handler_;
  int listen_fd_ = -1;
  std::thread accept_thread_;
  std::atomic<bool> running_{false};

  std::mutex clients_mu_;  // guards client_fds_ / client_threads_
  std::vector<int> client_fds_;
  std::vector<std::thread> client_threads_;
};

}  // namespace vision
}  // namespace ota
