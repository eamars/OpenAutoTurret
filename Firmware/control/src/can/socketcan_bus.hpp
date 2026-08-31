#pragma once
// Minimal production SocketCAN transport for the CyberGear bus
// (architecture §8.1).
//
// Design:
//  - one RX thread reads raw frames and invokes a callback (the RX path is
//    never blocked by control work);
//  - TX is a plain non-blocking send from any thread (control thread);
//  - RAW filters are installed so we only receive CyberGear traffic of
//    interest plus CAN error frames for diagnostics (§8.2);
//  - Frame.id is the 29-bit extended identifier WITHOUT CAN_EFF_FLAG.
#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "can/can_netlink.hpp"
#include "common/time.hpp"

namespace ota::can {

struct RawFrame {
  uint32_t id{0};
  uint8_t dlc{8};
  uint8_t data[8]{};
  ota::TimeNs rx_ns{0};  // monotonic timestamp at receipt
};

struct BusStats {
  uint64_t rx_frames{0};
  uint64_t rx_error_frames{0};
  uint64_t tx_frames{0};
  uint64_t tx_failed{0};
  ota::TimeNs last_rx_ns{0};
};

class SocketCanBus {
 public:
  struct Options {
    std::string iface = "can0";
    uint32_t bitrate = 1000000;
    // If the interface is DOWN, bring it up (IFF_UP) and set bitrate.
    // If it is already UP, never reconfigure it (report a mismatch instead).
    bool bring_up_if_down = false;
    // Install the default CyberGear RAW filters (feedback/discovery/reg
    // responses for any motor + error frames).
    bool install_filters = true;
  };

  SocketCanBus() = default;
  ~SocketCanBus();
  SocketCanBus(const SocketCanBus&) = delete;
  SocketCanBus& operator=(const SocketCanBus&) = delete;

  bool open(const Options& opts, std::string& err);
  void close();

  // Frame callback runs on the RX thread. Must be fast and non-blocking.
  using FrameCallback = std::function<void(const RawFrame&)>;
  void set_frame_callback(FrameCallback cb);

  bool start_rx(std::string& err);
  void stop_rx();

  // Non-blocking transmit. Returns false (and counts) on failure.
  bool send(uint32_t ext_id, const uint8_t data[8], std::string* err = nullptr);

  BusStats stats() const;
  bool is_up() const;
  uint32_t bitrate() const;  // 0 if unknown
  CanIfState can_state() const;  // driver state (CAN_ERROR_ACTIVE etc.)
  const std::string& iface() const { return opts_.iface; }

 private:
  void rx_loop();
  bool configure_interface(std::string& err);

  Options opts_{};
  int fd_ = -1;
  int wake_fd_[2] = {-1, -1};
  std::atomic<bool> running_{false};
  std::thread rx_thread_;
  std::mutex cb_mtx_;
  FrameCallback cb_;

  mutable std::mutex stats_mtx_;
  BusStats stats_{};

  bool up_state_ = false;
  uint32_t bitrate_state_ = 0;
  CanIfState can_state_ = CanIfState::Unknown;
};

}  // namespace ota::can
