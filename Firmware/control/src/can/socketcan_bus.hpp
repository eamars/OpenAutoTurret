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
#include "can/can_transport.hpp"
#include "common/time.hpp"

namespace ota::can {

// RawFrame / BusStats / CanTransport live in can_transport.hpp (the
// PHY-agnostic boundary); this class is the SocketCAN implementation.
class SocketCanBus : public CanTransport {
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
  // Configuration-carrying ctor so the transport can be created through the
  // CanTransport interface (start() then performs open+start_rx).
  explicit SocketCanBus(Options opts) : opts_(std::move(opts)) {}
  ~SocketCanBus() override;
  SocketCanBus(const SocketCanBus&) = delete;
  SocketCanBus& operator=(const SocketCanBus&) = delete;

  // CanTransport: open + start RX in one step (uses opts_ from the ctor).
  bool start(std::string& err) override;
  void stop() override;
  const char* kind() const override { return "socketcan"; }
  std::string device() const override { return opts_.iface; }

  bool open(const Options& opts, std::string& err);
  void close();

  // Frame callback runs on the RX thread. Must be fast and non-blocking.
  using FrameCallback = std::function<void(const RawFrame&)>;
  void set_frame_callback(FrameCallback cb) override;

  bool start_rx(std::string& err);
  void stop_rx();

  // Non-blocking transmit. Returns false (and counts) on failure.
  bool send(uint32_t ext_id, const uint8_t data[8],
            std::string* err = nullptr) override;

  BusStats stats() const override;
  bool is_up() const override;
  uint32_t bitrate() const;  // 0 if unknown
  CanIfState can_state() const override;  // driver state (CAN_ERROR_ACTIVE etc.)
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
