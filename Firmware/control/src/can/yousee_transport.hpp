#pragma once
// yousee USB-CAN (AT mode) transport — the second PHY (architecture §8.1).
//
// Hardware: "yousee/yourcee" USB-CAN AT adapter (CH340 CDC-serial). The
// adapter presents a UART; CAN frames are framed as
//     'A' 'T' | id_be32 = (ext_id << 3) | (1 << 2 /* extended */)
//           | dlc | data[dlc] | CRLF
// and plain AT text replies ('AT+...\r\n') may interleave. Recovered from
// legacy/yourcee_usb_to_can.py (the python-can SerialBus driver).
//
// Setup sequence (verified against the real adapter 2026-09-02):
//   AT+CG\r\n -> OK        AT+CAN_BAUD=<n>\r\n -> OK
//   AT+CAN_BAUD=?\r\n -> +CAN_BAUD:<n>         AT+AT\r\n -> OK (data mode)
//
// Limits accepted for development: the adapter exposes no bus error state
// (can_state() == Unknown; feedback-staleness supervision covers this) and
// corrupted frames are invisible to the host except as codec resyncs
// (surfaced via BusStats::rx_error_frames — an INDEPENDENT corruption
// counter for the bus, unlike the MCP2515 error counters).
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "can/can_transport.hpp"

namespace ota::can {

// Pure, hardware-free framing codec (unit-testable). Byte-exact with the
// legacy python driver.
class YouseeCodec {
 public:
  using FrameOut = std::function<void(const RawFrame&)>;
  explicit YouseeCodec(FrameOut out) : out_(std::move(out)) {}

  // Feed a serial chunk; emits every complete frame. Garbage is dropped with
  // resync accounting; AT text replies are consumed silently.
  void feed(const uint8_t* data, size_t n, ota::TimeNs now_ns);

  // Append the adapter framing for one CAN frame (dlc<=8) to `out`.
  static void encode(uint32_t ext_id, const uint8_t* data, uint8_t dlc,
                     std::vector<uint8_t>& out);

  uint64_t resyncs() const { return resyncs_; }

 private:
  std::vector<uint8_t> buf_;
  FrameOut out_;
  uint64_t resyncs_{0};
};

class YouseeTransport : public CanTransport {
 public:
  struct Options {
    std::string port = "/dev/ttyUSB0";
    int uart_baud = 921600;
    uint32_t can_bitrate = 1000000;
    bool skip_at_init = false;  // adapter already configured (tests/recovery)
  };

  explicit YouseeTransport(Options opts) : opts_(std::move(opts)),
                                           codec_([this](const RawFrame& f) {
                                             on_frame(f);
                                           }) {}
  ~YouseeTransport() override { stop(); }
  YouseeTransport(const YouseeTransport&) = delete;
  YouseeTransport& operator=(const YouseeTransport&) = delete;

  bool start(std::string& err) override;
  void stop() override;

  void set_frame_callback(FrameCallback cb) override;
  bool send(uint32_t ext_id, const uint8_t data[8],
            std::string* err = nullptr) override;

  BusStats stats() const override;
  bool is_up() const override { return fd_ >= 0 && running_.load(); }
  CanIfState can_state() const override { return CanIfState::Unknown; }
  const char* kind() const override { return "yousee"; }
  std::string device() const override { return opts_.port; }

  // Exposed for tests: map uart baud to termios speed_t (false if unknown).
  static bool uart_speed(int baud, uint32_t& out_speed);

 private:
  bool at_cmd(const std::string& cmd, const std::string& expect,
              std::string& err);
  void on_frame(const RawFrame& f);
  void rx_loop();

  Options opts_;
  int fd_ = -1;
  std::atomic<bool> running_{false};
  std::thread rx_thread_;

  std::mutex cb_mtx_;
  FrameCallback cb_;
  std::mutex tx_mtx_;  // serialize whole-frame writes

  YouseeCodec codec_;  // touched only by the RX thread

  mutable std::mutex stats_mtx_;
  BusStats stats_{};
};

}  // namespace ota::can
