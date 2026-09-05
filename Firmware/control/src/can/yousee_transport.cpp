#include "can/yousee_transport.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <poll.h>
#include <thread>

#include "common/time.hpp"
#include "spdlog/spdlog.h"

namespace ota::can {

// ---------------------------------------------------------------- YouseeCodec

void YouseeCodec::feed(const uint8_t* data, size_t n, ota::TimeNs now_ns) {
  buf_.insert(buf_.end(), data, data + n);
  size_t i = 0;
  while (true) {
    const size_t avail = buf_.size() - i;
    if (avail < 2) break;
    if (buf_[i] != 'A' || buf_[i + 1] != 'T') {  // resync: drop one byte
      ++i;
      ++resyncs_;
      continue;
    }
    if (avail >= 3 && buf_[i + 2] == '+') {  // AT text reply: consume line
      const size_t cr = buf_.size();
      size_t j = i + 2;
      bool found = false;
      for (; j + 1 < buf_.size(); ++j)
        if (buf_[j] == '\r' && buf_[j + 1] == '\n') {
          found = true;
          break;
        }
      if (!found) break;  // line incomplete; await more bytes
      i = j + 2;
      (void)cr;
      continue;
    }
    if (avail < 2 + 4 + 1) break;  // need header + dlc
    const uint8_t dlc = buf_[i + 6];
    if (dlc > 8) {  // bogus header (payload coincidence): skip past 'AT'
      i += 2;
      ++resyncs_;
      continue;
    }
    const size_t need = 2 + 4 + 1 + dlc + 2;
    if (avail < need) break;
    if (buf_[i + 7 + dlc] != '\r' || buf_[i + 8 + dlc] != '\n') {
      i += 2;
      ++resyncs_;
      continue;
    }
    RawFrame f{};
    const uint32_t cid = (static_cast<uint32_t>(buf_[i + 2]) << 24) |
                         (static_cast<uint32_t>(buf_[i + 3]) << 16) |
                         (static_cast<uint32_t>(buf_[i + 4]) << 8) |
                         static_cast<uint32_t>(buf_[i + 5]);
    f.id = cid >> 3;  // adapter packs (id << 3 | ext)
    f.dlc = dlc;
    std::memcpy(f.data, &buf_[i + 7], dlc);
    f.rx_ns = now_ns;
    out_(f);
    i += need;
  }
  buf_.erase(buf_.begin(), buf_.begin() + i);
}

void YouseeCodec::encode(uint32_t ext_id, const uint8_t* data, uint8_t dlc,
                         std::vector<uint8_t>& out) {
  const uint32_t cid = (ext_id << 3) | (1u << 2);  // extended frame
  out.clear();
  out.reserve(2 + 4 + 1 + dlc + 2);
  out.push_back('A');
  out.push_back('T');
  out.push_back(static_cast<uint8_t>(cid >> 24));
  out.push_back(static_cast<uint8_t>(cid >> 16));
  out.push_back(static_cast<uint8_t>(cid >> 8));
  out.push_back(static_cast<uint8_t>(cid));
  out.push_back(dlc);
  out.insert(out.end(), data, data + dlc);
  out.push_back('\r');
  out.push_back('\n');
}

// -------------------------------------------------------------- YouseeTransport

bool YouseeTransport::uart_speed(int baud, uint32_t& out_speed) {
  switch (baud) {
    case 9600: out_speed = B9600; return true;
    case 19200: out_speed = B19200; return true;
    case 38400: out_speed = B38400; return true;
    case 57600: out_speed = B57600; return true;
    case 115200: out_speed = B115200; return true;
    case 230400: out_speed = B230400; return true;
    case 460800: out_speed = B460800; return true;
    case 921600: out_speed = B921600; return true;
    default: return false;
  }
}

bool YouseeTransport::at_cmd(const std::string& cmd, const std::string& expect,
                             std::string& err) {
  size_t off = 0;
  while (off < cmd.size()) {
    const ssize_t w = ::write(fd_, cmd.data() + off, cmd.size() - off);
    if (w <= 0) {
      err = "yousee: write failed for " + cmd + ": " + std::strerror(errno);
      return false;
    }
    off += static_cast<size_t>(w);
  }
  const std::string err_line = "ERROR";
  std::string line;
  for (int tries = 0; tries < 30; ++tries) {  // 3 s budget (30 x 100 ms)
    char buf[64];
    const ssize_t n = ::read(fd_, buf, sizeof(buf));
    if (n > 0) {
      line.append(buf, static_cast<size_t>(n));
      // The adapter is a chatty AT device: it may emit unsolicited lines (a boot banner, a
      // stale ``OK`` from a previous ``AT+AT`` data-mode transition, or an interleaved AT
      // reply) before the response to THIS command. Reacting to the first complete line is
      // how a clean handshake intermittently falsifies — the very failure that left the CAN
      // link dead with "AT mismatch: sent 'AT+CG' expected 'OK' got 'ERROR'". Consume the
      // whole CRLF line and keep reading until the expected response (or a bare ERROR) shows
      // up, instead of trusting that the first line is ours.
      size_t cr;
      while ((cr = line.find("\r\n")) != std::string::npos) {
        std::string got = line.substr(0, cr);
        line.erase(0, cr + 2);
        if (got == expect) return true;
        if (got == err_line) {
          err = "yousee: AT failed: sent '" + cmd + "' got 'ERROR'";
          return false;
        }
        // Not ours (a boot banner, an interleaved AT reply, a stale OK): drop it and keep
        // reading. The command's own response is still to come.
      }
    }
  }
  err = "yousee: AT mismatch: sent '" + cmd + "' expected '" + expect + "' got '" + line + "'";
  return false;
}

bool YouseeTransport::start(std::string& err) {
  if (fd_ >= 0) return true;  // already started
  fd_ = ::open(opts_.port.c_str(), O_RDWR | O_NOCTTY);
  if (fd_ < 0) {
    err = "yousee: open " + opts_.port + ": " + std::strerror(errno);
    return false;
  }
  termios tio{};
  if (::tcgetattr(fd_, &tio) != 0) {
    err = "yousee: tcgetattr: " + std::string(std::strerror(errno));
    ::close(fd_);
    fd_ = -1;
    return false;
  }
  ::cfmakeraw(&tio);  // byte-pure: no echo, no CR/NL translation
  uint32_t spd = B921600;
  if (!uart_speed(opts_.uart_baud, spd)) {
    err = "yousee: unsupported uart_baud " + std::to_string(opts_.uart_baud);
    ::close(fd_);
    fd_ = -1;
    return false;
  }
  ::cfsetispeed(&tio, static_cast<speed_t>(spd));
  ::cfsetospeed(&tio, static_cast<speed_t>(spd));
  tio.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
  tio.c_cc[VMIN] = 0;   // read returns after VTIME deciseconds (0.1 s)
  tio.c_cc[VTIME] = 1;
  if (::tcsetattr(fd_, TCSANOW, &tio) != 0) {
    err = "yousee: tcsetattr: " + std::string(std::strerror(errno));
    ::close(fd_);
    fd_ = -1;
    return false;
  }
  ::tcflush(fd_, TCIOFLUSH);

  if (!opts_.skip_at_init) {
    const std::string br = std::to_string(opts_.can_bitrate);
    // Cold-start retry. The yousee adapter (CH340) and the CyberGear drives need a moment after
    // the port opens before they accept the AT handshake; a freshly-backed adapter can answer the
    // very first ``AT+CG`` with a bare ERROR (or nothing at all) while it enumerates, and then
    // answer OK a few dozen milliseconds later. The python bring-up tool always wins this race
    // because by the time a human runs it the adapter is already warm. Retry the whole sequence a
    // few times with a settle delay so a cold station still comes up, and only give up — and
    // report it as the fatal CAN open failure — once the budget is spent.
    constexpr int kInitAttempts = 3;
    std::string last_err;
    for (int attempt = 1; attempt <= kInitAttempts; ++attempt) {
      std::string aerr;
      const bool ok = at_cmd("AT+CG\r\n", "OK", aerr) &&
                      at_cmd("AT+CAN_BAUD=" + br + "\r\n", "OK", aerr) &&
                      at_cmd("AT+CAN_BAUD=?\r\n", "+CAN_BAUD:" + br, aerr) &&
                      at_cmd("AT+AT\r\n", "OK", aerr);
      if (ok) {
        last_err.clear();
        break;
      }
      last_err = aerr;
      spdlog::warn("yousee: AT init attempt {}/{} failed: {}", attempt, kInitAttempts, last_err);
      ::tcflush(fd_, TCIOFLUSH);
      std::this_thread::sleep_for(std::chrono::milliseconds(150));
      if (attempt == kInitAttempts) {
        err = last_err;
        ::close(fd_);
        fd_ = -1;
        return false;
      }
    }
  }

  running_.store(true);
  rx_thread_ = std::thread([this] { rx_loop(); });
  return true;
}

void YouseeTransport::stop() {
  const bool was = running_.exchange(false);
  if (rx_thread_.joinable()) rx_thread_.join();
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  (void)was;
}

void YouseeTransport::set_frame_callback(FrameCallback cb) {
  std::lock_guard<std::mutex> lk(cb_mtx_);
  cb_ = std::move(cb);
}

bool YouseeTransport::send(uint32_t ext_id, const uint8_t data[8],
                           std::string* err) {
  std::lock_guard<std::mutex> lk(tx_mtx_);
  if (fd_ < 0 || !running_.load()) {
    if (err) *err = "yousee: transport not open";
    std::lock_guard<std::mutex> sl(stats_mtx_);
    ++stats_.tx_failed;
    return false;
  }
  std::vector<uint8_t> buf;
  YouseeCodec::encode(ext_id, data, 8, buf);
  size_t off = 0;
  while (off < buf.size()) {
    const ssize_t w = ::write(fd_, buf.data() + off, buf.size() - off);
    if (w < 0 && (errno == EINTR)) continue;
    if (w <= 0) {
      if (err) *err = std::string("yousee: write: ") + std::strerror(errno);
      std::lock_guard<std::mutex> sl(stats_mtx_);
      ++stats_.tx_failed;
      return false;
    }
    off += static_cast<size_t>(w);
  }
  std::lock_guard<std::mutex> sl(stats_mtx_);
  ++stats_.tx_frames;
  return true;
}

BusStats YouseeTransport::stats() const {
  std::lock_guard<std::mutex> lk(stats_mtx_);
  BusStats s = stats_;
  s.rx_error_frames += codec_.resyncs();  // RX-thread value; benign race
  return s;
}

void YouseeTransport::on_frame(const RawFrame& f) {
  {
    std::lock_guard<std::mutex> lk(stats_mtx_);
    ++stats_.rx_frames;
    stats_.last_rx_ns = f.rx_ns;
  }
  FrameCallback cb;
  {
    std::lock_guard<std::mutex> lk(cb_mtx_);
    cb = cb_;
  }
  if (cb) cb(f);
}

void YouseeTransport::rx_loop() {
  uint8_t buf[512];
  while (running_.load()) {
    const ssize_t n = ::read(fd_, buf, sizeof(buf));
    if (n > 0) {
      codec_.feed(buf, static_cast<size_t>(n), ota::now_monotonic_ns());
      continue;
    }
    if (n < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK))
      continue;
    if (n < 0 && errno == EBADF) break;  // stop() closed the fd
    if (n < 0) {                          // EIO etc: device gone
      running_.store(false);
      break;
    }
    // n == 0: VTIME expired, nothing pending.
  }
}

}  // namespace ota::can
