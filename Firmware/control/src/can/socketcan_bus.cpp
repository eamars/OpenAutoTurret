#include "socketcan_bus.hpp"

#include <errno.h>
#include <net/if.h>
#include <poll.h>
#include <unistd.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <cstring>

#include "can/can_netlink.hpp"
#include "common/types.hpp"

namespace ota::can {

namespace {
constexpr uint32_t kEffFlag = CAN_EFF_FLAG;

// Build the default filter set: CyberGear comm types 0/2/17/18 (any motor)
// plus CAN error frames.
std::vector<struct can_filter> default_filters() {
  auto type_filter = [](uint8_t comm_type) {
    struct can_filter f{};
    f.can_id = (static_cast<uint32_t>(comm_type) << 24) | kEffFlag;
    // Match on comm type only; ignore data2 and target.
    f.can_mask = (0x1F << 24) | kEffFlag;
    return f;
  };
  std::vector<struct can_filter> v;
  v.push_back(type_filter(0));   // discovery responses
  v.push_back(type_filter(2));   // motor feedback
  v.push_back(type_filter(17));  // read-reg responses
  v.push_back(type_filter(18));  // write-reg echoes
  struct can_filter errf{};
  errf.can_id = CAN_ERR_FLAG;
  errf.can_mask = CAN_ERR_FLAG;
  v.push_back(errf);
  return v;
}
}  // namespace

SocketCanBus::~SocketCanBus() { close(); }

void SocketCanBus::close() {
  stop_rx();
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  if (wake_fd_[0] >= 0) {
    ::close(wake_fd_[0]);
    ::close(wake_fd_[1]);
    wake_fd_[0] = wake_fd_[1] = -1;
  }
}

bool SocketCanBus::configure_interface(std::string& err) {
    // State comes from netlink (RTM_GETLINK): UP flag, driver state, bitrate.
    CanIfInfo info{};
    if (!netlink_query_can(opts_.iface, info, err)) return false;
    if (!info.exists) {
      err = "interface " + opts_.iface + " does not exist";
      return false;
    }
    if (!info.is_can) {
      err = "interface " + opts_.iface + " is not a CAN interface";
      return false;
    }

    if (!info.up && opts_.bring_up_if_down) {
      // Bitrate changes require the interface down; the netlink helper does
      // the combined set-bitrate+up (RTM_NEWLINK, same as `ip link set ...
      // up type can bitrate N`).
      if (!netlink_can_set_bitrate_up(opts_.iface, opts_.bitrate, err)) {
        return false;
      }
    }

    // Re-read actual state after any bring-up.
    if (!netlink_query_can(opts_.iface, info, err)) return false;
    up_state_ = info.up;
    bitrate_state_ = info.bitrate;
    can_state_ = info.state;
    return true;
  }

  bool SocketCanBus::open(const Options& opts, std::string& err) {
  close();
  opts_ = opts;

  fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd_ < 0) {
    err = std::string("socket(PF_CAN) failed: ") + std::strerror(errno);
    return false;
  }

  if (!configure_interface(err)) return false;

  struct ifreq ifr{};
  std::strncpy(ifr.ifr_name, opts_.iface.c_str(), IFNAMSIZ - 1);
  if (::ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
    err = std::string("SIOCGIFINDEX failed: ") + std::strerror(errno);
    close();
    return false;
  }
  struct sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (::bind(fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) <
      0) {
    err = std::string("bind(") + opts_.iface + ") failed: " + std::strerror(errno);
    close();
    return false;
  }

  if (opts_.install_filters) {
    const auto filters = default_filters();
    if (::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(),
                     filters.size() * sizeof(struct can_filter)) < 0) {
      err = std::string("setsockopt(CAN_RAW_FILTER) failed: ") + std::strerror(errno);
      close();
      return false;
    }
  }

  // Non-blocking for TX; RX uses poll() on the same fd.
  const int fl = ::fcntl(fd_, F_GETFL, 0);
  ::fcntl(fd_, F_SETFL, fl | O_NONBLOCK);

  // Self-pipe for clean RX-thread wakeup/shutdown.
  if (pipe(wake_fd_) < 0) {
    err = std::string("pipe() failed: ") + std::strerror(errno);
    close();
    return false;
  }
  ::fcntl(wake_fd_[0], F_SETFL, O_NONBLOCK);
  ::fcntl(wake_fd_[1], F_SETFL, O_NONBLOCK);

  return true;
}

void SocketCanBus::set_frame_callback(FrameCallback cb) {
  std::lock_guard<std::mutex> lk(cb_mtx_);
  cb_ = std::move(cb);
}

bool SocketCanBus::start_rx(std::string& err) {
  if (fd_ < 0) {
    err = "bus not open";
    return false;
  }
  if (running_.load()) return true;
  running_.store(true);
  rx_thread_ = std::thread([this] { rx_loop(); });
  return true;
}

void SocketCanBus::stop_rx() {
  if (!running_.exchange(false)) {
    // Still join a thread that may be finishing.
    if (rx_thread_.joinable()) rx_thread_.join();
    return;
  }
  if (wake_fd_[1] >= 0) {
    char c = 1;
    (void)::write(wake_fd_[1], &c, 1);
  }
  if (rx_thread_.joinable()) rx_thread_.join();
}

void SocketCanBus::rx_loop() {
  uint8_t tmp[8];
  while (running_.load()) {
    struct pollfd pfds[2]{};
    pfds[0].fd = fd_;
    pfds[0].events = POLLIN;
    pfds[1].fd = wake_fd_[0];
    pfds[1].events = POLLIN;

    const int pr = ::poll(pfds, 2, 50);
    if (pr < 0) {
      if (errno == EINTR) continue;
      break;
    }
    if (pfds[1].revents & POLLIN) {
      (void)::read(wake_fd_[0], tmp, sizeof(tmp));  // drain
      if (!running_.load()) break;
    }
    if (!(pfds[0].revents & POLLIN)) continue;

    // Drain available frames (bounded batch to keep latency low).
    for (int i = 0; i < 64; ++i) {
      struct can_frame f{};
      const ssize_t n = ::recv(fd_, &f, sizeof(f), 0);
      if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) break;
        if (errno == EINTR) continue;
        break;
      }
      if (n < static_cast<ssize_t>(sizeof(struct can_frame))) break;

      if (f.can_id & CAN_ERR_FLAG) {
        std::lock_guard<std::mutex> lk(stats_mtx_);
        stats_.rx_error_frames++;
        stats_.last_rx_ns = ota::now_monotonic_ns();
        continue;
      }

      RawFrame rf;
      rf.id = f.can_id & 0x1FFFFFFFu;  // strip EFF/RTR flags
      rf.dlc = f.can_dlc;
      std::memcpy(rf.data, f.data, 8);
      rf.rx_ns = ota::now_monotonic_ns();

      {
        std::lock_guard<std::mutex> lk(stats_mtx_);
        stats_.rx_frames++;
        stats_.last_rx_ns = rf.rx_ns;
      }

      FrameCallback cb;
      {
        std::lock_guard<std::mutex> lk(cb_mtx_);
        cb = cb_;
      }
      if (cb) cb(rf);
    }
  }
}

bool SocketCanBus::send(uint32_t ext_id, const uint8_t data[8], std::string* err) {
  if (fd_ < 0) {
    if (err) *err = "bus not open";
    return false;
  }
  struct can_frame f{};
  f.can_id = (ext_id & 0x1FFFFFFFu) | CAN_EFF_FLAG;
  f.can_dlc = 8;
  std::memcpy(f.data, data, 8);

  for (int attempt = 0; attempt < 2; ++attempt) {
    const ssize_t n = ::send(fd_, &f, sizeof(f), 0);
    if (n == static_cast<ssize_t>(sizeof(f))) {
      std::lock_guard<std::mutex> lk(stats_mtx_);
      stats_.tx_frames++;
      return true;
    }
    if (errno == EINTR) continue;
    break;
  }
  std::lock_guard<std::mutex> lk(stats_mtx_);
  stats_.tx_failed++;
  if (err) *err = std::string("send failed: ") + std::strerror(errno);
  return false;
}

BusStats SocketCanBus::stats() const {
  std::lock_guard<std::mutex> lk(stats_mtx_);
  return stats_;
}

bool SocketCanBus::is_up() const { return up_state_; }
CanIfState SocketCanBus::can_state() const { return can_state_; }
uint32_t SocketCanBus::bitrate() const { return bitrate_state_; }

}  // namespace ota::can
