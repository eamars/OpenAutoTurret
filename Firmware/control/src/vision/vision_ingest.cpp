// OpenAutoTurret — vision ingest implementation (§6.1). See vision_ingest.hpp.
#include "vision/vision_ingest.hpp"

#include "common/time.hpp"

#include <spdlog/spdlog.h>

namespace ota {
namespace vision {

bool VisionIngest::start(std::string& err) {
  if (running_.load()) {
    err = "vision ingest already running";
    return false;
  }
  // A stale socket file from a crashed previous run would make bind() fail.
  ::unlink(cfg_.socket_path.c_str());
  listen_fd_ = ::socket(AF_UNIX, SOCK_SEQPACKET, 0);
  if (listen_fd_ < 0) {
    err = std::string("socket: ") + std::strerror(errno);
    return false;
  }
  sockaddr_un addr{};
  addr.sun_family = AF_UNIX;
  if (cfg_.socket_path.size() >= sizeof(addr.sun_path)) {
    err = "socket path too long";
    ::close(listen_fd_);
    listen_fd_ = -1;
    return false;
  }
  std::strncpy(addr.sun_path, cfg_.socket_path.c_str(),
               sizeof(addr.sun_path) - 1);
  if (::bind(listen_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) <
      0) {
    err = std::string("bind: ") + std::strerror(errno);
    ::close(listen_fd_);
    listen_fd_ = -1;
    return false;
  }
  if (::listen(listen_fd_, 4) < 0) {
    err = std::string("listen: ") + std::strerror(errno);
    ::close(listen_fd_);
    listen_fd_ = -1;
    return false;
  }
  running_.store(true);
  accept_thread_ = std::thread([this] { accept_loop(); });
  return true;
}

void VisionIngest::stop() {
  running_.store(false);
  if (listen_fd_ >= 0) {
    ::close(listen_fd_);
    listen_fd_ = -1;
    ::unlink(cfg_.socket_path.c_str());
  }
  if (accept_thread_.joinable()) accept_thread_.join();
  // Wake the reader threads by shutting their fds down. Join WITHOUT holding
  // clients_mu_: each reader takes that mutex to unregister itself.
  {
    std::lock_guard<std::mutex> lk(clients_mu_);
    for (int fd : client_fds_) ::shutdown(fd, SHUT_RDWR);
  }
  for (auto& t : client_threads_) {
    if (t.joinable()) t.join();
  }
  {
    std::lock_guard<std::mutex> lk(clients_mu_);
    client_threads_.clear();
    client_fds_.clear();
  }
}

void VisionIngest::accept_loop() {
  while (running_.load()) {
    pollfd pfd{listen_fd_, POLLIN, 0};
    int pr = ::poll(&pfd, 1, 100);  // 100 ms so stop() is observed promptly
    if (pr < 0) {
      if (errno == EINTR) continue;
      break;
    }
    if (pr == 0) continue;
    int cfd = ::accept(listen_fd_, nullptr, nullptr);
    if (cfd < 0) {
      if (errno == EINTR) continue;
      break;
    }
    {
      std::lock_guard<std::mutex> lk(clients_mu_);
      if (static_cast<int>(client_fds_.size()) >= cfg_.max_clients) {
        spdlog::warn("vision ingest: extra publisher rejected (max {})",
                     cfg_.max_clients);
        ::close(cfd);
        continue;
      }
      client_fds_.push_back(cfd);
      if (link_) link_->note_client_added();
      client_threads_.emplace_back(
          std::thread([this, cfd] { client_loop(cfd); }));
    }
  }
}

void VisionIngest::client_loop(int cfd) {
  // One datagram == one measurement. A larger buffer is harmless: any size
  // other than exactly kWireSize is counted as a drop (never silently parsed).
  uint8_t buf[128];
  while (running_.load()) {
    pollfd pfd{cfd, POLLIN, 0};
    int pr = ::poll(&pfd, 1, 100);
    if (pr < 0) {
      if (errno == EINTR) continue;
      break;
    }
    if (pr == 0) continue;
    if (pfd.revents & (POLLHUP | POLLERR | POLLNVAL)) break;
    const ssize_t n = ::recv(cfd, buf, sizeof(buf), 0);
    if (n <= 0) {
      if (n < 0 && errno == EINTR) continue;
      break;  // publisher closed / died
    }
    TargetMeasurement m;
    if (!TargetMeasurement::decode(buf, static_cast<size_t>(n), m)) {
      if (link_) link_->note_dropped();
      continue;
    }
    if (link_) link_->note_frame(m.frame_sequence, now_monotonic_ns());
    if (handler_) handler_(m);
  }
  // Unregister ourselves (mirrors WebServer::client_loop).
  std::lock_guard<std::mutex> lk(clients_mu_);
  for (auto it = client_fds_.begin(); it != client_fds_.end(); ++it) {
    if (*it == cfd) {
      ::close(cfd);
      client_fds_.erase(it);
      break;
    }
  }
  if (link_) link_->note_client_removed();
}

}  // namespace vision
}  // namespace ota
