#pragma once
// OpenAutoTurret — web/log telemetry + command server (architecture §5.3, §6.1,
// §42.2). Runs inside controld on a NON-real-time thread and:
//
//   * publishes downsampled TelemetrySnapshot JSON at 10-20 Hz (§6.3); and
//   * relays high-level developer commands from webd, through the validation
//     gate (command_validation.hpp), to the control loop (§42.2).
//
// Transport: Unix-domain socket, SOCK_SEQPACKET (message boundaries preserved,
// §6.1). webd is the ONLY client; it never opens can0 and never decides safety.
// Video frames do NOT traverse this socket (they are a separate low-priority
// path, §42.3).
//
// The class is deliberately decoupled: it takes a snapshot provider and a
// command handler callback, so it is unit-testable without CAN / a motor / a
// camera. The controld wires the real ControlLoop + Telemetry into those.
#include <cerrno>
#include <cstring>
#include <poll.h>
#include <sstream>
#include <string>
#include <sys/socket.h>
#include <sys/un.h>
#include <thread>
#include <unistd.h>
#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <vector>

#include "calibration/installation_pose.hpp"
#include "common/types.hpp"
#include "telemetry/telemetry.hpp"
#include "tracking/tracking_state_machine.hpp"
#include "web/command_validation.hpp"

namespace ota {
namespace web {

// Minimal flat-JSON string extraction (no external dependency). Finds
//   "key" : "value"  and returns the value (handles escaped quotes poorly,
// but our values are simple ASCII identifiers/paths).
inline bool json_get_string(const std::string& s, const std::string& key,
                            std::string& out) {
  const std::string pat = "\"" + key + "\"";
  auto p = s.find(pat);
  if (p == std::string::npos) return false;
  p = s.find(':', p + pat.size());
  if (p == std::string::npos) return false;
  p = s.find('"', p + 1);
  if (p == std::string::npos) return false;
  auto q = s.find('"', p + 1);
  if (q == std::string::npos) return false;
  out = s.substr(p + 1, q - p - 1);
  return true;
}

inline std::string json_escape(const std::string& s) {
  std::string o;
  o.reserve(s.size());
  for (char c : s) {
    switch (c) {
      case '"': o += "\\\""; break;
      case '\\': o += "\\\\"; break;
      case '\n': o += "\\n"; break;
      case '\t': o += "\\t"; break;
      default: o += c;
    }
  }
  return o;
}

// Format a TelemetrySnapshot as the published JSON telemetry object (§6.3).
inline std::string format_telemetry(const telemetry::TelemetrySnapshot& s) {
  std::ostringstream os;
  os << "{\"type\":\"telemetry\""
     << ",\"ts_ns\":" << s.timestamp_ns
     << ",\"phase\":\"" << json_escape(s.phase) << "\""
     << ",\"fault\":\"" << json_escape(s.fault_reason) << "\""
     << ",\"track_state\":\"" << track_state_name(s.track_state) << "\""
     << ",\"tracking_active\":" << (s.tracking_active ? "true" : "false")
     << ",\"target_confidence\":" << s.target_confidence
     << ",\"q_yaw_rad\":" << s.q_yaw_rad
     << ",\"v_yaw_rad_s\":" << s.v_yaw_rad_s
     << ",\"q_ref_yaw_rad\":" << s.q_ref_yaw_rad
     << ",\"q_pitch_rad\":" << s.q_pitch_rad
     << ",\"v_pitch_rad_s\":" << s.v_pitch_rad_s
     << ",\"q_ref_pitch_rad\":" << s.q_ref_pitch_rad
     << ",\"effort_yaw\":" << s.effort_yaw
     << ",\"effort_pitch\":" << s.effort_pitch
     << ",\"target_az_world_rad\":" << s.target_az_world_rad
     << ",\"target_el_world_rad\":" << s.target_el_world_rad
     << ",\"base_roll_rad\":" << s.base_roll_rad
     << ",\"base_pitch_rad\":" << s.base_pitch_rad
     << ",\"base_yaw_rad\":" << s.base_yaw_rad
     << ",\"installation_calibrated\":"
     << (s.installation_calibrated ? "true" : "false")
     << ",\"installation_source\":\""
     << pose_source_name(static_cast<PoseSource>(s.installation_source)) << "\""
     << ",\"safety_action\":\"" << safety_action_name(s.safety_action) << "\""
     << ",\"feedback_age_ms\":" << s.feedback_age_ms
     << ",\"control_cycle_us\":" << s.control_cycle_us
     << ",\"payload_profile_name\":\"" << json_escape(s.payload_profile_name) << "\""
     << ",\"payload_profile_status\":\"" << s.payload_profile_status << "\""
     << ",\"payload_derated\":" << (s.payload_derated ? "true" : "false")
     << ",\"payload_check_active\":" << (s.payload_check_active ? "true" : "false")
     << ",\"vision_connected\":" << (s.vision_connected ? "true" : "false")
     << ",\"vision_frames\":" << s.vision_frames
     << ",\"vision_dropped\":" << s.vision_dropped
     << ",\"vision_last_frame_sequence\":" << s.vision_last_frame_sequence
     << ",\"vision_measurement_age_ms\":" << s.vision_measurement_age_ms
     << "}";
  return os.str();
}

inline std::string format_response(const std::string& command,
                                   const CommandResult& r) {
  std::ostringstream os;
  os << "{\"type\":\"response\",\"command\":\"" << json_escape(command)
     << "\",\"ok\":" << (r.ok ? "true" : "false");
  if (!r.ok && !r.error.empty()) {
    os << ",\"error\":\"" << json_escape(r.error) << "\"";
  }
  os << "}";
  return os.str();
}

// The controld provides these; the server is otherwise agnostic to the
// control stack.
class WebServer {
 public:
  struct Config {
    std::string socket_path = "/run/ota/controld-web.sock";
    int telemetry_hz = 15;  // 10-20 Hz per §6.3
    int max_clients = 8;
  };
  using SnapshotProvider = std::function<telemetry::TelemetrySnapshot()>;
  using CommandHandler =
      std::function<CommandResult(const std::string& command,
                                  const std::string& arg)>;

  WebServer(Config cfg, SnapshotProvider provider, CommandHandler handler)
      : cfg_(cfg), provider_(std::move(provider)),
        handler_(std::move(handler)) {}

  ~WebServer() { stop(); }

  // Bind + listen + spawn the accept thread. Returns false + err on failure.
  bool start(std::string& err) {
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
    if (::bind(listen_fd_, reinterpret_cast<sockaddr*>(&addr),
               sizeof(addr)) < 0) {
      err = std::string("bind: ") + std::strerror(errno);
      ::close(listen_fd_);
      listen_fd_ = -1;
      return false;
    }
    if (::listen(listen_fd_, 8) < 0) {
      err = std::string("listen: ") + std::strerror(errno);
      ::close(listen_fd_);
      listen_fd_ = -1;
      return false;
    }
    running_ = true;
    accept_thread_ = std::thread([this] { accept_loop(); });
    return true;
  }

  void stop() {
    running_.store(false);
    if (listen_fd_ >= 0) {
      ::close(listen_fd_);
      listen_fd_ = -1;
      ::unlink(cfg_.socket_path.c_str());
    }
    if (accept_thread_.joinable()) accept_thread_.join();
    // Wake client threads by shutting their fds down.
    {
      std::lock_guard<std::mutex> lk(clients_mu_);
      for (int fd : client_fds_) ::shutdown(fd, SHUT_RDWR);
    }
    // Join WITHOUT holding clients_mu_: each client thread acquires that
    // mutex to unregister its fd, so holding it here would deadlock.
    // (client_threads_ is stable here — the accept thread is already joined,
    // and client threads never modify it.)
    for (auto& t : client_threads_) {
      if (t.joinable()) t.join();
    }
    {
      std::lock_guard<std::mutex> lk(clients_mu_);
      client_threads_.clear();
      client_fds_.clear();
    }
  }

  int connected_clients() const {
    std::lock_guard<std::mutex> lk(clients_mu_);
    return static_cast<int>(client_fds_.size());
  }

  const Config& config() const { return cfg_; }

 private:
  void accept_loop() {
    while (running_.load()) {
      pollfd pfd{listen_fd_, POLLIN, 0};
      int pr = ::poll(&pfd, 1, 100);  // 100 ms so we observe stop()
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
          ::close(cfd);  // over limit: drop
          continue;
        }
        client_fds_.push_back(cfd);
        client_threads_.emplace_back(
            std::thread([this, cfd] { client_loop(cfd); }));
      }
    }
  }

  void client_loop(int cfd) {
    int timeout_ms = std::max(1, 1000 / std::max(1, cfg_.telemetry_hz));
    while (running_.load()) {
      pollfd pfd{cfd, POLLIN, 0};
      int pr = ::poll(&pfd, 1, timeout_ms);
      if (pr < 0) {
        if (errno == EINTR) continue;
        break;
      }
      const bool peer_gone =
          (pfd.revents & (POLLHUP | POLLERR | POLLNVAL)) != 0;
      if (pr > 0 && (pfd.revents & POLLIN) && !peer_gone) {
        char buf[4096];
        ssize_t n = ::recv(cfd, buf, sizeof(buf) - 1, 0);
        if (n <= 0) {
          if (n < 0 && errno == EINTR) continue;
          break;  // closed
        }
        buf[n] = 0;
        handle_command(cfd, buf);
      }
      if (!peer_gone) publish_telemetry(cfd);
      if (peer_gone) break;
    }
    // Remove ourselves.
    {
      std::lock_guard<std::mutex> lk(clients_mu_);
      for (auto it = client_fds_.begin(); it != client_fds_.end(); ++it) {
        if (*it == cfd) {
          ::close(cfd);
          client_fds_.erase(it);
          break;
        }
      }
    }
  }

  void handle_command(int cfd, const char* json) {
    std::string command, arg;
    json_get_string(json, "command", command);
    json_get_string(json, "arg", arg);
    CommandResult result;
    if (handler_) result = handler_(command, arg);
    else {
      result.ok = false;
      result.error = "no command handler installed";
    }
    send_all(cfd, format_response(command, result));
  }

  void publish_telemetry(int cfd) {
    if (!provider_) return;
    telemetry::TelemetrySnapshot s = provider_();
    send_all(cfd, format_telemetry(s));
  }

  void send_all(int cfd, const std::string& msg) {
    size_t off = 0;
    while (off < msg.size()) {
      ssize_t n = ::send(cfd, msg.data() + off, msg.size() - off,
                         MSG_NOSIGNAL);
      if (n <= 0) {
        if (n < 0 && errno == EINTR) continue;
        return;  // peer gone
      }
      off += static_cast<size_t>(n);
    }
  }

  Config cfg_;
  SnapshotProvider provider_;
  CommandHandler handler_;
  int listen_fd_ = -1;
  std::thread accept_thread_;
  std::atomic<bool> running_{false};

  mutable std::mutex clients_mu_;
  std::vector<int> client_fds_;
  std::vector<std::thread> client_threads_;
};

}  // namespace web
}  // namespace ota
