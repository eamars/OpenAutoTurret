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
     << ",\"at_ready\":" << (s.at_ready ? "true" : "false")
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
     << ",\"can_available\":" << (s.can_available ? "true" : "false")
     << ",\"can_kind\":\"" << json_escape(s.can_kind) << "\""
     << ",\"can_device\":\"" << json_escape(s.can_device) << "\""
     << ",\"can_up\":" << (s.can_up ? "true" : "false")
     << ",\"can_state\":" << static_cast<int>(s.can_state)
     << ",\"can_rx_frames\":" << s.can_rx_frames
     << ",\"can_rx_error_frames\":" << s.can_rx_error_frames
     << ",\"can_tx_frames\":" << s.can_tx_frames
     << ",\"can_tx_failed\":" << s.can_tx_failed
     << ",\"can_last_rx_age_ms\":" << s.can_last_rx_age_ms
     << ",\"payload_profile_status\":\"" << s.payload_profile_status << "\""
     << ",\"payload_derated\":" << (s.payload_derated ? "true" : "false")
     << ",\"payload_check_active\":" << (s.payload_check_active ? "true" : "false")
     << ",\"vision_connected\":" << (s.vision_connected ? "true" : "false")
     << ",\"vision_frames\":" << s.vision_frames
     << ",\"vision_dropped\":" << s.vision_dropped
     << ",\"vision_last_frame_sequence\":" << s.vision_last_frame_sequence
     << ",\"vision_measurement_age_ms\":" << s.vision_measurement_age_ms
     // v3 §50/§52: the mode, the intent, and the answer to the last command.
     << ",\"vision_track_sets\":" << s.vision_track_sets
     << ",\"vision_sensor_age_ms\":" << s.vision_sensor_age_ms
     << ",\"vision_publish_to_receive_ms\":" << s.vision_publish_to_receive_ms
     << ",\"selected_track_id\":" << s.selected_track_id
     << ",\"selected_display_index\":" << s.selected_display_index
     << ",\"selected_descriptor\":\"" << json_escape(s.selected_descriptor) << "\""
     << ",\"selection_visibility\":\"" << json_escape(s.selection_visibility) << "\""
     << ",\"selection_ambiguous\":" << (s.selection_ambiguous ? 1 : 0)
     << ",\"reacquisition_score\":" << s.reacquisition_score
     << ",\"ambiguity_margin\":" << s.ambiguity_margin
     << ",\"event_generation\":" << s.event_generation
     // §79. The generation travels with the window so a reader can tell "nothing has
     // happened" from "I have not looked yet", and can tell a restart from a quiet
     // station: a generation that goes backwards means controld came back up.
     << ",\"events\":["
     << [&s]() {
          std::string out;
          for (int i = 0; i < s.event_tail_count; ++i) {
            const telemetry::TelemetrySnapshot::EventTail& e = s.event_tail[i];
            if (i) out += ",";
            out += "{\"t_ns\":" + std::to_string(e.t_ns) +
                   ",\"event\":\"" + json_escape(e.name) + "\"" +
                   ",\"detail\":\"" + json_escape(e.detail) + "\"}";
          }
          return out;
        }()
     << "]"
     << ",\"track_count\":" << s.track_count
     << ",\"track_list_age_ms\":" << s.track_list_age_ms
     << ",\"tracks\":["
     << [&s]() {
          std::string out;
          for (int i = 0; i < s.track_count; ++i) {
            const telemetry::TrackListing& t = s.tracks[i];
            if (i) out += ",";
            out += "{\"uuid\":" + std::to_string(t.uuid_lo) +
                   ",\"display_index\":" + std::to_string(t.display_index) +
                   ",\"label\":\"" + json_escape(t.label) + "\"" +
                   ",\"class_name\":\"" + json_escape(t.class_name) + "\"" +
                   ",\"state\":\"" + json_escape(t.state) + "\"" +
                   ",\"confidence\":" + std::to_string(t.confidence) +
                   ",\"anchor_x\":" + std::to_string(t.anchor_x) +
                   ",\"anchor_y\":" + std::to_string(t.anchor_y) +
                   ",\"selectable\":" + (t.selectable ? "true" : "false") +
                   ",\"selected\":" + (t.selected ? "true" : "false") + "}";
          }
          return out;
        }()
     << "]"
     << ",\"roam_target_yaw_rad\":" << s.roam_target_yaw_rad
     << ",\"roam_sweep_direction\":" << s.roam_sweep_direction
     << ",\"manual_lease_active\":" << (s.manual_lease_active ? 1 : 0)
     << ",\"manual_lease_remaining_ms\":" << s.manual_lease_remaining_ms
     << ",\"manual_profile\":\"" << json_escape(s.manual_profile) << "\""
     << ",\"confidence_band\":\"" << json_escape(s.confidence_band) << "\""
     << ",\"selected_confidence\":" << s.selected_confidence
     << ",\"operating_mode\":\"" << json_escape(s.operating_mode) << "\""
     << ",\"supervisory_state\":\"" << json_escape(s.supervisory_state) << "\""
     << ",\"mode_phase\":\"" << json_escape(s.mode_phase) << "\""
     << ",\"intent_source\":\"" << json_escape(s.intent_source) << "\""
     << ",\"intent_type\":\"" << json_escape(s.intent_type) << "\""
     << ",\"intent_reason\":\"" << json_escape(s.intent_reason) << "\""
     << ",\"intent_velocity_scale\":" << s.intent_velocity_scale
     << ",\"cmd_ack_command\":\"" << json_escape(s.cmd_ack_command) << "\""
     << ",\"cmd_ack_accepted\":" << static_cast<int>(s.cmd_ack_accepted)
     << ",\"cmd_ack_reason\":\"" << json_escape(s.cmd_ack_reason) << "\""
     << ",\"cmd_ack_controller_state\":\""
     << json_escape(s.cmd_ack_controller_state) << "\""
     << ",\"cmd_ack_safety_state\":\"" << json_escape(s.cmd_ack_safety_state)
     << "\""
     << ",\"cmd_ack_seq\":" << s.cmd_ack_seq
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
