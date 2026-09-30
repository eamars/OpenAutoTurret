#pragma once
#include "capture.hpp"
#include "readback.hpp"
#include <array>
#include <arpa/inet.h>
#include <cerrno>
#include <cmath>
#include <csignal>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>
#include <map>
#include <memory>
#include <net/if.h>
#include <poll.h>
#include <sstream>
#include <stdexcept>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>
#include "can/cybergear_protocol.hpp"
#include "can/gm6020_protocol.hpp"

namespace ota::commission {
namespace detail {
inline volatile sig_atomic_t interrupted = 0;
inline void on_signal(int) { interrupted = 1; }
inline void require(bool ok, const char* detail) {
  if (!ok) throw std::runtime_error(detail);
}
// All emitted machine-readable records use JSON, including exception text.
inline std::string quoted(const std::string& text) {
  std::ostringstream out; out << '"';
  for (const unsigned char c : text) {
    if (c == '"' || c == '\\') out << '\\' << c;
    else if (c >= 32 && c != 127) out << c; // Keep UTF-8 bytes intact; escape only JSON control bytes.
    else { const char* hex="0123456789abcdef"; out << "\\u00" << hex[c>>4] << hex[c&15]; }
  }
  out << '"'; return out.str();
}
// Manifest scalars retain their original spelling as JSON strings. In
// particular, YAML's null spelling '~' is not JSON and cannot appear here.
inline std::string json(const YAML::Node& node) {
  if (!node || node.IsNull()) return quoted("null");
  if (node.IsScalar()) return quoted(node.Scalar());
  std::string result=node.IsSequence()?"[":"{";
  bool first=true;
  for (const auto& item:node) {
    if (!first) result+=",";
    first=false;
    if (node.IsSequence()) result+=json(item);
    else result+=quoted(item.first.as<std::string>())+":"+json(item.second);
  }
  return result+(node.IsSequence()?"]":"}");
}
struct Fd {
  int value{-1};
  ~Fd() { if (value >= 0) close(value); }
};
struct Limits {
  int64_t clock, dequeue, can_gap, imu_gap, startup, duration, read_timeout, read_period, stop_period;
  int imu_status;
  explicit Limits(const YAML::Node& n) {
    auto ns=[&](const char* key) {
      const double seconds=n[key].as<double>();
      require(std::isfinite(seconds) && seconds>=1e-9 && seconds<=3600, "DATA_INVALID: capture timing bound (at least 1 ns)");
      return int64_t(seconds*1e9);
    };
    clock=ns("clock_uncertainty_s"); dequeue=ns("dequeue_age_s");
    can_gap=ns("can_gap_s"); imu_gap=ns("imu_gap_s");
    startup=ns("startup_s"); duration=ns("duration_s");
    read_timeout=ns("read_timeout_s"); read_period=ns("read_period_s");
    stop_period=ns("stop_period_s");
    imu_status=n["minimum_imu_status"].as<int>();
    require(imu_status>=0 && imu_status<=3 && duration>startup,
            "DATA_INVALID: capture duration or IMU status");
  }
};
// Physical sessions inherit the launcher's global motion lock (fd 8). A new
// independently opened descriptor must be unable to lock the same inode.
inline void check_launcher_lease() {
  const std::string path="/tmp/ota-motion-"+std::to_string(getuid())+".lock";
  struct stat inherited{}, named{};
  require(fstat(8,&inherited)==0 && stat(path.c_str(),&named)==0 &&
          S_ISREG(inherited.st_mode) && inherited.st_dev==named.st_dev && inherited.st_ino==named.st_ino,
          "HARD_ABORT: launcher motion lease is not inherited");
  require(flock(8,LOCK_EX|LOCK_NB)==0, "HARD_ABORT: launcher motion lease unavailable");
  Fd independent; independent.value=open(path.c_str(),O_RDWR|O_NOFOLLOW|O_CLOEXEC);
  require(independent.value>=0, "HARD_ABORT: cannot inspect motion lease");
  const auto result=flock(independent.value,LOCK_EX|LOCK_NB);
  require(result<0 && (errno==EWOULDBLOCK || errno==EAGAIN),
          "HARD_ABORT: motion lease exclusion failed");
}
struct Endpoint {
  Fd fd;
  std::unique_ptr<TimestampedReceiver> receiver;
  uint64_t sequence{};
  int64_t last_feedback{};
  std::string iface;
  uint64_t drops_begin{}, errors_begin{};
  sockaddr_in peer{};
  uint64_t counter(const char* field) const {
    std::ifstream input("/sys/class/net/"+iface+"/statistics/"+field);
    uint64_t value{};
    require(bool(input>>value),"DATA_INVALID: interface loss counter unavailable");
    return value;
  }
  void open(const YAML::Node& n, bool synthetic, const Limits& limits) {
    if (synthetic) {
      fd.value=socket(AF_INET,SOCK_DGRAM|SOCK_CLOEXEC,0);
      require(fd.value>=0,"DATA_INVALID: loopback socket unavailable");
      receiver=std::make_unique<TimestampedReceiver>(fd.value,limits.clock);
      sockaddr_in addr{}; addr.sin_family=AF_INET; addr.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
      const auto port=n["port"].as<unsigned>();
      require(port>0 && port<=65535,"DATA_INVALID: loopback port"); addr.sin_port=htons(port);
      require(bind(fd.value,reinterpret_cast<sockaddr*>(&addr),sizeof(addr))==0,
              "DATA_INVALID: loopback bind failed");
      if (n["peer_port"]) {
        const auto target=n["peer_port"].as<unsigned>();
        require(target>0 && target<=65535,"DATA_INVALID: loopback peer port");
        peer=addr; peer.sin_port=htons(target);
      }
    } else {
      iface=n["interface"].as<std::string>();
      require(iface=="can0" || iface=="can1","INTEGRATION_MISMATCH: unsupported station CAN interface");
      can::CanIfInfo info; std::string error;
      require(can::netlink_query_can(iface,info,error) && info.is_can && info.up && info.bitrate==1000000 &&
              info.state==can::CanIfState::ErrorActive,
              "HARD_ABORT: CAN interface is not UP at 1 Mbps");
      drops_begin=counter("rx_dropped"); errors_begin=counter("rx_errors");
      fd.value=socket(PF_CAN,SOCK_RAW|SOCK_CLOEXEC,CAN_RAW);
      require(fd.value>=0,"DATA_INVALID: SocketCAN open failed");
      receiver=std::make_unique<TimestampedReceiver>(fd.value,limits.clock);
      // Subscribe to all traffic and all errors; unexpected senders remain visible.
      const can_err_mask_t mask=CAN_ERR_MASK;
      require(setsockopt(fd.value,SOL_CAN_RAW,CAN_RAW_ERR_FILTER,&mask,sizeof(mask))==0,
              "DATA_INVALID: CAN error subscription failed");
      ifreq request{}; std::strncpy(request.ifr_name,iface.c_str(),IFNAMSIZ-1);
      require(ioctl(fd.value,SIOCGIFINDEX,&request)==0,"DATA_INVALID: CAN interface index");
      sockaddr_can addr{}; addr.can_family=AF_CAN; addr.can_ifindex=request.ifr_ifindex;
      require(bind(fd.value,reinterpret_cast<sockaddr*>(&addr),sizeof(addr))==0,
              "DATA_INVALID: SocketCAN bind failed");
    }
  }
  void send_baseline(const can::RawFrame& frame, bool synthetic) {
    const auto id=cybergear::unpack_ext_id(frame.id);
    require(frame.extended && !frame.error && !frame.rtr && frame.dlc==8 && id.target==127 && id.data2==0 &&
            (id.comm_type==17 || id.comm_type==0 || id.comm_type==4),
            "HARD_ABORT: baseline permits discovery, normal STOP and reads only");
    if (id.comm_type!=17) for (const auto b:frame.data)
      require(b==0,"HARD_ABORT: baseline must not clear faults");
    can_frame wire{}; wire.can_id=frame.id|CAN_EFF_FLAG; wire.can_dlc=frame.dlc;
    std::memcpy(wire.data,frame.data,8);
    ssize_t n;
    if (synthetic) {
      require(peer.sin_port!=0,"DATA_INVALID: synthetic register endpoint missing");
      n=sendto(fd.value,&wire,sizeof(wire),MSG_DONTWAIT,reinterpret_cast<sockaddr*>(&peer),sizeof(peer));
    } else n=send(fd.value,&wire,sizeof(wire),MSG_DONTWAIT);
    require(n==sizeof(wire),"DATA_INVALID: register request not accepted by kernel");
  }
};
struct ImuStream {
  struct Sensor { int generation{-1}, sequence{-1}; int64_t stamp{}, received{}; uint64_t count{}; };
  std::map<std::string,Sensor> sensors{{"accel",{}},{"gyro",{}},{"rv",{}},{"game_rv",{}}};
  std::string pending;
  int generation{-1};
  bool ready() const { for (const auto& [_,s]:sensors) if (!s.count) return false; return true; }
  void line(const std::string& raw, const Limits& limits, Journal& journal) {
    require(raw.size()<3000 && !raw.empty() && raw.front()=='{' && raw.back()=='}',
            "DATA_INVALID: invalid IMU JSON record");
    const auto n=YAML::Load(raw);
    const auto kind=n["kind"].as<std::string>();
    require(journal.append("{\"kind\":\"imu_raw\",\"dequeue_ns\":"+std::to_string(monotonic_ns())+
                           ",\"raw_json\":"+quoted(raw)+"}"),"DATA_INVALID: capture writer failed");
    require(kind!="trace_reset" && kind!="gap" && kind!="summary",
            "DATA_INVALID: IMU stream ended, recovered or discarded history");
    if (kind!="sample") return; // retain product/config/tare events; no calibration claim
    const auto name=n["sensor"].as<std::string>();
    require(sensors.count(name),"DATA_INVALID: unknown IMU sensor");
    auto& s=sensors.at(name);
    const auto stamp=n["sample_ns"].as<int64_t>(), rx=n["rx_ns"].as<int64_t>();
    const auto gen=n["generation"].as<int>(), seq=n["sequence"].as<int>();
    const auto status=n["status"].as<int>();
    require(gen>=0 && seq>=0 && seq<=255 && status>=limits.imu_status && status<=3,
            "DATA_INVALID: IMU generation, sequence or status");
    if (generation<0) generation=gen;
    require(gen==generation,"DATA_INVALID: IMU generation changed");
    require(stamp>0 && rx>=stamp && rx<=monotonic_ns() && rx-stamp<=limits.imu_gap &&
            monotonic_ns()-rx<=limits.dequeue,"DATA_INVALID: stale or invalid IMU clock");
    if (s.count) {
      require(gen==s.generation && seq==((s.sequence+1)&255),"DATA_INVALID: IMU sequence loss or reset");
      require(stamp>s.stamp && stamp-s.stamp<=limits.imu_gap && rx>=s.received,
              "MEASUREMENT_LIMITED: IMU sample gap or reordering");
    }
    const auto values=n["values"].as<std::vector<double>>();
    require(values.size()==(name=="gyro" || name=="accel" ? 3u:4u),"DATA_INVALID: IMU dimensions");
    double norm=0;
    for (const auto v:values) { require(std::isfinite(v),"DATA_INVALID: nonfinite IMU value"); norm+=v*v; }
    if (values.size()==4) require(norm>=.9801 && norm<=1.0201,"DATA_INVALID: IMU quaternion norm");
    s={gen,seq,stamp,rx,s.count+1};
  }
  void read_fd(int fd, const Limits& limits, Journal& journal) {
    char bytes[8192];
    const auto n=read(fd,bytes,sizeof(bytes));
    if (n<0 && (errno==EAGAIN || errno==EINTR)) return;
    require(n>0,"DATA_INVALID: IMU producer EOF or read failure");
    pending.append(bytes,n);
    size_t end;
    while ((end=pending.find('\n'))!=std::string::npos) {
      line(pending.substr(0,end),limits,journal); pending.erase(0,end+1);
    }
    require(pending.size()<3000,"DATA_INVALID: IMU line exceeded capture bound");
  }
};
}
}
