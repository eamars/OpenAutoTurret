#include "session.hpp"
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
namespace {
volatile sig_atomic_t interrupted = 0;
void on_signal(int) { interrupted = 1; }
void require(bool ok, const char* detail) {
  if (!ok) throw std::runtime_error(detail);
}
std::string json(const YAML::Node& node) {
  YAML::Emitter out;
  out << YAML::Flow << YAML::DoubleQuoted << node;
  return out.c_str();
}
// All emitted machine-readable records use JSON, including exception text.
std::string quoted(const std::string& text) {
  std::ostringstream out; out << '"';
  for (const unsigned char c : text) {
    if (c == '"' || c == '\\') out << '\\' << c;
    else if (c >= 32 && c < 127) out << c;
    else { const char* hex="0123456789abcdef"; out << "\\u00" << hex[c>>4] << hex[c&15]; }
  }
  out << '"'; return out.str();
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
void check_launcher_lease() {
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

int capture_session(const char* path) {
  std::unique_ptr<Journal> journal;
  try {
    const auto config=YAML::LoadFile(path);
    require(config["schema"].as<std::string>()=="adr0022.capture/2","INTEGRATION_MISMATCH: capture schema");
    const auto source=config["provenance"].as<std::string>();
    const bool synthetic=source=="SYNTHETIC";
    require(synthetic || source=="MEASURED","DATA_INVALID: capture provenance");
    require(config["transport"].as<std::string>()==(synthetic?"loopback_udp":"socketcan"),
            "INTEGRATION_MISMATCH: transport/provenance mismatch");
    const Limits limits(config["limits"]);
    const bool register_reads=config["register_reads"].as<bool>();
    const bool stop_poll=config["pitch_stop_poll"].as<bool>();
    require(!stop_poll || config["pitch_supported_when_disabled"].as<bool>(),
            "HARD_ABORT: pitch support must be confirmed before STOP polling");
    const auto uid_text=config["expected_pitch_uid"].as<std::string>();
    require(uid_text.size()==16 && uid_text.find_first_not_of("0123456789abcdef")==std::string::npos,
            "DATA_INVALID: expected pitch UID must be 16 lowercase hex digits");
    const uint64_t expected_uid=std::stoull(uid_text,nullptr,16);
    const int imu_fd=config["imu_fd"].as<int>();
    require(imu_fd>2 && imu_fd!=8,"DATA_INVALID: IMU descriptor");
    struct stat imu_info{};
    require(fstat(imu_fd,&imu_info)==0 && S_ISFIFO(imu_info.st_mode),"DATA_INVALID: IMU pipe required");
    require(fcntl(imu_fd,F_SETFL,fcntl(imu_fd,F_GETFL)|O_NONBLOCK)==0,"DATA_INVALID: IMU nonblocking pipe");
    if (!synthetic) check_launcher_lease();
    const std::string manifest=json(config); // original configuration retained separately by caller
    journal=std::make_unique<Journal>(config["output"].as<std::string>(),
       "{\"kind\":\"header\",\"schema\":\"adr0022.capture/2\",\"provenance\":"+quoted(source)+
       ",\"purpose\":\"baseline_acquisition\",\"parameter_qualified\":false,\"manifest_yaml\":"+quoted(manifest)+"}");
    std::array<Endpoint,2> buses;
    buses[0].open(config["yaw"],synthetic,limits); buses[1].open(config["pitch"],synthetic,limits);
    require(synthetic || buses[0].iface!=buses[1].iface,"INTEGRATION_MISMATCH: duplicate CAN bus");
    ImuStream imu;
    Readback readback(127,0);
    const std::array startup_registers{cybergear::Reg::RunMode,cybergear::Reg::CurFiltGain,
       cybergear::Reg::LimitCur,cybergear::Reg::CurKp,cybergear::Reg::CurKi,cybergear::Reg::MechPos,cybergear::Reg::VBus};
    size_t startup_index=0; uint64_t read_count=0;
    int64_t next_read=0, discovery_begin=0, stop_begin=0, next_stop=0;
    bool identified=false, stop_pending=false;
    uint64_t stop_count=0, stop_confirmed=0;
    interrupted=0; std::signal(SIGTERM,on_signal); std::signal(SIGINT,on_signal);
    std::array<pollfd,3> fds{{{buses[0].fd.value,POLLIN,0},{buses[1].fd.value,POLLIN,0},{imu_fd,POLLIN,0}}};
    const auto begin=monotonic_ns();
    std::cout << "{\"kind\":\"capture_ready\",\"excitation\":false,\"pitch_stop_requests\":"
              << (stop_poll?"true":"false") << ",\"provenance\":" << quoted(source) << "}\n" << std::flush;
    while (monotonic_ns()-begin < limits.duration) {
      require(!interrupted,"DATA_INVALID: capture interrupted");
      const int result=poll(fds.data(),fds.size(),2);
      if (result<0 && errno==EINTR) continue;
      require(result>=0,"DATA_INVALID: acquisition poll failure");
      for (size_t i=0;i<2;++i) {
        auto& bus=buses[i];
        require(!(fds[i].revents&(POLLERR|POLLHUP|POLLNVAL)),"HARD_ABORT: CAN endpoint failed");
        // Bounded drain preserves fairness between buses, IMU and supervision.
        for (unsigned batch=0;batch<64;++batch) {
          Receipt r;
          if (!bus.receiver->receive(r)) break;
          require(journal->append(receipt_json(r,i?"pitch":"yaw",++bus.sequence)),"DATA_INVALID: capture writer failed");
          require(!r.drop_delta,"DATA_INVALID: socket receive overflow");
          require(!r.frame.error,"HARD_ABORT: CAN error frame");
          require(r.dequeue_ns-r.kernel_monotonic_ns<=limits.dequeue,"DATA_INVALID: stale CAN dequeue");
          bool feedback=false;
          if (i==0) { gm6020::Feedback value; feedback=gm6020::decode(r.frame,1,value); }
          else {
            const auto id=cybergear::unpack_ext_id(r.frame.id);
            if (id.comm_type==0 && r.frame.extended && !r.frame.rtr && r.frame.dlc==8) {
              cybergear::CanFrame frame; frame.id=r.frame.id; std::memcpy(frame.data,r.frame.data,8);
              cybergear::DiscoveryResponse reply;
              require(!identified && discovery_begin && r.kernel_monotonic_ns>=discovery_begin &&
                      r.kernel_monotonic_ns<discovery_begin+limits.read_timeout && id.data2==127 &&
                      cybergear::parse_discovery_response(frame,reply) && reply.unique_id==expected_uid,
                      "INTEGRATION_MISMATCH: pitch discovery identity or correlation differs");
              identified=true;
              require(journal->append("{\"kind\":\"pitch_identity\",\"uid_hex\":"+quoted(uid_text)+
                        ",\"receive_ns\":"+std::to_string(r.kernel_monotonic_ns)+"}"),"DATA_INVALID: capture writer failed");
              continue;
            }
            feedback=r.frame.extended && !r.frame.rtr && r.frame.dlc==8 && id.comm_type==2 &&
                     id.target==0 && (id.data2&255)==127;
            require(!feedback || (id.data2&0x3f00)==0,"HARD_ABORT: pitch drive fault");
            if (feedback && stop_poll && identified) {
              const bool disabled=((id.data2>>14)&3)==0;
              require(disabled || !stop_confirmed,"HARD_ABORT: pitch became enabled during baseline capture");
              if (stop_pending && disabled && r.kernel_monotonic_ns>=stop_begin) {
                require(r.kernel_monotonic_ns<stop_begin+limits.read_timeout,"DATA_INVALID: stale STOP feedback");
                stop_pending=false; ++stop_confirmed;
                require(journal->append("{\"kind\":\"pitch_stop_confirmed\",\"sequence\":"+std::to_string(stop_count)+
                     ",\"request_begin_ns\":"+std::to_string(stop_begin)+",\"receive_ns\":"+
                     std::to_string(r.kernel_monotonic_ns)+",\"disabled\":true}"),"DATA_INVALID: capture writer failed");
              }
            }
          }
          if (!feedback && i==1 && register_reads) {
            const auto observation=readback.observe(r);
            require(journal->append(readback_json(observation)),"DATA_INVALID: capture writer failed");
            ++read_count;
            continue;
          }
          require(feedback,"HARD_ABORT: unexpected CAN traffic during exclusive capture");
          if (bus.last_feedback) require(r.kernel_monotonic_ns-bus.last_feedback<=limits.can_gap,
                                        "MEASUREMENT_LIMITED: CAN feedback gap");
          bus.last_feedback=r.kernel_monotonic_ns;
        }
      }
      require(!(fds[2].revents&(POLLERR|POLLNVAL)),"DATA_INVALID: IMU pipe failure");
      if (fds[2].revents&(POLLIN|POLLHUP)) imu.read_fd(imu_fd,limits,*journal);
      const auto now=monotonic_ns();
      readback.check_deadline(now);
      if (!discovery_begin) {
        const auto request=cybergear::make_discovery_request(0,127);
        can::RawFrame frame; frame.id=request.id; std::memcpy(frame.data,request.data,8);
        discovery_begin=now; buses[1].send_baseline(frame,synthetic);
        require(journal->append("{\"kind\":\"discovery_request\",\"begin_ns\":"+std::to_string(now)+"}"),
                "DATA_INVALID: capture writer failed");
      }
      require(identified || now<discovery_begin+limits.read_timeout,"MEASUREMENT_LIMITED: discovery timeout; no retry");
      require(!stop_pending || now<stop_begin+limits.read_timeout,"HARD_ABORT: STOP feedback timeout; no retry");
      if (identified && stop_poll && !stop_pending && now>=next_stop && now+limits.read_timeout<begin+limits.duration) {
        const auto request=cybergear::make_stop(0,127);
        can::RawFrame frame; frame.id=request.id; std::memcpy(frame.data,request.data,8);
        stop_begin=now; buses[1].send_baseline(frame,synthetic);
        stop_pending=true; ++stop_count; next_stop=now+limits.stop_period;
        require(journal->append("{\"kind\":\"pitch_stop_request\",\"sequence\":"+std::to_string(stop_count)+
              ",\"begin_ns\":"+std::to_string(now)+",\"clear_fault\":false}"),"DATA_INVALID: capture writer failed");
      }
      // Stop scheduling reads before the capture deadline so all accepted TXs
      // have a complete response window. No blocking RPC or retry in this loop.
      if (identified && (!stop_poll || stop_confirmed) && register_reads && !readback.pending() && now>=next_read &&
          now+limits.read_timeout<begin+limits.duration) {
        const auto reg=startup_index<startup_registers.size() ? startup_registers[startup_index++] :
                       (read_count%2 ? cybergear::Reg::Iqf : cybergear::Reg::MechPos);
        const auto request=readback.begin(reg,now,limits.read_timeout);
        buses[1].send_baseline(request,synthetic);
        const auto accepted=monotonic_ns(); readback.accepted(accepted,true);
        require(journal->append("{\"kind\":\"register_request\",\"request_sequence\":"+std::to_string(readback.sequence())+
             ",\"index\":"+std::to_string(unsigned(reg))+",\"begin_ns\":"+std::to_string(now)+
             ",\"kernel_accepted_ns\":"+std::to_string(accepted)+",\"motor_actuation\":false}"),
             "DATA_INVALID: capture writer failed");
        next_read=accepted+limits.read_period;
      }
      if (now-begin>limits.startup) {
        for (size_t i=0;i<buses.size();++i) {
          const auto& bus=buses[i];
          // STOP polling ends early enough to drain its response. Preserve that
          // planned tail explicitly instead of manufacturing fresh samples.
          const auto age_limit=(i==1 && stop_poll && now+limits.read_timeout>=begin+limits.duration) ?
                                limits.can_gap+limits.read_timeout : limits.can_gap;
          require(bus.last_feedback && now-bus.last_feedback<=age_limit,"MEASUREMENT_LIMITED: CAN feedback absent/stale");
        }
        require(imu.ready(),"MEASUREMENT_LIMITED: required IMU streams absent");
        for (const auto& [_,s]:imu.sensors)
          require(now-s.received<=limits.imu_gap,"MEASUREMENT_LIMITED: IMU stream stale");
      }
      require(journal->healthy(),"DATA_INVALID: capture writer failed");
    }
    require(imu.ready() && buses[0].sequence && buses[1].sequence,"DATA_INVALID: incomplete capture");
    require(!register_reads || (read_count>startup_registers.size() && !readback.pending()),
            "MEASUREMENT_LIMITED: incomplete register observations");
    require(identified && (!stop_poll || (stop_confirmed && stop_count==stop_confirmed && !stop_pending)),
            "HARD_ABORT: pitch identity/STOP evidence incomplete");
    if (!synthetic) for (const auto& bus:buses)
      require(bus.counter("rx_dropped")==bus.drops_begin && bus.counter("rx_errors")==bus.errors_begin,
              "DATA_INVALID: interface receive loss during capture");
    // Ancillary overflow metadata arrives on a later delivered packet. Check
    // the socket counters directly before certifying completion, including when
    // the dropped tail had no following packet on which to report the loss.
    const auto yaw_drops=buses[0].receiver->kernel_drops(), pitch_drops=buses[1].receiver->kernel_drops();
    require(!yaw_drops && !pitch_drops,"DATA_INVALID: final socket receive loss");
    std::string footer="{\"kind\":\"footer\",\"status\":\"COMPLETE\",\"parameter_qualified\":false,\"begin_ns\":"+
      std::to_string(begin)+",\"end_ns\":"+std::to_string(monotonic_ns())+",\"yaw_frames\":"+
      std::to_string(buses[0].sequence)+",\"pitch_frames\":"+std::to_string(buses[1].sequence)+
      ",\"register_reads\":"+std::to_string(read_count)+",\"pitch_stop_confirmed\":"+std::to_string(stop_confirmed)+
      ",\"socket_drops\":{\"yaw\":"+std::to_string(yaw_drops)+",\"pitch\":"+std::to_string(pitch_drops)+"}"+
      ",\"writer_queue_high_water\":"+std::to_string(journal->high_water())+"}";
    require(journal->finish(footer),"DATA_INVALID: capture flush failed");
    std::cout << footer << '\n'; return 0;
  } catch (const std::exception& e) {
    const std::string detail=journal && !journal->healthy() ? journal->failure_reason() : e.what();
    const std::string failure="{\"kind\":\"footer\",\"status\":\"INVALID\",\"detail\":"+quoted(detail)+"}";
    if (journal) journal->finish(failure);
    std::cerr << failure << '\n'; return 1;
  }
}
} // namespace ota::commission
