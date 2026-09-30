#include "session.hpp"
#include "io.hpp"

namespace ota::commission {
using namespace detail;

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
    std::vector startup_registers{cybergear::Reg::RunMode,cybergear::Reg::CurFiltGain,
       cybergear::Reg::LimitCur,cybergear::Reg::CurKp,cybergear::Reg::CurKi,cybergear::Reg::MechPos,cybergear::Reg::VBus,
       cybergear::Reg::Iqf};
    if (const auto extra=config["additional_startup_registers"]) {
      require(extra.IsSequence() && register_reads && stop_poll,
              "DATA_INVALID: additional native settings require disabled register acquisition");
      for (const auto& item:extra) {
        const auto index=item.as<unsigned>();
        require(index==unsigned(cybergear::Reg::LocKp) || index==unsigned(cybergear::Reg::SpdKp) ||
                index==unsigned(cybergear::Reg::SpdKi) || index==unsigned(cybergear::Reg::LimitSpd),
                "DATA_INVALID: unsupported additional baseline register");
        const auto reg=static_cast<cybergear::Reg>(index);
        require(std::find(startup_registers.begin(),startup_registers.end(),reg)==startup_registers.end(),
                "DATA_INVALID: duplicate additional baseline register");
        startup_registers.push_back(reg);
      }
    }
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
    size_t startup_index=0, periodic_index=0; uint64_t read_count=0, rejected_count=0;
    std::map<cybergear::Reg,bool> readable;
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
            readable[observation.reg]=observation.value.has_value();
            if (observation.value) ++read_count; else ++rejected_count;
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
        std::optional<cybergear::Reg> reg;
        if (startup_index<startup_registers.size()) reg=startup_registers[startup_index++];
        else for (unsigned attempt=0;attempt<2;++attempt) {
          const auto candidate=(periodic_index++%2) ? cybergear::Reg::MechPos : cybergear::Reg::Iqf;
          if (readable.at(candidate)) { reg=candidate; break; }
        }
        // A correlated negative response is a discovered capability limit.
        // Never retry that register in this disabled-baseline context.
        if (reg) {
          const auto request=readback.begin(*reg,now,limits.read_timeout);
          buses[1].send_baseline(request,synthetic);
          const auto accepted=monotonic_ns(); readback.accepted(accepted,true);
          require(journal->append("{\"kind\":\"register_request\",\"request_sequence\":"+std::to_string(readback.sequence())+
               ",\"index\":"+std::to_string(unsigned(*reg))+",\"begin_ns\":"+std::to_string(now)+
               ",\"kernel_accepted_ns\":"+std::to_string(accepted)+",\"motor_actuation\":false}"),
               "DATA_INVALID: capture writer failed");
          next_read=accepted+limits.read_period;
        } else next_read=now+limits.read_period;
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
    require(!register_reads || (readable.size()==startup_registers.size() && !readback.pending()),
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
      ",\"register_rejections\":"+std::to_string(rejected_count)+
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
