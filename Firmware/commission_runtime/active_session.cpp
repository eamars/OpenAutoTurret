#include "session.hpp"
#include "io.hpp"
#include <deque>

namespace ota::commission {
using namespace detail;
namespace {
can::RawFrame raw(const cybergear::CanFrame& source) {
  can::RawFrame frame; frame.id=source.id; frame.dlc=source.dlc;
  std::memcpy(frame.data,source.data,8); return frame;
}

// This entry observes the neutral mode transition. It has no nonzero-current,
// homing, position-reference, inner-loop-gain or persistent-register operation.
// Dynamic acquisition must obtain its own qualified envelope and homing identity.
struct Preparation {
  enum class State { Discover, Stop, Observe, ZeroBefore, Mode, CheckMode, CheckZero,
                     Enable, CheckEnabledMode, CheckEnabledZero, ObserveEnabled,
                     FinalStop, RestoreMode, CheckRestored, Done };
  State state=State::Discover;
  int64_t entered{}, pending_begin{}, next_ping{}, observed_since{};
  int original_mode=-1;
  double origin{}, actual_current{};
  int64_t actual_current_ns{}, status_ns{}, enable_begin_ns{};
  bool disabled=false, enabled=false, have_origin=false;
};

class NeutralSession {
 public:
  explicit NeutralSession(const YAML::Node& config, bool characterization=false)
      : config_(config), limits_(config["limits"]), synthetic_(config["provenance"].as<std::string>()=="SYNTHETIC"),
        characterization_(characterization), readback_(127,0) {
    schema_=characterization_?"adr0022.neutral-characterization/1":"adr0022.current-preparation/1";
    purpose_=characterization_?"neutral_current_measurement_characterization":"neutral_current_mode_verification";
    require(config["schema"].as<std::string>()==schema_,
            "INTEGRATION_MISMATCH: current preparation schema");
    require(synthetic_ || config["provenance"].as<std::string>()=="MEASURED", "DATA_INVALID: provenance");
    require(config["transport"].as<std::string>()==(synthetic_?"loopback_udp":"socketcan"),
            "INTEGRATION_MISMATCH: transport/provenance mismatch");
    require(config["pitch_supported_when_disabled"].as<bool>(), "HARD_ABORT: pitch support is required");
    require(config["purpose"].as<std::string>()==purpose_,
            "INTEGRATION_MISMATCH: unsupported active purpose");
    uid_text_=config["expected_pitch_uid"].as<std::string>();
    require(uid_text_.size()==16 && uid_text_.find_first_not_of("0123456789abcdef")==std::string::npos,
            "DATA_INVALID: pitch UID");
    expected_uid_=std::stoull(uid_text_,nullptr,16);
    protection_current_=positive("protection_current_bound_A");
    const auto observation_s=characterization_?positive("neutral_observation_s"):2.;
    require(protection_current_<=6.5,"DATA_INVALID: manufacturer continuous current protection required");
    observation_ns_=int64_t(observation_s*1e9);
    require(observation_ns_>0 &&
            limits_.duration>limits_.startup+observation_ns_,
            "DATA_INVALID: neutral observation does not fit session deadline");
    maximum_displacement_=positive("transition_displacement_bound_rad");
    maximum_temperature_=positive("pitch_maximum_temperature_C");
    imu_fd_=config["imu_fd"].as<int>();
    struct stat info{};
    require(imu_fd_>2 && imu_fd_!=8 && fstat(imu_fd_,&info)==0 && S_ISFIFO(info.st_mode),
            "DATA_INVALID: inherited IMU pipe required");
    require(fcntl(imu_fd_,F_SETFL,fcntl(imu_fd_,F_GETFL)|O_NONBLOCK)==0,"DATA_INVALID: IMU nonblocking pipe");
    if (!synthetic_) {
      check_launcher_lease();
      require(config["yaw"]["interface"].as<std::string>()=="can0" &&
              config["pitch"]["interface"].as<std::string>()=="can1", "INTEGRATION_MISMATCH: station topology");
    }
    journal_=std::make_unique<Journal>(config["output"].as<std::string>(),
      "{\"kind\":\"header\",\"schema\":"+quoted(schema_)+",\"provenance\":"+
      quoted(synthetic_?"SYNTHETIC":"MEASURED")+",\"purpose\":"+quoted(purpose_)+","
      "\"parameter_qualified\":false,\"manifest_yaml\":"+quoted(json(config))+"}");
    buses_[0].open(config["yaw"],synthetic_,limits_);
    buses_[1].open(config["pitch"],synthetic_,limits_);
  }

  int run() {
    interrupted=0; std::signal(SIGINT,on_signal); std::signal(SIGTERM,on_signal);
    begin_=monotonic_ns(); prep_.entered=begin_;
    std::cout<<"{\"kind\":\"capture_ready\",\"excitation\":false,\"purpose\":"+quoted(purpose_)+"}\n"<<std::flush;
    try {
      record("{\"kind\":\"session_begin\",\"time_ns\":"+std::to_string(begin_)+"}");
      while (prep_.state!=Preparation::State::Done) {
        pump();
        const auto now=monotonic_ns();
        require(!interrupted,"HARD_ABORT: current preparation interrupted");
        require(now-begin_<limits_.duration,"MEASUREMENT_LIMITED: preparation deadline");
        readback_.check_deadline(now);
        if (now-begin_>limits_.startup) check_streams(now);
        step(now);
        require(journal_->healthy(),"DATA_INVALID: capture writer failed");
      }
      return finish(true,"");
    } catch (const std::exception& e) {
      const std::string reason=e.what();
      // A failure never restores/enables a drive or retries a trial. Request
      // neutral output and STOP, preserving separately whether it was confirmed.
      // No claim is made about stopping after Pi/CAN/power loss.
      shutdown();
      return finish(false,reason);
    }
  }

 private:
  double positive(const char* key) const {
    const auto value=config_[key].as<double>();
    require(std::isfinite(value) && value>0,"DATA_INVALID: finite positive transition boundary required");
    return value;
  }
  void record(const std::string& row) {
    require(journal_->append(row),"DATA_INVALID: capture writer failed");
  }
  int64_t send(unsigned axis, const can::RawFrame& frame, const char* operation) {
    // A separate allowlist at the final send boundary prevents a future caller
    // from quietly turning neutral verification into an excitation interface.
    if (!axis) {
      require(!frame.extended && frame.id==0x1fe && frame.dlc==8,"HARD_ABORT: invalid yaw neutral frame");
      for (const auto b:frame.data) require(!b,"HARD_ABORT: nonzero yaw command in neutral verification");
    } else {
      const auto id=cybergear::unpack_ext_id(frame.id);
      require(frame.extended && !frame.error && !frame.rtr && frame.dlc==8 && id.target==127 && id.data2==0,
              "HARD_ABORT: invalid pitch command identity");
      require(id.comm_type==0 || id.comm_type==3 || id.comm_type==4 || id.comm_type==17 || id.comm_type==18,
              "HARD_ABORT: pitch command outside neutral verification contract");
      if (id.comm_type==18) {
        const auto index=uint16_t(frame.data[0])|(uint16_t(frame.data[1])<<8);
        require(index==unsigned(cybergear::Reg::IqRef) || index==unsigned(cybergear::Reg::RunMode),
                "HARD_ABORT: register write outside neutral verification contract");
        if (index==unsigned(cybergear::Reg::IqRef)) {
          float value; std::memcpy(&value,frame.data+4,4);
          require(value==0.,"HARD_ABORT: nonzero pitch current in neutral verification");
        } else {
          require(!buses_[0].receiver->kernel_drops() && !buses_[1].receiver->kernel_drops(),
                  "DATA_INVALID: socket receive loss before mode write");
          require(prep_.disabled && prep_.status_ns && monotonic_ns()-prep_.status_ns<=limits_.can_gap,
                  "HARD_ABORT: mode write without fresh disabled feedback");
          require(frame.data[4]<=3 && !frame.data[5] && !frame.data[6] && !frame.data[7],
                  "HARD_ABORT: invalid mode write");
        }
      } else if (id.comm_type!=17) for (const auto b:frame.data)
        require(!b,"HARD_ABORT: simple command payload must be zero");
    }
    auto& endpoint=buses_[axis];
    can_frame wire{}; wire.can_id=frame.id|(frame.extended?CAN_EFF_FLAG:0); wire.can_dlc=frame.dlc;
    std::memcpy(wire.data,frame.data,8);
    const auto before=monotonic_ns();
    ssize_t count;
    if (synthetic_) {
      require(endpoint.peer.sin_port!=0,"DATA_INVALID: synthetic command peer absent");
      count=sendto(endpoint.fd.value,&wire,sizeof(wire),MSG_DONTWAIT,
                   reinterpret_cast<sockaddr*>(&endpoint.peer),sizeof(endpoint.peer));
    } else count=::send(endpoint.fd.value,&wire,sizeof(wire),MSG_DONTWAIT);
    const auto after=monotonic_ns();
    require(count==sizeof(wire),"HARD_ABORT: current preparation TX failed");
    std::string bytes;
    for (const auto b:frame.data) { const char* digits="0123456789abcdef"; bytes+=digits[b>>4]; bytes+=digits[b&15]; }
    if (axis && cybergear::unpack_ext_id(frame.id).comm_type==18) {
      const auto index=uint16_t(frame.data[0])|(uint16_t(frame.data[1])<<8);
      writes_[index]={bytes,before};
    }
    record("{\"kind\":\"neutral_tx\",\"axis\":"+quoted(axis?"pitch":"yaw")+",\"operation\":"+quoted(operation)+
      ",\"begin_ns\":"+std::to_string(before)+",\"kernel_accepted_ns\":"+std::to_string(after)+
      ",\"id\":"+std::to_string(frame.id)+",\"data_hex\":"+quoted(bytes)+",\"success\":true}");
    return before;
  }
  int64_t pitch(const cybergear::CanFrame& frame, const char* operation) { return send(1,raw(frame),operation); }
  void read(cybergear::Reg reg, int64_t now) {
    send(1,readback_.begin(reg,now,limits_.read_timeout),"register_read");
    readback_.accepted(monotonic_ns(),true);
  }
  void change(Preparation::State state,int64_t now) {
    prep_.state=state; prep_.entered=now; prep_.pending_begin=0;
    record("{\"kind\":\"preparation_state\",\"state\":"+std::to_string(int(state))+",\"time_ns\":"+std::to_string(now)+"}");
  }
  bool confirm(cybergear::Reg reg, double expected, int64_t now) {
    if (!prep_.pending_begin) { read(reg,now); prep_.pending_begin=now; return false; }
    if (readback_.pending()) return false;
    require(latest_read_ && latest_read_->reg==reg && latest_read_->value &&
            *latest_read_->value==double(float(expected)),"INTEGRATION_MISMATCH: pitch register readback differs");
    return true;
  }
  void step(int64_t now) {
    using S=Preparation::State;
    if (now>=next_yaw_) { send(0,gm6020::current_zero_frame(1),"current_zero"); next_yaw_=now+limits_.stop_period; }
    if (prep_.state==S::Discover) {
      if (!prep_.pending_begin) { prep_.pending_begin=now; pitch(cybergear::make_discovery_request(0,127),"discovery"); }
      require(now-prep_.pending_begin<limits_.read_timeout,"MEASUREMENT_LIMITED: pitch discovery timeout");
      if (identified_) change(S::Stop,now);
    } else if (prep_.state==S::Stop || prep_.state==S::FinalStop) {
      if (!prep_.pending_begin) {
        prep_.disabled=false; prep_.pending_begin=now; pitch(cybergear::make_stop(0,127),"stop");
      }
      require(now-prep_.pending_begin<limits_.read_timeout,"HARD_ABORT: pitch STOP confirmation timeout");
      if (prep_.disabled && prep_.status_ns>=prep_.pending_begin)
        change(prep_.state==S::Stop?S::Observe:S::RestoreMode,now);
    } else if (prep_.state==S::Observe) {
      if (!prep_.pending_begin) { read(cybergear::Reg::RunMode,now); prep_.pending_begin=now; }
      if (!readback_.pending() && prep_.original_mode<0) {
        require(latest_read_ && latest_read_->value && *latest_read_->value>=0 && *latest_read_->value<=3 &&
                *latest_read_->value==std::floor(*latest_read_->value),"DATA_INVALID: initial RunMode");
        prep_.original_mode=int(*latest_read_->value);
      }
      if (now>=prep_.next_ping) { pitch(cybergear::make_stop(0,127),"stop_poll"); prep_.next_ping=now+limits_.stop_period; }
      if (now-prep_.entered>=2'000'000'000LL && now-begin_>=limits_.startup && prep_.original_mode>=0) {
        check_streams(now); change(S::ZeroBefore,now);
      }
    } else if (prep_.state==S::ZeroBefore) {
      pitch(cybergear::make_write_reg_float(cybergear::Reg::IqRef,0,0,127),"zero_before_mode"); change(S::Mode,now);
    } else if (prep_.state==S::Mode) {
      pitch(cybergear::make_write_reg_u8(cybergear::Reg::RunMode,3,0,127),"select_current_mode"); change(S::CheckMode,now);
    } else if (prep_.state==S::CheckMode) {
      if (confirm(cybergear::Reg::RunMode,3,now)) change(S::CheckZero,now);
    } else if (prep_.state==S::CheckZero) {
      if (confirm(cybergear::Reg::IqRef,0,now)) change(S::Enable,now);
    } else if (prep_.state==S::Enable) {
      require(!buses_[0].receiver->kernel_drops() && !buses_[1].receiver->kernel_drops(),
              "DATA_INVALID: socket receive loss before enable");
      require(prep_.have_origin && prep_.disabled && now-prep_.status_ns<=limits_.can_gap,
              "HARD_ABORT: enabling without fresh disabled feedback and position");
      prep_.enabled=false;
      prep_.enable_begin_ns=pitch(cybergear::make_enable(0,127),"enable_at_zero");
      change(S::CheckEnabledMode,now);
    } else if (prep_.state==S::CheckEnabledMode) {
      if (confirm(cybergear::Reg::RunMode,3,now)) change(S::CheckEnabledZero,now);
    } else if (prep_.state==S::CheckEnabledZero) {
      if (confirm(cybergear::Reg::IqRef,0,now)) {
        require(prep_.enabled && prep_.status_ns>=prep_.enable_begin_ns &&
                now-prep_.status_ns<=limits_.can_gap,
                "HARD_ABORT: fresh enabled status absent after mode readback");
        change(S::ObserveEnabled,now);
      }
    } else if (prep_.state==S::ObserveEnabled) {
      require(prep_.enabled && now-prep_.status_ns<=limits_.can_gap,
              "HARD_ABORT: enabled current mode was lost");
      if (now>=prep_.next_ping) {
        pitch(cybergear::make_write_reg_float(cybergear::Reg::IqRef,0,0,127),"neutral_keepalive");
        prep_.next_ping=now+limits_.stop_period;
      }
      require(now-prep_.entered<=limits_.read_timeout || (prep_.actual_current_ns>=prep_.entered &&
              now-prep_.actual_current_ns<limits_.read_timeout),"MEASUREMENT_LIMITED: neutral Iq feedback missing");
      if (now-prep_.entered>=observation_ns_ && !readback_.pending()) {
        change(S::FinalStop,now);
      } else if (!readback_.pending() && now>=next_read_) {
        read(cybergear::Reg::Iqf,now); next_read_=now+limits_.read_period;
      }
    } else if (prep_.state==S::RestoreMode) {
      pitch(cybergear::make_write_reg_u8(cybergear::Reg::RunMode,prep_.original_mode,0,127),"restore_original_mode_disabled");
      change(S::CheckRestored,now);
    } else if (prep_.state==S::CheckRestored && confirm(cybergear::Reg::RunMode,prep_.original_mode,now)) change(S::Done,now);
  }
  void check_streams(int64_t now) {
    for (const auto& bus:buses_) require(bus.last_feedback && now-bus.last_feedback<=limits_.can_gap,
                                        "MEASUREMENT_LIMITED: CAN feedback absent/stale");
    require(imu_.ready(),"MEASUREMENT_LIMITED: IMU streams absent");
    for (const auto& [_,s]:imu_.sensors) require(now-s.received<=limits_.imu_gap,"MEASUREMENT_LIMITED: IMU stream stale");
  }
  void pump() {
    std::array<pollfd,3> fds{{{buses_[0].fd.value,POLLIN,0},{buses_[1].fd.value,POLLIN,0},{imu_fd_,POLLIN,0}}};
    const auto result=poll(fds.data(),fds.size(),2);
    if (result<0 && errno==EINTR) return;
    require(result>=0,"DATA_INVALID: acquisition poll failed");
    for (unsigned i=0;i<2;++i) {
      auto& bus=buses_[i];
      require(!(fds[i].revents&(POLLERR|POLLHUP|POLLNVAL)),"HARD_ABORT: CAN endpoint failed");
      Receipt r;
      // Limit each drain so continuous traffic cannot starve IMU handling,
      // signals, command deadlines or the state machine's stop path.
      for (unsigned drained=0; drained<256 && bus.receiver->receive(r); ++drained) {
        record(receipt_json(r,i?"pitch":"yaw",++bus.sequence));
        require(!r.drop_delta,"DATA_INVALID: socket receive loss");
        require(!r.frame.error,"HARD_ABORT: CAN error frame");
        require(r.dequeue_ns-r.kernel_monotonic_ns<=limits_.dequeue,"DATA_INVALID: stale CAN dequeue");
        if (!i) {
          gm6020::Feedback value;
          require(gm6020::decode(r.frame,1,value),"HARD_ABORT: unexpected yaw traffic");
          require(yaw_encoder_.update(value.angle_count,r.kernel_monotonic_ns),"HARD_ABORT: invalid yaw encoder");
          require(std::abs(yaw_encoder_.relative_rad())<=maximum_displacement_,"HARD_ABORT: neutral yaw displacement");
          bus.last_feedback=r.kernel_monotonic_ns; continue;
        }
        const auto id=cybergear::unpack_ext_id(r.frame.id);
        require(r.frame.extended && r.frame.dlc==8 && !r.frame.rtr,"HARD_ABORT: invalid pitch frame");
        cybergear::CanFrame frame; frame.id=r.frame.id; std::memcpy(frame.data,r.frame.data,8);
        if (id.comm_type==0) {
          cybergear::DiscoveryResponse response;
          require(!identified_ && prep_.state==Preparation::State::Discover && prep_.pending_begin &&
                  r.kernel_monotonic_ns>=prep_.pending_begin && r.kernel_monotonic_ns<prep_.pending_begin+limits_.read_timeout &&
                  id.data2==127 && cybergear::parse_discovery_response(frame,response) && response.unique_id==expected_uid_,
                  "INTEGRATION_MISMATCH: pitch identity or discovery correlation");
          identified_=true;
          record("{\"kind\":\"pitch_identity\",\"uid_hex\":"+quoted(uid_text_)+",\"receive_ns\":"+std::to_string(r.kernel_monotonic_ns)+"}");
        } else if (id.comm_type==17) {
          latest_read_=readback_.observe(r); record(readback_json(*latest_read_));
          require(latest_read_->value.has_value(),"MEASUREMENT_LIMITED: required current-mode register rejected");
          if (latest_read_->reg==cybergear::Reg::Iqf) {
            prep_.actual_current=*latest_read_->value; prep_.actual_current_ns=latest_read_->receive_ns;
            ++current_samples_;
            observed_current_max_=std::max(observed_current_max_,std::abs(prep_.actual_current));
            // Neutral offsets and noise remain measurements. The manufacturer
            // electrical protection is independent of a zero-current command.
            require(std::abs(prep_.actual_current)<=protection_current_,
                    "HARD_ABORT: manufacturer current protection bound exceeded");
          }
        } else if (id.comm_type==18) {
          const auto index=uint16_t(r.frame.data[0])|(uint16_t(r.frame.data[1])<<8);
          std::string bytes;
          for (const auto b:r.frame.data) { const char* digits="0123456789abcdef"; bytes+=digits[b>>4]; bytes+=digits[b&15]; }
          require(id.target==0 && id.data2==127 && writes_.count(index) &&
                  writes_.at(index).first==bytes && r.kernel_monotonic_ns>=writes_.at(index).second &&
                  r.kernel_monotonic_ns<writes_.at(index).second+limits_.read_timeout,
                  "DATA_INVALID: unexpected pitch write echo");
          record("{\"kind\":\"write_echo\",\"index\":"+std::to_string(index)+
                 ",\"receive_ns\":"+std::to_string(r.kernel_monotonic_ns)+",\"readback_verified\":false}");
        } else if (id.comm_type==2) {
          cybergear::Feedback feedback;
          require(cybergear::parse_feedback(frame,feedback) && feedback.motor_id==127 && feedback.host_id==0,
                  "HARD_ABORT: unexpected pitch feedback identity");
          require(!feedback.faults && feedback.temp_c<maximum_temperature_,"HARD_ABORT: pitch fault or temperature");
          if (!prep_.have_origin) { prep_.origin=feedback.angle_rad; prep_.have_origin=true; }
          require(std::abs(feedback.angle_rad-prep_.origin)<=maximum_displacement_,"HARD_ABORT: neutral transition displacement");
          prep_.disabled=feedback.mode==cybergear::MotorMode::Reset;
          prep_.enabled=feedback.mode==cybergear::MotorMode::Motor;
          prep_.status_ns=r.kernel_monotonic_ns;
          bus.last_feedback=r.kernel_monotonic_ns;
          const auto state=prep_.state;
          if (state==Preparation::State::Observe || state==Preparation::State::ZeroBefore ||
              state==Preparation::State::Mode || state==Preparation::State::CheckMode ||
              state==Preparation::State::CheckZero || state==Preparation::State::Enable ||
              state==Preparation::State::RestoreMode || state==Preparation::State::CheckRestored)
            require(prep_.disabled,"HARD_ABORT: pitch re-enabled during disabled transition");
          if (state==Preparation::State::ObserveEnabled)
            require(prep_.enabled,"HARD_ABORT: enabled current mode was lost");
        } else require(false,"HARD_ABORT: unexpected pitch traffic");
      }
    }
    require(!(fds[2].revents&(POLLERR|POLLNVAL)),"DATA_INVALID: IMU pipe failed");
    if (fds[2].revents&(POLLIN|POLLHUP)) imu_.read_fd(imu_fd_,limits_,*journal_);
  }
  void shutdown() noexcept {
    try { send(0,gm6020::current_zero_frame(1),"abort_zero"); } catch (...) {}
    if (!identified_) return;
    try { pitch(cybergear::make_write_reg_float(cybergear::Reg::IqRef,0,0,127),"abort_zero"); } catch (...) {}
    const auto stop_begin=monotonic_ns();
    bool stop_sent=false;
    try { pitch(cybergear::make_stop(0,127),"abort_stop"); stop_sent=true; } catch (...) {}
    // Preserve the original failure. Drain directly because the failed stream,
    // register transaction or writer must not prevent a STOP acknowledgement
    // from being observed. This never restarts acquisition or sends an enable.
    while (monotonic_ns()-stop_begin<limits_.read_timeout) {
      try {
        Receipt r;
        if (buses_[1].receiver->receive(r)) {
          journal_->append(receipt_json(r,"pitch",++buses_[1].sequence));
          cybergear::CanFrame frame; frame.id=r.frame.id; frame.dlc=r.frame.dlc;
          std::memcpy(frame.data,r.frame.data,8);
          cybergear::Feedback feedback;
          if (stop_sent && r.frame.extended && !r.frame.error && !r.frame.rtr && r.frame.dlc==8 &&
              cybergear::parse_feedback(frame,feedback) && feedback.motor_id==127 && feedback.host_id==0 &&
              feedback.mode==cybergear::MotorMode::Reset && !feedback.faults &&
              r.kernel_monotonic_ns>=stop_begin && r.kernel_monotonic_ns<stop_begin+limits_.read_timeout &&
              r.dequeue_ns-r.kernel_monotonic_ns<=limits_.dequeue) {
            abort_stop_confirmed_=true; break;
          }
        } else {
          pollfd fd{buses_[1].fd.value,POLLIN,0}; poll(&fd,1,2);
        }
      } catch (...) { break; }
    }
  }
  int finish(bool complete,const std::string& detail) {
    uint32_t yaw_drops=0,pitch_drops=0;
    std::string interface_loss="{";
    std::string failure=detail;
    try {
      yaw_drops=buses_[0].receiver->kernel_drops(); pitch_drops=buses_[1].receiver->kernel_drops();
      require(!yaw_drops && !pitch_drops,"DATA_INVALID: final socket receive loss");
      if (!synthetic_) for (unsigned axis=0; axis<buses_.size(); ++axis) {
        const auto& bus=buses_[axis];
        const auto drops=bus.counter("rx_dropped"),errors=bus.counter("rx_errors");
        require(drops>=bus.drops_begin && errors>=bus.errors_begin,"DATA_INVALID: interface counters reset");
        if (axis) interface_loss+=",";
        interface_loss+=quoted(axis?"pitch":"yaw")+":{\"rx_dropped\":"+std::to_string(drops-bus.drops_begin)+
                        ",\"rx_errors\":"+std::to_string(errors-bus.errors_begin)+"}";
        require(drops==bus.drops_begin && errors==bus.errors_begin,"DATA_INVALID: interface receive loss");
      }
    } catch (const std::exception& e) { complete=false; if (failure.empty()) failure=e.what(); }
    interface_loss+="}";
    const std::string footer="{\"kind\":\"footer\",\"status\":"+quoted(complete?"COMPLETE":"INVALID")+
      ",\"detail\":"+quoted(failure)+",\"parameter_qualified\":false,\"motion_qualified\":false,\"original_mode\":"+
      std::to_string(prep_.original_mode)+",\"abort_stop_confirmed\":"+(abort_stop_confirmed_?"true":"false")+
      ",\"normal_stop_confirmed\":"+(complete && prep_.disabled?"true":"false")+
      ",\"characterization_only\":"+(characterization_?"true":"false")+
      ",\"neutral_current_qualified\":false"+
      ",\"current_sample_count\":"+std::to_string(current_samples_)+
      ",\"observed_current_max_abs_A\":"+std::to_string(observed_current_max_)+
      ",\"interface_loss_deltas\":"+interface_loss+
      ",\"writer_queue_high_water\":"+std::to_string(journal_->high_water())+
      ",\"socket_drops\":{\"yaw\":"+std::to_string(yaw_drops)+",\"pitch\":"+
      std::to_string(pitch_drops)+"},\"yaw_frames\":"+std::to_string(buses_[0].sequence)+",\"pitch_frames\":"+
      std::to_string(buses_[1].sequence)+",\"end_ns\":"+std::to_string(monotonic_ns())+"}";
    const bool written=journal_->finish(footer);
    if (!written) {
      std::cout<<"{\"kind\":\"footer\",\"status\":\"INVALID\",\"detail\":\"DATA_INVALID: capture journal did not finish\","
                  "\"parameter_qualified\":false,\"motion_qualified\":false,\"normal_stop_confirmed\":false,"
                  "\"abort_stop_confirmed\":"<<(abort_stop_confirmed_?"true":"false")<<"}\n";
      return 1;
    }
    std::cout<<footer<<'\n'; return complete?0:1;
  }
  YAML::Node config_;
  Limits limits_;
  bool synthetic_;
  bool characterization_;
  int64_t observation_ns_{};
  std::string schema_,purpose_;
  std::string uid_text_;
  uint64_t expected_uid_{};
  int imu_fd_{};
  double protection_current_{},observed_current_max_{},maximum_displacement_{},maximum_temperature_{};
  uint64_t current_samples_{};
  std::array<Endpoint,2> buses_;
  ImuStream imu_;
  Readback readback_;
  std::optional<ReadObservation> latest_read_;
  std::unique_ptr<Journal> journal_;
  Preparation prep_;
  gm6020::UnwrappedEncoder yaw_encoder_;
  std::map<unsigned,std::pair<std::string,int64_t>> writes_;
  bool abort_stop_confirmed_{};
  bool identified_{};
  int64_t begin_{},next_yaw_{},next_read_{};
};
}
int current_preparation_session(const char* path) {
  try { return NeutralSession(YAML::LoadFile(path)).run(); }
  catch (const std::exception& e) {
    std::cerr<<"{\"status\":\"INVALID\",\"detail\":"<<quoted(e.what())<<"}\n"; return 1;
  }
}
int current_characterization_session(const char* path) {
  try { return NeutralSession(YAML::LoadFile(path),true).run(); }
  catch (const std::exception& e) {
    std::cerr<<"{\"status\":\"INVALID\",\"detail\":"<<quoted(e.what())<<"}\n"; return 1;
  }
}
}
