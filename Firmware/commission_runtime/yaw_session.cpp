#include "session.hpp"
#include "io.hpp"
#include <algorithm>
#include <numbers>

namespace ota::commission {
using namespace detail;
namespace {
struct Segment { double duration, first, last; };

class YawSession {
 public:
  explicit YawSession(const YAML::Node& config)
      : config_(config), limits_(config["limits"]),
        synthetic_(config["provenance"].as<std::string>()=="SYNTHETIC"), readback_(127,0) {
    require(config["schema"].as<std::string>()=="adr0022.yaw-acquisition/1" &&
            config["purpose"].as<std::string>()=="yaw_current_identification",
            "INTEGRATION_MISMATCH: yaw acquisition manifest");
    require(synthetic_ || config["provenance"].as<std::string>()=="MEASURED", "DATA_INVALID: provenance");
    require(config["transport"].as<std::string>()==(synthetic_?"loopback_udp":"socketcan"),
            "INTEGRATION_MISMATCH: transport/provenance mismatch");
    require(config["pitch_supported_when_disabled"].as<bool>(), "HARD_ABORT: pitch support required");
    uid_text_=config["expected_pitch_uid"].as<std::string>();
    require(uid_text_.size()==16 && uid_text_.find_first_not_of("0123456789abcdef")==std::string::npos,
            "DATA_INVALID: pitch UID");
    expected_uid_=std::stoull(uid_text_,nullptr,16);
    baseline_s_=finite_positive("baseline_s");
    require(synthetic_ || baseline_s_>=2., "DATA_INVALID: measured baseline must cover two seconds");
    stop_s_=finite_positive("stop_observation_s");
    require(stop_s_==2., "DATA_INVALID: two-second stop observation required");
    current_bound_=finite_positive("yaw_current_bound_A");
    require(current_bound_<=gm6020::kAmpsFullScale, "HARD_ABORT: yaw protocol current range exceeded");
    if (config["yaw_displacement_target_deg"]) {
      displacement_target_deg_=config["yaw_displacement_target_deg"].as<double>();
      require(std::isfinite(displacement_target_deg_) && displacement_target_deg_!=0.,
              "DATA_INVALID: finite signed yaw displacement target required");
      have_displacement_target_=true;
    }
    pitch_maximum_temperature_=finite_positive("pitch_maximum_temperature_C");
    const auto plan=config["current_segments"];
    require(plan.IsSequence() && plan.size()>0, "DATA_INVALID: current segment sequence required");
    double episode_s=0.;
    for (const auto& item:plan) {
      Segment segment{item["duration_s"].as<double>(),item["start_A"].as<double>(),item["end_A"].as<double>()};
      require(std::isfinite(segment.duration) && segment.duration>0 &&
              std::isfinite(segment.first) && std::isfinite(segment.last), "DATA_INVALID: finite current segment required");
      if (segment.first==0. && segment.last==0.) episode_s=0.;
      else episode_s+=segment.duration;
      require(synthetic_ || current_bound_<=0.9 || episode_s<=0.5+1e-12,
              "HARD_ABORT: initial measured nonzero current episode exceeds 0.5 seconds");
      require(std::abs(segment.first)<=current_bound_ && std::abs(segment.last)<=current_bound_,
              "HARD_ABORT: current segment exceeds declared current authority");
      segments_.push_back(segment); excitation_s_+=segment.duration;
    }
    require((baseline_s_+excitation_s_+stop_s_)*1e9<limits_.duration,
            "DATA_INVALID: acquisition sequence does not fit session deadline");
    imu_fd_=config["imu_fd"].as<int>();
    struct stat info{};
    require(imu_fd_>2 && imu_fd_!=8 && fstat(imu_fd_,&info)==0 && S_ISFIFO(info.st_mode),
            "DATA_INVALID: inherited IMU pipe required");
    require(fcntl(imu_fd_,F_SETFL,fcntl(imu_fd_,F_GETFL)|O_NONBLOCK)==0,
            "DATA_INVALID: IMU nonblocking pipe");
    if (!synthetic_) {
      check_launcher_lease();
      require(config["yaw"]["interface"].as<std::string>()=="can0" &&
              config["pitch"]["interface"].as<std::string>()=="can1", "INTEGRATION_MISMATCH: station topology");
    }
    journal_=std::make_unique<Journal>(config["output"].as<std::string>(),
      "{\"kind\":\"header\",\"schema\":\"adr0022.yaw-acquisition/1\",\"purpose\":\"yaw_current_identification\","
      "\"provenance\":"+quoted(synthetic_?"SYNTHETIC":"MEASURED")+",\"parameter_qualified\":false,"
      "\"manifest_yaml\":"+quoted(json(config))+"}");
    buses_[0].open(config["yaw"],synthetic_,limits_);
    buses_[1].open(config["pitch"],synthetic_,limits_);
  }

  int run() {
    interrupted=0; std::signal(SIGINT,on_signal); std::signal(SIGTERM,on_signal);
    begin_=monotonic_ns();
    std::cout<<"{\"kind\":\"capture_ready\",\"excitation\":true,\"purpose\":\"yaw_current_identification\"}\n"<<std::flush;
    std::string failure;
    bool sequence_complete=false;
    try {
      record("{\"kind\":\"session_begin\",\"time_ns\":"+std::to_string(begin_)+"}");
      yaw(0.,"baseline");
      discovery_begin_=pitch(cybergear::make_discovery_request(0,127),"discovery");
      while (!sequence_complete) {
        pump();
        const auto now=monotonic_ns();
        require(!interrupted, "HARD_ABORT: yaw acquisition interrupted");
        require(now-begin_<limits_.duration, "MEASUREMENT_LIMITED: yaw acquisition deadline");
        require(identified_ || now-discovery_begin_<limits_.read_timeout,
                "MEASUREMENT_LIMITED: pitch discovery timeout");
        readback_.check_deadline(now);
        supervise(now);
        service_pitch(now);
        if (!excitation_begin_ && now-begin_>=int64_t(baseline_s_*1e9) &&
            identified_ && disabled_ && have_mode_) {
          check_streams(now);
          excitation_reference_counts_=encoder_counts_;
          excitation_begin_=now;
          record("{\"kind\":\"excitation_begin\",\"time_ns\":"+std::to_string(now)+"}");
          if (have_displacement_target_) {
            std::ostringstream row; row.precision(17);
            row<<"{\"kind\":\"yaw_displacement_reference\",\"target_deg\":"<<displacement_target_deg_
               <<",\"encoder_raw\":"<<previous_encoder_<<",\"encoder_unwrapped_counts\":"<<excitation_reference_counts_
               <<",\"kernel_monotonic_ns\":"<<buses_[0].last_feedback<<",\"excitation_begin_ns\":"<<now<<"}";
            record(row.str());
          }
        }
        const double elapsed=excitation_begin_?(now-excitation_begin_)*1e-9:0.;
        if (excitation_begin_ && (target_reached_ || elapsed>=excitation_s_)) sequence_complete=true;
        else if (now>=next_yaw_) {
          yaw(excitation_begin_?requested(elapsed):0.,excitation_begin_?"excitation":"baseline");
          next_yaw_=now+5'000'000;
        }
        require(journal_->healthy(), "DATA_INVALID: capture writer failed");
      }
    } catch (const std::exception& e) { failure=e.what(); }
    // Zero is requested before all other work, including failure reporting. The
    // ensuing interval records feedback; it does not certify physical stopping.
    const auto stop_begin=monotonic_ns();
    bool zero_completed=false;
    try { yaw(0.,"stop"); zero_completed=true; }
    catch (const std::exception& e) { if (failure.empty()) failure=e.what(); }
    excitation_end_displacement_deg_=displacement_deg();
    if (excitation_begin_ && have_displacement_target_) {
      std::ostringstream row; row.precision(17);
      row<<"{\"kind\":\"yaw_excitation_endpoint\",\"target_deg\":"<<displacement_target_deg_
         <<",\"target_reached\":"<<(target_reached_?"true":"false")
         <<",\"actual_displacement_deg\":"<<excitation_end_displacement_deg_
         <<",\"reason\":"<<quoted(target_reached_?"target_observed":(sequence_complete?"waveform_deadline":"failure"))
         <<",\"kernel_monotonic_ns\":"<<buses_[0].last_feedback<<",\"zero_request_begin_ns\":"<<stop_begin<<"}";
      record_best_effort(row.str());
    }
    record_best_effort("{\"kind\":\"stop_observation_begin\",\"time_ns\":"+std::to_string(stop_begin)+"}");
    next_yaw_=stop_begin+5'000'000;
    while (monotonic_ns()-stop_begin<int64_t(stop_s_*1e9)) {
      try {
        pump();
        const auto now=monotonic_ns();
        if (now>=next_yaw_) { yaw(0.,"stop"); zero_completed=true; next_yaw_=now+5'000'000; }
        if (identified_ && now>=next_stop_) {
          pitch(cybergear::make_stop(0,127),"stop_poll"); next_stop_=now+limits_.stop_period;
        }
        supervise(now);
      } catch (const std::exception& e) {
        if (failure.empty()) failure=e.what();
        // Keep attempting zero and recording received streams for the full
        // interval even if a previously failed stream cannot qualify the run.
        const auto now=monotonic_ns();
        if (now>=next_yaw_) {
          try { yaw(0.,"stop"); zero_completed=true; } catch (...) {}
          next_yaw_=now+5'000'000;
        }
      }
    }
    return finish(sequence_complete && zero_completed && failure.empty(),sequence_complete,zero_completed,failure);
  }

 private:
  double finite_positive(const char* key) const {
    const double value=config_[key].as<double>();
    require(std::isfinite(value) && value>0., "DATA_INVALID: finite positive timing or current required");
    return value;
  }
  void record(const std::string& row) { require(journal_->append(row),"DATA_INVALID: capture writer failed"); }
  void record_best_effort(const std::string& row) { journal_->append(row); }
  double displacement_deg() const { return (encoder_counts_-excitation_reference_counts_)*(360./8192.); }
  double requested(double elapsed) const {
    for (const auto& segment:segments_) {
      if (elapsed<segment.duration) return segment.first+(segment.last-segment.first)*elapsed/segment.duration;
      elapsed-=segment.duration;
    }
    return 0.;
  }
  std::pair<int64_t,int64_t> transmit(unsigned axis,const can::RawFrame& frame,bool& success) {
    auto& bus=buses_[axis];
    can_frame wire{}; wire.can_id=frame.id|(frame.extended?CAN_EFF_FLAG:0); wire.can_dlc=frame.dlc;
    std::memcpy(wire.data,frame.data,8);
    const auto before=monotonic_ns();
    ssize_t count;
    if (synthetic_) count=sendto(bus.fd.value,&wire,sizeof(wire),MSG_DONTWAIT,
                                 reinterpret_cast<sockaddr*>(&bus.peer),sizeof(bus.peer));
    else count=::send(bus.fd.value,&wire,sizeof(wire),MSG_DONTWAIT);
    const auto after=monotonic_ns();
    success=count==sizeof(wire); return {before,after};
  }
  void yaw(double requested_A,const char* phase) {
    const double limited=std::clamp(requested_A,-current_bound_,current_bound_);
    const int raw_current=gm6020::current_raw_uncapped(limited);
    auto frame=gm6020::current_zero_frame(1);
    const auto encoded=static_cast<uint16_t>(static_cast<int16_t>(raw_current));
    frame.data[0]=uint8_t(encoded>>8); frame.data[1]=uint8_t(encoded);
    bool success=false;
    const auto [before,after]=transmit(0,frame,success);
    std::ostringstream row; row.precision(17);
    row<<"{\"kind\":\"yaw_current_tx\",\"phase\":"<<quoted(phase)<<",\"requested_A\":"<<requested_A
       <<",\"limited_A\":"<<limited<<",\"successful_tx_A\":";
    if (success) row<<raw_current*gm6020::kAmpsPerRaw; else row<<"null";
    row<<",\"current_raw\":"<<raw_current<<",\"id\":510,\"unused_slots_zero\":true,\"begin_ns\":"<<before
       <<",\"kernel_accepted_ns\":"<<after<<",\"success\":"<<(success?"true":"false")<<"}";
    record(row.str());
    require(success,"HARD_ABORT: yaw current TX failed");
  }
  int64_t pitch(const cybergear::CanFrame& source,const char* operation) {
    can::RawFrame frame; frame.id=source.id; frame.dlc=source.dlc; std::memcpy(frame.data,source.data,8);
    const auto id=cybergear::unpack_ext_id(frame.id);
    require(id.target==127 && id.data2==0 &&
            (id.comm_type==0 || id.comm_type==4 || id.comm_type==17),
            "HARD_ABORT: pitch command must be discovery, STOP or read");
    bool success=false; const auto [before,after]=transmit(1,frame,success);
    std::string bytes;
    for (const auto b:frame.data) { const char* digits="0123456789abcdef"; bytes+=digits[b>>4]; bytes+=digits[b&15]; }
    record("{\"kind\":\"pitch_tx\",\"operation\":"+quoted(operation)+",\"id\":"+std::to_string(frame.id)+
           ",\"data_hex\":"+quoted(bytes)+",\"begin_ns\":"+std::to_string(before)+
           ",\"kernel_accepted_ns\":"+std::to_string(after)+",\"success\":"+(success?"true":"false")+"}");
    require(success,"HARD_ABORT: pitch STOP/read TX failed");
    return before;
  }
  void service_pitch(int64_t now) {
    if (!identified_) return;
    if (now>=next_stop_) {
      if (!stop_begin_) stop_begin_=now;
      pitch(cybergear::make_stop(0,127),"stop_poll"); next_stop_=now+limits_.stop_period;
    }
    require(disabled_ || now-stop_begin_<limits_.read_timeout,"HARD_ABORT: pitch STOP feedback timeout");
    if (disabled_ && !readback_.pending() && now>=next_read_) {
      const auto reg=have_mode_?((periodic_read_++%2)?cybergear::Reg::Iqf:cybergear::Reg::MechPos):cybergear::Reg::RunMode;
      const auto request=readback_.begin(reg,now,limits_.read_timeout);
      cybergear::CanFrame frame; frame.id=request.id; std::memcpy(frame.data,request.data,8);
      pitch(frame,"register_read"); readback_.accepted(monotonic_ns(),true); next_read_=now+limits_.read_period;
    }
  }
  void check_streams(int64_t now) {
    for (const auto& bus:buses_) require(bus.last_feedback && now-bus.last_feedback<=limits_.can_gap,
                                        "HARD_ABORT: CAN feedback absent/stale");
    require(disabled_,"HARD_ABORT: pitch is not disabled");
    require(imu_.ready(),"MEASUREMENT_LIMITED: IMU streams absent");
    for (const auto& [_,sensor]:imu_.sensors)
      require(now-sensor.received<=limits_.imu_gap,"MEASUREMENT_LIMITED: IMU stream stale");
  }
  void supervise(int64_t now) {
    for (const auto& bus:buses_) if (bus.last_feedback)
      require(now-bus.last_feedback<=limits_.can_gap,"HARD_ABORT: CAN feedback lost");
    for (const auto& [_,sensor]:imu_.sensors) if (sensor.count)
      require(now-sensor.received<=limits_.imu_gap,"MEASUREMENT_LIMITED: IMU stream stale");
    if (excitation_begin_) require(disabled_,"HARD_ABORT: pitch became enabled during yaw acquisition");
  }
  void yaw_feedback(const gm6020::Feedback& value,const Receipt& r) {
    if (have_encoder_) {
      int delta=int(value.angle_count)-int(previous_encoder_);
      if (delta>4096) delta-=8192;
      if (delta<-4096) delta+=8192;
      encoder_counts_+=delta;
    }
    previous_encoder_=value.angle_count; have_encoder_=true;
    std::ostringstream row; row.precision(17);
    row<<"{\"kind\":\"yaw_feedback\",\"encoder_raw\":"<<value.angle_count
       <<",\"encoder_unwrapped_counts\":"<<encoder_counts_<<",\"q_relative_rad\":"
       <<encoder_counts_*(2.*std::numbers::pi/8192.)<<",\"speed_rpm\":"<<value.speed_rpm
       <<",\"current_raw\":"<<value.current_raw<<",\"current_A\":"<<value.current_a()
       <<",\"temperature_raw\":"<<unsigned(value.temperature_raw)
       <<",\"kernel_realtime_ns\":"<<r.kernel_realtime_ns<<",\"kernel_monotonic_ns\":"<<r.kernel_monotonic_ns
       <<",\"dequeue_ns\":"<<r.dequeue_ns<<"}";
    record(row.str());
    if (have_displacement_target_ && excitation_begin_ && !target_reached_) {
      const double displacement=displacement_deg();
      if (displacement_target_deg_>0.?displacement>=displacement_target_deg_:displacement<=displacement_target_deg_) {
        target_reached_=true;
        target_observed_displacement_deg_=displacement;
        std::ostringstream observation; observation.precision(17);
        observation<<"{\"kind\":\"yaw_displacement_target_observed\",\"target_deg\":"<<displacement_target_deg_
                   <<",\"actual_displacement_deg\":"<<displacement
                   <<",\"encoder_unwrapped_counts\":"<<encoder_counts_
                   <<",\"kernel_monotonic_ns\":"<<r.kernel_monotonic_ns<<"}";
        record(observation.str());
      }
    }
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
      for (unsigned drained=0;drained<256 && bus.receiver->receive(r);++drained) {
        record(receipt_json(r,i?"pitch":"yaw",++bus.sequence));
        require(!r.frame.error,"HARD_ABORT: CAN error frame");
        require(!r.drop_delta,"DATA_INVALID: socket receive loss");
        if (!i) {
          gm6020::Feedback value;
          if (gm6020::decode(r.frame,1,value)) {
            yaw_feedback(value,r); bus.last_feedback=r.kernel_monotonic_ns;
          }
          continue;
        }
        if (!r.frame.extended || r.frame.rtr || r.frame.dlc!=8) continue;
        const auto id=cybergear::unpack_ext_id(r.frame.id);
        cybergear::CanFrame frame; frame.id=r.frame.id; std::memcpy(frame.data,r.frame.data,8);
        if (id.comm_type==0) {
          cybergear::DiscoveryResponse response;
          require(cybergear::parse_discovery_response(frame,response) && id.data2==127 &&
                  response.unique_id==expected_uid_,"INTEGRATION_MISMATCH: pitch identity");
          identified_=true;
          record("{\"kind\":\"pitch_identity\",\"uid_hex\":"+quoted(uid_text_)+
                 ",\"receive_ns\":"+std::to_string(r.kernel_monotonic_ns)+"}");
        } else if (id.comm_type==17) {
          const auto observation=readback_.observe(r); record(readback_json(observation));
          if (observation.reg==cybergear::Reg::RunMode) {
            require(observation.value.has_value(),"MEASUREMENT_LIMITED: pitch mode readback unavailable");
            have_mode_=true;
          }
        } else if (id.comm_type==2) {
          cybergear::Feedback feedback;
          require(cybergear::parse_feedback(frame,feedback) && feedback.motor_id==127 && feedback.host_id==0,
                  "INTEGRATION_MISMATCH: pitch feedback identity");
          require(!feedback.faults && feedback.temp_c<pitch_maximum_temperature_,"HARD_ABORT: pitch fault or temperature");
          const bool disabled=feedback.mode==cybergear::MotorMode::Reset;
          require(!disabled_ || disabled,"HARD_ABORT: pitch became enabled");
          if (stop_begin_ && r.kernel_monotonic_ns>=stop_begin_) disabled_=disabled;
          bus.last_feedback=r.kernel_monotonic_ns;
        } else if (id.comm_type==21) require(false,"HARD_ABORT: pitch fault response");
      }
    }
    require(!(fds[2].revents&(POLLERR|POLLNVAL)),"DATA_INVALID: IMU pipe failed");
    if (fds[2].revents&(POLLIN|POLLHUP)) imu_.read_fd(imu_fd_,limits_,*journal_);
  }
  int finish(bool complete,bool sequence_complete,bool zero_completed,std::string failure) {
    try {
      for (const auto& bus:buses_) {
        require(!bus.receiver->kernel_drops(),"DATA_INVALID: final socket receive loss");
        if (!synthetic_) require(bus.counter("rx_dropped")==bus.drops_begin && bus.counter("rx_errors")==bus.errors_begin,
                                 "DATA_INVALID: interface receive loss");
      }
    } catch (const std::exception& e) { complete=false; if (failure.empty()) failure=e.what(); }
    std::ostringstream displacement_fields; displacement_fields.precision(17);
    displacement_fields<<",\"yaw_displacement_target_deg\":";
    if (have_displacement_target_) displacement_fields<<displacement_target_deg_; else displacement_fields<<"null";
    displacement_fields<<",\"yaw_target_reached\":"<<(target_reached_?"true":"false")
                       <<",\"yaw_excitation_displacement_deg\":"<<excitation_end_displacement_deg_
                       <<",\"yaw_final_displacement_deg\":"<<displacement_deg()
                       <<",\"yaw_target_observed_displacement_deg\":";
    if (target_reached_) displacement_fields<<target_observed_displacement_deg_; else displacement_fields<<"null";
    const std::string footer="{\"kind\":\"footer\",\"status\":"+quoted(complete?"COMPLETE":"INVALID")+
      ",\"detail\":"+quoted(failure)+",\"sequence_complete\":"+(sequence_complete?"true":"false")+
      ",\"zero_request_completed\":"+(zero_completed?"true":"false")+
      ",\"stop_observation_s\":2,\"motion_qualified\":false,\"parameter_qualified\":false,"
      "\"yaw_frames\":"+std::to_string(buses_[0].sequence)+",\"pitch_frames\":"+std::to_string(buses_[1].sequence)+
      ",\"end_ns\":"+std::to_string(monotonic_ns())+displacement_fields.str()+"}";
    const bool written=journal_->finish(footer);
    std::cout<<footer<<'\n';
    return complete && written?0:1;
  }
  YAML::Node config_;
  Limits limits_;
  bool synthetic_;
  std::string uid_text_;
  uint64_t expected_uid_{};
  int imu_fd_{};
  double baseline_s_{},stop_s_{},current_bound_{},pitch_maximum_temperature_{},excitation_s_{};
  double displacement_target_deg_{},target_observed_displacement_deg_{},excitation_end_displacement_deg_{};
  std::vector<Segment> segments_;
  std::array<Endpoint,2> buses_;
  ImuStream imu_;
  Readback readback_;
  std::unique_ptr<Journal> journal_;
  bool identified_{},disabled_{},have_mode_{},have_encoder_{};
  bool have_displacement_target_{},target_reached_{};
  uint16_t previous_encoder_{};
  int64_t encoder_counts_{};
  int64_t excitation_reference_counts_{};
  uint64_t periodic_read_{};
  int64_t begin_{},discovery_begin_{},stop_begin_{},excitation_begin_{},next_yaw_{},next_stop_{},next_read_{};
};
}
int yaw_acquisition_session(const char* path) {
  try { return YawSession(YAML::LoadFile(path)).run(); }
  catch (const std::exception& e) {
    std::cerr<<"{\"status\":\"INVALID\",\"detail\":"<<quoted(e.what())<<"}\n";
    return 1;
  }
}
}
