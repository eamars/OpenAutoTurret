#include "session.hpp"
#include "io.hpp"
#include "session_parts.hpp"
#include "servo.hpp"
#include "servo_config.hpp"
#include <algorithm>
#include <deque>
#include <numbers>

namespace ota::commission {
using namespace detail;
namespace {
struct AppliedCurrent { bool success; double actual; int64_t accepted; };

// Yaw position servo session (tools/servo_commission): the shared Servo stepped on
// every GM6020 encoder frame against a manifest reference table, pitch held
// disabled and polled, then zero current and a two-second stop observation.
class YawControlSession {
 public:
  explicit YawControlSession(const YAML::Node& config)
      : config_(config),limits_(config["limits"]),
        synthetic_(config["provenance"].as<std::string>()=="SYNTHETIC"),readback_(127,0) {
    require(config["schema"].as<std::string>()=="adr0022.yaw-control/1" &&
            config["purpose"].as<std::string>()=="yaw_shared_core_3a","INTEGRATION_MISMATCH: yaw control manifest");
    require(synthetic_ || config["provenance"].as<std::string>()=="MEASURED","DATA_INVALID: provenance");
    require(config["transport"].as<std::string>()==(synthetic_?"loopback_udp":"socketcan"),
            "INTEGRATION_MISMATCH: transport/provenance mismatch");
    require(config["pitch_supported_when_disabled"].as<bool>(),"HARD_ABORT: pitch support required");
    candidate_=config["candidate_label"].as<std::string>();
    require(!candidate_.empty(),"DATA_INVALID: descriptive candidate label required");
    uid_text_=config["expected_pitch_uid"].as<std::string>();
    require(uid_text_.size()==16 && uid_text_.find_first_not_of("0123456789abcdef")==std::string::npos,
            "DATA_INVALID: pitch UID");
    uid_=std::stoull(uid_text_,nullptr,16);
    baseline_s_=positive("baseline_s"); stop_s_=positive("stop_observation_s");
    require(baseline_s_>=2. && stop_s_==2.,"DATA_INVALID: two-second baseline and zero observation required");
    current_bound_=positive("yaw_current_bound_A");
    // Owner ruling 2026-10-02: GM6020 peaks up to the 3 A protocol full scale, the
    // servo's RMS budget up to the 1.62 A continuous rating; the motor temperature trip
    // below is the thermal protection.
    require(current_bound_<=3.0,"HARD_ABORT: yaw current authority");
    pitch_temperature_=positive("pitch_maximum_temperature_C");
    require(std::isfinite(config["other_axis_posture_rad"].as<double>()),"DATA_INVALID: finite measured coordinates");
    try { servo_parameters_=axis::servo_from_yaml(config["servo_parameters"]); }
    catch (const std::exception& e) { throw std::runtime_error(std::string("DATA_INVALID: servo parameters: ")+e.what()); }
    require(servo_parameters_.current_cap==current_bound_,"INTEGRATION_MISMATCH: current authority differs from servo");
    require(servo_parameters_.rms_limit<=1.62 && servo_parameters_.rms_tau_s<=5.,"HARD_ABORT: servo RMS budget above the 1.62 A continuous rating");
    motor_temperature_limit_=config["servo_motor_temperature_limit_C"].as<double>();
    require(motor_temperature_limit_>0 && motor_temperature_limit_<=70.,"DATA_INVALID: servo motor temperature limit");
    require(servo_.configure(servo_parameters_),"DATA_INVALID: servo parameter apply failed");
    speed_limit_=config["servo_speed_limit_rad_s"].as<double>();
    // Owner rule 2026-10-02: stay below 100 RPM (10.47 rad/s).
    require(std::isfinite(speed_limit_) && speed_limit_>0 && speed_limit_<=10.,"DATA_INVALID: servo speed limit");
    hold_after_s_=config["servo_hold_after_s"].as<double>();
    require(std::isfinite(hold_after_s_) && hold_after_s_>=0 && hold_after_s_<=10.,"DATA_INVALID: servo hold window");
    // Optional limit-cycle guard (tools/servo_commission sets it): stop the session
    // when the current's fast RMS exceeds this many amperes.
    oscillation_=axis::OscillationMonitor(config["servo_oscillation_limit_A"]?config["servo_oscillation_limit_A"].as<double>():0.);
    if (const auto schedule=config["servo_gain_schedule"]) {
      for (const auto& item:schedule)
        gain_schedule_.push_back({item["begin_s"].as<double>(),item["kq"].as<double>(),item["kv"].as<double>(),item["ki"].as<double>()});
      require(!gain_schedule_.empty() && gain_schedule_.size()<=64,"DATA_INVALID: servo gain schedule");
    }
    if (const auto x=config["servo_excitation"]) {
      // Identification only: a log-swept sine added to the servo output.
      excitation_={x["amplitude_A"].as<double>(),x["f0_hz"].as<double>(),x["f1_hz"].as<double>(),
                   x["begin_s"].as<double>(),x["duration_s"].as<double>()};
      require(excitation_.valid(.4),"DATA_INVALID: servo excitation");
    }
    const auto calibration=config["gyro_calibration"];
    const auto column=calibration["yaw_column"].as<std::vector<double>>();
    const auto bias=calibration["baseline_sensor_bias"].as<std::vector<double>>();
    require(column.size()==3 && bias.size()==3,"DATA_INVALID: frozen yaw gyro column and bias required");
    double norm=0.; for (const auto value:column) { require(std::isfinite(value),"DATA_INVALID: gyro column"); norm+=value*value; }
    require(norm>0.,"DATA_INVALID: gyro column has no yaw projection");
    for (unsigned i=0;i<3;++i) { require(std::isfinite(bias[i]),"DATA_INVALID: gyro bias"); projection_[i]=column[i]/norm; bias_[i]=bias[i]; }
    const auto samples=config["reference_samples"];
    require(samples && samples.IsSequence() && samples.size()>=2,"DATA_INVALID: finite reference sample table required");
    try {
      for (const auto& item:samples)
        reference_.add({item["time_s"].as<double>(),item["position_rad"].as<double>(),
                        item["velocity_rad_s"].as<double>(),item["acceleration_rad_s2"].as<double>()});
    } catch (const std::runtime_error&) {
      require(false,"DATA_INVALID: reference samples must be finite, start at zero and increase in time");
    }
    require((baseline_s_+reference_.duration()+hold_after_s_+stop_s_)*1e9<limits_.duration,"DATA_INVALID: control sequence deadline");
    imu_fd_=config["imu_fd"].as<int>(); struct stat info{};
    require(imu_fd_>2 && imu_fd_!=8 && fstat(imu_fd_,&info)==0 && S_ISFIFO(info.st_mode),
            "DATA_INVALID: inherited IMU pipe required");
    require(fcntl(imu_fd_,F_SETFL,fcntl(imu_fd_,F_GETFL)|O_NONBLOCK)==0,"DATA_INVALID: IMU nonblocking pipe");
    if (!synthetic_) {
      check_launcher_lease();
      require(config["yaw"]["interface"].as<std::string>()=="can0" && config["pitch"]["interface"].as<std::string>()=="can1",
              "INTEGRATION_MISMATCH: station topology");
    }
    journal_=std::make_unique<Journal>(config["output"].as<std::string>(),
      "{\"kind\":\"header\",\"schema\":\"adr0022.yaw-control/1\",\"purpose\":\"yaw_shared_core_3a\",\"provenance\":"+
      quoted(synthetic_?"SYNTHETIC":"MEASURED")+",\"candidate_label\":"+quoted(candidate_)+",\"parameter_qualified\":false,\"manifest_yaml\":"+quoted(json(config))+"}",
      16384);  // 1 kHz sessions log ~3 rows/ms; absorb SD-card write stalls
    buses_[0].open(config["yaw"],synthetic_,limits_); buses_[1].open(config["pitch"],synthetic_,limits_);
  }
  int run() {
    interrupted=0; std::signal(SIGINT,on_signal); std::signal(SIGTERM,on_signal); begin_=monotonic_ns();
    std::cout<<"{\"kind\":\"capture_ready\",\"excitation\":true,\"purpose\":\"yaw_shared_core_3a\"}\n"<<std::flush;
    std::string failure; bool complete=false,zero_completed=false;
    try {
      record("{\"kind\":\"session_begin\",\"time_ns\":"+std::to_string(begin_)+",\"startup_discarded\":["+
             std::to_string(buses_[0].receiver->startup_discarded())+","+std::to_string(buses_[1].receiver->startup_discarded())+"]}");
      record("{\"kind\":\"controller_parameters_readback\",\"candidate_label\":"+quoted(candidate_)+
             ",\"source\":\"configured_servo\",\"parameters\":"+axis::servo_to_json(servo_.parameters())+"}");
      require(yaw(0.,"baseline").success,"HARD_ABORT: yaw baseline zero TX failed");
      discovery_begin_=pitch(cybergear::make_discovery_request(0,127),"discovery");
      while (!complete) {
        pump(); const auto now=monotonic_ns();
        require(!interrupted,"HARD_ABORT: yaw servo session interrupted");
        require(now-begin_<limits_.duration,"MEASUREMENT_LIMITED: yaw control deadline");
        require(identified_ || now-discovery_begin_<limits_.read_timeout,"MEASUREMENT_LIMITED: pitch discovery timeout");
        readback_.check_deadline(now); supervise(now); service_pitch(now);
        if (!control_begin_ && now-begin_>=int64_t(baseline_s_*1e9) && identified_ && disabled_ && have_mode_) {
          check_streams(now); initial_position_=position(); control_begin_=now;
          require(servo_.reset(seconds(now),initial_position_,0.,0.),"DATA_INVALID: servo reset failed");
          record("{\"kind\":\"yaw_control_begin\",\"time_ns\":"+std::to_string(now)+"}");
        }
        // The servo holds the final reference sample, then the session ends.
        if (control_begin_ && (now-control_begin_)*1e-9>=reference_.duration()+hold_after_s_) { complete=true; servo_done_=true; }
        if (!control_begin_ && now>=next_baseline_) {
          require(yaw(0.,"baseline").success,"HARD_ABORT: yaw baseline TX failed"); next_baseline_=now+5'000'000;
        }
        require(journal_->healthy(),"DATA_INVALID: capture writer failed");
      }
    } catch (const std::exception& e) { failure=e.what(); }
    servo_done_=true;  // nothing but zero current from here on, whatever ended the loop
    record_best("{\"kind\":\"servo_learned\",\"parameters\":"+axis::servo_to_json(servo_.learned())+"}");
    const auto stop_begin=monotonic_ns();
    try { zero_completed=yaw(0.,"stop").success; require(zero_completed,"HARD_ABORT: final zero TX failed"); }
    catch (const std::exception& e) { if (failure.empty()) failure=e.what(); }
    record_best("{\"kind\":\"stop_observation_begin\",\"time_ns\":"+std::to_string(stop_begin)+"}");
    int64_t next_zero=stop_begin+5'000'000;
    while (monotonic_ns()-stop_begin<int64_t(stop_s_*1e9)) {
      try {
        pump(); const auto now=monotonic_ns();
        if (now>=next_zero) { const auto sent=yaw(0.,"stop"); zero_completed|=sent.success; require(sent.success,"HARD_ABORT: stop zero TX failed"); next_zero=now+5'000'000; }
        service_pitch(now); supervise(now);
      } catch (const std::exception& e) {
        if (failure.empty()) failure=e.what();
        const auto now=monotonic_ns();
        if (now>=next_zero) { try { zero_completed|=yaw(0.,"stop").success; } catch (...) {} next_zero=now+5'000'000; }
      }
    }
    return finish(complete && failure.empty() && zero_completed,complete,zero_completed,failure);
  }
 private:
  double positive(const char* key) const { const auto value=config_[key].as<double>(); require(std::isfinite(value) && value>0,"DATA_INVALID: positive runtime boundary required"); return value; }
  double seconds(int64_t time) const { return (time-begin_)*1e-9; }
  // Absolute GM6020 angle (the crosstalk table and friction maps are indexed by it).
  double position() const { return position_offset_+encoder_counts_*(2.*std::numbers::pi/8192.); }
  void record(const std::string& row) { require(journal_->append(row),"DATA_INVALID: capture writer failed"); }
  void record_best(const std::string& row) { if (journal_->healthy()) journal_->append(row); }
  std::pair<bool,int64_t> transmit(unsigned axis,const can::RawFrame& frame) {
    can_frame wire{}; wire.can_id=frame.id|(frame.extended?CAN_EFF_FLAG:0); wire.can_dlc=frame.dlc;
    std::memcpy(wire.data,frame.data,8); const auto before=monotonic_ns(); ssize_t count;
    auto& bus=buses_[axis];
    if (synthetic_) count=sendto(bus.fd.value,&wire,sizeof(wire),MSG_DONTWAIT,reinterpret_cast<sockaddr*>(&bus.peer),sizeof(bus.peer));
    else count=::send(bus.fd.value,&wire,sizeof(wire),MSG_DONTWAIT);
    tx_begin_=before; tx_accepted_=monotonic_ns(); return {count==sizeof(wire),before};
  }
  AppliedCurrent yaw(double requested,const char* phase) {
    require(std::isfinite(requested),"DATA_INVALID: yaw current is not finite");
    const auto limited=std::clamp(requested,-current_bound_,current_bound_);
    const int raw=gm6020::current_raw_uncapped(limited); auto frame=gm6020::current_zero_frame(1);
    const auto encoded=static_cast<uint16_t>(static_cast<int16_t>(raw)); frame.data[0]=uint8_t(encoded>>8); frame.data[1]=uint8_t(encoded);
    const auto [success,_]=transmit(0,frame); const double actual=raw*gm6020::kAmpsPerRaw;
    std::ostringstream out; out.precision(17);
    out<<"{\"kind\":\"yaw_current_tx\",\"phase\":"<<quoted(phase)<<",\"requested_A\":"<<requested
       <<",\"limited_A\":"<<limited<<",\"successful_tx_A\":";
    if (success) out<<actual; else out<<"null";
    out<<",\"current_raw\":"<<raw<<",\"id\":510,\"unused_slots_zero\":true,\"begin_ns\":"<<tx_begin_
       <<",\"kernel_accepted_ns\":"<<tx_accepted_<<",\"success\":"<<(success?"true":"false")<<'}'; record(out.str());
    return {success,actual,tx_accepted_};
  }
  int64_t pitch(const cybergear::CanFrame& source,const char* operation) {
    const auto id=cybergear::unpack_ext_id(source.id);
    require(id.target==127 && id.data2==0 && (id.comm_type==0 || id.comm_type==4 || id.comm_type==17),
            "HARD_ABORT: pitch command must be discovery, STOP or read");
    can::RawFrame frame; frame.id=source.id; frame.dlc=source.dlc; std::memcpy(frame.data,source.data,8);
    const auto [success,before]=transmit(1,frame);
    record("{\"kind\":\"pitch_tx\",\"operation\":"+quoted(operation)+",\"id\":"+std::to_string(source.id)+
      ",\"begin_ns\":"+std::to_string(before)+",\"kernel_accepted_ns\":"+std::to_string(tx_accepted_)+",\"success\":"+(success?"true":"false")+"}");
    require(success,"HARD_ABORT: pitch STOP/read TX failed"); return before;
  }
  void control_servo(int64_t now) {
    const double since=(now-control_begin_)*1e-9;
    while (next_gain_<gain_schedule_.size() && since>=gain_schedule_[next_gain_][0]) {
      const auto& g=gain_schedule_[next_gain_++];
      require(servo_.set_gains(g[1],g[2],g[3]),"DATA_INVALID: servo gain schedule entry");
      record("{\"kind\":\"servo_gains\",\"time_ns\":"+std::to_string(now)+",\"kq\":"+std::to_string(g[1])+
             ",\"kv\":"+std::to_string(g[2])+",\"ki\":"+std::to_string(g[3])+"}");
    }
    const auto r=reference_.at(since);
    // References are relative to the position where control began.
    const auto out=servo_.step(seconds(now),initial_position_+r.position,r.velocity,r.acceleration);
    std::ostringstream log; log.precision(9);
    require(motor_temperature_<motor_temperature_limit_,"HARD_ABORT: yaw motor temperature limit");
    log<<"{\"kind\":\"servo_cycle\",\"time_ns\":"<<now<<",\"qr\":"<<initial_position_+r.position<<",\"vr\":"<<r.velocity<<",\"ar\":"<<r.acceleration
       <<",\"q\":"<<out.position<<",\"v\":"<<out.velocity<<",\"gyro\":"<<gyro_rate_<<",\"req\":"<<out.requested<<",\"u\":"<<out.limited
       <<",\"ff\":"<<out.feedforward<<",\"fr\":"<<out.friction<<",\"p\":"<<out.proportional<<",\"d\":"<<out.derivative
       <<",\"i\":"<<out.integral<<",\"rms\":"<<out.rms<<",\"cap\":"<<out.cap<<",\"sat\":"<<out.saturated<<",\"rock\":"<<out.rocking
       <<",\"stalls\":"<<out.stall_events<<",\"stale\":"<<servo_.stale_samples()<<",\"status\":"<<out.status<<'}'; record(log.str());
    require(out.status!=int(axis::ServoStatus::FollowingError),"HARD_ABORT: servo following error limit");
    require(out.status==int(axis::ServoStatus::Ok),"MEASUREMENT_LIMITED: servo sensor data stale or invalid");
    // Physical speed check: encoder displacement over >=20 ms and the independent
    // gyro. The observer's instantaneous estimate overshoots for a few ms after a
    // breakaway and is not, by itself, evidence of real overspeed.
    speed_history_.push_back({now,position()});
    while (speed_history_.size()>2 && now-speed_history_[1].first>=20'000'000) speed_history_.pop_front();
    const auto& [then,where]=speed_history_.front();
    const double encoder_speed=now-then>=20'000'000?(position()-where)/((now-then)*1e-9):0.;
    if (std::abs(encoder_speed)>speed_limit_ || std::abs(gyro_rate_)>speed_limit_)
      throw std::runtime_error("HARD_ABORT: yaw speed limit (encoder "+std::to_string(encoder_speed)+
                               " rad/s, gyro "+std::to_string(gyro_rate_)+" rad/s)");
    if (out.rocking) last_rock_=now;
    const bool rock_settling=last_rock_ && now-last_rock_<200'000'000;
    if (oscillation_.update(last_step_?(now-last_step_)*1e-9:0.,out.limited,rock_settling))
      throw std::runtime_error("HARD_ABORT: servo oscillation (fast current RMS "+std::to_string(oscillation_.rms())+" A)");
    last_step_=now;
    const double excitation=excitation_.at(since);
    const auto sent=yaw(std::clamp(out.limited+excitation,-current_bound_,current_bound_),excitation!=0.?"excitation":"control");
    servo_.acknowledge(sent.success,sent.actual);
    require(sent.success,"HARD_ABORT: yaw control TX failed"); ++control_cycles_;
  }
  void service_pitch(int64_t now) {
    if (!identified_) return;
    if (now>=next_stop_) { if (!pitch_stop_begin_) pitch_stop_begin_=now; pitch(cybergear::make_stop(0,127),"stop_poll"); next_stop_=now+limits_.stop_period; }
    require(disabled_ || now-pitch_stop_begin_<limits_.read_timeout,"HARD_ABORT: pitch STOP feedback timeout");
    if (disabled_ && !readback_.pending() && now>=next_read_) {
      const auto reg=have_mode_?((read_index_++%2)?cybergear::Reg::Iqf:cybergear::Reg::MechPos):cybergear::Reg::RunMode;
      const auto raw=readback_.begin(reg,now,limits_.read_timeout); cybergear::CanFrame frame; frame.id=raw.id; std::memcpy(frame.data,raw.data,8);
      pitch(frame,"register_read"); readback_.accepted(monotonic_ns(),true); next_read_=now+limits_.read_period;
    }
  }
  void check_streams(int64_t now) {
    for (const auto& bus:buses_) require(bus.last_feedback && now-bus.last_feedback<=limits_.can_gap,"HARD_ABORT: CAN feedback absent/stale");
    require(disabled_,"HARD_ABORT: pitch is not disabled"); require(imu_.ready() && gyro_sequence_,"MEASUREMENT_LIMITED: IMU streams absent");
    require(have_posture_,"MEASUREMENT_LIMITED: actual pitch posture readback absent");
    for (const auto& [_,sensor]:imu_.sensors) require(now-sensor.received<=limits_.imu_gap,"MEASUREMENT_LIMITED: IMU stream stale");
  }
  void supervise(int64_t now) {
    for (const auto& bus:buses_) if (bus.last_feedback) require(now-bus.last_feedback<=limits_.can_gap,"HARD_ABORT: CAN feedback lost");
    for (const auto& [_,sensor]:imu_.sensors) if (sensor.count) require(now-sensor.received<=limits_.imu_gap,"MEASUREMENT_LIMITED: IMU stream stale");
    if (control_begin_) require(disabled_,"HARD_ABORT: pitch became enabled during yaw control");
  }
  void imu_line(const std::string& raw) {
    imu_.line(raw,limits_,*journal_); const auto sample=YAML::Load(raw);
    if (sample["kind"].as<std::string>()!="sample" || sample["sensor"].as<std::string>()!="gyro") return;
    const int generation=sample["generation"].as<int>();
    require(!control_begin_ || generation==gyro_generation_,"HARD_ABORT: gyro generation changed during control");
    gyro_generation_=generation; gyro_time_=sample["sample_ns"].as<int64_t>();
    const auto values=sample["values"].as<std::vector<double>>(); gyro_rate_=0.;
    for (unsigned i=0;i<3;++i) gyro_rate_+=(values[i]-bias_[i])*projection_[i];
    ++gyro_sequence_;
    if (control_begin_) servo_.observe_gyro(seconds(gyro_time_),gyro_rate_);
  }
  void pump() {
    std::array<pollfd,3> fds{{{buses_[0].fd.value,POLLIN,0},{buses_[1].fd.value,POLLIN,0},{imu_fd_,POLLIN,0}}};
    const auto result=poll(fds.data(),fds.size(),1); if (result<0 && errno==EINTR) return;
    require(result>=0,"DATA_INVALID: acquisition poll failed");
    for (unsigned axis=0;axis<2;++axis) {
      auto& bus=buses_[axis]; require(!(fds[axis].revents&(POLLERR|POLLHUP|POLLNVAL)),"HARD_ABORT: CAN endpoint failed"); Receipt receipt;
      for (unsigned drained=0;drained<256 && bus.receiver->receive(receipt);++drained) {
        ++bus.sequence;
        // The decoded yaw_feedback row already carries the yaw receipt's kernel timestamps.
        if (axis) record(receipt_json(receipt,"pitch",bus.sequence));
        require(!receipt.frame.error,"HARD_ABORT: CAN error frame"); require(!receipt.drop_delta,"DATA_INVALID: socket receive loss");
        if (!axis) {
          gm6020::Feedback value;
          if (gm6020::decode(receipt.frame,1,value)) {
            if (have_encoder_) { int delta=int(value.angle_count)-int(previous_encoder_); if (delta>4096) delta-=8192; if (delta<-4096) delta+=8192; encoder_counts_+=delta; }
            else position_offset_=value.angle_count*(2.*std::numbers::pi/8192.);
            have_encoder_=true; previous_encoder_=value.angle_count; motor_temperature_=value.temperature_raw; encoder_time_=receipt.kernel_monotonic_ns; bus.last_feedback=encoder_time_;
            std::ostringstream out; out.precision(17);
            out<<"{\"kind\":\"yaw_feedback\",\"encoder_raw\":"<<value.angle_count<<",\"encoder_unwrapped_counts\":"<<encoder_counts_
               <<",\"q_relative_rad\":"<<position()<<",\"speed_rpm\":"<<value.speed_rpm<<",\"current_raw\":"<<value.current_raw
               <<",\"current_A\":"<<value.current_a()<<",\"temperature_raw\":"<<unsigned(value.temperature_raw)
               <<",\"kernel_realtime_ns\":"<<receipt.kernel_realtime_ns<<",\"kernel_monotonic_ns\":"<<encoder_time_
               <<",\"dequeue_ns\":"<<receipt.dequeue_ns<<'}'; record(out.str());
            if (control_begin_ && !servo_done_) {
              if (!servo_.observe_encoder(seconds(encoder_time_),position()))
                throw std::runtime_error("MEASUREMENT_LIMITED: servo encoder update rejected (ready="+
                  std::to_string(servo_.ready())+", t="+std::to_string(seconds(encoder_time_))+")");
              control_servo(monotonic_ns());
            }
          }
          continue;
        }
        if (!receipt.frame.extended || receipt.frame.rtr || receipt.frame.dlc!=8) continue;
        const auto id=cybergear::unpack_ext_id(receipt.frame.id); cybergear::CanFrame frame; frame.id=receipt.frame.id; std::memcpy(frame.data,receipt.frame.data,8);
        if (id.comm_type==0) {
          cybergear::DiscoveryResponse response;
          require(cybergear::parse_discovery_response(frame,response) && id.data2==127 && response.unique_id==uid_,"INTEGRATION_MISMATCH: pitch identity"); identified_=true;
          record("{\"kind\":\"pitch_identity\",\"uid_hex\":"+quoted(uid_text_)+",\"receive_ns\":"+std::to_string(receipt.kernel_monotonic_ns)+"}");
        } else if (id.comm_type==17) {
          const auto observation=readback_.observe(receipt); record(readback_json(observation));
          if (observation.reg==cybergear::Reg::RunMode) { require(observation.value.has_value(),"MEASUREMENT_LIMITED: pitch mode readback unavailable"); have_mode_=true; }
          if (observation.reg==cybergear::Reg::MechPos) {
            require(observation.value && std::isfinite(*observation.value),"MEASUREMENT_LIMITED: pitch posture readback unavailable");
            have_posture_=true;
          }
        } else if (id.comm_type==2) {
          cybergear::Feedback feedback;
          require(cybergear::parse_feedback(frame,feedback) && feedback.motor_id==127 && feedback.host_id==0,"INTEGRATION_MISMATCH: pitch feedback identity");
          require(!feedback.faults && feedback.temp_c<pitch_temperature_,"HARD_ABORT: pitch fault or temperature");
          const bool disabled=feedback.mode==cybergear::MotorMode::Reset; require(!disabled_ || disabled,"HARD_ABORT: pitch became enabled");
          if (pitch_stop_begin_ && receipt.kernel_monotonic_ns>=pitch_stop_begin_) disabled_=disabled;
          bus.last_feedback=receipt.kernel_monotonic_ns;
        } else if (id.comm_type==21) require(false,"HARD_ABORT: pitch fault response");
      }
    }
    require(!(fds[2].revents&(POLLERR|POLLNVAL)),"DATA_INVALID: IMU pipe failed");
    if (fds[2].revents&(POLLIN|POLLHUP)) {
      char bytes[8192]; const auto count=::read(imu_fd_,bytes,sizeof(bytes));
      if (count<0 && (errno==EAGAIN || errno==EINTR)) return;
      require(count>0,"DATA_INVALID: IMU producer EOF or read failure"); imu_pending_.append(bytes,count);
      size_t end; while ((end=imu_pending_.find('\n'))!=std::string::npos) { imu_line(imu_pending_.substr(0,end)); imu_pending_.erase(0,end+1); }
      require(imu_pending_.size()<3000,"DATA_INVALID: IMU line exceeded capture bound");
    }
  }
  int finish(bool complete,bool sequence,bool zero,std::string detail) {
    try {
      for (const auto& bus:buses_) {
        require(!bus.receiver->kernel_drops(),"DATA_INVALID: final socket receive loss");
        if (!synthetic_) require(bus.counter("rx_dropped")==bus.drops_begin && bus.counter("rx_errors")==bus.errors_begin,"DATA_INVALID: interface receive loss");
      }
    } catch (const std::exception& e) { complete=false; if (detail.empty()) detail=e.what(); }
    const auto footer="{\"kind\":\"footer\",\"status\":"+quoted(complete?"COMPLETE":"INVALID")+",\"detail\":"+quoted(detail)+
      ",\"sequence_complete\":"+(sequence?"true":"false")+",\"zero_request_completed\":"+(zero?"true":"false")+
      ",\"stop_observation_s\":2,\"motion_qualified\":false,\"parameter_qualified\":false,\"shared_core_control_cycles\":"+
      std::to_string(control_cycles_)+",\"yaw_frames\":"+std::to_string(buses_[0].sequence)+",\"pitch_frames\":"+std::to_string(buses_[1].sequence)+
      ",\"end_ns\":"+std::to_string(monotonic_ns())+"}";
    const bool written=journal_->finish(footer); std::cout<<footer<<'\n'; return complete && written?0:1;
  }
  YAML::Node config_; Limits limits_; bool synthetic_; Readback readback_;
  axis::ServoParameters servo_parameters_{}; axis::Servo servo_; bool servo_done_{};
  double speed_limit_{},hold_after_s_{},motor_temperature_{},motor_temperature_limit_{};
  std::vector<std::array<double,4>> gain_schedule_; std::size_t next_gain_{};
  std::deque<std::pair<int64_t,double>> speed_history_;
  axis::LogSweep excitation_; axis::ReferenceTable reference_; axis::OscillationMonitor oscillation_; int64_t last_step_{},last_rock_{};
  std::array<Endpoint,2> buses_; ImuStream imu_; std::unique_ptr<Journal> journal_;
  std::array<double,3> projection_{},bias_{};
  std::string candidate_,uid_text_,imu_pending_;
  uint64_t uid_{},gyro_sequence_{},read_index_{},control_cycles_{};
  int imu_fd_{},gyro_generation_{-1}; uint16_t previous_encoder_{}; int64_t encoder_counts_{};
  bool identified_{},disabled_{},have_mode_{},have_encoder_{},have_posture_{};
  double baseline_s_{},stop_s_{},current_bound_{},pitch_temperature_{},position_offset_{},initial_position_{},gyro_rate_{};
  int64_t begin_{},discovery_begin_{},pitch_stop_begin_{},control_begin_{},next_baseline_{},next_stop_{},next_read_{},encoder_time_{},gyro_time_{},tx_begin_{},tx_accepted_{};
};
}
int yaw_control_session(const char* path) {
  try { return YawControlSession(YAML::LoadFile(path)).run(); }
  catch (const std::exception& e) { std::cerr<<"{\"status\":\"INVALID\",\"detail\":"<<quoted(e.what())<<"}\n"; return 1; }
}
}
