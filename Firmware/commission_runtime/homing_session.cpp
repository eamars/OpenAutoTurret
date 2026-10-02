#include "session.hpp"
#include "io.hpp"
#include "calibration/full_axis_homing.hpp"
#include "can/pitch_current_policy.hpp"
#include "position_loop.hpp"
#include "session_parts.hpp"
#include <algorithm>
#include <deque>
#include <vector>

namespace ota::commission {
using namespace detail;
namespace {
can::RawFrame raw(const cybergear::CanFrame& source) {
  can::RawFrame frame; frame.id=source.id; frame.dlc=source.dlc;
  std::memcpy(frame.data,source.data,8); return frame;
}
double number(const YAML::Node& node,const char* key,bool zero=false) {
  const auto value=node[key].as<double>();
  require(std::isfinite(value) && (zero?value>=0:value>0),"DATA_INVALID: explicit finite homing parameter required");
  return value;
}
FullAxisHomingParams parameters(const YAML::Node& config) {
  const auto n=config["homing"], c=n["contact"];
  FullAxisHomingParams full; auto& p=full.homing;
  p.observe_only=true;
  p.motion_checks_abort=n["motion_checks_abort"].as<bool>();
  require(p.motion_checks_abort,"HARD_ABORT: sensorless homing requires aborting motion checks");
#define PARAM(field) p.field=number(n,#field)
  PARAM(coarse_speed_rad_s); PARAM(fine_speed_rad_s); PARAM(backoff_speed_rad_s);
  PARAM(backoff_rad); PARAM(small_backoff_rad); PARAM(repeatability_rad);
  PARAM(settle_time_s); PARAM(approach_timeout_s); PARAM(max_travel_rad);
  PARAM(arrival_tol_rad); PARAM(backoff_timeout_s); PARAM(backoff_arrival_tol_rad);
  PARAM(backoff_arrive_vel_rad_s); PARAM(limit_cur_initial_a); PARAM(limit_cur_max_a);
  PARAM(max_rotation_rad); PARAM(torque_safety_nm);
#undef PARAM
  p.limit_cur_step_a=number(n,"limit_cur_step_a",true);
  p.repeatability_retries=n["repeatability_retries"].as<int>();
  p.rearm_before_start=n["rearm_before_start"].as<bool>();
  require(p.repeatability_retries>=0 && p.repeatability_retries<=2 && p.rearm_before_start,
          "DATA_INVALID: explicit bounded homing rearm/repeatability contract");
#define CONTACT(field) p.contact.field=number(c,#field)
  CONTACT(v_stall_threshold_rad_s); CONTACT(q_stall_threshold_rad); CONTACT(progress_window_s);
  CONTACT(effort_contact_threshold_nm); CONTACT(effort_hard_contact_nm);
  CONTACT(motion_history_vel_rad_s); CONTACT(effort_hard_abort_nm);
  CONTACT(v_move_threshold_rad_s); CONTACT(a_peak_rad_s2);
#undef CONTACT
  p.contact.contact_dwell_ms=c["contact_dwell_ms"].as<int>();
  p.contact.min_command_active_ms=c["min_command_active_ms"].as<int>();
  p.contact.jitter_window_ms=c["jitter_window_ms"].as<int>();
  require(p.contact.contact_dwell_ms>0 && p.contact.min_command_active_ms>0 &&
          p.contact.jitter_window_ms>0 && p.contact.jitter_window_ms<=640,
          "DATA_INVALID: homing contact timing");
  full.dir_endpoint_a=n["dir_endpoint_a"].as<int>(); full.dir_endpoint_b=n["dir_endpoint_b"].as<int>();
  require(std::abs(full.dir_endpoint_a)==1 && full.dir_endpoint_b==-full.dir_endpoint_a,
          "DATA_INVALID: opposing pitch endpoint directions required");
  require(can::valid_pitch_current_limit(p.limit_cur_initial_a) &&
          p.limit_cur_initial_a==p.limit_cur_max_a && p.limit_cur_step_a==0,
          "HARD_ABORT: fixed existing pitch homing current cap required");
  return full;
}
std::string scalar(double value) { std::ostringstream out; out.precision(17); out<<value; return out.str(); }

// This executor uses the existing sensorless FSM and the existing receipt/readback
// machinery. It is the session's only CAN sender. No encoder-zero command or
// retained-calibration writer exists in this path.
class HomingSession {
 public:
  explicit HomingSession(const YAML::Node& config,bool validate_only=false)
      : config_(config), limits_(config["limits"]), params_(parameters(config)),
        homing_(AxisId::Pitch,params_), synthetic_(config["provenance"].as<std::string>()=="SYNTHETIC"),readback_(127,0) {
    require(config["schema"].as<std::string>()=="adr0022.sensorless-homing/1" &&
            config["purpose"].as<std::string>()=="pitch_sensorless_homing","INTEGRATION_MISMATCH: homing schema/purpose");
    require(synthetic_ || config["provenance"].as<std::string>()=="MEASURED","DATA_INVALID: provenance");
    require(config["transport"].as<std::string>()==(synthetic_?"loopback_udp":"socketcan"),"INTEGRATION_MISMATCH: homing transport");
    require(config["pitch_supported_when_disabled"].as<bool>(),"HARD_ABORT: pitch support required");
    uid_=config["expected_pitch_uid"].as<std::string>();
    require(uid_.size()==16 && uid_.find_first_not_of("0123456789abcdef")==std::string::npos,"DATA_INVALID: pitch UID");
    expected_uid_=std::stoull(uid_,nullptr,16);
    const auto n=config["native_settings"],g=config["guards"];
    expected_mode_=n["expected_original_mode"].as<int>();
    require(expected_mode_>=1 && expected_mode_<=3,"DATA_INVALID: expected original mode");
    original_={{cybergear::Reg::RunMode,double(expected_mode_)},
      {cybergear::Reg::LimitCur,number(n,"original_limit_cur_A")},
      {cybergear::Reg::SpdKp,number(n,"original_speed_kp",true)},
      {cybergear::Reg::SpdKi,number(n,"original_speed_ki",true)},
      {cybergear::Reg::LocKp,number(n,"original_position_kp",true)}};
    desired_={{cybergear::Reg::LimitCur,number(n,"homing_limit_cur_A")},
      {cybergear::Reg::SpdKp,number(n,"homing_speed_kp")},
      {cybergear::Reg::SpdKi,number(n,"homing_speed_ki")},
      {cybergear::Reg::LocKp,number(n,"homing_position_kp")}};
    require(desired_[0].second==params_.homing.limit_cur_initial_a && can::valid_pitch_current_limit(desired_[0].second),
            "HARD_ABORT: homing current cap differs from explicit native setting");
    current_bound_=number(g,"current_bound_A"); torque_bound_=number(g,"torque_bound_Nm");
    temp_bound_=number(g,"pitch_temperature_C"); yaw_bound_=number(g,"yaw_displacement_rad");
    travel_bound_=number(g,"total_displacement_rad"); speed_bound_=number(g,"maximum_encoder_speed_rad_s");
    transition_bound_=number(g,"mode_transition_displacement_rad");
    midpoint_speed_=number(g,"midpoint_speed_rad_s");
    midpoint_dwell_=int64_t(number(g,"midpoint_dwell_s")*1e9);
    midpoint_timeout_=int64_t(number(g,"midpoint_timeout_s")*1e9);
    // Measured Iqf protection is a rating boundary, independent of the
    // established 5 A homing command/LimitCur ceiling checked above.
    require(current_bound_<=6.5,"DATA_INVALID: homing current protection exceeds manufacturer continuous rating");
    if (!synthetic_) {
      const auto basis=config["protection_limit_basis"];
      require(current_bound_==6.5 && basis && basis.IsMap() &&
          basis["kind"].as<std::string>("")=="manufacturer_continuous_current_rating" &&
          basis["document"].as<std::string>("")=="docs/references/cybergear/CyberGear微电机使用说明书.pdf" &&
          basis["continuous_current_A"].as<double>(0)==6.5,
          "DATA_INVALID: measured homing requires manufacturer continuous-current protection basis");
    }
    require(torque_bound_<=params_.homing.torque_safety_nm &&
            torque_bound_<=params_.homing.contact.effort_hard_abort_nm &&
            speed_bound_>std::max({params_.homing.coarse_speed_rad_s,params_.homing.fine_speed_rad_s,
                                 params_.homing.backoff_speed_rad_s,midpoint_speed_}),"DATA_INVALID: incompatible homing guards");
    if (!synthetic_) require(config["yaw"]["interface"].as<std::string>()=="can0" &&
        config["pitch"]["interface"].as<std::string>()=="can1","INTEGRATION_MISMATCH: station topology");
    if (const auto t=config["servo_trial"]) {
      // Pitch speed-mode servo trial: the drive keeps its own velocity loop; the host
      // streams SpdRef = v_ref + kp*(q_ref-q) + integral (axis::PositionLoop) at a fixed period.
      trial_=true;
      axis::PositionLoopParameters loop{number(t,"kp_per_s"),number(t,"ki_per_s2",true),number(t,"integral_clamp_rad_s",true),
                                         number(t,"speed_limit_rad_s")};
      require(loop.speed_limit<=1.0 && loop_.configure(loop),"DATA_INVALID: pitch servo trial loop");
      trial_period_=int64_t(number(t,"command_period_s")*1e9); trial_hold_=int64_t(number(t,"hold_after_s",true)*1e9);
      trial_follow_=number(t,"following_error_rad"); trial_approach_speed_=number(t,"approach_speed_rad_s");
      require(trial_period_>=500'000 && trial_period_<=20'000'000 && trial_approach_speed_<=0.2,"DATA_INVALID: pitch servo trial timing");
      home_first_=t["home_first"].as<bool>(false);
      if (home_first_) {
        // The working window comes from this session's own sensorless homing:
        // the measured endpoints inset by window_margin_rad, centred between them.
        trial_margin_=number(t,"window_margin_rad");
      } else {
        // Without homing the absolute window comes from the manifest (production's homed
        // endstops, inset). The pitch may rest anywhere up to its endstops, so the approach
        // may start up to window_margin_rad (+0.02) outside the window.
        trial_min_=t["window_min_rad"].as<double>(); trial_max_=t["window_max_rad"].as<double>();
        trial_center_=t["center_rad"].as<double>(); trial_slack_=number(t,"window_margin_rad")+0.02;
        require(std::isfinite(trial_min_) && std::isfinite(trial_max_) && trial_min_<trial_max_ &&
                trial_center_>trial_min_ && trial_center_<trial_max_,"DATA_INVALID: pitch servo trial bounds");
      }
      if (const auto x=t["excitation"]) {
        // Identification only: a log-swept sine added to SpdRef.
        excitation_={x["amplitude_rad_s"].as<double>(),x["f0_hz"].as<double>(),x["f1_hz"].as<double>(),
                     x["begin_s"].as<double>(),x["duration_s"].as<double>()};
        require(excitation_.valid(.2),"DATA_INVALID: pitch servo trial excitation");
      }
      if (const auto schedule=t["gain_schedule"]) {
        for (const auto& g:schedule) trial_schedule_.push_back({g["begin_s"].as<double>(),g["kp"].as<double>(),g["ki"].as<double>()});
        require(!trial_schedule_.empty() && trial_schedule_.size()<=64,"DATA_INVALID: pitch servo trial gain schedule");
        for (const auto& g:trial_schedule_) require(std::isfinite(g[1]) && g[1]>0 && std::isfinite(g[2]) && g[2]>=0,
                                                    "DATA_INVALID: pitch servo trial gain schedule entry");
      }
      try {
        for (const auto& item:t["reference_samples"])
          trial_ref_.add({item["time_s"].as<double>(),item["position_rad"].as<double>(),
                          item["velocity_rad_s"].as<double>(),item["acceleration_rad_s2"].as<double>()});
      } catch (const std::runtime_error&) { require(false,"DATA_INVALID: pitch trial reference"); }
      require(trial_ref_.size()>=2,"DATA_INVALID: pitch trial reference table");
      trial_active_=!home_first_;
    }
    if (validate_only) return; // Same numerical contract, before any device or output I/O.
    imu_fd_=config["imu_fd"].as<int>(); struct stat info{};
    require(imu_fd_>2 && imu_fd_!=8 && fstat(imu_fd_,&info)==0 && S_ISFIFO(info.st_mode),"DATA_INVALID: inherited IMU pipe required");
    require(fcntl(imu_fd_,F_SETFL,fcntl(imu_fd_,F_GETFL)|O_NONBLOCK)==0,"DATA_INVALID: IMU nonblocking pipe");
    if (!synthetic_) {
      check_launcher_lease();
      require(config["yaw"]["interface"].as<std::string>()=="can0" && config["pitch"]["interface"].as<std::string>()=="can1",
              "INTEGRATION_MISMATCH: station topology");
    }
    journal_=std::make_unique<Journal>(config["output"].as<std::string>(),
      "{\"kind\":\"header\",\"schema\":\"adr0022.sensorless-homing/1\",\"provenance\":"+quoted(synthetic_?"SYNTHETIC":"MEASURED")+
      ",\"purpose\":\"pitch_sensorless_homing\",\"parameter_qualified\":false,\"manifest_yaml\":"+quoted(json(config))+"}");
    buses_[0].open(config["yaw"],synthetic_,limits_); buses_[1].open(config["pitch"],synthetic_,limits_);
  }
  int run() {
    interrupted=0; std::signal(SIGINT,on_signal); std::signal(SIGTERM,on_signal); begin_=monotonic_ns(); entered_=begin_;
    std::cout<<"{\"kind\":\"capture_ready\",\"excitation\":true,\"purpose\":\"pitch_sensorless_homing\"}\n"<<std::flush;
    try {
      record("{\"kind\":\"session_begin\",\"time_ns\":"+std::to_string(begin_)+",\"startup_discarded\":["+
             std::to_string(buses_[0].receiver->startup_discarded())+","+std::to_string(buses_[1].receiver->startup_discarded())+"]}");
      while (state_!=State::Done) {
        pump(); const auto now=monotonic_ns();
        require(!interrupted,"HARD_ABORT: homing interrupted");
        require(now-begin_<limits_.duration,"HARD_ABORT: homing overall deadline"); readback_.check_deadline(now);
        if (now-begin_>limits_.startup) check_streams(now);
        if (transition_ && have_pose_) require(std::abs(pose_-transition_origin_)<=transition_bound_,"HARD_ABORT: mode transition displacement");
        step(now); require(journal_->healthy(),"DATA_INVALID: capture writer failed");
      }
      return finish(true,"");
    } catch (const std::exception& e) { const std::string reason=e.what(); shutdown(); return finish(false,reason); }
  }
 private:
  enum class State { Discover, InitialStop, Observe, Snapshot, Run, HoldPose, HoldVerify, TransitionStop, TransitionPose, TransitionWrite,
                     TransitionVerify, Enable, EnabledVerify, EnabledPose, EnabledPinVerify, FinalStop,
                     MidpointVerify,
                     RestoreWrite, RestoreVerify, Done };
  using Settings=std::vector<std::pair<cybergear::Reg,double>>;
  void record(const std::string& row) { require(journal_->append(row),"DATA_INVALID: capture writer failed"); }
  void state(State next,int64_t now) {
    state_=next; entered_=now; pending_begin_=0; verification_.clear(); verify_index_=0;
    record("{\"kind\":\"homing_executor_state\",\"state\":"+std::to_string(int(next))+",\"time_ns\":"+std::to_string(now)+"}");
  }
  void send(unsigned axis,const can::RawFrame& frame,const char* operation) {
    if (!axis) {
      require(!frame.extended && frame.id==0x1fe && frame.dlc==8,"HARD_ABORT: invalid yaw neutral frame");
      for (const auto b:frame.data) require(!b,"HARD_ABORT: nonzero yaw homing command");
    } else {
      const auto id=cybergear::unpack_ext_id(frame.id);
      require(frame.extended && !frame.error && !frame.rtr && frame.dlc==8 && id.target==127 && id.data2==0,
              "HARD_ABORT: invalid pitch command identity");
      require(id.comm_type==0 || id.comm_type==3 || id.comm_type==4 || id.comm_type==17 || id.comm_type==18,
              "HARD_ABORT: pitch command outside homing contract");
      if (id.comm_type==18) {
        const auto reg=cybergear::Reg(uint16_t(frame.data[0])|(uint16_t(frame.data[1])<<8));
        require(reg==cybergear::Reg::RunMode || reg==cybergear::Reg::IqRef || reg==cybergear::Reg::SpdRef ||
                reg==cybergear::Reg::LocRef || reg==cybergear::Reg::LimitSpd || reg==cybergear::Reg::LimitCur ||
                reg==cybergear::Reg::SpdKp || reg==cybergear::Reg::SpdKi || reg==cybergear::Reg::LocKp,
                "HARD_ABORT: unrelated pitch register write");
        if (reg==cybergear::Reg::IqRef) { float value; std::memcpy(&value,frame.data+4,4); require(value==0,"HARD_ABORT: nonzero Iq command"); }
        if (reg==cybergear::Reg::RunMode || reg==cybergear::Reg::LimitCur || reg==cybergear::Reg::SpdKp ||
            reg==cybergear::Reg::SpdKi || reg==cybergear::Reg::LocKp) {
          require(disabled_ && status_ns_ && monotonic_ns()-status_ns_<=limits_.can_gap,
                  "HARD_ABORT: mode/gain/current-limit write without fresh disabled feedback");
          require(!buses_[0].receiver->kernel_drops() && !buses_[1].receiver->kernel_drops(),"DATA_INVALID: receive loss before transition write");
        }
      } else if (id.comm_type!=17) for (const auto b:frame.data) require(!b,"HARD_ABORT: homing simple payload must be zero");
    }
    auto& bus=buses_[axis]; can_frame wire{}; wire.can_id=frame.id|(frame.extended?CAN_EFF_FLAG:0); wire.can_dlc=frame.dlc;
    std::memcpy(wire.data,frame.data,8); const auto before=monotonic_ns();
    const bool register_write=axis && cybergear::unpack_ext_id(frame.id).comm_type==18;
    if (register_write) {
      // Keep older accepted writes while their receipts may still be queued.
      // Matching below uses receipt time and the narrower read-timeout window.
      while (!writes_.empty() && before-writes_.front().begin_ns>=limits_.read_timeout+limits_.dequeue) writes_.pop_front();
      require(writes_.size()<4096,"DATA_INVALID: bounded homing write history exhausted");  // 1 kHz servo trial writes
    }
    const auto count=synthetic_?sendto(bus.fd.value,&wire,sizeof(wire),MSG_DONTWAIT,reinterpret_cast<sockaddr*>(&bus.peer),sizeof(bus.peer)):
                               ::send(bus.fd.value,&wire,sizeof(wire),MSG_DONTWAIT);
    require(count==sizeof(wire),"HARD_ABORT: homing TX failed"); std::string bytes;
    for (const auto b:frame.data) { const char* digits="0123456789abcdef"; bytes+=digits[b>>4]; bytes+=digits[b&15]; }
    if (register_write) writes_.push_back({unsigned(uint16_t(frame.data[0])|(uint16_t(frame.data[1])<<8)),bytes,before});
    record("{\"kind\":\"homing_tx\",\"axis\":"+quoted(axis?"pitch":"yaw")+",\"operation\":"+quoted(operation)+
      ",\"begin_ns\":"+std::to_string(before)+",\"kernel_accepted_ns\":"+std::to_string(monotonic_ns())+
      ",\"id\":"+std::to_string(frame.id)+",\"data_hex\":"+quoted(bytes)+",\"success\":true}");
  }
  void pitch(const cybergear::CanFrame& frame,const char* operation) { send(1,raw(frame),operation); }
  void write(cybergear::Reg reg,double value,const char* operation) {
    if (reg==cybergear::Reg::RunMode) pitch(cybergear::make_write_reg_u8(reg,uint8_t(value),0,127),operation);
    else pitch(cybergear::make_write_reg_float(reg,float(value),0,127),operation);
    if (reg==cybergear::Reg::LimitSpd) position_limit_=value;
  }
  void read(cybergear::Reg reg,int64_t now) {
    send(1,readback_.begin(reg,now,limits_.read_timeout),"register_read"); readback_.accepted(monotonic_ns(),true);
  }
  bool verify(int64_t now) {
    if (verify_index_==verification_.size()) return true;
    const auto [reg,expected]=verification_[verify_index_];
    if (!pending_begin_) { read(reg,now); pending_begin_=now; return false; }
    if (readback_.pending()) return false;
    require(latest_read_ && latest_read_->reg==reg && latest_read_->value,
            "INTEGRATION_MISMATCH: homing native register readback missing");
    if (reg==cybergear::Reg::LocRef) {
      record("{\"kind\":\"homing_reference_observation\",\"expected_rad\":"+scalar(expected)+
        ",\"observed_rad\":"+scalar(*latest_read_->value)+",\"observed_minus_expected_rad\":"+scalar(*latest_read_->value-expected)+
        ",\"receive_ns\":"+std::to_string(latest_read_->receive_ns)+",\"reference_qualified\":false}");
    } else {
      const auto encoded=reg==cybergear::Reg::RunMode?double(uint8_t(expected)):double(float(expected));
      require(reg==cybergear::Reg::Iqf?std::abs(*latest_read_->value)<=current_bound_:*latest_read_->value==encoded,
              "INTEGRATION_MISMATCH: homing native register readback differs from encoded setting");
    }
    if (state_==State::Snapshot && reg!=cybergear::Reg::Iqf) {
      observed_.push_back({reg,*latest_read_->value});
      record("{\"kind\":\"homing_original_setting\",\"index\":"+std::to_string(unsigned(reg))+",\"value\":"+scalar(*latest_read_->value)+"}");
    }
    ++verify_index_; pending_begin_=0; return verify_index_==verification_.size();
  }
  Settings guarded(const Settings& settings) const {
    Settings result;
    for (const auto& setting:settings) { result.push_back(setting); result.push_back({cybergear::Reg::Iqf,0}); }
    return result;
  }
  void stop_poll(int64_t now) {
    if (now>=next_ping_) { pitch(cybergear::make_stop(0,127),"stop_poll"); next_ping_=now+limits_.stop_period; }
  }
  void begin_transition(const DesiredState& desired,int mode,int64_t now) {
    require(have_pose_,"HARD_ABORT: transition without measured pose");
    latched_=desired; transition_=true; transition_origin_=pose_; transition_begin_=now; next_mode_=mode;
    state(State::TransitionStop,now);
  }
  bool measured_pin(int64_t now) {
    if (!pending_begin_) { read(cybergear::Reg::MechPos,now); pending_begin_=now; return false; }
    if (readback_.pending()) return false;
    require(latest_read_ && latest_read_->reg==cybergear::Reg::MechPos && latest_read_->value &&
            now-latest_read_->receive_ns<=limits_.read_timeout && now-status_ns_<=limits_.can_gap,
            "DATA_INVALID: fresh native MechPos/pitch status required");
    const auto residual=*latest_read_->value-pose_;
    record("{\"kind\":\"homing_position_observation\",\"native_mechpos_rad\":"+scalar(*latest_read_->value)+
      ",\"type2_pose_rad\":"+scalar(pose_)+",\"register_minus_type2_rad\":"+scalar(residual)+
      ",\"read_receive_ns\":"+std::to_string(latest_read_->receive_ns)+",\"status_receive_ns\":"+std::to_string(status_ns_)+
      ",\"mapping_qualified\":false}");
    pinned_pose_=*latest_read_->value; return true;
  }
  void complete_transition(int64_t now) {
    transition_=false; mode_=next_mode_;
    auto desired=latched_; desired.enter_pos_mode=false; desired.rearm_speed_mode=false;
    state(State::Run,now); holding_=true; command_speed_=0; apply(desired,now);
  }
  void record_endpoint(const HomingController& endpoint,const char* name,bool& recorded) {
    if (recorded || endpoint.state()!=AxisHomeState::Complete) return;
    const auto& observed=endpoint.result(); recorded=true;
    record("{\"kind\":\"homing_endpoint_observation\",\"endpoint\":"+quoted(name)+
      ",\"coarse_contact_rad\":"+scalar(observed.coarse_contact_rad)+
      ",\"fine_contact1_rad\":"+scalar(observed.fine_contact1_rad)+
      ",\"fine_contact2_rad\":"+scalar(observed.fine_contact2_rad)+
      ",\"fine_contact_rad\":"+scalar(observed.fine_contact_rad)+
      ",\"repeatability_rad\":"+scalar(observed.repeatability_rad)+",\"endpoint_qualified\":false}");
  }
  void apply(const DesiredState& desired,int64_t now) {
    require(enabled_ && status_ns_ && now-status_ns_<=limits_.can_gap,"HARD_ABORT: homing command without fresh enabled feedback");
    if (desired.limit_cur_a) require(desired.limit_cur_a==desired_[0].second,"HARD_ABORT: FSM changed fixed homing current cap");
    const int requested=desired.position_move?1:(desired.hold?mode_:2);
    if (desired.enter_pos_mode || desired.rearm_speed_mode || requested!=mode_) { begin_transition(desired,requested,now); return; }
    if (desired.hold) {
      // The FSM's hold target defaults to zero. Pin the real pose when entering
      // a position hold; do not forward that zero or chase receipt noise.
      if (!holding_) {
        if (mode_==1) { state(State::HoldPose,now); return; }
        else write(cybergear::Reg::SpdRef,0,"hold_zero_speed");
      }
      holding_=true; if (mode_==2) command_speed_=0;
    } else if (desired.position_move) {
      if (holding_ || command_target_!=desired.target_rad || command_speed_!=desired.speed_rad_s) {
        write(cybergear::Reg::LimitSpd,desired.speed_rad_s,"position_speed_limit");
        write(cybergear::Reg::LocRef,desired.target_rad,"position_target");
        if (midpoint_) {
          midpoint_command_ns_=now;
          midpoint_travel_ns_=int64_t(std::abs(desired.target_rad-pinned_pose_)/desired.speed_rad_s*1e9);
          state(State::MidpointVerify,now);
          verification_={{cybergear::Reg::LocRef,desired.target_rad},{cybergear::Reg::Iqf,0}};
        }
      }
      holding_=false; command_target_=desired.target_rad; command_speed_=desired.speed_rad_s;
    } else {
      if (holding_ || command_speed_!=desired.velocity_rad_s || now>=next_command_) {
        write(cybergear::Reg::SpdRef,desired.velocity_rad_s,"approach_speed"); next_command_=now+limits_.stop_period;
      }
      holding_=false; command_speed_=desired.velocity_rad_s;
    }
    if (last_message_!=desired.message) {
      last_message_=desired.message;
      record("{\"kind\":\"homing_desired_state\",\"time_ns\":"+std::to_string(now)+",\"message\":"+quoted(desired.message)+
        ",\"target_rad\":"+scalar(desired.target_rad)+",\"speed_rad_s\":"+scalar(desired.speed_rad_s)+
        ",\"velocity_rad_s\":"+scalar(desired.velocity_rad_s)+",\"hold\":"+(desired.hold?"true":"false")+"}");
    }
  }
  void step(int64_t now) {
    if (now>=next_yaw_) { send(0,gm6020::current_zero_frame(1),"current_zero"); next_yaw_=now+limits_.stop_period; }
    if (enabled_ && now>=next_command_) {
      if (state_==State::EnabledVerify || state_==State::EnabledPose || state_==State::EnabledPinVerify) {
        write(next_mode_==1?cybergear::Reg::LimitSpd:cybergear::Reg::SpdRef,0,"enabled_neutral_keepalive");
        next_command_=now+limits_.stop_period;
      } else if (state_==State::Run || state_==State::HoldPose || state_==State::HoldVerify || state_==State::MidpointVerify) {
        // CyberGear status is command-triggered. Repeat an inert speed-limit
        // write in position mode; rewriting LocRef restarts its profile.
        write(mode_==1?cybergear::Reg::LimitSpd:cybergear::Reg::SpdRef,
              mode_==1?position_limit_:command_speed_,"homing_status_keepalive");
        next_command_=now+limits_.stop_period;
      }
    }
    if (state_==State::Discover) {
      if (!pending_begin_) { pending_begin_=now; pitch(cybergear::make_discovery_request(0,127),"discovery"); }
      require(now-pending_begin_<limits_.read_timeout,"MEASUREMENT_LIMITED: pitch discovery timeout");
      if (identified_) state(State::InitialStop,now);
    } else if (state_==State::InitialStop || state_==State::TransitionStop || state_==State::FinalStop) {
      if (!pending_begin_) {
        pending_begin_=now; disabled_=false;
        if (state_!=State::InitialStop) { write(cybergear::Reg::SpdRef,0,"neutral_speed_before_stop"); write(cybergear::Reg::IqRef,0,"neutral_current_before_stop"); }
        pitch(cybergear::make_stop(0,127),"stop");
        next_ping_=now+limits_.stop_period;
      }
      require(now-pending_begin_<limits_.read_timeout,"HARD_ABORT: pitch STOP confirmation timeout");
      stop_poll(now);
      if (disabled_ && status_ns_>=pending_begin_) {
        if (state_==State::InitialStop) state(State::Observe,now);
        else if (state_==State::FinalStop) {
          // 1 kHz trial writes can still have Motor-state replies queued behind
          // the STOP reply; let them drain before the disabled-only restore.
          if (!trial_ || now-status_ns_>=100'000'000 || now-pending_begin_>=limits_.read_timeout/2) {
            normal_stop_confirmed_=true; state(State::RestoreWrite,now);
          }
        }
        else state(next_mode_==1?State::TransitionPose:State::TransitionWrite,now);
      }
    } else if (state_==State::Observe) {
      stop_poll(now);
      if (now-begin_>=limits_.startup && imu_.ready()) {
        check_streams(now); state(State::Snapshot,now); verification_=guarded(original_);
      }
    } else if (state_==State::Snapshot) {
      stop_poll(now);
      if (verify(now)) {
        DesiredState initial; initial.rearm_speed_mode=true; initial.hold=true;
        begin_transition(initial,2,now);
      }
    } else if (state_==State::HoldPose) {
      require(enabled_,"HARD_ABORT: enabled homing mode was lost");
      require(current_ns_ && now-current_ns_<=limits_.read_timeout,"MEASUREMENT_LIMITED: measured homing current stale");
      if (measured_pin(now)) {
        write(cybergear::Reg::LocRef,pinned_pose_,"hold_measured_pose");
        state(State::HoldVerify,now); verification_={{cybergear::Reg::LocRef,pinned_pose_}};
      }
    } else if (state_==State::HoldVerify) {
      require(enabled_,"HARD_ABORT: enabled homing mode was lost");
      require(current_ns_ && now-current_ns_<=limits_.read_timeout,"MEASUREMENT_LIMITED: measured homing current stale");
      if (verify(now)) { state(State::Run,now); holding_=true; }
    } else if (state_==State::TransitionPose) {
      stop_poll(now); if (measured_pin(now)) state(State::TransitionWrite,now);
    } else if (state_==State::TransitionWrite) {
      write(cybergear::Reg::SpdRef,0,"transition_zero_speed"); write(cybergear::Reg::IqRef,0,"transition_zero_current");
      write(cybergear::Reg::RunMode,next_mode_,"select_homing_mode");
      for (const auto& [reg,value]:desired_) write(reg,value,"homing_native_setting");
      if (next_mode_==1) {
        write(cybergear::Reg::LimitSpd,0,"transition_zero_speed_limit");
        write(cybergear::Reg::LocRef,pinned_pose_,"transition_pin_measured_pose");
      }
      state(State::TransitionVerify,now); verification_=guarded(desired_); verification_.push_back({cybergear::Reg::RunMode,double(next_mode_)});
      verification_.push_back({cybergear::Reg::SpdRef,0});
      // In speed/position mode IqRef reads back the drive's own loop output, not our write.
      if (next_mode_==3) verification_.push_back({cybergear::Reg::IqRef,0});
      if (next_mode_==1) { verification_.push_back({cybergear::Reg::LocRef,pinned_pose_}); verification_.push_back({cybergear::Reg::LimitSpd,0}); }
    } else if (state_==State::TransitionVerify) {
      stop_poll(now); if (verify(now)) state(State::Enable,now);
    } else if (state_==State::Enable) {
      check_streams(now); require(disabled_ && now-status_ns_<=limits_.can_gap,"HARD_ABORT: enable without fresh disabled status");
      enabled_=false; enable_seen_=false; enable_begin_=now; pitch(cybergear::make_enable(0,127),"enable_homing_neutral");
      state(State::EnabledVerify,now); verification_=guarded(desired_); verification_.push_back({cybergear::Reg::RunMode,double(next_mode_)});
      verification_.push_back({cybergear::Reg::SpdRef,0});
      if (next_mode_==3) verification_.push_back({cybergear::Reg::IqRef,0});
      if (next_mode_==1) verification_.push_back({cybergear::Reg::LimitSpd,0});
    } else if (state_==State::EnabledVerify) {
      if (verify(now)) {
        require(enabled_ && status_ns_>=enable_begin_,"HARD_ABORT: enabled homing status absent");
        if (next_mode_==1) state(State::EnabledPose,now); else complete_transition(now);
      }
    } else if (state_==State::EnabledPose) {
      require(enabled_,"HARD_ABORT: enabled homing mode was lost");
      if (measured_pin(now)) {
        write(cybergear::Reg::LocRef,pinned_pose_,"enabled_repin_measured_pose");
        state(State::EnabledPinVerify,now); verification_={{cybergear::Reg::LocRef,pinned_pose_},{cybergear::Reg::LimitSpd,0}};
      }
    } else if (state_==State::EnabledPinVerify) {
      require(enabled_,"HARD_ABORT: enabled homing mode was lost");
      if (verify(now)) complete_transition(now);
    } else if (state_==State::MidpointVerify) {
      require(enabled_,"HARD_ABORT: enabled homing mode was lost");
      require(now-midpoint_begin_<midpoint_timeout_,"HARD_ABORT: measured midpoint timeout");
      if (verify(now)) state(State::Run,now);
    } else if (state_==State::Run) {
      require(enabled_,"HARD_ABORT: enabled homing mode was lost");
      require(now-entered_<=limits_.read_timeout || (current_ns_>=entered_ && now-current_ns_<=limits_.read_timeout),
              "MEASUREMENT_LIMITED: measured homing current stale");
      if (trial_active_) { trial_step(now); if (state_!=State::Run) return; }
      if (readback_.pending()) return;
      if (!trial_active_ && status_ns_>fsm_status_ns_) {
        fsm_status_ns_=status_ns_;
        if (!midpoint_) {
        // Preserve actual receipt time. Transition waits pause calls to the FSM,
        // never rewrite measurement time or hide time consumed by setup.
        const auto desired=homing_.step({status_ns_,pose_,velocity_,torque_,false});
        record_endpoint(homing_.home_a(),"A",endpoint_a_recorded_);
        record_endpoint(homing_.home_b(),"B",endpoint_b_recorded_);
        if (homing_.terminal()) {
          if (homing_.phase()!=FullAxisPhase::Complete)
            throw std::runtime_error("HARD_ABORT: sensorless homing FSM failed: "+homing_.result().fail_reason);
          const auto& result=homing_.result(); midpoint_target_=0.5*(result.endpoint_a_rad+result.endpoint_b_rad);
          record("{\"kind\":\"homing_endpoints\",\"endpoint_a_rad\":"+scalar(result.endpoint_a_rad)+
            ",\"endpoint_b_rad\":"+scalar(result.endpoint_b_rad)+",\"measured_travel_deg\":"+scalar(result.measured_travel_deg)+
            ",\"repeatability_rad\":"+scalar(result.repeatability_rad)+",\"midpoint_rad\":"+scalar(midpoint_target_)+
            ",\"encoder_mechpos_agreement_qualified\":false,\"calibration_qualified\":false}");
          if (trial_) { begin_trial_after_homing(result.endpoint_a_rad,result.endpoint_b_rad,now); return; }
          midpoint_=true; midpoint_begin_=now;
          DesiredState move; move.position_move=true; move.enter_pos_mode=true; move.target_rad=midpoint_target_;
          move.speed_rad_s=midpoint_speed_; move.message="measured midpoint"; begin_transition(move,1,now);
        } else apply(desired,now);
        } else {
        require(now-midpoint_begin_<midpoint_timeout_,"HARD_ABORT: measured midpoint timeout");
        if (midpoint_command_ns_ && now-midpoint_command_ns_>=midpoint_travel_ns_+midpoint_dwell_) {
          record("{\"kind\":\"homing_midpoint_dwell\",\"target_rad\":"+scalar(midpoint_target_)+",\"observed_rad\":"+scalar(pose_)+
            ",\"observed_minus_target_rad\":"+scalar(pose_-midpoint_target_)+",\"command_ns\":"+std::to_string(midpoint_command_ns_)+
            ",\"trajectory_duration_ns\":"+std::to_string(midpoint_travel_ns_)+
            ",\"begin_ns\":"+std::to_string(midpoint_command_ns_+midpoint_travel_ns_)+",\"end_ns\":"+std::to_string(now)+
            ",\"settled_qualified\":false}");
          state(State::FinalStop,now);
        }
        }
      }
      // A reply slower than read_period must still allow the FSM to advance.
      // Start the next measurement only after consuming the completed one.
      if (state_==State::Run && !readback_.pending() && now>=next_read_) {
        read(cybergear::Reg::Iqf,now); next_read_=now+limits_.read_period;
      }
    } else if (state_==State::RestoreWrite) {
      for (const auto& [reg,value]:observed_) write(reg,value,"restore_original_disabled_setting");
      state(State::RestoreVerify,now); verification_=observed_;
    } else if (state_==State::RestoreVerify) {
      stop_poll(now); if (verify(now)) state(State::Done,now);
    }
  }
  void begin_trial_after_homing(double a,double b,int64_t now) {
    const double low=std::min(a,b),high=std::max(a,b);
    require(high-low>2*trial_margin_+0.1,"HARD_ABORT: measured pitch travel too short for the servo trial window");
    trial_min_=low+trial_margin_; trial_max_=high-trial_margin_; trial_center_=0.5*(low+high); trial_slack_=trial_margin_+0.02;
    record("{\"kind\":\"pitch_trial_window\",\"endpoint_low_rad\":"+scalar(low)+",\"endpoint_high_rad\":"+scalar(high)+
           ",\"window_min_rad\":"+scalar(trial_min_)+",\"window_max_rad\":"+scalar(trial_max_)+",\"center_rad\":"+scalar(trial_center_)+"}");
    trial_active_=true;
    // Re-arm speed mode: the drive's speed integral is still wound up against the last stop.
    DesiredState initial; initial.rearm_speed_mode=true; initial.hold=true;
    begin_transition(initial,2,now);
  }
  void trial_step(int64_t now) {
    if (!trial_begin_) {
      trial_origin_=pose_; next_trial_=now;
      trial_approach_=std::max(0.5,std::abs(trial_center_-pose_)/trial_approach_speed_*1.5);
      trial_begin_=now+int64_t(trial_approach_*1e9);  // script time zero starts after the approach
      record("{\"kind\":\"pitch_trial_approach\",\"from_rad\":"+scalar(pose_)+",\"center_rad\":"+scalar(trial_center_)+
             ",\"duration_s\":"+scalar(trial_approach_)+"}");
    }
    const double elapsed=(now-trial_begin_)*1e-9;
    // During the approach the axis may start outside the working window (still
    // inside the endstops); the script itself is strict.
    const double slack=elapsed<0?trial_slack_:0.;
    require(pose_>=trial_min_-slack && pose_<=trial_max_+slack,"HARD_ABORT: pitch outside servo trial window");
    if (now-trial_begin_>=int64_t(trial_ref_.duration()*1e9)+trial_hold_) {
      command_speed_=0; write(cybergear::Reg::SpdRef,0,"trial_end_zero_speed");
      record("{\"kind\":\"pitch_trial_end\",\"time_ns\":"+std::to_string(now)+",\"pose_rad\":"+scalar(pose_)+"}");
      state(State::FinalStop,now); return;
    }
    if (now<next_trial_) return;
    next_trial_+=trial_period_; if (next_trial_<now) next_trial_=now+trial_period_;
    while (next_gain_<trial_schedule_.size() && elapsed>=trial_schedule_[next_gain_][0]) {
      const auto& g=trial_schedule_[next_gain_++]; loop_.set_gains(g[1],g[2]);
      record("{\"kind\":\"pitch_trial_gains\",\"time_ns\":"+std::to_string(now)+",\"kp\":"+scalar(g[1])+",\"ki\":"+scalar(g[2])+"}");
    }
    const auto r=trial_ref_.at(std::max(0.,elapsed),true);
    double q_ref=trial_center_+r.position, v_ref=r.velocity;
    if (elapsed<0) {
      // Smoothstep approach from the starting pose to the centre.
      const double x=std::clamp(1+elapsed/trial_approach_,0.,1.), d=trial_center_-trial_origin_;
      q_ref=trial_origin_+d*(3*x*x-2*x*x*x); v_ref=d*(6*x-6*x*x)/trial_approach_;
    }
    const double error=q_ref-pose_;
    require(std::abs(error)<=trial_follow_,"HARD_ABORT: pitch servo following error");
    const double dt=trial_last_?(now-trial_last_)*1e-9:0.; trial_last_=now;
    const double x=excitation_.at(elapsed);
    const double limit=loop_.parameters().speed_limit;
    const double speed=std::clamp(loop_.step(dt,q_ref,v_ref,pose_)+x,-limit,limit);
    command_speed_=speed; write(cybergear::Reg::SpdRef,speed,"trial_speed");
    std::ostringstream out; out.precision(9);
    out<<"{\"kind\":\"pitch_trial\",\"time_ns\":"<<now<<",\"qr\":"<<q_ref<<",\"vr\":"<<v_ref<<",\"q\":"<<pose_
       <<",\"status_ns\":"<<status_ns_<<",\"torque\":"<<torque_<<",\"cmd\":"<<speed<<",\"x\":"<<x<<",\"i\":"<<loop_.integral()<<'}';
    record(out.str());
  }
  void check_streams(int64_t now) {
    for (const auto& bus:buses_) require(bus.last_feedback && now-bus.last_feedback<=limits_.can_gap,"MEASUREMENT_LIMITED: CAN feedback absent/stale");
    require(imu_.ready(),"MEASUREMENT_LIMITED: IMU streams absent");
    for (const auto& [_,sensor]:imu_.sensors) require(now-sensor.received<=limits_.imu_gap,"MEASUREMENT_LIMITED: IMU stream stale");
    require(!buses_[0].receiver->kernel_drops() && !buses_[1].receiver->kernel_drops(),"DATA_INVALID: socket receive loss");
  }
  void pump() {
    std::array<pollfd,3> fds{{{buses_[0].fd.value,POLLIN,0},{buses_[1].fd.value,POLLIN,0},{imu_fd_,POLLIN,0}}};
    const auto result=poll(fds.data(),fds.size(),2); if (result<0 && errno==EINTR) return;
    require(result>=0,"DATA_INVALID: homing poll failed");
    for (unsigned axis=0;axis<2;++axis) {
      auto& bus=buses_[axis]; require(!(fds[axis].revents&(POLLERR|POLLHUP|POLLNVAL)),"HARD_ABORT: CAN endpoint failed"); Receipt receipt;
      for (unsigned drained=0;drained<256 && bus.receiver->receive(receipt);++drained) {
        record(receipt_json(receipt,axis?"pitch":"yaw",++bus.sequence));
        require(!receipt.drop_delta,"DATA_INVALID: socket receive loss"); require(!receipt.frame.error,"HARD_ABORT: CAN error frame");
        require(receipt.dequeue_ns-receipt.kernel_monotonic_ns<=limits_.dequeue,"DATA_INVALID: stale CAN dequeue");
        if (!axis) {
          gm6020::Feedback feedback; require(gm6020::decode(receipt.frame,1,feedback),"HARD_ABORT: unexpected yaw traffic");
          require(yaw_encoder_.update(feedback.angle_count,receipt.kernel_monotonic_ns),"HARD_ABORT: invalid yaw encoder");
          require(std::abs(yaw_encoder_.relative_rad())<=yaw_bound_,"HARD_ABORT: yaw homing displacement");
          bus.last_feedback=receipt.kernel_monotonic_ns; continue;
        }
        const auto id=cybergear::unpack_ext_id(receipt.frame.id);
        require(receipt.frame.extended && receipt.frame.dlc==8 && !receipt.frame.rtr,"HARD_ABORT: invalid pitch frame");
        cybergear::CanFrame frame; frame.id=receipt.frame.id; frame.dlc=receipt.frame.dlc; std::memcpy(frame.data,receipt.frame.data,8);
        if (id.comm_type==0) {
          cybergear::DiscoveryResponse response;
          require(!identified_ && state_==State::Discover && pending_begin_ && receipt.kernel_monotonic_ns>=pending_begin_ &&
                  receipt.kernel_monotonic_ns<pending_begin_+limits_.read_timeout && id.data2==127 &&
                  cybergear::parse_discovery_response(frame,response) && response.unique_id==expected_uid_,"INTEGRATION_MISMATCH: pitch UID/discovery correlation");
          identified_=true; record("{\"kind\":\"pitch_identity\",\"uid_hex\":"+quoted(uid_)+",\"receive_ns\":"+std::to_string(receipt.kernel_monotonic_ns)+"}");
        } else if (id.comm_type==17) {
          latest_read_=readback_.observe(receipt); record(readback_json(*latest_read_));
          require(latest_read_->value.has_value(),"MEASUREMENT_LIMITED: required homing register rejected");
          if (latest_read_->reg==cybergear::Reg::Iqf) { current_ns_=latest_read_->receive_ns;
            require(std::abs(*latest_read_->value)<=current_bound_,"HARD_ABORT: measured homing current exceeds guard"); }
        } else if (id.comm_type==18) {
          const unsigned index=uint16_t(frame.data[0])|(uint16_t(frame.data[1])<<8); std::string bytes;
          for (const auto b:frame.data) { const char* digits="0123456789abcdef"; bytes+=digits[b>>4]; bytes+=digits[b&15]; }
          const auto matched=std::any_of(writes_.begin(),writes_.end(),[&](const auto& write) {
            return write.index==index && write.bytes==bytes && receipt.kernel_monotonic_ns>=write.begin_ns &&
                   receipt.kernel_monotonic_ns<write.begin_ns+limits_.read_timeout;
          });
          require(id.target==0 && id.data2==127 && matched,
                  "DATA_INVALID: unexpected homing write echo");
          record("{\"kind\":\"write_echo\",\"index\":"+std::to_string(index)+",\"receive_ns\":"+std::to_string(receipt.kernel_monotonic_ns)+",\"readback_verified\":false}");
        } else if (id.comm_type==2) {
          cybergear::Feedback feedback;
          require(cybergear::parse_feedback(frame,feedback) && feedback.motor_id==127 && feedback.host_id==0,"HARD_ABORT: unexpected pitch feedback identity");
          require(!feedback.faults && feedback.temp_c<temp_bound_ && std::abs(feedback.torque_nm)<=torque_bound_,"HARD_ABORT: pitch fault/temperature/torque guard");
          const int count=(int(frame.data[0])<<8)|frame.data[1]; const auto stamp=receipt.kernel_monotonic_ns;
          if (have_pose_) {
            require(stamp>status_ns_,"DATA_INVALID: pitch status time reordered");
            const auto dt=double(stamp-encoder_anchor_ns_)*1e-9;
            const auto displacement=(count-encoder_anchor_count_)*(25./65535.);
            // Command acknowledgements can repeat the same quantized position
            // a fraction of a millisecond apart. Compare the raw displacement
            // with the elapsed-time corridor plus one-count quantization;
            // adding a count to velocity would invent motion at rest. Keep the
            // anchor through a full command period to expose sustained motion.
            // Replies 0.1-1 ms apart can differ by a couple of counts from sampling jitter.
            require(std::abs(displacement)<=speed_bound_*dt+3*25./65535.,"HARD_ABORT: pitch encoder speed guard");
            if (stamp-encoder_anchor_ns_>=limits_.stop_period) {
              velocity_=displacement/dt; encoder_anchor_count_=count; encoder_anchor_ns_=stamp;
            }
          } else {
            origin_=feedback.angle_rad; low_=high_=origin_; have_pose_=true;
            encoder_anchor_count_=count; encoder_anchor_ns_=stamp;
          }
          pose_=feedback.angle_rad; last_count_=count; torque_=feedback.torque_nm; status_ns_=stamp;
          low_=std::min(low_,pose_); high_=std::max(high_,pose_);
          require(high_-low_<=travel_bound_,"HARD_ABORT: pitch total displacement guard");
          disabled_=feedback.mode==cybergear::MotorMode::Reset; enabled_=feedback.mode==cybergear::MotorMode::Motor;
          require(disabled_ || enabled_,"HARD_ABORT: unexpected pitch drive state"); bus.last_feedback=stamp;
          // A last STOP response can arrive after enable was transmitted.
          // Keep the command neutral until a fresh Motor acknowledgement;
          // once seen, any subsequent Reset is still an enabled-state loss.
          if (state_==State::EnabledVerify && stamp>=enable_begin_ && enabled_) enable_seen_=true;
          if (state_==State::Observe || state_==State::Snapshot || state_==State::TransitionPose || state_==State::TransitionWrite ||
              state_==State::TransitionVerify || state_==State::Enable || state_==State::RestoreWrite || state_==State::RestoreVerify)
            require(disabled_,"HARD_ABORT: pitch re-enabled during disabled transition");
          if (state_==State::Run || state_==State::HoldPose || state_==State::HoldVerify ||
              state_==State::MidpointVerify ||
              state_==State::EnabledPose || state_==State::EnabledPinVerify ||
              (state_==State::EnabledVerify && enable_seen_))
            require(enabled_,"HARD_ABORT: enabled homing mode was lost");
        } else require(false,"HARD_ABORT: unexpected pitch traffic");
      }
    }
    require(!(fds[2].revents&(POLLERR|POLLNVAL)),"DATA_INVALID: IMU pipe failed");
    if (fds[2].revents&(POLLIN|POLLHUP)) imu_.read_fd(imu_fd_,limits_,*journal_);
  }
  void shutdown() noexcept {
    try { send(0,gm6020::current_zero_frame(1),"abort_zero"); } catch (...) {}
    if (!identified_) return;
    try { write(cybergear::Reg::SpdRef,0,"abort_zero_speed"); } catch (...) {}
    try { write(cybergear::Reg::IqRef,0,"abort_zero_current"); } catch (...) {}
    const auto begin=monotonic_ns(); bool sent=false;
    try { pitch(cybergear::make_stop(0,127),"abort_stop"); sent=true; } catch (...) {}
    while (monotonic_ns()-begin<limits_.read_timeout) {
      try {
        Receipt receipt;
        if (buses_[1].receiver->receive(receipt)) {
          journal_->append(receipt_json(receipt,"pitch",++buses_[1].sequence)); cybergear::CanFrame frame;
          frame.id=receipt.frame.id; frame.dlc=receipt.frame.dlc; std::memcpy(frame.data,receipt.frame.data,8); cybergear::Feedback feedback;
          if (sent && receipt.frame.extended && !receipt.frame.error && !receipt.frame.rtr && receipt.frame.dlc==8 &&
              cybergear::parse_feedback(frame,feedback) && feedback.motor_id==127 && feedback.host_id==0 &&
              feedback.mode==cybergear::MotorMode::Reset && !feedback.faults && receipt.kernel_monotonic_ns>=begin &&
              receipt.kernel_monotonic_ns<begin+limits_.read_timeout && receipt.dequeue_ns-receipt.kernel_monotonic_ns<=limits_.dequeue) {
            abort_stop_confirmed_=true; break;
          }
        } else { pollfd fd{buses_[1].fd.value,POLLIN,0}; poll(&fd,1,2); }
      } catch (...) { break; }
    }
  }
  int finish(bool complete,const std::string& detail) {
    uint32_t yaw_drops=0,pitch_drops=0; std::string loss="{"; std::string failure=detail;
    try {
      yaw_drops=buses_[0].receiver->kernel_drops(); pitch_drops=buses_[1].receiver->kernel_drops();
      require(!yaw_drops && !pitch_drops,"DATA_INVALID: final socket receive loss");
      if (!synthetic_) for (unsigned axis=0;axis<2;++axis) {
        const auto& bus=buses_[axis]; const auto drops=bus.counter("rx_dropped"),errors=bus.counter("rx_errors");
        require(drops>=bus.drops_begin && errors>=bus.errors_begin,"DATA_INVALID: interface counters reset");
        if (axis) loss+=",";
        loss+=quoted(axis?"pitch":"yaw")+":{\"rx_dropped\":"+std::to_string(drops-bus.drops_begin)+",\"rx_errors\":"+std::to_string(errors-bus.errors_begin)+"}";
        require(drops==bus.drops_begin && errors==bus.errors_begin,"DATA_INVALID: interface receive loss");
      }
    } catch (const std::exception& e) { complete=false; if (failure.empty()) failure=e.what(); }
    loss+="}";
    const auto& result=homing_.result();
    const std::string footer="{\"kind\":\"footer\",\"status\":"+quoted(complete?"COMPLETE":"INVALID")+",\"detail\":"+quoted(failure)+
      ",\"parameter_qualified\":false,\"motion_qualified\":false,\"current_mode_qualified\":false,\"encoder_mechpos_agreement_qualified\":false,\"calibration_qualified\":false,"+
      "\"homing_observed\":"+(complete?"true":"false")+",\"endpoint_a_rad\":"+scalar(result.endpoint_a_rad)+",\"endpoint_b_rad\":"+scalar(result.endpoint_b_rad)+
      ",\"measured_travel_deg\":"+scalar(result.measured_travel_deg)+",\"repeatability_rad\":"+scalar(result.repeatability_rad)+",\"midpoint_rad\":"+scalar(midpoint_target_)+
      ",\"expected_original_mode\":"+std::to_string(expected_mode_)+",\"observed_original_mode\":"+(observed_.empty()?"null":scalar(observed_[0].second))+
      ",\"normal_stop_confirmed\":"+(complete && normal_stop_confirmed_ && disabled_?"true":"false")+
      ",\"abort_stop_confirmed\":"+(abort_stop_confirmed_?"true":"false")+",\"interface_loss_deltas\":"+loss+
      ",\"writer_queue_high_water\":"+std::to_string(journal_->high_water())+",\"socket_drops\":{\"yaw\":"+std::to_string(yaw_drops)+",\"pitch\":"+std::to_string(pitch_drops)+
      "},\"end_ns\":"+std::to_string(monotonic_ns())+"}";
    if (!journal_->finish(footer)) { std::cout<<"{\"kind\":\"footer\",\"status\":\"INVALID\",\"detail\":\"DATA_INVALID: capture journal did not finish\"}\n"; return 1; }
    std::cout<<footer<<'\n'; return complete?0:1;
  }
  YAML::Node config_; Limits limits_; FullAxisHomingParams params_; FullAxisHoming homing_; bool synthetic_; Readback readback_;
  std::string uid_; uint64_t expected_uid_{}; int expected_mode_{},imu_fd_{},mode_{},next_mode_{},last_count_{},encoder_anchor_count_{};
  double current_bound_{},torque_bound_{},temp_bound_{},yaw_bound_{},travel_bound_{},speed_bound_{},transition_bound_{};
  double midpoint_speed_{},midpoint_target_{},origin_{},pose_{},velocity_{},torque_{},low_{},high_{},transition_origin_{},pinned_pose_{};
  double command_target_{},command_speed_{},position_limit_{};
  int64_t begin_{},entered_{},pending_begin_{},next_ping_{},next_yaw_{},next_command_{},next_read_{},current_ns_{},status_ns_{},fsm_status_ns_{};
  int64_t encoder_anchor_ns_{};
  int64_t transition_begin_{},enable_begin_{},midpoint_begin_{},midpoint_command_ns_{},midpoint_travel_ns_{},midpoint_dwell_{},midpoint_timeout_{};
  bool identified_{},disabled_{},enabled_{},enable_seen_{},have_pose_{},transition_{},holding_{true},midpoint_{},abort_stop_confirmed_{},normal_stop_confirmed_{};
  bool endpoint_a_recorded_{},endpoint_b_recorded_{};
  bool trial_{},trial_active_{},home_first_{}; axis::ReferenceTable trial_ref_; axis::PositionLoop loop_; axis::LogSweep excitation_;
  std::vector<std::array<double,3>> trial_schedule_; std::size_t next_gain_{};
  double trial_min_{},trial_max_{},trial_follow_{},trial_origin_{},trial_margin_{},trial_slack_{};
  int64_t trial_period_{},trial_hold_{},trial_begin_{},next_trial_{},trial_last_{};
  double trial_center_{},trial_approach_speed_{},trial_approach_{};
  State state_{State::Discover}; DesiredState latched_; Settings original_,desired_,observed_,verification_; size_t verify_index_{};
  std::array<Endpoint,2> buses_; ImuStream imu_; gm6020::UnwrappedEncoder yaw_encoder_;
  std::optional<ReadObservation> latest_read_; std::unique_ptr<Journal> journal_; std::string last_message_;
  struct SentWrite { unsigned index; std::string bytes; int64_t begin_ns; };
  std::deque<SentWrite> writes_;
};
}
int sensorless_homing_session(const char* path) {
  try { return HomingSession(YAML::LoadFile(path)).run(); }
  catch (const std::exception& e) { std::cerr<<"{\"status\":\"INVALID\",\"detail\":"<<quoted(e.what())<<"}\n"; return 1; }
}
int validate_homing_manifest(const char* path) {
  try {
    HomingSession checked(YAML::LoadFile(path),true);
    std::cout<<"{\"status\":\"VALID_MANIFEST\",\"hardware_accessed\":false,\"parameter_qualified\":false}\n";
    return 0;
  } catch (const std::exception& e) { std::cerr<<"{\"status\":\"INVALID\",\"detail\":"<<quoted(e.what())<<"}\n"; return 1; }
}
}
