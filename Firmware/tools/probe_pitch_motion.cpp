// Continuous pitch step/return session using the production current interlock.
// No homing, encoder zero, calibration persistence, or yaw transmitter.
#include <atomic>
#include <charconv>
#include <chrono>
#include <cmath>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <numbers>
#include <thread>
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>
#include "can/cybergear_system.hpp"
#include "can/socketcan_bus.hpp"
#include "control/can_motor_backend.hpp"

using namespace std::chrono_literals;
constexpr auto axis=ota::AxisId::Pitch;
constexpr double rad=std::numbers::pi/180.0;
constexpr double commanded_speed=10*rad;
static volatile std::sig_atomic_t interrupted;
static void stop(int) { interrupted=1; }

int main(int argc, char** argv) {
  if (argc!=3 && argc!=4) { std::cerr<<"Usage: probe-pitch-motion STEP_MILLIDEGREES TRACE.csv [tuned|restore] (max +/-15000 mdeg)\n"; return 2; }
  const bool tune=argc==4 && std::string(argv[3])=="tuned";
  const bool restore=argc==4 && std::string(argv[3])=="restore";
  if (argc==4 && !tune && !restore) return 2;
  int step=0;
  const std::string arg=argv[1];
  auto parsed=std::from_chars(arg.data(),arg.data()+arg.size(),step);
  if (parsed.ec!=std::errc{} || parsed.ptr!=arg.data()+arg.size() || step < -15000 || step > 15000) return 2;
  std::signal(SIGINT,stop); std::signal(SIGTERM,stop);
  int ownership=-1;
  ota::can::CyberGearSystem system;
  ota::CanMotorBackend backend(system);
  bool identified=false;
  bool original_gains_read=false;
  double original_kp=0, original_ki=0;
  try {
    const auto lock="/tmp/ota-mixed-can-"+std::to_string(getuid())+".lock";
    ownership=::open(lock.c_str(),O_CREAT|O_RDWR|O_CLOEXEC|O_NOFOLLOW,0600);
    if (ownership<0 || flock(ownership,LOCK_EX|LOCK_NB)) throw std::runtime_error("CAN probe ownership unavailable");
    if (std::filesystem::canonical("/sys/class/net/can1/device").filename()!="spi1.0")
      throw std::runtime_error("can1 SPI parent mismatch");
    std::ofstream trace(argv[2]);
    if (!trace) throw std::runtime_error("trace unavailable");
    trace<<"time_ns,phase,pitch_relative_deg,q_rad,speed_deg_s,temperature_c,disabled,feedback_age_ms\n";
    ota::can::CyberGearSystemConfig cfg;
    cfg.iface="can1"; cfg.pitch_motor_id=127; cfg.bring_up_if_down=false;
    // The unused logical yaw slot is never queried or commanded by this probe.
    std::string error;
    if (!system.open(cfg,error)) throw std::runtime_error(error);
    const auto* can=dynamic_cast<const ota::can::SocketCanBus*>(&system.bus());
    if (!can || !system.bus().is_up() || can->bitrate()!=1000000 ||
        system.bus().can_state()!=ota::can::CanIfState::ErrorActive)
      throw std::runtime_error("can1 health/bitrate mismatch");
    uint64_t uid=0;
    if (!backend.discover(axis,uid,error) || uid!=0x7216313130333105ULL)
      throw std::runtime_error("pitch identity mismatch: "+error);
    identified=true;
    if (!system.send_stop(axis,&error)) throw std::runtime_error(error);
    std::this_thread::sleep_for(20ms);
    const auto initial=backend.snapshot(axis,ota::now_monotonic_ns());
    if (!initial.has_feedback || !initial.disabled || initial.faults || !std::isfinite(initial.q_rad))
      throw std::runtime_error("disabled fault-free pitch feedback required");
    using Reg=ota::cybergear::Reg;
    // Clear volatile gains left by an interrupted earlier commissioning trial
    // during the single disabled setup; no disable between movement stages.
    if (tune && !backend.restore_stopped_pitch_gains(1,.002,error))
      throw std::runtime_error("initial trial gain restoration failed: "+error);
    for (const auto r : {Reg::RunMode,Reg::LocRef,Reg::LimitSpd,Reg::LimitCur,Reg::MechPos,
                         Reg::LocKp,Reg::SpdKp,Reg::SpdKi,Reg::Iqf,Reg::VBus}) {
      double value=0;
      if (!backend.read_register(axis,r,value,200,error)) throw std::runtime_error("diagnostic read failed: "+error);
      std::cout<<"PITCH_REG name="<<ota::cybergear::reg_name(r)<<" value="<<value<<'\n';
      if (r==Reg::SpdKp) original_kp=value;
      if (r==Reg::SpdKi) original_ki=value;
    }
    original_gains_read=true;
    if (step==0) {
      if (restore && !backend.restore_stopped_pitch_gains(1,.002,error)) throw std::runtime_error(error);
      if (restore) std::cout<<"PITCH_GAINS restored_kp=1 restored_ki=0.002 without_enable=1\n";
      backend.deenergize(axis); system.close(); ::close(ownership);
      std::cout<<"PITCH_DIAGNOSTICS completed; no enable or movement command\n"; return 0;
    }
    // Diagnostic reads do not elicit COMM_TYPE_2; refresh stopped feedback.
    if (!system.send_stop(axis,&error)) throw std::runtime_error(error);
    std::this_thread::sleep_for(20ms);
    std::cout<<"PITCH identified_uid=0x"<<std::hex<<uid<<std::dec<<" q0_rad="<<initial.q_rad
             <<" step_deg="<<step/1000.0<<" speed_limit_deg_s=10 current_ceiling_a=5\n"
             <<"PITCH_LIMITS trusted_pitch_endpoints=0 limitation=unhomed_pitch_mechanical_envelope_uncommissioned "
               "direct_targets=1 operator_clearance_check_required=1\n"<<std::flush;
    if (tune) std::cout<<"PITCH_TRIAL_GAINS kp=4 ki=0.05; restore original gains before stop\n";
    std::mutex commands;
    std::atomic<ota::TimeNs> heartbeat{ota::now_monotonic_ns()};
    std::atomic<bool> trip{false}, stop_failed{false};
    const auto started=ota::now_monotonic_ns();
    std::atomic<int> trip_reason{0};
    std::atomic<double> active_target{initial.q_rad};
    std::atomic<double> progress_q{initial.q_rad};
    std::atomic<ota::TimeNs> progress_ns{started};
    std::atomic<double> sampled_iqf{NAN};
    std::atomic<ota::TimeNs> sampled_iqf_ns{0};
    std::atomic<bool> target_move_active{false};
    // Guard owns only the pitch stop and serializes it with every setup/target
    // command. It cannot survive process/Pi loss; this is bounded commissioning.
    std::jthread guard([&](std::stop_token done) {
      double last_q=initial.q_rad, measured_speed=0;
      auto last_q_ns=started;
      while (!done.stop_requested()) {
        {
          std::lock_guard lock(commands);
          const auto now=ota::now_monotonic_ns();
          const auto s=backend.snapshot(axis,now);
          if (s.has_feedback && s.rx_ns-last_q_ns>=50000000LL) {
            measured_speed=(s.q_rad-last_q)/((s.rx_ns-last_q_ns)*1e-9);
            last_q=s.q_rad; last_q_ns=s.rx_ns;
          }
          if (target_move_active && s.has_feedback && s.rx_ns>progress_ns.load() &&
              std::abs(s.q_rad-progress_q.load())>=0.12*rad) {
            progress_q=s.q_rad;
            progress_ns=s.rx_ns;
          }
          int reason=0;
          if (interrupted) reason=1;
          else if (now-started>40000000000LL) reason=2;
          else if (now-heartbeat.load()>100000000LL) reason=3;
          else if (!s.has_feedback || s.rx_ns>now || now-s.rx_ns>100000000LL) reason=4;
          else if (s.faults) reason=5;
          else if (target_move_active && s.disabled) reason=13;
          // This is a bounded excursion guard, not a trusted mechanical limit. The
          // station's pitch soft endpoints are uncommissioned until homing; do not
          // pretend expected_travel_deg is an absolute coordinate envelope.
          else if (!std::isfinite(s.q_rad) ||
                   std::abs(s.q_rad-initial.q_rad)>(std::abs(step)/1000.0+2.0)*rad) reason=6;
          else if (!std::isfinite(measured_speed) || std::abs(measured_speed)>20*rad) reason=7;
          else if (!std::isfinite(s.temp_c) || s.temp_c>45) reason=8;
          else if (system.bus().stats().rx_error_frames) reason=9;
          else if (target_move_active && std::abs(active_target.load()-s.q_rad)>0.75*rad &&
                   now-progress_ns.load()>1500000000LL &&
                   now-sampled_iqf_ns.load()>500000000LL) reason=12;
          else if (target_move_active && std::abs(active_target.load()-s.q_rad)>0.75*rad &&
                   now-progress_ns.load()>1500000000LL &&
                   std::abs(sampled_iqf.load())>=4.5) reason=10;
          else if (target_move_active && std::abs(active_target.load()-s.q_rad)>0.75*rad &&
                   now-progress_ns.load()>2500000000LL) reason=11;
          if (reason && !trip) {
            trip_reason=reason;
            trip=true;
            if (reason==10 || reason==11 || reason==12) {
              std::cerr<<"PITCH_STALL target_error_deg="
                       <<std::abs(active_target.load()-s.q_rad)/rad
                       <<" encoder_no_progress_ms="<<(now-progress_ns.load())/1e6
                       <<" iqf_a="<<sampled_iqf.load()
                       <<" iqf_age_ms="<<(now-sampled_iqf_ns.load())/1e6
                       <<" trip_reason="<<reason<<'\n'<<std::flush;
            }
          }
          if (trip && !system.send_stop(axis)) stop_failed=true;
        }
        std::this_thread::sleep_for(5ms);
      }
      for (int i=0;i<5;++i) { if (!system.send_stop(axis)) stop_failed=true; std::this_thread::sleep_for(10ms); }
    });
    auto record=[&](const char* phase) {
      const auto now=ota::now_monotonic_ns(); const auto s=backend.snapshot(axis,now);
      trace<<now<<','<<phase<<','<<(s.q_rad-initial.q_rad)/rad<<','<<s.q_rad<<','<<s.v_rad_s/rad
           <<','<<s.temp_c<<','<<s.disabled<<','<<(now-s.rx_ns)*1e-6<<'\n';
    };
    auto mode=ota::MotorBackend::Transition::Pending;
    while (!trip && mode==ota::MotorBackend::Transition::Pending) {
      {
        std::lock_guard lock(commands);
        heartbeat=ota::now_monotonic_ns();
        if (!trip) mode=backend.transition_mode(axis,true,commanded_speed,heartbeat.load(),error,tune ? .05:-1,tune ? 4:1,true);
      }
      record("setup"); std::this_thread::sleep_for(5ms);
    }
    if (trip || mode!=ota::MotorBackend::Transition::Complete)
      throw std::runtime_error("pitch setup stopped: "+error);
    const auto q0=backend.snapshot(axis,ota::now_monotonic_ns()).q_rad;
    // The production transition writes and reads back exactly 5 A before the
    // position-mode enable. The pitch endpoints are not homed/commissioned.
    std::cout<<"PITCH_CURRENT_LIMIT verified_a=5 source=production_backend_pre_enable_readback\n"
             <<std::flush;
    // This is a bounded motion check, not a homing or limit-calibration run.
    const Reg observed_regs[]={Reg::LocRef,Reg::LimitSpd,Reg::Iqf,Reg::MechVel};
    unsigned observed=0; bool waiting=false; ota::TimeNs read_deadline=0;
    // The absolute pitch envelope is not commissioned; don't misrepresent the
    // homing expected_travel range as an absolute soft limit. The operator must
    // check clearance. Runtime supervision is by feedback/fault/temperature,
    // encoder-derived speed, lack of progress under a nonzero target, and a
    // hard excursion ceiling of requested amplitude + 2 degrees.
    const double amplitude=std::abs(step)*rad/1000.0;
    const double first_direction=step<0 ? -1.0:1.0;
    const double endpoints[]={q0+first_direction*amplitude,q0,
                              q0-first_direction*amplitude,q0};
    const char* legs[]={"outbound_first","return_origin",
                        "outbound_opposite","return_origin"};
    int stage_number=0;
    for (int stage=0;stage<4 && !trip;++stage) {
      const double target=endpoints[stage];
      const std::string phase=legs[stage];
      ++stage_number;
      const auto before=backend.snapshot(axis,ota::now_monotonic_ns());
      active_target=target;
      progress_q=before.q_rad;
      progress_ns=before.rx_ns;
      target_move_active=(std::abs(target-before.q_rad)>0.5*rad);
      std::cout<<"PITCH_STAGE stage="<<stage_number<<" leg="<<legs[stage]
               <<" target_rad="<<target<<" target_delta_deg="<<(target-before.q_rad)/rad
               <<" speed_limit_deg_s=10 enabled_continuously=1 hold_until_settled=1\n"
               <<std::flush;
      const auto stage_started=ota::now_monotonic_ns();
      const auto stage_timeout=10000000000LL;
      ota::TimeNs settled_since=0;
      double settled_q=before.q_rad;
      while (!trip && ota::now_monotonic_ns()-stage_started<stage_timeout) {
        const auto now=ota::now_monotonic_ns();
        const auto sample=backend.snapshot(axis,now);
        // Encoder position is the stillness measure. CyberGear's raw speed
        // estimate is noisy at rest, so it is deliberately not a settling gate.
        if (sample.has_feedback && sample.rx_ns<=now && now-sample.rx_ns<=100000000LL &&
            std::abs(sample.q_rad-target)<=0.5*rad) {
          if (!settled_since) { settled_since=now; settled_q=sample.q_rad; }
          else if (std::abs(sample.q_rad-settled_q)>0.12*rad) {
            settled_since=now; settled_q=sample.q_rad;
          }
        } else {
          settled_since=0;
        }
        if (settled_since && now-settled_since>=200000000LL) break;
        {
          std::lock_guard lock(commands);
          heartbeat=now;
          if (!trip) backend.command(axis,target,commanded_speed);
        }
        {
          if (!waiting) {
            if (!system.begin_register_read(axis,observed_regs[observed],error))
              throw std::runtime_error(error);
            waiting=true;
            read_deadline=ota::now_monotonic_ns()+100000000LL;
          } else {
            double value=0;
            const int result=system.poll_register_read(value,error);
            if (result<0 || (result==0 && ota::now_monotonic_ns()>read_deadline))
              throw std::runtime_error("active diagnostic read failed");
            if (result==1) {
              std::cout<<"PITCH_ACTIVE_REG name="
                       <<ota::cybergear::reg_name(observed_regs[observed])
                       <<" value="<<value<<'\n';
              if (observed_regs[observed]==Reg::Iqf &&
                  (!std::isfinite(value) || std::abs(value)>5))
                throw std::runtime_error("observed filtered current exceeds 5 A");
              if (observed_regs[observed]==Reg::Iqf) {
                sampled_iqf=value;
                sampled_iqf_ns=ota::now_monotonic_ns();
              }
              waiting=false;
              observed=(observed+1)%4;
            }
          }
        }
        record(phase.c_str());
        if (settled_since && ota::now_monotonic_ns()-settled_since>=200000000LL) break;
        std::this_thread::sleep_for(5ms);
      }
      const auto reached=backend.snapshot(axis,ota::now_monotonic_ns());
      const auto reached_at=ota::now_monotonic_ns();
      const bool arrived=reached.has_feedback && std::abs(reached.q_rad-target)<=0.5*rad &&
                         reached.rx_ns<=reached_at && reached_at-reached.rx_ns<=100000000LL &&
                         settled_since!=0 && reached_at-settled_since>=200000000LL &&
                         !reached.disabled && !reached.faults;
      std::cout<<"PITCH_STAGE_RESULT stage="<<stage_number<<" error_deg="<<(reached.q_rad-target)/rad
               <<" speed_deg_s="<<reached.v_rad_s/rad<<" arrived_settled="<<arrived
               <<" disabled="<<reached.disabled<<" faults="<<reached.faults<<'\n'<<std::flush;
      target_move_active=false;
      if (!trip && !arrived) throw std::runtime_error("pitch target failed to settle before timeout");
    }
    system.cancel_register_read();
    if (tune && !trip) {
      {
        std::lock_guard lock(commands);
        heartbeat=ota::now_monotonic_ns();
        backend.command(axis,backend.snapshot(axis,heartbeat.load()).q_rad,0);
        backend.set_speed_loop_gains(axis,original_kp,original_ki);
      }
      for (const auto r:{Reg::SpdKp,Reg::SpdKi}) {
        heartbeat=ota::now_monotonic_ns(); double actual=0;
        const double expected=r==Reg::SpdKp ? original_kp:original_ki;
        if (!backend.read_register(axis,r,actual,80,error) || std::abs(actual-expected)>1e-6)
          throw std::runtime_error("trial gains restore not verified");
      }
      std::cout<<"PITCH_GAINS restored_kp="<<original_kp<<" restored_ki="<<original_ki<<'\n';
    }
    {
      std::lock_guard lock(commands); system.cancel_register_read(); backend.deenergize(axis);
    }
    const auto observe_until=ota::now_monotonic_ns()+2000000000LL;
    while (ota::now_monotonic_ns()<observe_until) {
      heartbeat=ota::now_monotonic_ns();
      if (!system.send_stop(axis)) stop_failed=true;
      record("observe"); std::this_thread::sleep_for(20ms);
    }
    guard.request_stop(); guard.join();
    if (tune && trip && !backend.restore_stopped_pitch_gains(original_kp,original_ki,error))
      throw std::runtime_error("stopped trial gain restoration failed: "+error);
    const auto final=backend.snapshot(axis,ota::now_monotonic_ns());
    double cap=0;
    const bool cap_ok=backend.read_register(axis,ota::cybergear::Reg::LimitCur,cap,200,error) &&
                      std::isfinite(cap) && std::abs(cap-5.0)<=1e-6;
    std::cout<<"PITCH_RESULT guard_trip="<<trip<<" trip_reason="<<trip_reason<<" stop_failed="<<stop_failed<<" disabled="<<final.disabled
             <<" faults="<<final.faults<<" delta_deg="<<(final.q_rad-initial.q_rad)/rad
             <<" final_limit_a="<<cap<<" current_limit_verified="<<cap_ok<<std::endl;
    system.close(); ::close(ownership);
    return !trip && !stop_failed && final.disabled && !final.faults && cap_ok ? 0:1;
  } catch (const std::exception& e) {
    if (identified) {
      backend.deenergize(axis);
      if (tune && original_gains_read) {
        std::string restore_error;
        if (!backend.restore_stopped_pitch_gains(original_kp,original_ki,restore_error))
          std::cerr<<"PITCH_GAINS_EXCEPTION_RESTORE_FAILED "<<restore_error<<'\n';
      }
    }
    std::cerr<<"PITCH_PROBE_FAILED "<<e.what()<<std::endl;
    system.close(); if (ownership>=0) ::close(ownership); return 1;
  }
}
