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
  if (argc!=3 && argc!=4) { std::cerr<<"Usage: probe-pitch-motion STEP_MILLIDEGREES TRACE.csv [tuned]\n"; return 2; }
  const bool tune=argc==4 && std::string(argv[3])=="tuned";
  const bool restore=argc==4 && std::string(argv[3])=="restore";
  if (argc==4 && !tune && !restore) return 2;
  int step=0;
  const std::string arg=argv[1];
  auto parsed=std::from_chars(arg.data(),arg.data()+arg.size(),step);
  if (parsed.ec!=std::errc{} || parsed.ptr!=arg.data()+arg.size() || step < -3000 || step > 3000) return 2;
  std::signal(SIGINT,stop); std::signal(SIGTERM,stop);
  int ownership=-1;
  ota::can::CyberGearSystem system;
  ota::CanMotorBackend backend(system);
  bool identified=false;
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
    double original_kp=0, original_ki=0;
    for (const auto r : {Reg::RunMode,Reg::LocRef,Reg::LimitSpd,Reg::LimitCur,Reg::MechPos,
                         Reg::LocKp,Reg::SpdKp,Reg::SpdKi,Reg::Iqf,Reg::VBus}) {
      double value=0;
      if (!backend.read_register(axis,r,value,200,error)) throw std::runtime_error("diagnostic read failed: "+error);
      std::cout<<"PITCH_REG name="<<ota::cybergear::reg_name(r)<<" value="<<value<<'\n';
      if (r==Reg::SpdKp) original_kp=value;
      if (r==Reg::SpdKi) original_ki=value;
    }
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
             <<" step_deg="<<step/1000.0<<" speed_limit_deg_s=10 current_ceiling_a=5\n"<<std::flush;
    if (tune) std::cout<<"PITCH_TRIAL_GAINS kp=4 ki=0.05; restore original gains before stop\n";
    std::mutex commands;
    std::atomic<ota::TimeNs> heartbeat{ota::now_monotonic_ns()};
    std::atomic<bool> trip{false}, stop_failed{false};
    const auto started=ota::now_monotonic_ns();
    std::atomic<int> trip_reason{0};
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
          int reason=0;
          if (interrupted) reason=1;
          else if (now-started>15000000000LL) reason=2;
          else if (now-heartbeat.load()>100000000LL) reason=3;
          else if (!s.has_feedback || s.rx_ns>now || now-s.rx_ns>100000000LL) reason=4;
          else if (s.faults) reason=5;
          else if (!std::isfinite(s.q_rad) || std::abs(s.q_rad-initial.q_rad)>4*rad) reason=6;
          else if (!std::isfinite(measured_speed) || std::abs(measured_speed)>20*rad) reason=7;
          else if (!std::isfinite(s.temp_c) || s.temp_c>45) reason=8;
          else if (system.bus().stats().rx_error_frames) reason=9;
          if (reason && !trip) { trip_reason=reason; trip=true; }
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
    // Owner's probe-first contract: full authorized current headroom (5 A),
    // sufficient demand to prove motion, and a short bounded experiment.
    const Reg observed_regs[]={Reg::LocRef,Reg::LimitSpd,Reg::Iqf,Reg::MechVel};
    unsigned observed=0; bool waiting=false; ota::TimeNs read_deadline=0;
    // Two outward/return pairs in one enabled session. Each stage includes
    // settling at its target; CAN, IMU and feedback stay live throughout.
    for (int stage=0;stage<4 && !trip;++stage) {
    const double target=q0+(stage%2==0 ? step*rad/1000.0:0);
    const auto phase="stage"+std::to_string(stage+1);
    std::cout<<"PITCH_STAGE stage="<<stage+1<<" target_rad="<<target
             <<" enabled_continuously=1\n"<<std::flush;
    const auto until=ota::now_monotonic_ns()+1500000000LL;
    while (!trip && ota::now_monotonic_ns()<until) {
      {
        std::lock_guard lock(commands);
        heartbeat=ota::now_monotonic_ns();
        if (!trip) backend.command(axis,target,commanded_speed);
      }
      {
        if (!waiting) {
          if (!system.begin_register_read(axis,observed_regs[observed],error)) throw std::runtime_error(error);
          waiting=true; read_deadline=ota::now_monotonic_ns()+100000000LL;
        } else {
          double value=0; const int result=system.poll_register_read(value,error);
          if (result<0 || (result==0 && ota::now_monotonic_ns()>read_deadline)) throw std::runtime_error("active diagnostic read failed");
          if (result==1) {
            std::cout<<"PITCH_ACTIVE_REG name="<<ota::cybergear::reg_name(observed_regs[observed])<<" value="<<value<<'\n';
            if (observed_regs[observed]==Reg::Iqf && (!std::isfinite(value) || std::abs(value)>5))
              throw std::runtime_error("observed filtered current exceeds 5 A");
            waiting=false; observed=(observed+1)%4;
          }
        }
      }
      record(phase.c_str()); std::this_thread::sleep_for(5ms);
    }
    const auto reached=backend.snapshot(axis,ota::now_monotonic_ns());
    std::cout<<"PITCH_STAGE_RESULT stage="<<stage+1<<" error_deg="<<(reached.q_rad-target)/rad
             <<" disabled="<<reached.disabled<<" faults="<<reached.faults<<'\n';
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
    const bool cap_ok=backend.read_register(axis,ota::cybergear::Reg::LimitCur,cap,200,error) && cap>0 && cap<=5;
    std::cout<<"PITCH_RESULT guard_trip="<<trip<<" trip_reason="<<trip_reason<<" stop_failed="<<stop_failed<<" disabled="<<final.disabled
             <<" faults="<<final.faults<<" delta_deg="<<(final.q_rad-initial.q_rad)/rad
             <<" final_limit_a="<<cap<<" current_limit_verified="<<cap_ok<<std::endl;
    system.close(); ::close(ownership);
    return !trip && !stop_failed && final.disabled && !final.faults && cap_ok ? 0:1;
  } catch (const std::exception& e) {
    if (identified) backend.deenergize(axis);
    std::cerr<<"PITCH_PROBE_FAILED "<<e.what()<<std::endl;
    system.close(); if (ownership>=0) ::close(ownership); return 1;
  }
}
