// Real CAN backend and register protocol, simulated motor responses only.
// A suspended load can move while disabled even if COMM_TYPE_2 stays frozen.
#include <cstring>
#include <iostream>
#include <map>
#include <thread>
#include "control/can_motor_backend.hpp"

using namespace ota;
class MotorReplies : public can::CanTransport {
 public:
  FrameCallback callback;
  double drift = 0, q = 0;
  bool frozen_feedback = false, enabled = true, silent_position = false;
  bool ignore_stop = false;
  uint8_t motor_id = 100;
  int enables = 0, position_reads = 0, mode_writes = 0;
  std::map<uint16_t,double> regs;
  MotorReplies() {
    regs[uint16_t(cybergear::Reg::RunMode)]=1;
    regs[uint16_t(cybergear::Reg::LimitCur)]=5;
    regs[uint16_t(cybergear::Reg::LimitSpd)]=.05;
    regs[uint16_t(cybergear::Reg::SpdKp)]=4;
    regs[uint16_t(cybergear::Reg::SpdKi)]=.05;
  }
  bool start(std::string&) override { return true; }
  void stop() override {}
  void set_frame_callback(FrameCallback cb) override { callback=std::move(cb); }
  void feedback() {
    can::RawFrame f{}; f.id=cybergear::pack_ext_id(2,motor_id,0) | (enabled ? 2u<<22 : 0);
    f.dlc=8; f.rx_ns=now_monotonic_ns();
    auto u=cybergear::encode_u16(frozen_feedback ? 0 : q,-12.5,12.5);
    f.data[0]=u>>8; f.data[1]=u&255;
    f.data[2]=f.data[4]=0x80; f.data[7]=250;
    callback(f);
  }
  bool send(uint32_t id,const uint8_t data[8],std::string*) override {
    auto e=cybergear::unpack_ext_id(id);
    auto reg=uint16_t(data[0]) | (uint16_t(data[1])<<8);
    if (e.comm_type==4 && !ignore_stop) { if(enabled) q+=drift; enabled=false; }
    if (e.comm_type==3) { enabled=true; ++enables; }
    if (e.comm_type==18) {
      if (reg==uint16_t(cybergear::Reg::RunMode)) ++mode_writes;
      float value=0; std::memcpy(&value,data+4,4);
      regs[reg]=reg==uint16_t(cybergear::Reg::RunMode) ? data[4] : value;
    }
    feedback();
    if (e.comm_type==17) {
      if (reg==uint16_t(cybergear::Reg::MechPos)) {
        ++position_reads;
        if (silent_position) return true;
      }
      can::RawFrame r{}; r.id=cybergear::pack_ext_id(17,motor_id,0);
      r.dlc=8; r.rx_ns=now_monotonic_ns(); std::memcpy(r.data,data,8);
      float value=reg==uint16_t(cybergear::Reg::MechPos) ? q : regs[reg];
      std::memcpy(r.data+4,&value,4);
      if (reg==uint16_t(cybergear::Reg::RunMode)) r.data[4]=uint8_t(regs[reg]);
      callback(r);
    }
    return true;
  }
  can::BusStats stats() const override { return {}; }
  bool is_up() const override { return true; }
  can::CanIfState can_state() const override { return can::CanIfState::Unknown; }
  const char* kind() const override { return "offline-motor-replies"; }
  std::string device() const override { return "no-hardware"; }
};
bool run(AxisId axis, double drift, bool frozen, bool silent, bool position, bool ignore_stop=false) {
  auto transport=std::make_unique<MotorReplies>(); auto* motor=transport.get();
  motor->drift=drift; motor->frozen_feedback=frozen; motor->silent_position=silent;
  motor->motor_id=axis==AxisId::Pitch ? 100 : 101; motor->ignore_stop=ignore_stop;
  can::CyberGearSystem system; std::string error;
  if (!system.open({},error,std::move(transport))) return false;
  motor->feedback(); CanMotorBackend backend(system);
  auto status=MotorBackend::Transition::Pending;
  const auto begin=now_monotonic_ns();
  while(status==MotorBackend::Transition::Pending && now_monotonic_ns()-begin<3'000'000'000) {
    status=backend.transition_mode(axis,position,position ? .05 : 5,now_monotonic_ns(),error,.05,4);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  const bool expected_failure=drift!=0 || silent || ignore_stop;
  const bool ok=expected_failure
      ? status==MotorBackend::Transition::Failed && motor->enables==0 && (!ignore_stop || motor->mode_writes==0)
      : status==MotorBackend::Transition::Complete && motor->enables==1 && motor->position_reads>0;
  std::cout << "axis=" << axis_name(axis) << " ignore_stop=" << ignore_stop
            << " drift_deg=" << drift*kRad2Deg << " frozen_feedback=" << frozen
            << " silent_position=" << silent << " position_mode=" << position
            << " enable_frames=" << motor->enables << " position_reads=" << motor->position_reads
            << " error='" << error << "' pass=" << ok << '\n';
  system.close(); return ok;
}
int main() {
  bool ok=true;
  for (auto axis : {AxisId::Pitch,AxisId::Yaw}) for (bool position : {false,true}) {
    ok=run(axis,0,false,false,position) && ok;
    for (int dir : {-1,1}) {
      ok=run(axis,dir*2.9*kDeg2Rad,false,false,position) && ok;
      ok=run(axis,dir*2.9*kDeg2Rad,true,false,position) && ok;
    }
    ok=run(axis,0,true,true,position) && ok;
    ok=run(axis,0,false,false,position,true) && ok;
  }
  return ok ? 0 : 1;
}
