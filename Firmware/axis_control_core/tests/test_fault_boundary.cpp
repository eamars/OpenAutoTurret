#include "axis_control_core.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace ota::axis;

namespace {
void expect(bool value,const char* message) {
  if(!value) throw std::runtime_error(message);
}
Parameters fixture(bool censored=false) {
  Parameters p{};
  p.model.q={-100.,-50.,0.,50.,100.};p.model.z={-.3,0.,.3};
  for(int k=0;k<3;++k) {p.model.theta[k]=.1;p.model.theta[3+k]=.06;}
  for(int k=0;k<15;++k) {
    p.model.theta[6+k]=-.12;p.model.theta[21+k]=.12;
    p.start_total[k]=-.16;p.start_total[15+k]=.16;
    p.start_censored[15+k]=censored;
  }
  p.model.theta[36]=.008;
  p.observer={4e-10,6.4e-9,.1,.03,.03,4e-10,6.4e-9,0};
  p.kp=p.ki=p.kpos=p.kaw=1.;p.current_cap=.9;p.slew=1.;p.integral_cap=.5;
  p.velocity_cap=1.;p.dt_min=.0001;p.dt_max=.03;p.intent_threshold=.00001;
  p.rest_speed=.001;p.sustained_s=.06;p.start_timeout_s=.15;
  return p;
}
Observation sample(double t,std::uint64_t sequence,std::uint64_t generation=1) {
  return {t,t,t,0.,0.,sequence,sequence,generation,1,1};
}
void inhibited(const Output& out) {
  expect(out.status==static_cast<int>(Status::HardAbort),"startup fault is not latched HardAbort");
  expect(out.requested==0. && out.limited==0. && out.sequence==0,"fault emitted an active command");
}
void timeout_latch() {
  Controller c;expect(c.configure(fixture()),"fixture configuration failed");
  expect(c.reset(0.,0.,0.,0.,1),"reset failed");
  std::uint64_t last=0;bool fault=false;
  for(std::uint64_t k=1;k<=60;++k) {
    const auto out=c.step(sample(k*.005,k),{0.,.1,0.,0.});
    if(out.status==0) {
      expect(!fault,"controller re-armed itself");last=out.sequence;
      expect(c.acknowledge(out.sequence,true,out.limited),"normal command ACK failed");
    } else {fault=true;inhibited(out);expect(!c.acknowledge(out.sequence,true,0.),"fault ACK accepted");}
  }
  expect(fault,"stalled start never timed out");
  expect(!c.reset(.3,0.,0.,0.,2,.4),"invalid reset accepted");
  inhibited(c.step(sample(.305,61,2),{0.,-.1,0.,0.}));
  expect(!c.switch_parameters(fixture(),{}),"parameter switch cleared startup fault");
  expect(c.reset(.3,0.,0.,0.,2),"explicit reset failed");
  auto out=c.step(sample(.305,1,2),{0.,-.1,0.,0.});
  expect(out.status==0 && out.sequence>last,"explicit reset failed to issue a fresh token");
}
void censored_start() {
  Controller c;expect(c.configure(fixture(true)),"fixture configuration failed");
  expect(c.reset(0.,0.,0.,0.,1),"reset failed");
  inhibited(c.step(sample(.005,1),{0.,.1,0.,0.}));
  inhibited(c.step(sample(.01,2),{0.,-.1,0.,0.}));
  expect(c.reset(.01,0.,0.,0.,2),"explicit reset failed");
  expect(c.step(sample(.015,1,2),{0.,-.1,0.,0.}).status==0,"uncensored direction refused after reset");
}
void old_ack_and_delayed_ack() {
  Controller c;expect(c.configure(fixture()),"fixture configuration failed");
  expect(c.reset(0.,0.,0.,0.,1),"reset failed");
  auto old=c.step(sample(.005,1),{0.,.1,0.,0.});
  expect(c.reset(.005,0.,0.,0.,2),"reset failed");
  auto next=c.step(sample(.01,1,2),{0.,-.1,0.,0.});
  expect(next.sequence>old.sequence,"reset reused command token");
  expect(!c.acknowledge_at(old.sequence,true,old.limited,.011),"pre-reset ACK accepted");
  expect(!c.acknowledge_at(next.sequence,true,next.limited,.009),"ACK before command accepted");
  expect(c.acknowledge_at(next.sequence,true,next.limited,.012),"valid delayed ACK refused");
  expect(!c.acknowledge_at(next.sequence,true,next.limited,.013),"duplicate ACK accepted");
  auto out=c.step(sample(.015,2,2),{0.,-.1,0.,0.});
  expect(out.status==0,"valid delayed ACK prevented subsequent command");
  expect(std::abs(out.limited-next.limited)<=.005+1e-12,"accepted current not retained for slew");
  expect(c.reset(.015,0.,0.,0.,2),"same-generation explicit reset failed");
  expect(c.step(sample(.02,1,2),{0.,.1,0.,0.}).sequence>out.sequence,"same generation reset reused token");
}
void failed_tx() {
  Controller c;expect(c.configure(fixture()),"fixture configuration failed");
  expect(c.reset(0.,0.,0.,0.,1),"reset failed");
  auto out=c.step(sample(.005,1),{0.,.1,0.,0.});
  expect(!c.acknowledge_at(out.sequence,false,out.limited,.006),"failed send accepted");
  auto inhibited_output=c.step(sample(.01,2),{0.,.1,0.,0.});
  expect(inhibited_output.status!=0 && inhibited_output.requested==0. && inhibited_output.limited==0. &&
         inhibited_output.sequence==0,"failed send permitted another command");
  expect(!c.acknowledge_at(out.sequence,true,out.limited,.011),"late success after failed send accepted");
}
}

int main() {
  try {timeout_latch();censored_start();old_ack_and_delayed_ack();failed_tx();}
  catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
  std::cout<<"4 native fault boundary scenarios passed; physical stopping remains unverified\n";
}
