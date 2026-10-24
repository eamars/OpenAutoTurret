#include "capture.hpp"
#include "readback.hpp"
#include <arpa/inet.h>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <linux/can.h>
#include <linux/sock_diag.h>
#include <poll.h>
#include <sys/resource.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

using namespace ota::commission;
namespace {
struct Directory {
  std::string path;
  Directory() { char name[]="/tmp/ota-capture-test-XXXXXX"; auto* p=mkdtemp(name); if (!p) throw std::runtime_error("mkdtemp"); path=p; }
  ~Directory() { std::filesystem::remove_all(path); }
};
struct Loopback {
  int rx{socket(AF_INET,SOCK_DGRAM,0)}, tx{socket(AF_INET,SOCK_DGRAM,0)};
  sockaddr_in addr{};
  Loopback() {
    addr.sin_family=AF_INET; addr.sin_addr.s_addr=htonl(INADDR_LOOPBACK); socklen_t len=sizeof(addr);
    if (rx<0 || tx<0 || bind(rx,reinterpret_cast<sockaddr*>(&addr),len) || getsockname(rx,reinterpret_cast<sockaddr*>(&addr),&len))
      throw std::runtime_error("loopback");
  }
  ~Loopback() { close(rx); close(tx); }
  bool send(canid_t id=0) {
    can_frame frame{}; frame.can_id=id; frame.can_dlc=8;
    return sendto(tx,&frame,sizeof(frame),0,reinterpret_cast<const sockaddr*>(&addr),sizeof(addr))==sizeof(frame);
  }
};
// Loopback delivery is not synchronous: sendto() can return before the datagram reaches the
// receiving socket (WSL2's 6.18 kernel, 2026-10-24: not one lone datagram in 20 was readable at
// once, and 853 of a 1000 burst had landed when the loop ended). Reading or counting straight
// after sending is reading a race, so the tests first wait, up to a second, until every datagram
// sent has landed: queued (receive-queue bytes over one frame's truesize) or dropped.
std::array<uint32_t,SK_MEMINFO_VARS> meminfo(int fd) {
  std::array<uint32_t,SK_MEMINFO_VARS> m{}; socklen_t size=sizeof(m);
  if (getsockopt(fd,SOL_SOCKET,SO_MEMINFO,m.data(),&size)) throw std::runtime_error("SO_MEMINFO");
  return m;
}
uint32_t frame_truesize() {
  static const uint32_t size=[] {
    Loopback probe; pollfd ready{probe.rx,POLLIN,0};
    if (!probe.send() || poll(&ready,1,1000)<=0) throw std::runtime_error("loopback probe");
    return meminfo(probe.rx)[SK_MEMINFO_RMEM_ALLOC];  // exactly one frame has landed
  }();
  return size;
}
bool landed(int rx, uint32_t sent) {
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(1);
  do {
    const auto m=meminfo(rx);
    if (m[SK_MEMINFO_RMEM_ALLOC]/frame_truesize()+m[SK_MEMINFO_DROPS]>=sent) return true;
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  } while (std::chrono::steady_clock::now()<deadline);
  return false;
}
Receipt reply(int64_t t, int type=17, int motor=127) {
  Receipt r; r.frame.id=ota::cybergear::pack_ext_id(type,motor,0);
  r.frame.data[0]=0x1a; r.frame.data[1]=0x70;
  const float current=.25f; std::memcpy(r.frame.data+4,&current,4);
  r.kernel_monotonic_ns=t; return r;
}
}
TEST(Readback, AcceptsOnlyCorrelatedReadAndRecordsUnknownDeviceTime) {
  Readback read(127,0);
  auto frame=read.begin(ota::cybergear::Reg::Iqf,100,50);
  EXPECT_EQ(ota::cybergear::unpack_ext_id(frame.id).comm_type,17);
  read.accepted(110,true);
  const auto result=read.observe(reply(105)); // response may queue before send returns
  ASSERT_TRUE(result.value); EXPECT_DOUBLE_EQ(*result.value,.25); EXPECT_EQ(result.request_sequence,1);
  EXPECT_NE(readback_json(result).find("\"device_sample_ns\":null"),std::string::npos);
  EXPECT_FALSE(read.pending());
}
TEST(Readback, RejectsEchoWrongSourceStaleOrWrongIndex) {
  for (int scenario=0;scenario<5;++scenario) {
    Readback read(127,0); read.begin(ota::cybergear::Reg::Iqf,100,50); read.accepted(101,true);
    auto r=reply(scenario==2?99:scenario==3?150:110,scenario==0?18:17,scenario==1?126:127);
    if (scenario==4) r.frame.data[0]=0x19;
    EXPECT_THROW(read.observe(r),std::runtime_error);
    EXPECT_THROW(read.begin(ota::cybergear::Reg::Iqf,200,50),std::runtime_error);
  }
}
TEST(Readback, FailedTransmitOrTimeoutCannotRetry) {
  Readback read(127,0); read.begin(ota::cybergear::Reg::Iqf,100,50);
  EXPECT_THROW(read.accepted(101,false),std::runtime_error);
  EXPECT_THROW(read.begin(ota::cybergear::Reg::Iqf,200,50),std::runtime_error);
  Readback timeout(127,0); timeout.begin(ota::cybergear::Reg::Iqf,100,50); timeout.accepted(101,true);
  EXPECT_THROW(timeout.check_deadline(150),std::runtime_error);
  EXPECT_THROW(timeout.observe(reply(151)),std::runtime_error);
}
TEST(Readback, NegativeReplyPreservesUnavailableValueAndDoesNotRetryThatRegister) {
  Readback read(127,0); read.begin(ota::cybergear::Reg::MechPos,100,50); read.accepted(101,true);
  auto r=reply(110);
  // Captured factory rejection: trailing bytes are stale UID data, not a float.
  r.frame.id=0x11017f00;
  const uint8_t data[]{0x19,0x70,0,0,0x30,0x33,0x31,0x05};
  std::memcpy(r.frame.data,data,8);
  const auto result=read.observe(r);
  EXPECT_FALSE(result.value); EXPECT_EQ(result.device_error_flag,1);
  EXPECT_NE(readback_json(result).find("\"value\":null"),std::string::npos);
  EXPECT_NE(readback_json(result).find("\"kind\":\"register_rejected\""),std::string::npos);
  EXPECT_FALSE(read.pending());
  EXPECT_THROW(read.begin(ota::cybergear::Reg::MechPos,200,50),std::runtime_error);
  EXPECT_NO_THROW(read.begin(ota::cybergear::Reg::Iqf,200,50));
  read.accepted(201,true);
  ASSERT_TRUE(read.observe(reply(210)).value);
}
TEST(Readback, NegativeReplyStillRequiresExactSourceIndexAndStatus) {
  for (int scenario=0;scenario<5;++scenario) {
    Readback read(127,0); read.begin(ota::cybergear::Reg::Iqf,100,50); read.accepted(101,true);
    auto r=reply(110); r.frame.id=0x11017f00;
    if (scenario==0) r.frame.id=0x11017e00;
    if (scenario==1) r.frame.data[0]=0x19;
    if (scenario==2) r.frame.id=0x11027f00;
    if (scenario==3) r.frame.data[2]=1;
    if (scenario==4) r.frame.id=0x11017f01;
    EXPECT_THROW(read.observe(r),std::runtime_error);
  }
}
TEST(Journal, PreservesEvidenceAndRejectsReopen) {
  Directory dir; const auto path=dir.path+"/capture";
  { Journal j(path,"header"); EXPECT_TRUE(j.append("sample")); EXPECT_TRUE(j.finish("footer")); EXPECT_EQ(j.written(),3); }
  EXPECT_THROW(Journal j(path,"replacement"),std::runtime_error);
  std::ifstream input(path); const std::string text((std::istreambuf_iterator<char>(input)),{});
  EXPECT_EQ(text,"header\nsample\nfooter\n");
}
TEST(Journal, IncompleteOrOversizedCaptureCannotGetSuccessFooter) {
  Directory dir; const auto path=dir.path+"/capture";
  { Journal j(path,"header"); EXPECT_FALSE(j.append(std::string(4096,'a'))); EXPECT_FALSE(j.finish("success")); }
  std::ifstream input(path); const std::string text((std::istreambuf_iterator<char>(input)),{});
  EXPECT_EQ(text,"header\n");
}
TEST(Journal, ActualFilesystemWriteFailureIsNotSuccess) {
  Directory dir;
  const auto child=fork(); ASSERT_GE(child,0);
  if (!child) {
    signal(SIGXFSZ,SIG_IGN);
    rlimit limit{1024,1024}; if (setrlimit(RLIMIT_FSIZE,&limit)) _exit(3);
    Journal j(dir.path+"/capture","header"); j.append(std::string(2048,'x'));
    _exit(j.finish("success") ? 4:0);
  }
  int status; ASSERT_EQ(waitpid(child,&status,0),child);
  ASSERT_TRUE(WIFEXITED(status)); EXPECT_EQ(WEXITSTATUS(status),0);
}
TEST(Receiver, KernelReportsRealDatagramOverflow) {
  const int rx=socket(AF_INET,SOCK_DGRAM,0),tx=socket(AF_INET,SOCK_DGRAM,0);
  ASSERT_GE(rx,0); ASSERT_GE(tx,0);
  sockaddr_in addr{}; addr.sin_family=AF_INET; addr.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
  int buffer=1024; ASSERT_EQ(setsockopt(rx,SOL_SOCKET,SO_RCVBUF,&buffer,sizeof(buffer)),0);
  ASSERT_EQ(bind(rx,reinterpret_cast<sockaddr*>(&addr),sizeof(addr)),0);
  socklen_t len=sizeof(addr); ASSERT_EQ(getsockname(rx,reinterpret_cast<sockaddr*>(&addr),&len),0);
  TimestampedReceiver receiver(rx);
  Receipt r; uint64_t observed_drops=0;
  can_frame frame{}; frame.can_id=0; frame.can_dlc=8;
  for (int i=0;i<1000;++i) ASSERT_EQ(sendto(tx,&frame,sizeof(frame),0,reinterpret_cast<sockaddr*>(&addr),len),sizeof(frame));
  ASSERT_TRUE(landed(rx,1000));
  auto receive=[&]() {
    const auto previous=r.kernel_monotonic_ns;
    try { return receiver.receive(r); }
    catch (const std::exception& error) {
      ADD_FAILURE()<<error.what()<<" kernel_ns="<<r.kernel_monotonic_ns<<" previous_ns="<<previous
                   <<" dequeue_ns="<<r.dequeue_ns<<" uncertainty_ns="<<r.clock_uncertainty_ns;
      throw;
    }
  };
  while (receive()) observed_drops+=r.drop_delta;
  // Linux attaches the accumulated loss counter to the next enqueued packet.
  ASSERT_EQ(sendto(tx,&frame,sizeof(frame),0,reinterpret_cast<sockaddr*>(&addr),len),sizeof(frame));
  pollfd ready{rx,POLLIN,0}; ASSERT_GT(poll(&ready,1,1000),0);
  ASSERT_TRUE(receiver.receive(r)); observed_drops+=r.drop_delta;
  EXPECT_GT(observed_drops,0); EXPECT_GT(r.socket_drops,0);
  close(rx); close(tx);
}
TEST(Receiver, FinalCounterDetectsLossBeforeAnyOverflowNotificationIsDequeued) {
  const int rx=socket(AF_INET,SOCK_DGRAM,0),tx=socket(AF_INET,SOCK_DGRAM,0);
  ASSERT_GE(rx,0); ASSERT_GE(tx,0);
  sockaddr_in addr{}; addr.sin_family=AF_INET; addr.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
  int buffer=1024; ASSERT_EQ(setsockopt(rx,SOL_SOCKET,SO_RCVBUF,&buffer,sizeof(buffer)),0);
  ASSERT_EQ(bind(rx,reinterpret_cast<sockaddr*>(&addr),sizeof(addr)),0);
  socklen_t len=sizeof(addr); ASSERT_EQ(getsockname(rx,reinterpret_cast<sockaddr*>(&addr),&len),0);
  TimestampedReceiver receiver(rx);
  EXPECT_EQ(receiver.kernel_drops(),0u);
  can_frame frame{}; frame.can_dlc=8;
  for (int i=0;i<1000;++i) ASSERT_EQ(sendto(tx,&frame,sizeof(frame),0,reinterpret_cast<sockaddr*>(&addr),len),sizeof(frame));
  // No receive() call and no subsequent trigger packet: once the burst has landed, loss is observable.
  ASSERT_TRUE(landed(rx,1000));
  EXPECT_GT(receiver.kernel_drops(),0u);
  close(rx); close(tx);
}
// Linux switches receive stamping on for the system's first timestamping socket through deferred
// work, and a datagram queued before that work runs is stamped when it is read. The receiver must
// return from construction only once datagrams are stamped as they queue, and must not deliver
// anything that was queued before then (station 2026-10-02: read-time stamps followed by earlier
// queue-time stamps aborted the overflow test as a backwards clock, 2 of 300 lone runs).
TEST(Receiver, StampsAtQueueTimeFromConstructionAndDiscardsWhatWasAlreadyQueued) {
  Loopback link;
  for (int i=0;i<5;++i) ASSERT_TRUE(link.send());
  ASSERT_TRUE(landed(link.rx,5)) << "queued before construction";
  TimestampedReceiver receiver(link.rx);
  EXPECT_EQ(receiver.startup_discarded(),5u);
  Receipt r; EXPECT_FALSE(receiver.receive(r));
  // Sent straight after construction, before any deferred work could have been waited for here;
  // read 2 ms after it queued, so a queue-time stamp is at least that old.
  ASSERT_TRUE(link.send());
  ASSERT_TRUE(landed(link.rx,1));
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  ASSERT_TRUE(receiver.receive(r));
  EXPECT_GE(r.dequeue_ns-r.kernel_monotonic_ns,1'000'000) << "stamped when read, not when queued";
  for (int i=0;i<50;++i) ASSERT_TRUE(link.send());
  ASSERT_TRUE(landed(link.rx,50));
  int received=0;
  while (receiver.receive(r)) ++received;
  EXPECT_EQ(received,50);
}
TEST(Receiver, ErrorFrameQueuedBeforeCaptureIsNotDiscardedSilently) {
  Loopback link;
  ASSERT_TRUE(link.send()); ASSERT_TRUE(link.send(CAN_ERR_FLAG));
  EXPECT_THROW(TimestampedReceiver receiver(link.rx),std::runtime_error);
}
