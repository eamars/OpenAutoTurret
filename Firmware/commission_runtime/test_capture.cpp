#include "capture.hpp"
#include "readback.hpp"
#include <arpa/inet.h>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <linux/can.h>
#include <poll.h>
#include <sys/resource.h>
#include <sys/wait.h>
#include <unistd.h>

using namespace ota::commission;
namespace {
struct Directory {
  std::string path;
  Directory() { char name[]="/tmp/ota-capture-test-XXXXXX"; auto* p=mkdtemp(name); if (!p) throw std::runtime_error("mkdtemp"); path=p; }
  ~Directory() { std::filesystem::remove_all(path); }
};
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
  EXPECT_DOUBLE_EQ(result.value,.25); EXPECT_EQ(result.request_sequence,1);
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
  TimestampedReceiver receiver(rx,1000000);
  can_frame frame{}; frame.can_id=0; frame.can_dlc=8;
  for (int i=0;i<1000;++i) ASSERT_EQ(sendto(tx,&frame,sizeof(frame),0,reinterpret_cast<sockaddr*>(&addr),len),sizeof(frame));
  Receipt r; uint64_t observed_drops=0;
  while (receiver.receive(r)) observed_drops+=r.drop_delta;
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
  TimestampedReceiver receiver(rx,1000000);
  EXPECT_EQ(receiver.kernel_drops(),0u);
  can_frame frame{}; frame.can_dlc=8;
  for (int i=0;i<1000;++i) ASSERT_EQ(sendto(tx,&frame,sizeof(frame),0,reinterpret_cast<sockaddr*>(&addr),len),sizeof(frame));
  // No receive() call and no subsequent trigger packet: loss is observable now.
  EXPECT_GT(receiver.kernel_drops(),0u);
  close(rx); close(tx);
}
