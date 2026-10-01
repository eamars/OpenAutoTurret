// Local executable boundary probe: real Linux recvmsg timestamps and real disk
// recording, with CAN payloads transported only over loopback UDP. No motor I/O.
#include "capture.hpp"
#include <arpa/inet.h>
#include <chrono>
#include <iostream>
#include <linux/can.h>
#include <stdexcept>
#include <unistd.h>

int main(int argc, char** argv) {
  if (argc != 2) return 2;
  int rx = -1, tx = -1;
  try {
    rx = socket(AF_INET, SOCK_DGRAM|SOCK_CLOEXEC, 0);
    tx = socket(AF_INET, SOCK_DGRAM|SOCK_CLOEXEC, 0);
    sockaddr_in addr{}; addr.sin_family = AF_INET; addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (rx < 0 || tx < 0 || bind(rx, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)))
      throw std::runtime_error("loopback socket failed");
    socklen_t len = sizeof(addr); getsockname(rx, reinterpret_cast<sockaddr*>(&addr), &len);
    ota::commission::TimestampedReceiver receiver(rx);
    ota::commission::Journal journal(argv[1], "{\"kind\":\"header\",\"provenance\":\"SYNTHETIC\",\"transport\":\"loopback_udp\"}");
    for (unsigned n = 0; n<2000; ++n) {
      can_frame f{}; f.can_id = 0x205; f.can_dlc = 8;
      f.data[0] = (n%8192)>>8; f.data[1] = n; f.data[6] = 37;
      if (sendto(tx, &f, sizeof(f), 0, reinterpret_cast<sockaddr*>(&addr), len) != sizeof(f))
        throw std::runtime_error("synthetic datagram TX failed");
      // Deliberately defer userspace RX. The timestamp must describe receipt,
      // not the later dequeue; this is the acquisition design hypothesis.
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      ota::commission::Receipt r;
      if (!receiver.receive(r) || r.drop_delta || r.frame.data[1] != uint8_t(n) ||
          r.dequeue_ns-r.kernel_monotonic_ns < 500000 ||
          !journal.append(ota::commission::receipt_json(r, "yaw", n+1)))
        throw std::runtime_error("timestamp/recording probe failed");
    }
    if (!journal.finish("{\"kind\":\"footer\",\"status\":\"COMPLETE\",\"frames\":2000}"))
      throw std::runtime_error("capture flush failed");
    close(rx); close(tx);
    std::cout << "PASS: 2000 kernel-timestamped synthetic frames durably recorded; no device access\n";
    return 0;
  } catch (const std::exception& e) {
    if (rx>=0) close(rx); if (tx>=0) close(tx);
    std::cerr << e.what() << '\n'; return 1;
  }
}
