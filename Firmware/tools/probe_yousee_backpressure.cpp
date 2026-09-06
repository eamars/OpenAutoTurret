// A real OS TTY queue, deliberately never drained. This used to block send().
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include "can/yousee_transport.hpp"
int main() {
  const int master = posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (master < 0 || grantpt(master) || unlockpt(master)) return 2;
  ota::can::YouseeTransport::Options options;
  options.port = ptsname(master);
  options.skip_at_init = true;
  ota::can::YouseeTransport transport(options);
  std::string error;
  if (!transport.start(error)) return 2;
  uint8_t data[8]{};
  bool bounded_failure = false;
  double max_ms = 0;
  for (int i=0; i<10000; ++i) {
    const auto start = ota::now_monotonic_ns();
    const bool sent = transport.send(0x18000064, data, &error);
    const double ms = (ota::now_monotonic_ns()-start)*1e-6;
    max_ms = std::max(max_ms, ms);
    if (!sent) { bounded_failure = ms < 20 && transport.stats().tx_failed > 0; break; }
  }
  transport.stop();
  close(master);
  std::cout << "{\"bounded_tx_failure\":" << bounded_failure << ",\"max_send_ms\":" << max_ms << "}\n";
  return bounded_failure ? 0 : 1;
}
