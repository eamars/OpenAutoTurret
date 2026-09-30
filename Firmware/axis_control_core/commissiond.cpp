#include "replay.hpp"
#include <cstring>
#include <iostream>
#ifdef OTA_COMMISSION_ACQUISITION
#include "session.hpp"
#endif
int main(int argc,char** argv) {
  if(argc==3 && std::strcmp(argv[1],"--axis-core-replay")==0)
    return ota::axis::replay_file(argv[2]);
#ifdef OTA_COMMISSION_ACQUISITION
  if(argc==3 && std::strcmp(argv[1],"--capture-baseline")==0)
    return ota::commission::capture_session(argv[2]);
#endif
  std::cerr<<"commissiond --axis-core-replay <synthetic-file>\n"
           <<"Full firmware build: --capture-baseline <capture-manifest> (discovery, STOP, reads; no enable)\n";
  return 2;
}
