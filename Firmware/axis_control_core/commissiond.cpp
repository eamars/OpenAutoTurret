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
  if(argc==3 && std::strcmp(argv[1],"--prepare-current")==0)
    return ota::commission::current_preparation_session(argv[2]);
  if(argc==3 && std::strcmp(argv[1],"--characterize-current")==0)
    return ota::commission::current_characterization_session(argv[2]);
  if(argc==3 && std::strcmp(argv[1],"--acquire-yaw")==0)
    return ota::commission::yaw_acquisition_session(argv[2]);
  if(argc==3 && std::strcmp(argv[1],"--control-yaw")==0)
    return ota::commission::yaw_control_session(argv[2]);
  if(argc==3 && std::strcmp(argv[1],"--establish-homing")==0)
    return ota::commission::sensorless_homing_session(argv[2]);
  if(argc==3 && std::strcmp(argv[1],"--validate-homing")==0)
    return ota::commission::validate_homing_manifest(argv[2]);
#endif
  std::cerr<<"commissiond --axis-core-replay <synthetic-file>\n"
           <<"Full firmware build: --capture-baseline <capture-manifest> (discovery, STOP, reads; no enable)\n"
           <<"--prepare-current <manifest> (neutral current-mode verification; no excitation)\n"
           <<"--characterize-current <manifest> (neutral current measurement characterization)\n"
           <<"--acquire-yaw <manifest> (finite yaw current acquisition; pitch disabled)\n"
           <<"--control-yaw <manifest> (finite shared-core yaw 3a; pitch disabled)\n"
           <<"--establish-homing <manifest> (bounded pitch sensorless homing)\n"
           <<"--validate-homing <manifest> (no-I/O homing parameter validation)\n";
  return 2;
}
