#include <cstring>
#include <iostream>
#ifdef OTA_COMMISSION_ACQUISITION
#include "session.hpp"
#endif
int main(int argc,char** argv) {
#ifdef OTA_COMMISSION_ACQUISITION
  if(argc==3 && std::strcmp(argv[1],"--control-yaw")==0)
    return ota::commission::yaw_control_session(argv[2]);
  if(argc==3 && std::strcmp(argv[1],"--establish-homing")==0)
    return ota::commission::sensorless_homing_session(argv[2]);
  if(argc==3 && std::strcmp(argv[1],"--validate-homing")==0)
    return ota::commission::validate_homing_manifest(argv[2]);
#else
  (void)argc; (void)argv;
#endif
  std::cerr<<"commissiond --control-yaw <manifest> (finite yaw position servo session; pitch disabled)\n"
           <<"--establish-homing <manifest> (bounded pitch sensorless homing, optional servo trial)\n"
           <<"--validate-homing <manifest> (no-I/O homing parameter validation)\n";
  return 2;
}
