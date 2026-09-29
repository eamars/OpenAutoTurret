#include "replay.hpp"
#include <cstring>
#include <iostream>
int main(int argc,char** argv) {
  if(argc==3 && std::strcmp(argv[1],"--axis-core-replay")==0)
    return ota::axis::replay_file(argv[2]);
  std::cerr<<"Stage 1 local mathematics only: commissiond --axis-core-replay <synthetic-file>\n";
  return 2;
}
