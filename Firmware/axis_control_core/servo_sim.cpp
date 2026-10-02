// servo-sim REQUEST.json OUT.npy -- closed-loop session simulation (simulate.hpp).
// Writes the rows as a float64 .npy matrix and one JSON status line on stdout.
#include "simulate.hpp"
#include <cstdio>
#include <iostream>
#include <string>

int main(int argc,char** argv) {
  if (argc!=3) { std::cerr<<"servo-sim REQUEST.json OUT.npy\n"; return 2; }
  try {
    const auto result=ota::axis::simulate(YAML::LoadFile(argv[1]));
    std::string header="{'descr': '<f8', 'fortran_order': False, 'shape': ("+std::to_string(result.rows.size())+", "+
                       std::to_string(result.columns.size())+"), }";
    while ((10+header.size()+1)%64) header+=' ';
    header+='\n';
    std::FILE* out=std::fopen(argv[2],"wb");
    if (!out) throw std::runtime_error("cannot write output");
    const unsigned char magic[]={0x93,'N','U','M','P','Y',1,0};
    std::fwrite(magic,1,8,out);
    const unsigned short length=static_cast<unsigned short>(header.size());
    std::fwrite(&length,2,1,out); std::fwrite(header.data(),1,header.size(),out);
    for (const auto& row:result.rows) std::fwrite(row.data(),sizeof(double),row.size(),out);
    if (std::fclose(out)) throw std::runtime_error("output write failed");
    std::cout<<"{\"status\":\""<<result.status<<"\",\"rows\":"<<result.rows.size()<<",\"columns\":[";
    for (std::size_t k=0;k<result.columns.size();++k) std::cout<<(k?",":"")<<'"'<<result.columns[k]<<'"';
    std::cout<<"]";
    if (!result.learned.empty()) std::cout<<",\"learned\":"<<result.learned;
    std::cout<<"}\n";
    return 0;
  } catch (const std::exception& e) {
    std::cout<<"{\"status\":\"INVALID\",\"detail\":\""<<e.what()<<"\"}\n"; return 1;
  }
}
