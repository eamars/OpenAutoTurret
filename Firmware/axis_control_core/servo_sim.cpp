// servo-sim REQUEST.json OUT.npy -- closed-loop session simulation (simulate.hpp).
// Writes the rows as a float64 .npy matrix and one JSON status line on stdout.
#include "npy.hpp"
#include "simulate.hpp"
#include <iostream>
#include <string>

int main(int argc,char** argv) {
  if (argc!=3) { std::cerr<<"servo-sim REQUEST.json OUT.npy\n"; return 2; }
  try {
    const auto result=ota::axis::simulate(YAML::LoadFile(argv[1]));
    ota::axis::write_npy(argv[2],result.rows,result.columns.size());
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
