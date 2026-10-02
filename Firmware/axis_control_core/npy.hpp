#pragma once
#include <cstdio>
#include <stdexcept>
#include <string>
#include <vector>

namespace ota::axis {
// Write rows (each `columns` doubles) as a float64 .npy matrix, as the simulators' output.
inline void write_npy(const std::string& path,const std::vector<std::vector<double>>& rows,std::size_t columns) {
  std::string header="{'descr': '<f8', 'fortran_order': False, 'shape': ("+std::to_string(rows.size())+", "+
                     std::to_string(columns)+"), }";
  while ((10+header.size()+1)%64) header+=' ';
  header+='\n';
  std::FILE* out=std::fopen(path.c_str(),"wb");
  if (!out) throw std::runtime_error("cannot write "+path);
  const unsigned char magic[]={0x93,'N','U','M','P','Y',1,0};
  std::fwrite(magic,1,8,out);
  const unsigned short length=static_cast<unsigned short>(header.size());
  std::fwrite(&length,2,1,out); std::fwrite(header.data(),1,header.size(),out);
  for (const auto& row:rows) {
    if (row.size()!=columns) { std::fclose(out); throw std::runtime_error("npy row width"); }
    std::fwrite(row.data(),sizeof(double),row.size(),out);
  }
  if (std::fclose(out)) throw std::runtime_error("write failed: "+path);
}
}
