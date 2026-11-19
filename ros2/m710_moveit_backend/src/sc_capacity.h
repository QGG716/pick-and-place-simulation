#pragma once
#include <cmath>
namespace m710 {
inline int singleCartonCapacityTier(double margin) {
  if(!std::isfinite(margin) || margin<1.)return -1;
  return margin>=2.?2:margin>=1.5?1:0;
}
}
