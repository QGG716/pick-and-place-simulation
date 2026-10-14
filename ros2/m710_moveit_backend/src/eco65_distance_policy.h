#pragma once
#include <cmath>

namespace m710 {
// Arithmetic epsilon only; never changes the 2.4 mm ordinary gap or .4 mm
// named-interface penetration. Shared by the actual ECO65 process checker.
inline bool eco65DistanceAccepted(double distance,double threshold) {
  return std::isfinite(distance) && distance+1e-9>=threshold;
}
}
