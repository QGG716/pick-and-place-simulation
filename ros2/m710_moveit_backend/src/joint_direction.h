#pragma once
#include <cmath>
namespace m710 {
constexpr double kJ1DirectionEpsilon=1e-7; // numerical FK/IK noise, radians, no wrapping
inline bool directedJ1(double from,double to,int direction) {
  return std::isfinite(from)&&std::isfinite(to)&&(!direction||direction*(to-from)>=-kJ1DirectionEpsilon);
}
}
