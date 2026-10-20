#pragma once
#include <cmath>
#include <algorithm>
#include <stdexcept>
namespace m710 {
constexpr double kJ1DirectionEpsilon=1e-7; // numerical FK/IK noise, radians, no wrapping
inline bool directedJ1(double from,double to,int direction) {
  return std::isfinite(from)&&std::isfinite(to)&&(!direction||direction*(to-from)>=-kJ1DirectionEpsilon);
}
inline double j1ReverseTravel(double from,double to,int direction) {
  if(!std::isfinite(from)||!std::isfinite(to)||direction<-1||direction>1)
    throw std::runtime_error("INVALID_J1_REVERSE_INPUT");
  return direction?std::max(0.,-direction*(to-from)):0.;
}
}
