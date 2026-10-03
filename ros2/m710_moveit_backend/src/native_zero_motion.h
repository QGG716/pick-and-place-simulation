#pragma once
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <cmath>
#include <string>
#include <vector>

namespace m710 {
// Validate a solver-returned stationary event without inventing a motion edge.
// Goal constraints, target ownership and Task lineage are checked by the caller.
inline std::string stationaryPointFailure(const trajectory_msgs::msg::JointTrajectory& trajectory,
                                          const std::vector<double>& start, bool goal_satisfied) {
  if(trajectory.points.size()!=1 || start.size()!=6) return "NOT_A_SINGLETON";
  const auto& p=trajectory.points.front();
  if(!goal_satisfied) return "SINGLETON_GOAL_NOT_SATISFIED";
  if(p.positions.size()!=start.size() || p.velocities.size()!=start.size() || p.accelerations.size()!=start.size())
    return "SINGLETON_INVALID_DIMENSIONS";
  if(p.time_from_start.sec<0 || p.time_from_start.nanosec>=1000000000) return "SINGLETON_INVALID_TIME";
  for(size_t i=0;i<start.size();++i) {
    if(!std::isfinite(p.positions[i]) || p.positions[i]!=start[i]) return "SINGLETON_START_CHANGED";
    if(!std::isfinite(p.velocities[i]) || !std::isfinite(p.accelerations[i]) || p.velocities[i]!=0. || p.accelerations[i]!=0.)
      return "SINGLETON_NOT_STATIONARY";
  }
  return {};
}
}
