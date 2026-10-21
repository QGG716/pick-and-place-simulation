#include <pilz_industrial_motion_planner/trajectory_functions.h>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace pilz_industrial_motion_planner;
void require(bool value, const char* why) { if (!value) throw std::runtime_error(why); }
int main() {
  JointLimitsContainer limits;
  JointLimit bound;
  bound.has_velocity_limits = true; bound.max_velocity = 4.53786;
  bound.has_acceleration_limits = true; bound.max_acceleration = .5;
  bound.has_deceleration_limits = true; bound.max_deceleration = -.5;
  bound.has_position_limits = true; bound.min_position = -8.; bound.max_position = 8.;
  for (const char* name : {"J3", "J4", "J5"}) require(limits.addLimit(name,bound), "ADD_LIMIT");
  unsetenv("M710_PLANNING_ONLY_J4_VELOCITY_EXCEPTION");
  require(!limits.verifyVelocityLimit("J4",31.3), "DEFAULT_MUST_REJECT");
  setenv("M710_PLANNING_ONLY_J4_VELOCITY_EXCEPTION","carton_l04_c03",1);
  require(!limits.verifyVelocityLimit("J4",31.3), "OTHER_TARGET_MUST_REJECT");
  setenv("M710_PLANNING_ONLY_J4_VELOCITY_EXCEPTION","carton_l05_c03",1);
  require(limits.verifyVelocityLimit("J4",31.3), "J4_DIAGNOSTIC_MUST_BYPASS");
  require(!limits.verifyVelocityLimit("J3",31.3) && !limits.verifyVelocityLimit("J5",31.3), "OTHER_JOINT_MUST_REJECT");
  require(!limits.verifyVelocityLimit("J4",std::numeric_limits<double>::infinity()), "NONFINITE_MUST_REJECT");
  require(!limits.verifyPositionLimit("J4",9.), "POSITION_LIMIT_RETAINED");
  const std::map<std::string,double> p0{{"J4",0.}}, v0{{"J4",0.}}, p1{{"J4",3.13}};
  require(!verifySampleJointLimits(p0,v0,p1,.1,.1,limits), "ACCELERATION_MUST_REJECT_AFTER_VELOCITY_BYPASS");
  const std::map<std::string,double> fast{{"J4",31.3}};
  require(verifySampleJointLimits(p0,fast,p1,.1,.1,limits), "CONSTANT_SPEED_DIAGNOSTIC_SAMPLE");
  require(!verifySampleJointLimits(p0,fast,p0,.1,.1,limits), "DECELERATION_MUST_REJECT");
  std::cout << "PASS: default, target, joint, finite, position, acceleration and deceleration boundaries\n";
}
