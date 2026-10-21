#include <pilz_industrial_motion_planner/trajectory_functions.h>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace pilz_industrial_motion_planner;
void require(bool v,const char* why) { if (!v) throw std::runtime_error(why); }
int main() {
  JointLimitsContainer limits;
  JointLimit b;b.has_velocity_limits=true;b.max_velocity=4.53786;
  b.has_acceleration_limits=true;b.max_acceleration=.5;
  b.has_deceleration_limits=true;b.max_deceleration=-.5;
  b.has_position_limits=true;b.min_position=-8.;b.max_position=8.;
  for (const char* j:{"J1","J2","J3","J4","J5","J6"}) require(limits.addLimit(j,b),"ADD_LIMIT");
  std::map<std::string,double> zero,fast,position;
  for (const auto& entry:limits) {zero[entry.first]=0.;fast[entry.first]=31.3;position[entry.first]=3.13;}
  unsetenv("M710_PLANNING_ONLY_ALL_JOINT_RATES_EXCEPTION");
  require(!verifySampleJointLimits(zero,zero,position,.1,.1,limits),"DEFAULT_RETAINS_LIMITS");
  setenv("M710_PLANNING_ONLY_ALL_JOINT_RATES_EXCEPTION","carton_l04_c03",1);
  require(!verifySampleJointLimits(zero,zero,position,.1,.1,limits),"OTHER_TARGET_RETAINS_LIMITS");
  setenv("M710_PLANNING_ONLY_ALL_JOINT_RATES_EXCEPTION","carton_l05_c03",1);
  require(verifySampleJointLimits(zero,zero,position,.1,.1,limits),"ALL_JOINT_VELOCITY_AND_ACCELERATION_EXCEPTION");
  require(verifySampleJointLimits(zero,fast,zero,.1,.1,limits),"ALL_JOINT_DECELERATION_EXCEPTION");
  require(!verifySampleJointLimits(zero,zero,position,.1,0.,limits),"DURATION_RETAINED");
  auto invalid=position;invalid["J4"]=std::numeric_limits<double>::quiet_NaN();
  require(!verifySampleJointLimits(zero,zero,invalid,.1,.1,limits),"FINITE_REQUIRED");
  for (const auto& entry:limits) {
    require(!limits.verifyPositionLimit(entry.first,9.),"POSITION_LIMIT_UNCHANGED");
    require(!limits.verifyVelocityLimit(entry.first,31.3),"ORIGINAL_CONTAINER_UNCHANGED");
    require(limits.getLimit(entry.first).has_acceleration_limits,"ORIGINAL_ACCELERATION_METADATA_UNCHANGED");
  }
  std::cout<<"PASS: explicit target, six-joint rate exception, position, finite, duration and original metadata\n";
}
