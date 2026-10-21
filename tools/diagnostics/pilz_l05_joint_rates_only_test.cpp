#include <pilz_industrial_motion_planner/trajectory_functions.h>
#include <dlfcn.h>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <moveit/robot_state/robot_state.h>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
using namespace pilz_industrial_motion_planner;
void require(bool v,const char* why) { if (!v) throw std::runtime_error(why); }
int main() {
  // Also run this test linked only to the diagnostic library. Load the real
  // Pilz implementation locally, matching the worker's plugin visibility.
  require(dlopen("libtrajectory_generation_common.so",RTLD_NOW|RTLD_LOCAL)!=nullptr,"LOCAL_PILZ_LOAD");
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
  auto urdf=urdf::parseURDF(R"(<robot name="bounds"><link name="base"/><link name="tip"/>
    <joint name="J4" type="revolute"><parent link="base"/><child link="tip"/><axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/></joint></robot>)");
  auto srdf=std::make_shared<srdf::Model>();
  require(urdf && srdf->initString(*urdf,R"(<robot name="bounds"><group name="arm"><joint name="J4"/></group></robot>)"),"STATE_MODEL");
  auto model=std::make_shared<moveit::core::RobotModel>(urdf,srdf);
  moveit::core::RobotState state(model);state.setToDefaultValues();
  state.setVariableVelocity("J4",100.);const auto* group=model->getJointModelGroup("arm");
  unsetenv("M710_PLANNING_ONLY_ALL_JOINT_RATES_EXCEPTION");
  require(!state.satisfiesBounds() && !state.satisfiesBounds(group),"DEFAULT_STATE_VELOCITY_RETAINED");
  setenv("M710_PLANNING_ONLY_ALL_JOINT_RATES_EXCEPTION","carton_l04_c03",1);
  require(!state.satisfiesBounds() && !state.satisfiesBounds(group),"OTHER_TARGET_STATE_VELOCITY_RETAINED");
  setenv("M710_PLANNING_ONLY_ALL_JOINT_RATES_EXCEPTION","carton_l05_c03",1);
  require(state.satisfiesBounds() && state.satisfiesBounds(group),"SCOPED_STATE_VELOCITY_EXCEPTION");
  require(state.hasVelocities() && state.getVariableVelocity("J4")==100.,"ORIGINAL_STATE_VELOCITY_UNCHANGED");
  state.setVariablePosition("J4",2.);
  require(!state.satisfiesBounds() && !state.satisfiesBounds(group),"STATE_POSITION_LIMIT_RETAINED");
  state.setVariablePosition("J4",0.);state.setVariableVelocity("J4",std::numeric_limits<double>::quiet_NaN());
  require(!state.satisfiesBounds() && !state.satisfiesBounds(group),"STATE_FINITE_RETAINED");
  std::cout<<"PASS: explicit target, six-joint rate exception, original state position predicate, finite, duration and unchanged trajectory/metadata\n";
}
