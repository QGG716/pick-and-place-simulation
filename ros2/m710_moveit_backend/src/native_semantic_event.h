#pragma once
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <moveit/kinematic_constraints/utils.h>
#include <nlohmann/json.hpp>
#include <string>

namespace m710 {
// These are the existing Pilz Cartesian goal tolerances, not new allowances.
constexpr double kNativeGoalPositionToleranceM=1e-6;
constexpr double kNativeGoalOrientationToleranceRad=1e-6;

inline bool semanticPlaceScope(bool planning_only,const std::string& mode,
    const nlohmann::json& req,bool attached_target,bool target_in_world) {
  return planning_only && mode=="static_prior_fast" && req.at("stage")=="place" &&
    req.at("pipeline_id")=="pilz_industrial_motion_planner" && req.at("planner_id")=="LIN" &&
    req.contains("goal_pose") && !req.at("attachment").is_null() &&
    req.at("attachment").at("id")==req.at("clearance_policy").at("target_id") &&
    !req.at("parent_stage_id").get<std::string>().empty() && attached_target && !target_in_world;
}

inline bool placeGoalSatisfied(const planning_scene::PlanningSceneConstPtr& scene,
    const std::string& tcp,const geometry_msgs::msg::Pose& pose) {
  if(!scene->getRobotModel()->getLinkModel(tcp)) return false;
  geometry_msgs::msg::PoseStamped target;target.header.frame_id=scene->getPlanningFrame();target.pose=pose;
  const auto constraints=kinematic_constraints::constructGoalConstraints(
    tcp,target,kNativeGoalPositionToleranceM,kNativeGoalOrientationToleranceRad);
  kinematic_constraints::KinematicConstraintSet goal(scene->getRobotModel());
  return goal.add(constraints,scene->getTransforms()) && goal.decide(scene->getCurrentState()).satisfied;
}
}
