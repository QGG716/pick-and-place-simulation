#pragma once
#include "process_geometry.h"
#include <fcl/fcl.h>

namespace m710 {
// Exact occupied volume of one horizontal box translated only along world Z.
// This is NOT a conservative robot envelope or an assumed lift mechanism.
inline ProcessJson receiverSweep(const ProcessJson& before,const ProcessJson& after) {
  auto a=processBox(before),b=processBox(after);
  auto unchanged=after;unchanged["pose"]=before.at("pose");
  if(unchanged!=before || a.id!="conveyor_transverse" ||
     (a.pose.linear()-b.pose.linear()).norm()>1e-12 ||
     (a.pose.linear().col(2)-Eigen::Vector3d::UnitZ()).norm()>1e-12 ||
     (a.pose.translation()-b.pose.translation()).head<2>().norm()>1e-12 ||
     b.pose.translation().z()<=a.pose.translation().z())
    throw std::runtime_error("RECEIVER_Z_TRANSITION_INVALID");
  const double dz=b.pose.translation().z()-a.pose.translation().z();
  auto sweep=before;sweep["size"][2]=2*a.half.z()+dz;
  sweep["pose"][2][3]=a.pose.translation().z()+dz/2;
  return sweep;
}
inline ProcessJson receiverTransitionWorld(const ProcessJson& request,const ProcessJson& binding) {
  if(binding.is_null() || !request.value("planning_only",false) ||
     request.value("planning_mode","")!="cold_from_scratch" ||
     !request.at("attachment").is_null() || request.value("parent_stage_id","").empty() ||
     request.at("q_start")!=request.at("q_goal") || request.contains("goal_pose") ||
     request.at("process_policy").at("target_id")!=binding.at("target_id") ||
     (request.at("stage")!="pregrasp" && request.at("stage")!="contact") ||
     request.at("pipeline_id")!="external_axis" || request.at("planner_id")!="LIFT_Z" ||
     !request.at("ownership_transition").is_null() ||
     request.value("receiver_end_fingerprint","").empty() ||
     request.at("receiver_end_fingerprint")==request.at("scene_fingerprint"))
    throw std::runtime_error("RECEIVER_TRANSITION_SCOPE_INVALID");
  const auto& end=request.at("receiver_end_world");auto expected=request.at("world");
  size_t count=0;
  for(auto& box:expected)if(box.at("id")=="conveyor_transverse") {
    if(box!=binding.at("low"))throw std::runtime_error("RECEIVER_START_CONTEXT_CHANGED");
    box=binding.at("high");++count;
  }
  if(count!=1 || expected!=end)throw std::runtime_error("RECEIVER_OTHER_WORLD_CHANGED");
  const auto target=processBox(request.at("process_policy").at("target"));
  const auto high=processBox(binding.at("high"));
  const double bottom=target.pose.translation().z()-target.pose.linear().row(2).cwiseAbs().dot(target.half);
  if(std::abs(bottom-.05-high.pose.translation().z()-high.half.z())>1e-7)
    throw std::runtime_error("RECEIVER_HMAX_TARGET_MISMATCH");
  receiverSweep(binding.at("low"),binding.at("high"));
  return expected;
}
inline double receiverBoxDistance(const ProcessJson& first,const ProcessJson& second) {
  const auto a=processBox(first),b=processBox(second);
  fcl::Boxd ga(2*a.half.x(),2*a.half.y(),2*a.half.z()),gb(2*b.half.x(),2*b.half.y(),2*b.half.z());
  fcl::DistanceRequestd req;req.gjk_solver_type=fcl::GST_LIBCCD;
  fcl::DistanceResultd result; // untruncated result, including first qualification cost
  const double value=fcl::distance(&ga,a.pose,&gb,b.pose,req,result);
  if(!std::isfinite(value) || !std::isfinite(result.min_distance))
    throw std::runtime_error("RECEIVER_DISTANCE_UNKNOWN");
  return std::min(value,result.min_distance);
}
}
