#pragma once
#include "nominal_release.h"
namespace m710 {
inline void validateWaitOutfeed(const ProcessJson& req) {
  const auto& path=req.at("outfeed");const auto& samples=path.at("samples");
  if(path.at("schema")!="fixed40_wait_outfeed_v1" ||
      (path.at("tool_timing")!="WAIT_AT_RELEASE_UNTIL_FULL_ENVELOPE_OUTFED_THEN_WITHDRAW" &&
       path.at("tool_timing")!="WITHDRAW_THEN_WAIT_FULL_OUTFEED" &&
       path.at("tool_timing")!="WITHDRAW_PARK_THEN_WAIT_FULL_OUTFEED") ||
      path.at("continuous_swept_proof")!=false || path.at("output_plane_x_m")!=-3.2 ||
      !(path.at("maximum_displacement_m").get<double>()>0) || path.at("maximum_displacement_m").get<double>()>.005 ||
      !(path.at("speed_m_s").get<double>()>0) || !std::isfinite(path.at("speed_m_s").get<double>()) || samples.size()<2 || samples.size()>4096 ||
      !req.at("attachment").is_null() || req.at("q_start")!=req.at("q_goal"))
    throw std::runtime_error("WAIT_OUTFEED_CONTRACT_INVALID");
  auto previous=processBox(samples.front().at("box"));const auto original=processBox(req.at("process_policy").at("target"));
  if(previous.id!=original.id || previous.half!=original.half ||
      (previous.pose.matrix()-original.pose.matrix()).cwiseAbs().maxCoeff()>1e-9 || samples.front().at("t")!=0.)
    throw std::runtime_error("OUTFEED_INITIAL_POSE_CHANGED");
  ProcessBox belt,longitudinal;bool has_belt=false,has_long=false;
  for(const auto& b:req.at("world")) {
    if(b.at("id")=="conveyor_transverse"){belt=processBox(b);has_belt=true;}
    if(b.at("id")=="conveyor_longitudinal"){longitudinal=processBox(b);has_long=true;}
  }
  if(!has_belt || !has_long)throw std::runtime_error("OUTFEED_RECEIVERS_MISSING");
  const double seam=belt.pose.translation().x()-belt.half.x();
  const double height=belt.pose.translation().z()+belt.half.z();
  const double slope=longitudinal.pose.linear()(2,0)/longitudinal.pose.linear()(0,0);
  const double long_min_x=longitudinal.pose.translation().x()-longitudinal.pose.linear().row(0).cwiseAbs().dot(longitudinal.half);
  bool x_started=false;double last_t=0.;
  for(size_t i=1;i<samples.size();++i) {
    const auto b=processBox(samples[i].at("box"));const double t=samples[i].at("t");
    const auto delta=b.pose.translation()-previous.pose.translation();
    const auto extent=b.pose.linear().cwiseAbs()*b.half;
    if(b.id!=original.id || b.half!=original.half || (b.pose.linear()-original.pose.linear()).norm()>1e-12 ||
        !std::isfinite(t) || t<=last_t || delta.norm()>.005000001 || delta.x()>1e-10 || delta.y()>1e-10 ||
        (std::abs(delta.x())>1e-10 && std::abs(delta.y())>1e-10) || (x_started && std::abs(delta.y())>1e-10) ||
        std::abs(delta.norm()-(t-last_t)*path.at("speed_m_s").get<double>())>1e-8)
      throw std::runtime_error("OUTFEED_SAMPLE_OR_TIMING_CHANGED");
    if(delta.x()<0)x_started=true;
    double support=height;
    if(b.pose.translation().x()-extent.x()<seam) {
      const double x=slope>=0?std::min(b.pose.translation().x()+extent.x(),seam):std::max(b.pose.translation().x()-extent.x(),long_min_x);
      support=height+(x-seam)*slope;
      if(b.pose.translation().x()+extent.x()>=seam)support=std::max(support,height);
    }
    if(std::abs(b.pose.translation().z()-extent.z()-support)>1e-8)
      throw std::runtime_error("OUTFEED_SUPPORT_ENVELOPE_CHANGED");
    previous=b;last_t=t;
  }
  if(previous.pose.translation().x()+previous.pose.linear().row(0).cwiseAbs().dot(previous.half)>=-3.2)
    throw std::runtime_error("OUTFEED_FULL_ENVELOPE_NOT_OUTSIDE");
}
inline void validateOutfeedPark(const ProcessJson& req,const ProcessJson& previous,const ProcessJson& drop,const ProcessJson& binding) {
  const auto pipeline=req.value("pipeline_id","");const auto planner=req.value("planner_id","");
  if(req.value("purpose","")!="FREE_UNLOADED_REPOSITION" || req.value("stage","")!="residence" ||
      !req.value("planning_only",false) || req.value("planning_mode","")!="cold_from_scratch" ||
      !req.at("attachment").is_null() || req.contains("goal_pose") || !req.contains("q_goal") ||
      !req.at("process_policy").at("support_names").empty() || !req.at("process_policy").at("initial_proximity").is_null() ||
      !binding.contains("outfeed_park_q_rad") || req.at("q_goal")!=binding.at("outfeed_park_q_rad") ||
      binding.at("joint_search_layout").at("schema")!="fixed40_joint_search_20261028_v1" ||
      !((pipeline=="pilz_industrial_motion_planner" && planner=="PTP") || (pipeline=="ompl" && planner=="RRTConnectkConfigDefault")) ||
      previous.value("stage","")!="withdrawal" || previous.value("planner_id","")!="LIN" ||
      previous.value("pipeline_id","")!="pilz_industrial_motion_planner" || !previous.at("attachment").is_null() ||
      req.at("parent_stage_id")!=previous.at("stage_id") ||
      drop.is_null() || drop.value("planner_id","")!="NOMINAL_DROP_RELEASE" ||
      previous.at("parent_stage_id")!=drop.at("stage_id"))
    throw std::runtime_error("OUTFEED_PARK_SCOPE_INVALID");
}
inline void validateWaitOutfeedParent(const ProcessJson& req,const ProcessJson& previous,const ProcessJson& grandparent,const ProcessJson& greatgrandparent=nullptr) {
  const bool park=req.at("outfeed").at("tool_timing")=="WITHDRAW_PARK_THEN_WAIT_FULL_OUTFEED";
  const bool first=park || req.at("outfeed").at("tool_timing")=="WITHDRAW_THEN_WAIT_FULL_OUTFEED";
  const auto& withdrawal=park?grandparent:previous;
  const auto& drop=park?greatgrandparent:(first?grandparent:previous);
  if(drop.is_null() || drop.value("planner_id","")!="NOMINAL_DROP_RELEASE" ||
      drop.value("pipeline_id","")!="semantic_event" || drop.value("stage","")!="extraction")
    throw std::runtime_error("OUTFEED_DROP_PARENT_REQUIRED");
  if(first && (withdrawal.value("stage","")!="withdrawal" || withdrawal.value("planner_id","")!="LIN" ||
      withdrawal.value("pipeline_id","")!="pilz_industrial_motion_planner" || !withdrawal.at("attachment").is_null() ||
      withdrawal.at("parent_stage_id")!=drop.at("stage_id")))
    throw std::runtime_error("OUTFEED_CHECKED_STATIONARY_CARTON_WITHDRAWAL_REQUIRED");
  if(park && (previous.value("stage","")!="residence" || previous.value("purpose","")!="FREE_UNLOADED_REPOSITION" ||
      !previous.at("attachment").is_null() || previous.at("parent_stage_id")!=withdrawal.at("stage_id")))
    throw std::runtime_error("OUTFEED_CHECKED_PARK_PARENT_REQUIRED");
}
inline ProcessJson outfeedWorldFailure(const ProcessJson& target,const ProcessJson& world) {
  const auto box=processBox(target);
  for(const auto& other:world) {
    if(other.at("id")==box.id)continue;
    const auto b=processBox(other);const double sat=processSignedGap(box,b);
    if((b.id=="conveyor_transverse" || b.id=="conveyor_longitudinal") && sat>=-1e-9)continue;
    if(sat>.005000001)continue;
    const double gap=receiverBoxDistance(target,other);
    if(gap+1e-9<.005)return {{"reason","OUTFEED_WORLD_CLEARANCE"},{"pair",{box.id,b.id}},{"gap_m",gap}};
  }
  return nullptr;
}
}
