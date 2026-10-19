#pragma once
#include "lifted_receiver.h"
namespace m710 {
inline bool coupledBinding(const ProcessJson& binding) {
  return binding.is_object() && binding.value("schema","")=="m710_coupled_receiver_v2";
}
inline void validateCoupledBinding(const ProcessJson& b) {
  if(!coupledBinding(b) || b.at("samples").empty() || b.at("samples").size()>4096 ||
      b.at("continuous_swept_proof")!=false || b.at("maximum_height_step_m").get<double>()>.010000000001)
    throw std::runtime_error("COUPLED_BINDING_INVALID");
  const auto& first=b.at("samples").front();
  std::map<std::string,ProcessJson> original;
  for(const auto& box:first.at("world")) {processBox(box);original[box.at("id")]=box;}
  const auto moving=b.at("moving_ids").get<std::set<std::string>>();
  double previous=first.at("height_m");
  for(const auto& sample:b.at("samples")) {
    const double h=sample.at("height_m");
    if(!std::isfinite(h) || std::abs(h-previous)>.010000000001 || sample.at("world").size()!=original.size())
      throw std::runtime_error("COUPLED_SAMPLE_COVERAGE_INVALID");
    previous=h;std::set<std::string> seen;
    for(const auto& box:sample.at("world")) {
      const auto parsed=processBox(box);const auto id=parsed.id;
      if(!seen.insert(id).second || !original.count(id))throw std::runtime_error("COUPLED_OBJECT_IDENTITY_CHANGED");
      auto unchanged=box;unchanged["pose"]=original.at(id).at("pose");
      if(unchanged!=original.at(id) || (!moving.count(id) && box!=original.at(id)))
        throw std::runtime_error("COUPLED_FIXED_GEOMETRY_CHANGED");
    }
  }
}
inline ProcessJson coupledTransitionWorld(const ProcessJson& req,const ProcessJson& b) {
  if(!coupledBinding(b) || !req.value("planning_only",false) || req.value("planning_mode","")!="cold_from_scratch" ||
      req.at("planner_id")!="COUPLED_LIFT" || req.at("q_start")!=req.at("q_goal") ||
      !req.at("attachment").is_null() || !req.at("ownership_transition").is_null() ||
      req.value("parent_stage_id","").empty() || (req.at("stage")!="pregrasp" && req.at("stage")!="contact") ||
      req.at("world")!=b.at("samples").front().at("world") ||
      req.at("receiver_end_world")!=b.at("samples").back().at("world") ||
      req.at("process_policy").at("target_id")!=b.at("target_id"))
    throw std::runtime_error("COUPLED_TRANSITION_SCOPE_OR_WORLD_INVALID");
  return req.at("receiver_end_world");
}
inline ProcessJson coupledWorldFailure(const ProcessJson& sample,const ProcessJson& binding) {
  const auto& world=sample.at("world");
  std::set<std::set<std::string>> mates,planes;
  for(const auto& p:binding.at("assembly_pairs"))mates.insert(p.get<std::set<std::string>>());
  for(const auto& p:binding.at("designed_contacts"))planes.insert(p.get<std::set<std::string>>());
  std::vector<ProcessBox> parsed;for(const auto& b:world)parsed.push_back(processBox(b));
  for(size_t i=0;i<world.size();++i)for(size_t j=i+1;j<world.size();++j) {
    const auto& a=parsed[i];const auto& b=parsed[j];
    if(world[i].value("category","")!="lift_mechanism" && world[j].value("category","")!="lift_mechanism" &&
        a.id!="conveyor_transverse" && a.id!="conveyor_longitudinal" && b.id!="conveyor_transverse" && b.id!="conveyor_longitudinal")continue;
    const std::set<std::string> pair{a.id,b.id};if(mates.count(pair))continue;
    const Eigen::Vector3d ea=a.pose.linear().cwiseAbs()*a.half,eb=b.pose.linear().cwiseAbs()*b.half;
    if(((a.pose.translation()-b.pose.translation()).cwiseAbs()-ea-eb).cwiseMax(0.).norm()>.005000001)continue;
    const double sat=processSignedGap(a,b);
    if(planes.count(pair) && sat>=-1e-9) {
      bool plane=false;for(int k=0;k<2;++k)if(std::abs(std::abs(a.pose.translation()[k]-b.pose.translation()[k])-ea[k]-eb[k])<=1e-9)plane=true;
      if(plane)continue;
    }
    const double gap=receiverBoxDistance(world[i],world[j]);
    if(sat<0. || gap+1e-9<.005)return {{"reason","COUPLED_WORLD_CLEARANCE"},{"pair",{a.id,b.id}},{"surface_distance_m",gap},{"height_m",sample.at("height_m")}};
  }
  return nullptr;
}
}
