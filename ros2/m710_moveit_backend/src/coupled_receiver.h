#pragma once
#include "lifted_receiver.h"
namespace m710 {
inline bool coupledBinding(const ProcessJson& binding) {
  return binding.is_object() && (binding.value("schema","")=="m710_coupled_receiver_v2" ||
      binding.value("schema","")=="m710_coupled_receiver_v3");
}
inline const ProcessJson& activeCoupledBinding(const ProcessJson& req,const ProcessJson& binding) {
  if(req.value("coupled_leg","")=="PRE_APPROACH")return binding.at("pre_approach_transition");
  if(req.contains("coupled_leg"))throw std::runtime_error("COUPLED_LEG_UNKNOWN");
  return binding;
}
inline void validateCoupledBinding(const ProcessJson& b) {
  if(!coupledBinding(b) || b.at("samples").empty() || b.at("samples").size()>4096 ||
      b.at("continuous_swept_proof")!=false || b.at("maximum_height_step_m").get<double>()>.010000000001)
    throw std::runtime_error("COUPLED_BINDING_INVALID");
  const auto& first=b.at("samples").front();
  std::map<std::string,ProcessJson> original;
  for(const auto& box:first.at("world")) {processBox(box);validateSolid(box);original[box.at("id")]=box;}
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
  if(b.contains("pre_approach_transition")) {
    const auto& pre=b.at("pre_approach_transition");
    if(pre.contains("pre_approach_transition") || pre.at("target_id")!=b.at("target_id") ||
        pre.at("samples").back().at("world")!=b.at("samples").front().at("world"))
      throw std::runtime_error("COUPLED_PRE_APPROACH_CONTINUITY_INVALID");
    validateCoupledBinding(pre);
  }
}
inline ProcessJson coupledTransitionWorld(const ProcessJson& req,const ProcessJson& b) {
  const bool attached=!req.at("attachment").is_null();
  if(req.at("stage")=="extraction")
    throw std::runtime_error("EXTRACTION_THEN_LIFT_DISABLED");
  auto withoutTarget=[&](const ProcessJson& world){auto out=ProcessJson::array();for(const auto& box:world)
    if(!attached || box.at("id")!=b.at("target_id"))out.push_back(box);return out;};
  if(!coupledBinding(b) || !req.value("planning_only",false) || req.value("planning_mode","")!="cold_from_scratch" ||
      req.at("planner_id")!="COUPLED_LIFT" || req.at("q_start")!=req.at("q_goal") ||
      !req.at("ownership_transition").is_null() ||
      (req.value("parent_stage_id","").empty() && req.value("coupled_leg","")!="PRE_APPROACH") ||
      (req.at("stage")!="pregrasp" && req.at("stage")!="contact") ||
      attached || // this existing protocol establishes ATTACH at extraction start
      req.at("world")!=withoutTarget(b.at("samples").front().at("world")) ||
      req.at("receiver_end_world")!=withoutTarget(b.at("samples").back().at("world")) ||
      req.at("process_policy").at("target_id")!=b.at("target_id"))
    throw std::runtime_error("COUPLED_TRANSITION_SCOPE_OR_WORLD_INVALID");
  return req.at("receiver_end_world");
}
inline void validateL01Hmax(const ProcessJson& req,const ProcessJson& binding) {
  const auto& contract=binding.at("receive_contract");
  const auto& p=contract.at("l01_hmax_permission");
  const auto target=processBox(contract.at("initial_target"));
  if(p.at("schema")!="frozen_l01_hmax_v1" || p.at("planning_only")!=true ||
      p.at("stationary_receiver")!=true || p.at("height_source")!="FROZEN_INITIAL_HMAX" ||
      !req.value("planning_only",false) || req.value("planning_mode","")!="cold_from_scratch" ||
      binding.at("target_id")!=target.id || req.at("process_policy").at("target_id")!=target.id ||
      p.at("original_cartons").size()!=40)
    throw std::runtime_error("L01_HMAX_SCOPE_CHANGED");
  std::map<std::string,ProcessJson> original;
  for(const auto& b:p.at("original_cartons")) {
    const auto parsed=processBox(b);
    if(b.at("category")!="carton" || !original.emplace(parsed.id,b).second)
      throw std::runtime_error("L01_HMAX_ORIGINAL_POPULATION_CHANGED");
  }
  const auto anchor=processBox(original.at(p.at("anchor_id").get<std::string>()));
  auto height=[](const ProcessBox& b){return 2*b.pose.linear().row(2).cwiseAbs().dot(b.half);};
  const double ah=height(anchor),az=anchor.pose.translation().z()-ah/2;
  std::set<std::string> allowed;
  for(const auto& item:original) {
    const auto b=processBox(item.second);const double h=height(b),z=b.pose.translation().z()-h/2;
    if(std::abs(h-ah)<1e-9 && std::abs(z-az)<1e-9)allowed.insert(b.id);
  }
  if(std::abs(ah-.3)>1e-9 || std::abs(az-.3)>1e-9 || allowed.size()!=5 ||
      allowed!=p.at("allowed_target_ids").get<std::set<std::string>>() || !allowed.count(target.id) ||
      original.at(target.id)!=contract.at("initial_target") ||
      std::abs(contract.at("height_m").get<double>()-(target.pose.translation().z()-height(target)/2-.05))>1e-8)
    throw std::runtime_error("L01_HMAX_TARGET_OR_HEIGHT_CHANGED");
  // Frozen permission cannot hide a changed root carton or moving receiver.
  for(const auto& sample:binding.at("samples")) {
    bool found=false;
    if(std::abs(sample.at("height_m").get<double>()-contract.at("height_m").get<double>())>1e-8)
      throw std::runtime_error("L01_HMAX_RECEIVER_NOT_STATIONARY");
    for(const auto& b:sample.at("world"))if(b.at("id")==target.id) {
      if(b!=original.at(target.id))throw std::runtime_error("L01_HMAX_INITIAL_TARGET_CHANGED");
      found=true;
    }
    if(!found)throw std::runtime_error("L01_HMAX_INITIAL_TARGET_MISSING");
  }
}
inline void validateReceiveHeight(const ProcessJson& req,const ProcessJson& binding) {
  if(!binding.contains("receive_contract") || req.at("stage")!="extraction")return;
  const auto& contract=binding.at("receive_contract");
  const double expected=contract.at("height_m");bool found=false;
  for(const auto& raw:req.at("world"))if(raw.at("id")=="conveyor_transverse") {
    const auto belt=processBox(raw);const double h=belt.pose.translation().z()+belt.half.z();
    if(!std::isfinite(expected) || std::abs(h-expected)>1e-8)
      throw std::runtime_error("EXTRACTION_BEFORE_REQUIRED_RECEIVE_HEIGHT");
    found=true;
  }
  if(!found)throw std::runtime_error("RECEIVE_HEIGHT_WORLD_IDENTITY");
  // Exact whole-mechanism world is bound at init and checked along the native
  // parent chain. Payload ownership may differ; no fixed body may be omitted.
  ProcessJson fixed=ProcessJson::array(),required=ProcessJson::array();
  for(const auto& b:req.at("world"))if(b.value("category","")!="carton")fixed.push_back(b);
  for(const auto& b:binding.at("samples").back().at("world"))if(b.value("category","")!="carton")required.push_back(b);
  if(fixed!=required)throw std::runtime_error("RECEIVE_MECHANISM_WORLD_CHANGED");
  const bool l01_hmax=contract.contains("l01_hmax_permission");
  if(l01_hmax)validateL01Hmax(req,binding);
  const bool search=contract.contains("joint_search_contract");
  if(search) {
    const auto& policy=contract.at("joint_search_contract");const auto target=processBox(contract.at("initial_target"));
    const double h=2*target.pose.linear().row(2).cwiseAbs().dot(target.half),bottom=target.pose.translation().z()-h/2;
    const std::string mode=policy.at("mode");const double minimum=policy.at("minimum_m"),maximum=policy.at("maximum_m");
    double height=0.;
    if(mode=="Hmax")height=bottom-.05;
    else if(mode=="Hmin")height=bottom-h-.05;
    else if(mode=="LIFTED_TOP_FIXED" && policy.at("face")=="z+")height=minimum;
    else if(mode=="LIFTED_TOP_EXPLICIT" && policy.at("face")=="z+" && policy.contains("explicit_receive_height_m")) {
      height=policy.at("explicit_receive_height_m");
      if((height!=.65 && height!=.85) || !(height>bottom-.05+1e-9))
        throw std::runtime_error("SEARCH_EXPLICIT_TOP_HEIGHT_SCOPE");
    }
    else throw std::runtime_error("SEARCH_RECEIVE_MODE");
    const double release=policy.at("face")=="z+"?std::max(bottom+.05,height+.05):bottom;
    const auto& layout=binding.at("joint_search_layout");
    if(layout.at("schema")!="fixed40_joint_search_20261028_v1" ||
        minimum!=layout.at("minimum_surface_height_m").get<double>() ||
        maximum!=layout.at("maximum_surface_height_m").get<double>())
      throw std::runtime_error("SEARCH_RECEIVE_LAYOUT_LIMITS_CHANGED");
    if(policy.at("schema")!="fixed40_joint_search_20261028_v1" || policy.at("target_id")!=target.id ||
        policy.at("d_m")!=.05 || expected<minimum-1e-9 || expected>maximum+1e-9 ||
        std::abs(height-expected)>1e-8 || std::abs(policy.at("height_m").get<double>()-expected)>1e-8 ||
        std::abs(policy.at("release_bottom_m").get<double>()-release)>1e-8)
      throw std::runtime_error("SEARCH_RECEIVE_FORMULA_OR_RANGE_CHANGED");
  }
  if(contract.value("junction_front",false)) {
    const auto target=processBox(contract.at("initial_target"));
    const double h=2*target.pose.linear().row(2).cwiseAbs().dot(target.half);
    const double hmin=target.pose.translation().z()-h/2-h-.05;
    if(!l01_hmax && !search && std::abs(expected-hmin)>1e-8)throw std::runtime_error("JUNCTION_FRONT_REQUIRES_HMIN");
    const auto current=processBox(req.at("process_policy").at("target"));
    if(std::abs(current.pose.translation().y()-target.pose.translation().y())>1e-5 ||
        (current.pose.linear()-target.pose.linear()).cwiseAbs().maxCoeff()>2e-4)
      throw std::runtime_error("JUNCTION_FRONT_Y_OR_ORIENTATION_CHANGED");
  }
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
    const double sat=(world[i].contains("solid") || world[j].contains("solid"))?
      solidSignedGap(world[i],world[j]):processSignedGap(a,b);
    if(binding.value("geometry_revision","")=="upper_seam_lower_slider_v3" &&
       pair==std::set<std::string>{"conveyor_transverse","conveyor_longitudinal"} &&
       sat>=-1e-9)continue; // named, nonpenetrating top edge; penetration still rejects
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
