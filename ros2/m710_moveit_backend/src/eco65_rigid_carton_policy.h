#pragma once
#include "eco65_collision_policy.h"
#include <iomanip>
#include <sstream>

namespace m710 {
inline std::set<std::string> eco65RigidToolIds() {
  std::set<std::string> ids;
  for(int i=0;i<43;++i)if(i!=41) {
    std::ostringstream name;name<<"part_"<<std::setw(3)<<std::setfill('0')<<i;
    if(i>=2 && i<=24 && i%2==0)name<<"_rigid";
    ids.insert(name.str());
  }
  for(int i=0;i<3;++i)ids.insert("part_041_region_"+std::to_string(i));
  return ids;
}
inline bool eco65HasRigidCartonPolicy(const EcoPolicyJson& p) {
  return p.value("schema",std::string())=="eco65_pair_collision_policy_v6";
}
inline const EcoPolicyJson& eco65RigidCartonRule(const EcoPolicyJson& p) {
  const auto& e=p.at("rigid_tool_carton_contact");
  const auto ids=eco65RigidToolIds();std::set<std::string> members;
  for(const auto& row:e.at("rigid_tool_members")) {
    const std::string id=row.at("id"),sha=row.at("mesh_sha256");
    if(!members.insert(id).second || row.at("semantic")!="conservative_rigid" ||
        sha.size()!=64 || sha.find_first_not_of("0123456789abcdef")!=std::string::npos)
      throw std::runtime_error("ECO65_RIGID_CARTON_SOURCE_INVALID");
  }
  if(e.at("schema")!="eco65_rigid_tool_carton_contact_v1" ||
      e.at("scope")!="ENUMERATED_RIGID_TOOL_VS_ORIGINAL_CARTONS_WORLD_OR_ATTACHED" ||
      e.at("continuous_task_mode")!="eco65_coupled_cycle_v1" ||
      e.at("rigid_tool_links")!=EcoPolicyJson(ids) || members!=ids ||
      e.at("original_carton_ids")!=EcoPolicyJson(eco65ExtractionOriginalIds(p)) ||
      e.at("original_carton_inventory_fingerprint")!=p.at("extraction_carton_contact").at("original_carton_inventory_fingerprint") ||
      e.at("include_current_target")!=true || e.at("include_attached_target")!=true ||
      e.at("include_released_target")!=true || e.at("geometry_retained")!=true ||
      e.at("allow_intersection")!=true || e.at("physical_execution")!="NOT_RUN" || e.at("execution_ready")!=false)
    throw std::runtime_error("ECO65_RIGID_CARTON_SCOPE_INVALID");
  return e;
}
inline void validateEco65RigidCartonPolicy(const EcoPolicyJson& p,
    const moveit::core::RobotModelConstPtr& model,const EcoPolicyJson& geometry,
    const EcoPolicyJson& tools,const EcoPolicyJson& compliant) {
  if(!eco65HasRigidCartonPolicy(p))return;
  const auto& e=eco65RigidCartonRule(p);const auto ids=eco65RigidToolIds();
  auto rigid=tools.get<std::set<std::string>>();
  for(const auto& band:compliant)rigid.erase(band.get<std::string>());
  if(rigid!=ids || geometry.at("rigid_tool_carton_contact")!=e)
    throw std::runtime_error("ECO65_RIGID_CARTON_TOOL_BINDING_CHANGED");
  for(const auto& id:ids) {
    const auto* link=model->getLinkModel(id);
    if(!link || link->getShapes().empty() || !link->getParentLinkModel() ||
        link->getParentLinkModel()->getName()!="link_6")
      throw std::runtime_error("ECO65_RIGID_CARTON_ACTUAL_LINK_MISSING");
  }
}
inline EcoPolicyJson eco65RigidCartonPairs(const EcoPolicyJson& req) {
  EcoPolicyJson pairs=EcoPolicyJson::array();
  if(req.is_null() || !req.contains("clearance_policy"))return pairs;
  const auto& p=req.at("clearance_policy").at("source_policy");
  if(!eco65HasRigidCartonPolicy(p) || req.value("continuous_task_mode",std::string())!="eco65_coupled_cycle_v1")return pairs;
  const auto& e=eco65RigidCartonRule(p);const auto& process=req.at("process_policy");
  if(process.at("rigid_tool_carton_contact")!=e)
    throw std::runtime_error("ECO65_RIGID_CARTON_REQUEST_BINDING_CHANGED");
  const auto original=e.at("original_carton_ids").get<std::set<std::string>>();
  const auto completed=req.at("completed_ids").get<std::set<std::string>>();
  std::set<std::string> expected,actual;
  for(const auto& id:completed)if(!original.count(id))throw std::runtime_error("ECO65_RIGID_CARTON_UNKNOWN_COMPLETED");
  for(const auto& id:original)if(!completed.count(id))expected.insert(id);
  for(const auto& row:req.at("world")) {
    const std::string id=row.at("id");
    if(original.count(id) && !actual.insert(id).second)throw std::runtime_error("ECO65_RIGID_CARTON_DUPLICATE");
  }
  if(!req.at("attachment").is_null()) {
    const std::string id=req.at("attachment").at("id");
    if(id!=process.at("target_id") || !original.count(id) || !actual.insert(id).second)
      throw std::runtime_error("ECO65_RIGID_CARTON_ATTACHED_OWNERSHIP_INVALID");
  }
  if(actual!=expected || process.at("task_contract").at("stack_carton_ids")!=EcoPolicyJson(expected))
    throw std::runtime_error("ECO65_RIGID_CARTON_WORLD_OWNERSHIP_INVALID");
  for(const auto& tool:e.at("rigid_tool_links"))for(const auto& box:actual)pairs.push_back({tool,box});
  return pairs;
}
inline void setEco65RigidCartonPairs(const planning_scene::PlanningScenePtr& scene,const EcoPolicyJson& pairs) {
  for(const auto& pair:pairs) {
    const std::string tool=pair.at(0),box=pair.at(1);
    const auto* link=scene->getRobotModel()->getLinkModel(tool);
    const bool world=scene->getWorld()->hasObject(box),attached=scene->getCurrentState().hasAttachedBody(box);
    if(!link || link->getShapes().empty() || scene->getRobotModel()->getLinkModel(box) || world==attached)
      throw std::runtime_error("ECO65_RIGID_CARTON_PHYSICAL_OWNERSHIP_INVALID");
    scene->getAllowedCollisionMatrixNonConst().setEntry(tool,box,true);
  }
}
inline EcoPolicyJson eco65RigidCartonEvidence(const EcoPolicyJson& pairs) {
  return {{"active",!pairs.empty()},{"pairs",pairs},{"policy_schema","eco65_rigid_tool_carton_contact_v1"},
    {"scope","ENUMERATED_RIGID_TOOL_VS_ORIGINAL_CARTONS_WORLD_OR_ATTACHED"}};
}
}
