#pragma once
// mast_contact_record_only (user decision 2026-11-25, "所有与视觉桅杆的碰撞都只记录，
// 不否决"): physical single carton only. Every moving robot link (all links
// below joint_2, the tool included) and the attached payload may touch every
// vision-mast solid (robot links named eco65_camera_* / eco65_mast_* fixed to
// link_1). The caller records the contacts with a separate check without this
// permission; no other pair rule, clearance or solid changes.
#include <moveit/planning_scene/planning_scene.h>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <vector>

namespace m710 {

inline std::vector<std::string> mastContactLinks(const moveit::core::RobotModelConstPtr& model) {
  const auto* base=model->getLinkModel("link_1");
  if(!base)throw std::runtime_error("MAST_CONTACT_LINK_1_MISSING");
  std::vector<std::string> out;
  for(const auto* link:model->getLinkModelsWithCollisionGeometry()) {
    const std::string& name=link->getName();
    if(name.rfind("eco65_camera_",0)!=0 && name.rfind("eco65_mast_",0)!=0)continue;
    const auto* at=link;
    while(at!=base) {
      const auto* joint=at->getParentJointModel();
      if(!joint || joint->getType()!=moveit::core::JointModel::FIXED)throw std::runtime_error("MAST_CONTACT_LINK_NOT_FIXED_TO_LINK_1:"+name);
      at=joint->getParentLinkModel();
      if(!at)throw std::runtime_error("MAST_CONTACT_LINK_NOT_FIXED_TO_LINK_1:"+name);
    }
    out.push_back(name);
  }
  if(out.empty())throw std::runtime_error("MAST_CONTACT_NO_MAST_LINKS");
  return out;
}

inline std::vector<std::string> mastContactMovingLinks(const moveit::core::RobotModelConstPtr& model) {
  const auto* joint=model->getJointModel("joint_2");
  if(!joint)throw std::runtime_error("MAST_CONTACT_JOINT_2_MISSING");
  std::vector<std::string> out;
  for(const auto* link:joint->getDescendantLinkModels())if(!link->getShapes().empty())out.push_back(link->getName());
  if(out.empty())throw std::runtime_error("MAST_CONTACT_NO_MOVING_LINKS");
  return out;
}

// Adds the permission to the scene ACM when the request's clearance policy
// carries it; returns the number of permitted pairs (0 when absent). The
// attachment, when present, is the payload id carried by the tool.
inline size_t applyMastContactRecordOnly(planning_scene::PlanningScene& scene,const nlohmann::json& req) {
  if(!req.contains("clearance_policy") || !req.at("clearance_policy").contains("mast_contact_record_only"))return 0;
  const auto& policy=req.at("clearance_policy");
  const auto& spec=policy.at("mast_contact_record_only");
  if(policy.value("schema",std::string())!="eco65_native_single_carton_clearance_v1")
    throw std::runtime_error("MAST_CONTACT_SCOPE_INVALID");
  if(!spec.is_object() || spec.size()!=1 || !spec.contains("basis") || !spec.at("basis").is_string() ||
     spec.at("basis").get<std::string>().empty())
    throw std::runtime_error("MAST_CONTACT_BASIS_REQUIRED");
  const auto mast=mastContactLinks(scene.getRobotModel());
  auto moving=mastContactMovingLinks(scene.getRobotModel());
  if(req.contains("attachment") && !req.at("attachment").is_null())moving.push_back(req.at("attachment").at("id").get<std::string>());
  auto& acm=scene.getAllowedCollisionMatrixNonConst();
  for(const auto& a:mast)for(const auto& b:moving)acm.setEntry(a,b,true);
  return mast.size()*moving.size();
}

}  // namespace m710
