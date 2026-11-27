#pragma once
// layout_fixed_mounts (2026-11-27, site STEP static layer built from the user's
// layout with the CAD design transform): the robot base stands on its STEP
// mounting plate. That plate/base_link pair is an exact fixed mating interface,
// the same kind as the existing base_link/robot_mount_plate permission. Only
// base_link may be named, only against an imported world object of the site
// STEP layer ("site_layout_" prefix); every other link, tool solid, payload
// and every other pair keep all rules and clearances.
#include <moveit/planning_scene/planning_scene.h>
#include <nlohmann/json.hpp>
#include <set>
#include <stdexcept>
#include <string>

namespace m710 {

inline size_t applyLayoutFixedMounts(planning_scene::PlanningScene& scene,const nlohmann::json& req) {
  if(!req.contains("clearance_policy") || !req.at("clearance_policy").contains("layout_fixed_mounts"))return 0;
  const auto& policy=req.at("clearance_policy");
  const auto& spec=policy.at("layout_fixed_mounts");
  if(policy.value("schema",std::string())!="eco65_native_single_carton_clearance_v1")
    throw std::runtime_error("LAYOUT_FIXED_MOUNTS_SCOPE_INVALID");
  if(!spec.is_object() || spec.size()!=2 || !spec.contains("pairs") || !spec.at("pairs").is_array() || spec.at("pairs").empty() ||
     !spec.contains("basis") || !spec.at("basis").is_string() || spec.at("basis").get<std::string>().empty())
    throw std::runtime_error("LAYOUT_FIXED_MOUNTS_INVALID");
  const auto model=scene.getRobotModel();
  std::set<std::string> world;
  for(const auto& id:scene.getWorld()->getObjectIds())world.insert(id);
  auto& acm=scene.getAllowedCollisionMatrixNonConst();size_t count=0;
  for(const auto& pair:spec.at("pairs")) {
    if(!pair.is_array() || pair.size()!=2 || !pair[0].is_string() || !pair[1].is_string())
      throw std::runtime_error("LAYOUT_FIXED_MOUNT_PAIR_INVALID");
    const std::string link=pair[0].get<std::string>(),object=pair[1].get<std::string>();
    const auto* l=model->getLinkModel(link);
    // base_link must not move: the model root or fixed to it.
    bool fixed=l!=nullptr;
    for(const auto* x=l;fixed && x;x=x->getParentLinkModel()) {
      const auto* joint=x->getParentJointModel();
      if(joint && joint->getType()!=moveit::core::JointModel::FIXED)fixed=false;
    }
    if(link!="base_link" || !l || l->getShapes().empty() || !fixed)
      throw std::runtime_error("LAYOUT_FIXED_MOUNT_LINK_INVALID:"+link);
    if(object.rfind("site_layout_",0)!=0 || !world.count(object))throw std::runtime_error("LAYOUT_FIXED_MOUNT_OBJECT_INVALID:"+object);
    acm.setEntry(link,object,true);++count;
  }
  return count;
}

}  // namespace m710
