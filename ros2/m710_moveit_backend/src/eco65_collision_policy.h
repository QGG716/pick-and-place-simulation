#pragma once
#include <moveit/robot_model/robot_model.h>
#include <nlohmann/json.hpp>
#include <set>
#include <string>

namespace m710 {
using EcoPolicyJson = nlohmann::json;
inline bool eco65MastExempt(const EcoPolicyJson& policy,const std::string& a,const std::string& b) {
  if(policy.value("schema",std::string())!="eco65_pair_collision_policy_v2")return false;
  const auto& exemption=policy.at("robot_mast_exemption");
  for(const auto& pair:exemption.at("exempt_pairs"))
    if((pair.at(0)==a && pair.at(1)==b)||(pair.at(0)==b && pair.at(1)==a))return true;
  return false;
}
// Called once against the actual imported model, before any request can use it.
inline void validateEco65MastExemption(const EcoPolicyJson& policy,
    const moveit::core::RobotModelConstPtr& model,const EcoPolicyJson& geometry,
    const EcoPolicyJson& tools) {
  if(policy.value("schema",std::string())!="eco65_pair_collision_policy_v2")return;
  const auto& e=policy.at("robot_mast_exemption");
  const EcoPolicyJson robots={"base_link","link_1","link_2","link_3","link_4","link_5","link_6"};
  if(policy.at("mode")!="robot_body_vision_mast_exempt_v2" ||
      e.at("schema")!="eco65_robot_vision_mast_exemption_v1" ||
      e.at("scope")!="ROBOT_BODY_VS_ENUMERATED_VISION_MAST_ASSEMBLY_ONLY" ||
      e.at("robot_body_links")!=robots || geometry.at("robot_mast_exemption")!=e ||
      e.at("geometry_retained")!=true || e.at("execution_ready")!=false ||
      e.at("physical_execution")!="NOT_RUN")
    throw std::runtime_error("ECO65_MAST_POLICY_BINDING_INVALID");
  const auto owned=tools.get<std::set<std::string>>();
  std::set<std::string> names;
  EcoPolicyJson exact=EcoPolicyJson::array();
  for(const auto& item:e.at("assembly_members")) {
    const std::string id=item.at("id"),role=item.at("role");
    const auto* link=model->getLinkModel(id);
    const auto* joint=link?link->getParentJointModel():nullptr;
    if(id=="robot_mount_plate" || owned.count(id) || !names.insert(id).second ||
        !link || link->getShapes().empty() || !joint || joint->getType()!=moveit::core::JointModel::FIXED ||
        !link->getParentLinkModel() || link->getParentLinkModel()->getName()!="link_1" ||
        item.at("parent_link")!="link_1" || item.at("source_fingerprint").get<std::string>().size()!=64 ||
        item.at("mesh_sha256").get<std::string>().size()!=64 ||
        (role!="permanent_mast" && role!="permanent_mounting_seat" &&
         role!="permanent_camera" && role!="permanent_camera_adapter"))
      throw std::runtime_error("ECO65_MAST_ASSEMBLY_LINK_INVALID:"+id);
  }
  if(names.empty() || EcoPolicyJson(names)!=e.at("assembly_links"))
    throw std::runtime_error("ECO65_MAST_ASSEMBLY_INVENTORY_MISMATCH");
  for(const auto& robot:robots) {
    const auto* link=model->getLinkModel(robot.get<std::string>());
    if(!link || link->getShapes().empty())throw std::runtime_error("ECO65_ROBOT_BODY_MISSING");
    for(const auto& name:names)exact.push_back({robot,name});
  }
  if(exact!=e.at("exempt_pairs"))throw std::runtime_error("ECO65_MAST_EXEMPTION_PAIR_SCOPE_CHANGED");
}
}
