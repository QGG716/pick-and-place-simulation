#pragma once
// absent_tool_parts (user decision 2026-11-25, "把吸具零件 part_001从layout中去掉";
// confirmed "实物没有，去掉"): physical single carton only. Named rigid tool
// solids that the field tool does not have take part in no collision pair:
// every pair of such a part with any robot link, tool solid, world object or
// the attached payload is allowed. The model and every other solid, pair rule
// and clearance are unchanged; cup lips/bodies can never be named.
#include <moveit/planning_scene/planning_scene.h>
#include <nlohmann/json.hpp>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace m710 {

inline size_t applyAbsentToolParts(planning_scene::PlanningScene& scene,const nlohmann::json& req) {
  if(!req.contains("clearance_policy") || !req.at("clearance_policy").contains("absent_tool_parts"))return 0;
  const auto& policy=req.at("clearance_policy");
  const auto& spec=policy.at("absent_tool_parts");
  if(policy.value("schema",std::string())!="eco65_native_single_carton_clearance_v1")
    throw std::runtime_error("ABSENT_TOOL_PARTS_SCOPE_INVALID");
  if(!spec.is_object() || spec.size()!=2 || !spec.contains("parts") || !spec.at("parts").is_array() || spec.at("parts").empty() ||
     !spec.contains("basis") || !spec.at("basis").is_string() || spec.at("basis").get<std::string>().empty())
    throw std::runtime_error("ABSENT_TOOL_PARTS_INVALID");
  const auto tools=policy.at("tool_links").get<std::set<std::string>>();
  const auto model=scene.getRobotModel();
  std::vector<std::string> parts;
  for(const auto& p:spec.at("parts")) {
    const std::string name=p.get<std::string>();
    const auto* link=model->getLinkModel(name);
    if(!tools.count(name) || !link || link->getShapes().empty() || name.find("_lip")!=std::string::npos)
      throw std::runtime_error("ABSENT_TOOL_PART_INVALID:"+name);
    parts.push_back(name);
  }
  std::vector<std::string> others;
  for(const auto* link:model->getLinkModelsWithCollisionGeometry())others.push_back(link->getName());
  // World objects as imported in this scene (requests may carry a scene handle instead of the world).
  for(const auto& id:scene.getWorld()->getObjectIds())others.push_back(id);
  if(req.contains("attachment") && !req.at("attachment").is_null())others.push_back(req.at("attachment").at("id").get<std::string>());
  auto& acm=scene.getAllowedCollisionMatrixNonConst();size_t count=0;
  for(const auto& a:parts)for(const auto& b:others)if(a!=b){acm.setEntry(a,b,true);++count;}
  return count;
}

}  // namespace m710
