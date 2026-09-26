// Synthetic geometry tests: implementation evidence, never scene success rates.
#include "clearance.h"
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <iostream>
#include <sstream>
#include <iomanip>
using J=nlohmann::json;
void require(bool value,const char* message) {if(!value) throw std::runtime_error(message);}
planning_scene::PlanningScenePtr scene(double gap) {
  std::ostringstream xml;xml<<std::setprecision(17);
  xml<<R"(<robot name="test"><link name="world"/><link name="robot"><collision><geometry><box size="0.01 0.01 0.01"/></geometry></collision></link><joint name="J1" type="prismatic"><parent link="world"/><child link="robot"/><axis xyz="1 0 0"/><limit lower="-1" upper="1" velocity="1" effort="1"/></joint><link name="tool"><collision><geometry><box size="0.01 0.01 0.01"/></geometry></collision></link><joint name="mount" type="fixed"><parent link="robot"/><child link="tool"/><origin xyz=")"<<.01+gap<<R"( 0 0"/></joint></robot>)";
  auto urdf=urdf::parseURDF(xml.str());auto srdf=std::make_shared<srdf::Model>();
  require(bool(urdf),"URDF");require(srdf->initString(*urdf,R"(<robot name="test"><group name="manipulator"><joint name="J1"/></group></robot>)"),"SRDF");
  auto model=std::make_shared<moveit::core::RobotModel>(urdf,srdf);
  auto result=std::make_shared<planning_scene::PlanningScene>(model);
  result->getCurrentStateNonConst().setToDefaultValues();result->getCurrentStateNonConst().update();return result;
}
J policy() {return {{"schema","m710_native_free_clearance_v1"},{"stage","transit"},
  {"source_policy",{{"schema","m710_poc_pair_collision_policy_v4"},{"required_pair_clearance_m",.005},{"self_collision_clearance_m",0.}}},
  {"numerical_gap_tolerance_m",1e-9},{"tool_links",{"tool"}},{"conveyor_ids",J::array()},{"payload_id",""},{"receiver_reserve_m",0.}};}
moveit_msgs::msg::CollisionObject box(const std::string& name,double x) {
  moveit_msgs::msg::CollisionObject b;b.id=name;b.header.frame_id="world";b.operation=b.ADD;
  shape_msgs::msg::SolidPrimitive shape;shape.type=shape.BOX;shape.dimensions={.01,.01,.01};b.primitives.push_back(shape);
  geometry_msgs::msg::Pose pose;pose.orientation.w=1;pose.position.x=x;b.primitive_poses.push_back(pose);return b;
}
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);J out;out["scope"]="synthetic implementation checks only";out["threshold_cases"]=J::array();
  for(double gap:{.004,.005-2e-9,.005-.5e-9,.005,.005+.5e-9,.006}) {
    auto s=scene(gap);m710::Clearance c(s,policy());bool valid=c.check(s->getCurrentState());
    require(valid==(gap+1e-9>=.005),"THRESHOLD_CLASSIFICATION");out["threshold_cases"].push_back({{"gap",gap},{"valid",valid},{"failure",c.last_failure}});
  }
  auto s=scene(.002);auto p=policy();p["tool_links"]=J::array();
  m710::Clearance ordinary(s,p);require(ordinary.check(s->getCurrentState()),"ORDINARY_SELF_MUST_NOT_GET_5MM");
  m710::Clearance tool(s,policy());require(!tool.check(s->getCurrentState()),"ROBOT_TOOL_MUST_GET_5MM");
  auto diff=s->diff();diff->decoupleParent();diff->getAllowedCollisionMatrixNonConst().setEntry("robot","tool",true);
  m710::Clearance exempt(diff,policy());require(exempt.check(diff->getCurrentState()),"SCOPED_ASSEMBLY_ALLOWANCE");
  require(!tool.check(s->getCurrentState()),"ACM_LEAK");out["self_tool_and_diff"]="PASS";
  s=scene(.02);s->processCollisionObjectMsg(box("neighbor",.042));
  auto cupdiff=s->diff();cupdiff->decoupleParent();cupdiff->getAllowedCollisionMatrixNonConst().setEntry("tool","neighbor",true);
  m710::Clearance cup(cupdiff,policy()),rigid(s,policy());
  require(cup.check(cupdiff->getCurrentState()),"EXACT_CUP_PAIR");require(!rigid.check(s->getCurrentState()),"RIGID_NOT_EXEMPT");
  out["cup_permission_not_rigid"]="PASS";
  s=scene(.02);s->processCollisionObjectMsg(box("wall",.07));
  m710::Clearance edge(s,policy());auto state=s->getCurrentState();
  for(double q:{-.1,.1}) {state.setJointGroupPositions("manipulator",std::vector<double>{q});state.update();require(edge.check(state),"EDGE_ENDPOINT");}
  edge.phase="output";auto failure=edge.checkPath(s->getCurrentState(),J::array({J::array({-.1}),J::array({.1})}),.04);
  require(!failure.is_null(),"EDGE_INTERIOR_MISSED");out["valid_endpoints_invalid_edge"]=failure;out["edge_counts"]=edge.evidence();
  // A checker is rebuilt after attachment; the world must lose that same ID.
  s=scene(.02);s->processCollisionObjectMsg(box("payload",.08));
  auto attached=s->diff();attached->decoupleParent();attached->getWorldNonConst()->removeObject("payload");
  moveit_msgs::msg::AttachedCollisionObject a;a.link_name="robot";a.object=box("payload",.08);a.object.header.frame_id="robot";
  require(attached->processAttachedCollisionObjectMsg(a),"ATTACH");attached->processCollisionObjectMsg(box("neighbor",.092));
  p=policy();p["payload_id"]="payload";m710::Clearance payload(attached,p);
  require(!payload.check(attached->getCurrentState()),"ATTACHED_PAYLOAD_GAP");
  require(!attached->getWorld()->hasObject("payload") && s->getWorld()->hasObject("payload"),"ATTACH_DIFF_LEAK");
  out["attached_failure"]=payload.last_failure;out["status"]="PASS";std::cout<<out.dump(2)<<std::endl;rclcpp::shutdown();
}
