// Ordered-contact counterexamples. Synthetic model; not a carton-run claim.
#include "process_policy.h"
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <iostream>
using J=nlohmann::json;
void require(bool value,const char* reason) {if(!value) throw std::runtime_error(reason);}
J transform(double x,double z=0) {return {{1.,0.,0.,x},{0.,1.,0.,0.},{0.,0.,1.,z},{0.,0.,0.,1.}};}
J box(const std::string& id,double x) {return {{"id",id},{"pose",transform(x)},{"size",{.2,.4,.6}}};}
moveit_msgs::msg::CollisionObject object(const J& b) {
  moveit_msgs::msg::CollisionObject o;o.id=b.at("id");o.header.frame_id="world";o.operation=o.ADD;
  shape_msgs::msg::SolidPrimitive s;s.type=s.BOX;const auto dimensions=b.at("size").get<std::vector<double>>();s.dimensions.assign(dimensions.begin(),dimensions.end());o.primitives.push_back(s);
  geometry_msgs::msg::Pose p;p.orientation.w=1.;p.position.x=b.at("pose").at(0).at(3);o.primitive_poses.push_back(p);return o;
}
planning_scene::PlanningScenePtr scene() {
  auto u=urdf::parseURDF(R"(<robot name="test"><link name="world"/><link name="flange"/><joint name="J1" type="prismatic"><parent link="world"/><child link="flange"/><axis xyz="1 0 0"/><limit lower="-1" upper="1" velocity="1" effort="1"/></joint><link name="cup"><collision><geometry><box size="0.01 0.01 0.01"/></geometry></collision></link><joint name="mount" type="fixed"><parent link="flange"/><child link="cup"/><origin xyz="0 0 .305"/></joint></robot>)");
  auto srdf=std::make_shared<srdf::Model>();require(bool(u),"URDF");
  require(srdf->initString(*u,R"(<robot name="test"><group name="manipulator"><joint name="J1"/></group></robot>)"),"SRDF");
  auto model=std::make_shared<moveit::core::RobotModel>(u,srdf);auto s=std::make_shared<planning_scene::PlanningScene>(model);
  s->getCurrentStateNonConst().setToDefaultValues();s->getCurrentStateNonConst().update();
  s->processCollisionObjectMsg(object(box("neighbor",.2)));
  moveit_msgs::msg::AttachedCollisionObject a;a.link_name="flange";a.object=object(box("target",0.));a.object.header.frame_id="flange";a.touch_links={"cup"};
  require(s->processAttachedCollisionObjectMsg(a),"ATTACH");return s;
}
J request() {
  J source={{"schema","m710_poc_pair_collision_policy_v4"},{"required_pair_clearance_m",.005},{"self_collision_clearance_m",0.},
    {"stack_contact_stages",{"support-release","extraction"}},{"stack_contact_mode","planner_relaxed_physics_checked"},
    {"compliant_cup_neighbor_contact_mode","ignore"},{"maximum_compliant_cup_additional_compression_m",.005},
    {"maximum_planned_stack_penetration_m",.01},{"free_space_clearance_m",.0052}};
  J cups=J::array(),active=J::array(),eligible=J::array();
  for(int i=0;i<72;++i) {cups.push_back({{"index",i},{"cup_id","cup_"+std::to_string(i)},
    {"center_contact_frame_m",{0.,0.,0.}},{"seal_radius_m",.0215}});active.push_back(i==0);eligible.push_back(true);}
  J physical={{1.,0.,0.,0.},{0.,-1.,0.,0.},{0.,0.,-1.,.3},{0.,0.,0.,1.}};
  J spec={{"schema","m710_native_process_v1"},{"stage","extraction"},{"target_id","target"},{"target",box("target",0.)},
    {"target_face","top"},{"flange_from_physical_contact",physical},{"cups",cups},{"commanded_active_mask",active},
    {"geometrically_eligible_mask",eligible},{"actual_contact_mask",active},{"max_attachment_gap_m",.002},{"maximum_penetration_m",.0002},
    {"max_normal_misalignment_rad",.08726646259971647},{"suction_edge_margin_m",0.},{"support_names",J::array()},
    {"stack_carton_ids",{"target","neighbor"}},{"collision_policy",source},
    {"initial_proximity",{{"mode","planner_relaxed_physics_checked"},{"target","target"},{"stack_carton_names",{"neighbor"}},{"fully_released",false}}}};
  return {{"stage","extraction"},{"process_policy",spec},{"world",J::array({box("neighbor",.2)})},{"allowed_pairs",J::array()},
    {"clearance_policy",{{"source_policy",source},{"compliant_tool_links",{"cup"}},{"target_id","target"}}}};
}
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);auto s=scene();auto req=request();m710::ProcessPolicy p(s,req);
  require(p.check(s->getCurrentState()),"INITIAL_STACK_CONTACT_ALLOWED");
  auto q=s->getCurrentState();q.setJointGroupPositions("manipulator",std::vector<double>{.011});q.update();
  require(!p.check(q) && p.last_failure.at("reason")=="GROSS_PLANNED_STACK_PENETRATION","GROSS_STACK_REJECTED");
  auto forward=J::array({J::array({0.}),J::array({-.01})});
  require(p.checkPath(s->getCurrentState(),forward,.01).is_null(),"SEPARATING_PATH_ALLOWED");
  require(p.terminalProximity().at("fully_released")==true,"RESTORED_FREE_SPACE_STATE");
  m710::ProcessPolicy backtrack(s,req);auto reentry=J::array({J::array({0.}),J::array({-.01}),J::array({-.003})});
  require(backtrack.checkPath(s->getCurrentState(),reentry,.01).at("reason")=="PAYLOAD_COLLISION_FREE_SPACE_RULES_RESTORED","REENTRY_REJECTED");
  auto free=scene();req["stage"]="transit";req["process_policy"]["stage"]="transit";req["process_policy"]["initial_proximity"]=nullptr;
  m710::ProcessPolicy transit(free,req);collision_detection::AllowedCollision::Type allowed;
  require(!free->getAllowedCollisionMatrix().getEntry("target","neighbor",allowed) || allowed!=collision_detection::AllowedCollision::ALWAYS,"NO_STACK_ACM_IN_TRANSIT");
  auto bad=request();bad["allowed_pairs"]={{"flange","neighbor"}};bool rejected=false;
  try {m710::ProcessPolicy unsafe(scene(),bad);} catch(const std::exception&) {rejected=true;}
  require(rejected,"RIGID_LINK_ACM_REJECTED");
  req["process_policy"]["support_names"]={"neighbor"};rejected=false;
  try {m710::ProcessPolicy free_support(scene(),req);} catch(const std::exception& e) {rejected=std::string(e.what())=="NATIVE_FREE_SPACE_SUPPORT_PERMISSION_FORBIDDEN";}
  require(rejected,"FREE_TRANSIT_SUPPORT_PERMISSION_REJECTED");
  std::cout<<J({{"status","PASS"},{"scope","synthetic_native_process_policy_only"},{"cases",7}}).dump()<<std::endl;rclcpp::shutdown();
}
