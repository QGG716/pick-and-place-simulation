// Ordered-contact counterexamples. Synthetic model; not a carton-run claim.
#include "process_policy.h"
#include "scene_geometry.h"
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
bool optimized=false;
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
  return {{"native_check_reuse",optimized},{"stage","extraction"},{"process_policy",spec},{"world",J::array({box("neighbor",.2)})},{"allowed_pairs",J::array()},
    {"clearance_policy",{{"source_policy",source},{"compliant_tool_links",{"cup"}},{"target_id","target"}}}};
}
void checkWorldShapePoses() {
  auto s=scene();
  auto imported=object(box("frame_probe",.07));
  imported.pose.orientation.w=1.;imported.pose.position.x=.30;
  require(s->processCollisionObjectMsg(imported),"WORLD_FRAME_IMPORT");
  const auto original=s->getWorld()->getObject("frame_probe");
  const auto world_pose=m710::worldShapePose(*original,0);
  require(std::abs(world_pose.translation().x()-.37)<1e-12 &&
    std::abs(original->shape_poses_.at(0).translation().x()-.07)<1e-12,"WORLD_SHAPE_USES_OBJECT_AND_LOCAL_POSES");
  imported.pose.position.x=.40;
  require(s->processCollisionObjectMsg(imported),"MOVED_WORLD_FRAME_IMPORT");
  const auto moved=s->getWorld()->getObject("frame_probe");
  require((original->shape_poses_.at(0).matrix()-moved->shape_poses_.at(0).matrix()).norm()<1e-12 &&
    std::abs(m710::worldShapePose(*moved,0).translation().x()-.47)<1e-12 &&
    (world_pose.matrix()-m710::worldShapePose(*moved,0).matrix()).cwiseAbs().maxCoeff()>1e-7,
    "NON_TARGET_OBJECT_FRAME_MOTION_IS_VISIBLE");
  s->getWorldNonConst()->removeObject("frame_probe");
  auto& state=s->getCurrentStateNonConst();state.setJointGroupPositions("manipulator",std::vector<double>{.11});state.update();
  moveit_msgs::msg::AttachedCollisionObject attachment;attachment.link_name="flange";
  attachment.object=object(box("frame_probe",.26));attachment.object.header.frame_id="flange";
  require(s->processAttachedCollisionObjectMsg(attachment),"WORLD_FRAME_ATTACH");
  const auto attached_pose=s->getCurrentState().getAttachedBody("frame_probe")->getGlobalCollisionBodyTransforms().at(0);
  require((world_pose.matrix()-attached_pose.matrix()).cwiseAbs().maxCoeff()<1e-12,"WORLD_TO_ATTACHED_POSE_PRESERVED");
  s->getCurrentStateNonConst().clearAttachedBody("frame_probe");
  imported.pose.position.x=.30;require(s->processCollisionObjectMsg(imported),"WORLD_FRAME_RELEASE");
  require((attached_pose.matrix()-m710::worldShapePose(*s->getWorld()->getObject("frame_probe"),0).matrix()).cwiseAbs().maxCoeff()<1e-12,
    "ATTACHED_TO_WORLD_POSE_PRESERVED");
}
int main(int argc,char** argv) {
  const J mount={{"robot_mount_contact_exemption",true}};
  require(m710::authorizedRobotMountPair(mount,"J2_link","chassis"),"EXPLICIT_ROBOT_MOUNT_ALLOWED");
  require(m710::authorizedRobotMountPair(mount,"chassis","J6_link"),"MOUNT_PAIR_SYMMETRIC");
  require(!m710::authorizedRobotMountPair(J::object(),"J2_link","chassis"),"LEGACY_MOUNT_POLICY_UNCHANGED");
  for(const auto& pair:std::vector<std::pair<std::string,std::string>>{{"cup","chassis"},{"flange","chassis"},
      {"J2_link","conveyor_transverse"},{"J2_link","trailer_floor"},{"J2_link","other_chassis"},{"J2_link","J3_link"}})
    require(!m710::authorizedRobotMountPair(mount,pair.first,pair.second),"MOUNT_PERMISSION_NARROW");
  optimized=argc>1&&std::string(argv[1])=="optimized";rclcpp::init(argc,argv);checkWorldShapePoses();auto s=scene();auto req=request();m710::ProcessPolicy p(s,req);
  auto tool_req=request();
  tool_req["planning_only"]=true;tool_req["planning_mode"]="cold_from_scratch";
  tool_req["clearance_policy"]["tool_links"]={"cup"};
  tool_req["clearance_policy"]["source_policy"]["tool_chassis_contact_target_ids"]={"target"};
  tool_req["process_policy"]["collision_policy"]=tool_req["clearance_policy"]["source_policy"];
  tool_req["allowed_pairs"]={{"cup","chassis"}};
  {m710::ProcessPolicy permitted(scene(),tool_req);}
  for(int invalid=0;invalid<5;++invalid) {
    auto r=tool_req;
    if(invalid==0)r["planning_only"]=false;
    if(invalid==1)r["allowed_pairs"]={{"cup","trailer_left_wall"}};
    if(invalid==2)r["allowed_pairs"]={{"target","chassis"}};
    if(invalid==3)r["allowed_pairs"]={{"flange","chassis"}};
    if(invalid==4) {
      r["clearance_policy"]["source_policy"]["tool_chassis_contact_target_ids"]={"other_layer"};
      r["process_policy"]["collision_policy"]=r["clearance_policy"]["source_policy"];
    }
    bool denied=false;try {m710::ProcessPolicy forbidden(scene(),r);}
    catch(const std::exception& e) {denied=std::string(e.what())=="PROCESS_UNSCOPED_ACM_PERMISSION";}
    require(denied,"TOOL_CHASSIS_SCOPE_MUST_FAIL_CLOSED");
  }
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

  auto mask_scene=scene();auto masks=request();
  for(int i=0;i<72;++i)masks["process_policy"]["commanded_active_mask"][i]=false;
  m710::ProcessPolicy no_cups(mask_scene,masks);
  require(!no_cups.check(mask_scene->getCurrentState())&&no_cups.last_failure.at("reason")=="NO_COMMANDED_CUP","MASK_CHANGE_REUSED");
  // Compression comparison at both sides of the unchanged 5 mm boundary.
  for(double shift:{-.005-2e-9,-.005,-.005+2e-9}) {
    auto cs=scene();auto cr=request();
    // Move target within the flange frame. Keep request's actual target pose bound.
    cs->getCurrentStateNonConst().clearAttachedBody("target");
    auto b=box("target",0.);b["pose"]=transform(0.,-shift);
    moveit_msgs::msg::AttachedCollisionObject aa;aa.link_name="flange";aa.object=object(b);aa.object.header.frame_id="flange";
    aa.object.primitive_poses[0].position.z=-shift;aa.touch_links={"cup"};
    require(cs->processAttachedCollisionObjectMsg(aa),"BOUNDARY_ATTACH");cr["process_policy"]["target"]=b;
    auto rr=cr;rr["native_check_reuse"]=false;auto oo=cr;oo["native_check_reuse"]=true;
    m710::ProcessPolicy reference(cs,rr),compiled(cs,oo);
    require(reference.check(cs->getCurrentState())==compiled.check(cs->getCurrentState()),"COMPRESSION_BOUNDARY_DIFFERENCE");
    require(reference.last_failure==compiled.last_failure,"COMPRESSION_WITNESS_DIFFERENCE");
  }
  for(const std::string stage:{"pregrasp","contact","withdrawal"}) {
    auto empty=scene();empty->getCurrentStateNonConst().clearAttachedBody("target");
    require(empty->processCollisionObjectMsg(object(box("target",0.))),"RELEASE_TARGET");
    auto r=request();r["stage"]=stage;r["process_policy"]["stage"]=stage;r["process_policy"]["initial_proximity"]=nullptr;
    r["world"].push_back(box("target",0.));
    m710::ProcessPolicy check_empty(empty,r);require(check_empty.check(empty->getCurrentState(),false),"UNLOADED_CHECK");
    require(check_empty.evidence().at("seal_geometry_evaluations")==0,"HELD_SEAL_USED_WITHOUT_ATTACHMENT");
  }
  std::cout<<J({{"status","PASS"},{"scope","synthetic_native_process_policy_and_scene_frames_only"},{"cases",24}}).dump()<<std::endl;rclcpp::shutdown();
}
