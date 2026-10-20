// Synthetic geometry tests: implementation evidence, never scene success rates.
#include "clearance.h"
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <iostream>
#include <sstream>
#include <iomanip>
using J=nlohmann::json;
void require(bool value,const char* message) {if(!value) throw std::runtime_error(message);}
planning_scene::PlanningScenePtr scene(double gap,bool cups=false) {
  std::ostringstream xml;xml<<std::setprecision(17);
  xml<<R"(<robot name="test"><link name="world"/><link name="robot"><collision><geometry><box size="0.01 0.01 0.01"/></geometry></collision></link><joint name="J1" type="prismatic"><parent link="world"/><child link="robot"/><axis xyz="1 0 0"/><limit lower="-1" upper="1" velocity="1" effort="1"/></joint><link name="tool"><collision><geometry><box size="0.01 0.01 0.01"/></geometry></collision></link><joint name="mount" type="fixed"><parent link="robot"/><child link="tool"/><origin xyz=")"<<.01+gap<<R"( 0 0"/></joint></robot>)";
  auto source=xml.str();
  if(cups) {
    source.resize(source.size()-8);
    for(int i=1;i<12;++i) {
      const auto name="cup_"+std::to_string(i);
      source+="<link name=\""+name+"\"><collision><geometry><box size=\"0.01 0.01 0.01\"/></geometry></collision></link>"
        "<joint name=\"mount_"+name+"\" type=\"fixed\"><parent link=\"robot\"/><child link=\""+name+
        "\"/><origin xyz=\""+std::to_string(2.+.2*i)+" 0 0\"/></joint>";
    }
    source+="</robot>";
  }
  auto urdf=urdf::parseURDF(source);auto srdf=std::make_shared<srdf::Model>();
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
void extractionContactCases(J& out) {
  auto rejects=[](const std::function<void()>& f) {
    bool rejected=false;try{f();}catch(const std::exception&){rejected=true;}
    require(rejected,"EXTRACTION_SCOPE_MUST_REJECT");
  };
  std::set<std::string> original={"payload","neighbor"};
  for(int i=2;i<12;++i)original.insert("carton_"+std::to_string(i));
  J exemption={{"schema","eco65_attached_target_stack_exemption_v1"},
    {"continuous_task_mode","eco65_coupled_cycle_v1"},{"stage","extraction"},
    {"scope","ATTACHED_TARGET_VS_REMAINING_ORIGINAL_CARTONS_ONLY"},{"original_carton_ids",original},
    {"allow_intersection",true},{"geometry_retained",true},{"physical_execution","NOT_RUN"},{"execution_ready",false}};
  J source={{"schema","eco65_pair_collision_policy_v3"},{"collision_margin_m",.002},
    {"mesh_error_reserve_m",.0002},{"required_pair_clearance_m",.0024},{"self_collision_clearance_m",.0024},
    {"contact_tolerance_m",.0004},{"robot_mast_exemption",{{"exempt_pairs",J::array()}}},
    {"stack_contact_mode","attached_target_remaining_original_cartons_during_extraction"},
    {"stack_contact_stages",{"extraction"}},{"extraction_carton_contact",exemption}};
  auto p=policy();p["schema"]="eco65_native_process_clearance_v1";p["stage"]="extraction";
  p["source_policy"]=source;p["target_id"]="payload";p["payload_id"]="payload";
  J req={{"stage","extraction"},{"continuous_task_mode","eco65_coupled_cycle_v1"},{"attachment",{{"id","payload"}}},
    {"completed_ids",J::array()},{"clearance_policy",p},{"world",J::array()},
    {"process_policy",{{"stage","extraction"},{"target_id","payload"},{"extraction_carton_contact",exemption},
      {"task_contract",{{"target_id","payload"},{"continuous_task_mode","eco65_coupled_cycle_v1"},
        {"stack_carton_ids",original}}}}}};
  auto s=scene(.02);
  moveit_msgs::msg::AttachedCollisionObject attached;attached.link_name="robot";
  attached.object=box("payload",.08);attached.object.header.frame_id="robot";
  require(s->processAttachedCollisionObjectMsg(attached),"EXTRACTION_ATTACH");
  int index=0;
  for(const auto& id:original)if(id!="payload") {
    s->processCollisionObjectMsg(box(id,id=="neighbor"?.085:1.+.05*index++));
    req["world"].push_back({{"id",id}});
  }
  const auto neighbors=m710::eco65ExtractionNeighbors(req);
  require(neighbors.size()==11 && !neighbors.count("payload") && !neighbors.count("tool"),"EXACT_EXTRACTION_PAIRS");
  J inventory=J::array();for(const auto& id:original)inventory.push_back({{"id",id}});
  m710::validateEco65ExtractionContact(source,{{"extraction_carton_contact",exemption}},inventory);
  auto bad_inventory=inventory;bad_inventory[0]["id"]="conveyor";
  rejects([&]{m710::validateEco65ExtractionContact(source,{{"extraction_carton_contact",exemption}},bad_inventory);});
  auto legacy=p;legacy["source_policy"]["schema"]="eco65_pair_collision_policy_v2";
  m710::Clearance old(s,legacy);require(!old.check(s->getCurrentState()),"OLD_POLICY_RETAINS_PAYLOAD_COLLISION");
  collision_detection::CollisionRequest collision_request;collision_detection::CollisionResult collision_result;
  s->checkCollision(collision_request,collision_result);
  require(collision_result.collision,"SYNTHETIC_PAYLOAD_ACTUALLY_INTERSECTS_NEIGHBOR");
  rejects([&]{m710::Clearance missing(s,p,"optimized",nullptr,req);});
  auto enabled=s->diff();enabled->decoupleParent();m710::setEco65ExtractionPairs(enabled,"payload",neighbors);
  m710::Clearance extraction(enabled,p,"optimized",nullptr,req);
  require(extraction.check(enabled->getCurrentState()),"EXTRACTION_ACTUAL_OVERLAP_ALLOWED");
  collision_result.clear();enabled->checkCollision(collision_request,collision_result);
  require(!collision_result.collision,"ORDINARY_COLLISION_USES_SAME_EXTRACTION_PAIRS");
  // The same actual overlapping solids must be rejected at free transit.
  auto free=p;free["stage"]="transit";auto free_req=req;free_req["stage"]="transit";
  free_req["clearance_policy"]=free;free_req["process_policy"]["stage"]="transit";
  require(m710::eco65ExtractionNeighbors(free_req).empty(),"TRANSIT_HAS_NO_EXTRACTION_PAIRS");
  m710::Clearance transit(s,free,"optimized",nullptr,free_req);
  require(!transit.check(s->getCurrentState()),"TRANSIT_RESTORES_PAYLOAD_COLLISION");
  rejects([&]{m710::Clearance leak(enabled,free,"optimized",nullptr,free_req);});
  auto wall=enabled->diff();wall->decoupleParent();wall->processCollisionObjectMsg(box("conveyor",.085));
  m710::Clearance environment(wall,p,"optimized",nullptr,req);
  require(!environment.check(wall->getCurrentState()),"EXTRACTION_CONVEYOR_STILL_CHECKED");
  auto tool=enabled->diff();tool->decoupleParent();tool->processCollisionObjectMsg(box("neighbor",.03));
  m710::Clearance tool_collision(tool,p,"optimized",nullptr,req);
  require(!tool_collision.check(tool->getCurrentState()),"EXTRACTION_TOOL_NEIGHBOR_STILL_CHECKED");
  auto bad=req;bad["attachment"]=nullptr;
  require(m710::eco65ExtractionNeighbors(bad).empty(),"UNATTACHED_HAS_NO_EXTRACTION_PAIRS");
  bad=req;bad["continuous_task_mode"]="legacy";
  require(m710::eco65ExtractionNeighbors(bad).empty(),"LEGACY_HAS_NO_EXTRACTION_PAIRS");
  bad=req;bad["attachment"]["id"]="neighbor";rejects([&]{m710::eco65ExtractionNeighbors(bad);});
  bad=req;bad["process_policy"]["task_contract"]["stack_carton_ids"].push_back("conveyor");
  rejects([&]{m710::eco65ExtractionNeighbors(bad);});
  bad=req;bad["completed_ids"].push_back("neighbor");rejects([&]{m710::eco65ExtractionNeighbors(bad);});
  bad=req;bad["world"].push_back({{"id","payload"}});rejects([&]{m710::eco65ExtractionNeighbors(bad);});
  out["eco65_extraction_contact"]={{"status","PASS"},{"scope","SYNTHETIC_ACTUAL_OVERLAP_AND_RUNTIME_SCOPE"},
    {"pairs",m710::eco65ExtractionEvidence("payload",neighbors)},{"tool_failure",tool_collision.last_failure},
    {"environment_failure",environment.last_failure},{"transit_failure",transit.last_failure}};
}
void cupNeighborCases(J& out) {
  std::set<std::string> ids={"target","neighbor"},bands={"tool"};
  for(int i=2;i<12;++i)ids.insert("box_"+std::to_string(i));
  for(int i=1;i<12;++i)bands.insert("cup_"+std::to_string(i));
  J rule={{"schema","eco65_cup_neighbor_contact_v1"},{"continuous_task_mode","eco65_coupled_cycle_v1"},
    {"scope","ENUMERATED_CUP_TERMINAL_BANDS_VS_REMAINING_NON_TARGET_ORIGINAL_CARTONS"},
    {"terminal_band_links",bands},{"original_carton_ids",ids},{"original_carton_inventory_fingerprint","synthetic"},
    {"geometry_retained",true},{"enabled_and_disabled_cups",true},{"allow_intersection",true},
    {"physical_execution","NOT_RUN"},{"execution_ready",false}};
  J source={{"schema","eco65_pair_collision_policy_v4"},
    {"compliant_cup_neighbor_contact_mode","named_terminal_bands_remaining_original_cartons"},
    {"cup_neighbor_contact",rule},{"extraction_carton_contact",{{"original_carton_ids",ids},
      {"original_carton_inventory_fingerprint","synthetic"}}}};
  J process={{"cup_neighbor_contact",rule},{"target_id","target"},
    {"task_contract",{{"stack_carton_ids",ids}}},{"terminal_band_by_contour",J::object()}};
  for(const auto& name:bands)process["terminal_band_by_contour"][name+"_seal"]=name;
  J request={{"continuous_task_mode","eco65_coupled_cycle_v1"},{"stage","contact"},
    {"completed_ids",J::array()},{"process_policy",process},
    {"clearance_policy",{{"source_policy",source},{"compliant_tool_links",bands}}},{"world",J::array()}};
  auto s=scene(.02,true);int index=0;
  for(const auto& id:ids) {
    s->processCollisionObjectMsg(box(id,id=="neighbor"?.03:10.+.1*index++));
    request["world"].push_back({{"id",id}});
  }
  const auto pairs=m710::eco65CupNeighborPairs(request);
  require(pairs.size()==132,"CUP_NEIGHBOR_EXACT_PAIR_COUNT");
  for(const auto& pair:pairs)require(pair.at(1)!="target" && pair.at(0)!="robot","CUP_SCOPE");
  // Ordinary FCL and the optimized pair workspace see the identical ACM.
  auto p=policy();p["tool_links"]=bands;
  m710::Clearance before(s,p);require(!before.check(s->getCurrentState()),"CUP_CONTACT_REJECTED_BEFORE");
  m710::setEco65CupNeighborPairs(s,pairs);
  m710::Clearance after(s,p);require(after.check(s->getCurrentState()),"CUP_CONTACT_ACCEPTED_AFTER");
  collision_detection::CollisionRequest cr;collision_detection::CollisionResult result;
  s->checkCollision(cr,result);require(!result.collision,"CUP_ACM_FCL_CONSISTENT");
  for(const auto& name:{"target","unknown_environment","neighbor"}) {
    auto changed=s->diff();changed->decoupleParent();
    changed->processCollisionObjectMsg(box(name,std::string(name)=="neighbor"?0.:.03));
    m710::Clearance kept(changed,p);
    require(!kept.check(changed->getCurrentState()),"TARGET_ENVIRONMENT_ROBOT_CHECK_RETAINED");
  }
  auto bad=request;bad["process_policy"]["terminal_band_by_contour"]["tool_seal"]="robot";
  bool rejected=false;try{m710::eco65CupNeighborPairs(bad);}catch(const std::exception&){rejected=true;}
  require(rejected,"CUP_RIGID_RECLASSIFICATION_REJECTED");
  out["eco65_cup_neighbor"]={{"status","PASS"},{"pairs",pairs.size()},
    {"scope","SYNTHETIC_ACTUAL_CONTACT_ACM_AND_RETAINED_TARGET_ROBOT_ENVIRONMENT_CHECKS"}};
}
int main(int argc,char** argv) {
  const std::string mode=argc>1?argv[1]:"optimized";rclcpp::init(argc,argv);J out;out["mode"]=mode;out["scope"]="synthetic implementation checks only";out["threshold_cases"]=J::array();
  for(double gap:{.004,.005-2e-9,.005-.5e-9,.005,.005+.5e-9,.006}) {
    auto s=scene(gap);m710::Clearance c(s,policy(),mode);bool valid=c.check(s->getCurrentState());
    require(valid==(gap+1e-9>=.005),"THRESHOLD_CLASSIFICATION");out["threshold_cases"].push_back({{"gap",gap},{"valid",valid},{"failure",c.last_failure}});
  }
  auto s=scene(.002);auto p=policy();p["tool_links"]=J::array();
  m710::Clearance ordinary(s,p,mode);require(ordinary.check(s->getCurrentState()),"ORDINARY_SELF_MUST_NOT_GET_5MM");
  m710::Clearance tool(s,policy(),mode);require(!tool.check(s->getCurrentState()),"ROBOT_TOOL_MUST_GET_5MM");
  auto diff=s->diff();diff->decoupleParent();diff->getAllowedCollisionMatrixNonConst().setEntry("robot","tool",true);
  m710::Clearance exempt(diff,policy(),mode);require(exempt.check(diff->getCurrentState()),"SCOPED_ASSEMBLY_ALLOWANCE");
  require(!tool.check(s->getCurrentState()),"ACM_LEAK");out["self_tool_and_diff"]="PASS";
  s=scene(.02);s->processCollisionObjectMsg(box("neighbor",.042));
  auto cupdiff=s->diff();cupdiff->decoupleParent();cupdiff->getAllowedCollisionMatrixNonConst().setEntry("tool","neighbor",true);
  m710::Clearance cup(cupdiff,policy(),mode),rigid(s,policy(),mode);
  require(cup.check(cupdiff->getCurrentState()),"EXACT_CUP_PAIR");require(!rigid.check(s->getCurrentState()),"RIGID_NOT_EXEMPT");
  out["cup_permission_not_rigid"]="PASS";
  s=scene(.02);s->processCollisionObjectMsg(box("wall",.07));
  m710::Clearance edge(s,policy(),mode);auto state=s->getCurrentState();
  for(double q:{-.1,.1}) {state.setJointGroupPositions("manipulator",std::vector<double>{q});state.update();require(edge.check(state),"EDGE_ENDPOINT");}
  edge.phase="output";auto failure=edge.checkPath(s->getCurrentState(),J::array({J::array({-.1}),J::array({.1})}),.04);
  require(!failure.is_null(),"EDGE_INTERIOR_MISSED");out["valid_endpoints_invalid_edge"]=failure;out["edge_counts"]=edge.evidence();
  // A checker is rebuilt after attachment; the world must lose that same ID.
  s=scene(.02);s->processCollisionObjectMsg(box("payload",.08));
  auto attached=s->diff();attached->decoupleParent();attached->getWorldNonConst()->removeObject("payload");
  moveit_msgs::msg::AttachedCollisionObject a;a.link_name="robot";a.object=box("payload",.08);a.object.header.frame_id="robot";
  require(attached->processAttachedCollisionObjectMsg(a),"ATTACH");attached->processCollisionObjectMsg(box("neighbor",.092));
  p=policy();p["payload_id"]="payload";m710::Clearance payload(attached,p,mode);
  require(!payload.check(attached->getCurrentState()),"ATTACHED_PAYLOAD_GAP");
  require(!attached->getWorld()->hasObject("payload") && s->getWorld()->hasObject("payload"),"ATTACH_DIFF_LEAK");
  out["attached_failure"]=payload.last_failure;
  // Same geometry, different stage/policy thresholds must not share results.
  auto receiver=attached->diff();receiver->decoupleParent();
  receiver->getWorldNonConst()->removeObject("neighbor");receiver->processCollisionObjectMsg(box("conveyor",.096));
  p["conveyor_ids"]={"conveyor"};p["receiver_reserve_m"]=.003;
  m710::Clearance transit(receiver,p,mode);require(!transit.check(receiver->getCurrentState()),"PAYLOAD_RECEIVER_RESERVE");
  p["stage"]="pregrasp";m710::Clearance pregrasp(receiver,p,mode);
  require(pregrasp.check(receiver->getCurrentState()),"STAGE_THRESHOLD_LEAK");
  require(!transit.check(receiver->getCurrentState()),"POLICY_CACHE_LEAK");
  out["payload_receiver_stage_thresholds"]="PASS";
  if(mode=="optimized") {
    auto changed=receiver->getCurrentState();changed.clearAttachedBody("payload");changed.update();
    require(!transit.check(changed),"STALE_ATTACHMENT_ACCEPTED");
    require(transit.last_failure.at("reason")=="DISTANCE_QUERY_FAILED","STALE_ATTACHMENT_REASON");
    require(!transit.check(receiver->getCurrentState()),"RESTORED_ATTACHMENT_LEAK");
    out["stale_attachment_fail_closed"]="PASS";
    extractionContactCases(out);
    cupNeighborCases(out);
  }
out["status"]="PASS";std::cout<<out.dump(2)<<std::endl;rclcpp::shutdown();
}
