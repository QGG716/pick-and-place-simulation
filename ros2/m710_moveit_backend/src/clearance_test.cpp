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
void rigidCartonCases(J& out) {
  const auto tools=m710::eco65RigidToolIds();std::set<std::string> ids;
  for(int i=0;i<12;++i)ids.insert("box_"+std::to_string(i));
  J extraction={{"schema","eco65_attached_target_stack_exemption_v1"},
    {"continuous_task_mode","eco65_coupled_cycle_v1"},{"stage","extraction"},
    {"scope","ATTACHED_TARGET_VS_REMAINING_ORIGINAL_CARTONS_ONLY"},{"original_carton_ids",ids},
    {"original_carton_inventory_fingerprint",std::string(64,'b')},{"allow_intersection",true},
    {"geometry_retained",true},{"physical_execution","NOT_RUN"},{"execution_ready",false}};
  J rule={{"schema","eco65_rigid_tool_carton_contact_v1"},
    {"scope","ENUMERATED_RIGID_TOOL_VS_ORIGINAL_CARTONS_WORLD_OR_ATTACHED"},
    {"continuous_task_mode","eco65_coupled_cycle_v1"},{"rigid_tool_links",tools},
    {"rigid_tool_members",J::array()},{"original_carton_ids",ids},
    {"original_carton_inventory_fingerprint",std::string(64,'b')},{"include_current_target",true},
    {"include_attached_target",true},{"include_released_target",true},{"geometry_retained",true},
    {"allow_intersection",true},{"physical_execution","NOT_RUN"},{"execution_ready",false}};
  for(const auto& id:tools)rule["rigid_tool_members"].push_back({{"id",id},{"semantic","conservative_rigid"},{"mesh_sha256",std::string(64,'a')}});
  J source={{"schema","eco65_pair_collision_policy_v6"},{"rigid_tool_carton_contact",rule},
    {"extraction_carton_contact",extraction},{"stack_contact_mode","attached_target_remaining_original_cartons_during_extraction"},
    {"stack_contact_stages",{"extraction"}}};
  J req={{"clearance_policy",{{"source_policy",source}}},{"continuous_task_mode","eco65_coupled_cycle_v1"},
    {"process_policy",{{"rigid_tool_carton_contact",rule},{"target_id","box_0"},{"task_contract",{{"stack_carton_ids",ids}}}}},
    {"completed_ids",J::array()},{"attachment",nullptr},{"world",J::array()}};
  for(const auto& id:ids)req["world"].push_back({{"id",id}});
  const auto world=m710::eco65RigidCartonPairs(req);require(world.size()==540,"RIGID_CARTON_EXACT_540_PAIRS");
  auto attached=req;attached["world"].erase(0);attached["attachment"]={{"id","box_0"}};
  require(m710::eco65RigidCartonPairs(attached)==world,"RIGID_CARTON_ATTACHED_CONTINUITY");
  auto drained=attached;drained["attachment"]=nullptr;drained["completed_ids"]={"box_0"};
  ids.erase("box_0");drained["process_policy"]["task_contract"]["stack_carton_ids"]=ids;
  require(m710::eco65RigidCartonPairs(drained).size()==495,"RIGID_CARTON_DRAIN_REMOVES_PAIRS");
  auto bad=req;bad["attachment"]={{"id","box_0"}};bool rejected=false;
  try{m710::eco65RigidCartonPairs(bad);}catch(const std::exception&){rejected=true;}
  require(rejected,"RIGID_CARTON_DUPLICATE_OWNERSHIP_REJECTED");
  bad=req;bad["clearance_policy"]["source_policy"]["rigid_tool_carton_contact"]["rigid_tool_links"].push_back("link_3");rejected=false;
  try{m710::eco65RigidCartonPairs(bad);}catch(const std::exception&){rejected=true;}
  require(rejected,"RIGID_CARTON_ROBOT_WIDENING_REJECTED");out["rigid_carton_policy"]="PASS";
}
void controlledRepeatedPathCases(J& out) {
  auto p=policy();p["schema"]="eco65_native_empty_clearance_v1";
  p["source_policy"]={{"schema","eco65_pair_collision_policy_v1"},{"collision_margin_m",.002},
    {"mesh_error_reserve_m",.0002},{"required_pair_clearance_m",.0024},
    {"self_collision_clearance_m",.0024},{"contact_tolerance_m",.0004}};
  J req={{"attachment",nullptr},{"allowed_pairs",J::array({J::array({"base_link","robot_mount_plate"})})},
    {"identity",{{"policy_scope","CONTROLLED_EMPTY_NO_CONTACT"}}}};
  auto s=scene(.02);J path={{0.},{.0001},{.0002},{.0002}};
  m710::Clearance c(s,p,"optimized",nullptr,req);c.phase="output";
  require(c.checkPath(s->getCurrentState(),path,.0005).is_null(),"CONTROLLED_PATH_VALID");
  require(c.evidence()["counts"]["output"]["queries"]==5,"ALL_DISTINCT_STATES_CHECKED_ONCE");
  require(c.evidence()["exact_repeated_path_states_reused"]==4,"ONLY_EXACT_REPEATS_REUSED");
  c.checkPath(s->getCurrentState(),path,.0005);
  require(c.evidence()["counts"]["output"]["queries"]==10,"NO_CROSS_REQUEST_REUSE");
  m710::Clearance tiny(s,p,"optimized",nullptr,req);tiny.phase="output";
  tiny.checkPath(s->getCurrentState(),J{{0.},{1e-9}},.0005);
  require(tiny.evidence()["counts"]["output"]["queries"]==3,"NO_APPROXIMATE_STATE_REUSE");
  m710::Clearance legacy(s,policy());legacy.phase="output";
  legacy.checkPath(s->getCurrentState(),path,.0005);
  require(legacy.evidence()["counts"]["output"]["queries"]==9,"LEGACY_SAMPLING_UNCHANGED");
  require(legacy.evidence()["exact_repeated_path_states_reused"]==0,"LEGACY_REUSE_DISABLED");
  auto changed=s->diff();changed->decoupleParent();changed->processCollisionObjectMsg(box("wall",0.));
  m710::Clearance obstructed(changed,p,"optimized",nullptr,req);
  require(!obstructed.checkPath(changed->getCurrentState(),path,.0005).is_null(),"SCENE_CHANGE_NOT_CACHED");
  s->processCollisionObjectMsg(box("wall",.07));
  m710::Clearance edge(s,p,"optimized",nullptr,req);auto state=s->getCurrentState();
  for(double q:{-.1,.1}) {state.setJointGroupPositions("manipulator",std::vector<double>{q});state.update();require(edge.check(state),"CONTROLLED_FREE_ENDPOINTS");}
  require(!edge.checkPath(s->getCurrentState(),J{{-.1},{.1}},.04).is_null(),"CONTROLLED_INTERIOR_COLLISION_RETAINED");
  out["controlled_exact_repeat_reuse"]="PASS";
}

void conservativeBoundsCases(J& out) {
  auto shape=std::make_shared<fcl::Boxd>(2.,.02,.04);
  Eigen::Isometry3d T=Eigen::Isometry3d::Identity();
  T.linear()=Eigen::AngleAxisd(M_PI/4,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  fcl::CollisionObjectd object(shape,T);auto bound=m710::conservativeWorldBounds(object);
  require(std::abs(bound.max_.x()-.714177848998413)<1e-10 &&
          std::abs(bound.max_.y()-.714177848998413)<1e-10 &&
          std::abs(bound.max_.z()-.02)<1e-10,"ROTATED_LONG_SOLID_BOUND_NUMERIC");
  require(object.getAABB().max_.z()>1.,"FCL_RADIUS_CUBE_BASELINE");
  // Literal local corner extents remain enclosed for 200 deterministic poses.
  for(int k=0;k<200;++k) {
    T.linear()=(Eigen::AngleAxisd(k*.137,Eigen::Vector3d::UnitX())*
      Eigen::AngleAxisd(k*.113,Eigen::Vector3d::UnitY())*
      Eigen::AngleAxisd(k*.071,Eigen::Vector3d::UnitZ())).toRotationMatrix();
    T.translation()=Eigen::Vector3d(k*.03,-k*.02,k*.01);object.setTransform(T);
    bound=m710::conservativeWorldBounds(object);
    for(double x:{-1.,1.})for(double y:{-.01,.01})for(double z:{-.02,.02}) {
      Eigen::Vector3d p=T*Eigen::Vector3d(x,y,z);
      require((p.array()>=bound.min_.array()).all()&&(p.array()<=bound.max_.array()).all(),"BOUNDS_MUST_CONTAIN_ALL_CORNERS");
    }
  }
  out["conservative_rotated_local_bounds"]="PASS";
}

void physicalSingleCartonCases(J& out) {
  J I={{1.,0.,0.,0.},{0.,1.,0.,0.},{0.,0.,1.,0.},{0.,0.,0.,1.}};
  J pose={{1.,0.,0.,0.},{0.,0.,1.,-.05},{0.,-1.,0.,0.},{0.,0.,0.,1.}};
  J g={{"connected_cup_ids",{"seal"}},{"topology",{{"uncovered_cups_self_close",true}}},
       {"normal_cad",{0.,-1.,0.}},{"terminal_band_by_contour",{{"seal","tool"}}},
       {"flange_from_contact_frame",I},
       {"contours",J::array({{{"id","seal"},{"points_m",{{-.01,0.,-.01},{.01,0.,-.01},{.01,0.,.01},{-.01,0.,.01}}}}})}};
  J target={{"id","target"},{"size",{.15,.1,.1}},{"pose",pose}};
  J spec={{"geometry",g},{"target_id","target"},{"target_pose",pose},{"size_m",{.15,.1,.1}},
    {"face",{{"center_m",{0.,0.,0.}},{"normal",{0.,1.,0.}},{"u",{1.,0.,0.}},{"v",{0.,0.,-1.}},{"size_m",{.15,.1}}}},
    {"active_ids",{"seal"}},{"allow_contact",true},{"supports",J::array()}};
  J req={{"identity",{{"policy_scope","PHYSICAL_SINGLE_CARTON"}}},{"single_carton_contact",spec},
    {"attachment",nullptr},{"world",J::array({target})},
    {"allowed_pairs",J::array({J::array({"base_link","robot_mount_plate"}),J::array({"target","tool"})})}};
  auto rejects=[](const J& request) {bool bad=false;try{m710::SingleCartonContact c(request);}catch(const std::exception&){bad=true;}require(bad,"PHYSICAL_CONTACT_MUST_REJECT");};
  auto old=m710::nativeFrames();m710::nativeFrames().flange_link="robot";
  auto s=scene(.02);auto state=s->getCurrentState();m710::SingleCartonContact c(req);
  require(c.check(state).is_null(),"PHYSICAL_SEAL_CONTACT_VALID");
  state.setJointGroupPositions("manipulator",std::vector<double>{.08});state.update();
  require(!c.check(state).is_null(),"PHYSICAL_EDGE_OVERHANG_REJECTED");
  auto bad=req;bad["single_carton_contact"]["geometry"]["flange_from_contact_frame"][1][3]=-.001;
  m710::SingleCartonContact penetration(bad);require(!penetration.check(s->getCurrentState()).is_null(),"PHYSICAL_CUP_PENETRATION_REJECTED");
  bad=req;bad["allowed_pairs"].push_back({"target","robot"});rejects(bad);
  bad=req;bad["single_carton_contact"]["active_ids"].push_back("seal");rejects(bad);
  bad=req;bad["world"][0]["pose"][0][3]=.01;rejects(bad);
  bad=req;bad["single_carton_contact"]["face"]["normal"]={0.,0.,1.};rejects(bad);
  // Exact named support can contact the loaded box, never penetrate its top.
  auto loaded=req;loaded["attachment"]={{"id","target"},{"size",{.15,.1,.1}},{"pose",pose},{"touch_links",{"tool"}}};
  auto floorpose=I;floorpose[2][3]=-.055;
  loaded["world"].push_back({{"id","support"},{"size",{1.,1.,.01}},{"pose",floorpose}});
  loaded["single_carton_contact"]["supports"]=J::array({{{"id","support"},{"top_world_m",-.05}}});
  loaded["allowed_pairs"].push_back({"target","support"});
  m710::SingleCartonContact supporting(loaded);require(supporting.check(s->getCurrentState()).is_null(),"PHYSICAL_SUPPORT_CONTACT");
  bad=loaded;bad["attachment"]["pose"][2][3]=-.001;m710::SingleCartonContact under(bad);
  require(!under.check(s->getCurrentState()).is_null(),"PHYSICAL_PAYLOAD_SUPPORT_PENETRATION");
  bad=loaded;bad["single_carton_contact"]["supports"][0]["top_world_m"]=-.1;rejects(bad);
  // A passive, nonsealing ring crosses the target edge. Its terminal skirt
  // can touch the target plane, but contributes no anticipated seal.
  auto passive=req;
  auto& pg=passive["single_carton_contact"]["geometry"];
  pg["connected_cup_ids"].push_back("edge");pg["terminal_band_by_contour"]["edge"]="passive_lip";
  pg["contours"].push_back({{"id","edge"},{"points_m",{{.06,0.,-.01},{.09,0.,-.01},{.09,0.,.01},{.06,0.,.01}}}});
  passive["single_carton_contact"]["target_lip_contact_ids"]={"seal","edge"};
  passive["allowed_pairs"].push_back({"target","passive_lip"});
  m710::SingleCartonContact edge(passive);require(edge.check(s->getCurrentState()).is_null(),"PASSIVE_EDGE_LIP_CONTACT_ALLOWED");
  bad=passive;bad["single_carton_contact"]["active_ids"].push_back("edge");
  m710::SingleCartonContact falseSeal(bad);require(!falseSeal.check(s->getCurrentState()).is_null(),"PASSIVE_EDGE_IS_NOT_SEAL");
  bad=passive;bad["single_carton_contact"]["geometry"]["contours"][1]["points_m"][0][1]=-.001;
  m710::SingleCartonContact edgeUnder(bad);require(!edgeUnder.check(s->getCurrentState()).is_null(),"PASSIVE_EDGE_CANNOT_PENETRATE");
  bad=passive;bad["single_carton_contact"]["geometry"]["topology"]["uncovered_cups_self_close"]=false;rejects(bad);
  bad=passive;bad["allowed_pairs"].push_back({"neighbor","passive_lip"});rejects(bad);
  bad=passive;bad["allowed_pairs"].push_back({"target","rigid_insert"});rejects(bad);
  bad=passive;bad["single_carton_contact"]["target_lip_contact_ids"].push_back("unknown");rejects(bad);
  bad=passive;bad["single_carton_contact"]["target_lip_contact_ids"].push_back("edge");rejects(bad);
  bad=passive;bad["single_carton_contact"]["target_lip_contact_ids"]={"edge"};rejects(bad);
  bad=passive;bad["single_carton_contact"]["allow_contact"]=false;rejects(bad); // no approach exemption
  auto pload=passive;pload["single_carton_contact"]["allow_contact"]=false;
  pload["attachment"]={{"id","target"},{"size",{.15,.1,.1}},{"pose",pose},{"touch_links",{"tool","passive_lip"}}};
  m710::SingleCartonContact held(pload);require(held.check(s->getCurrentState()).is_null(),"LOADED_PASSIVE_EDGE_CONTACT");
  bad=pload;bad["attachment"]["touch_links"].push_back("robot");rejects(bad);
  bad=pload;bad["single_carton_contact"]["active_ids"].push_back("edge");
  m710::SingleCartonContact badLoadedSeal(bad);require(!badLoadedSeal.check(s->getCurrentState()).is_null(),"LOADED_ACTIVE_EDGE_REJECTED");
  bad=pload;bad["attachment"]["pose"][1][3]=-.051;
  m710::SingleCartonContact gap(bad);require(!gap.check(s->getCurrentState()).is_null(),"LOADED_ATTACHMENT_GAP_REJECTED");
  bad=pload;bad["attachment"]["pose"][1][3]=-.049;
  m710::SingleCartonContact loadedUnder(bad);require(!loadedUnder.check(s->getCurrentState()).is_null(),"LOADED_ATTACHMENT_PENETRATION_REJECTED");
  bad=pload;bad["attachment"]["pose"][0][3]=.08;
  m710::SingleCartonContact slide(bad);require(!slide.check(s->getCurrentState()).is_null(),"LOADED_ATTACHMENT_SLIDE_REJECTED");
  bad=pload;bad["single_carton_contact"]["geometry"]["contours"][1]["points_m"][0][1]=-.001;
  m710::SingleCartonContact loadedEdgeUnder(bad);require(!loadedEdgeUnder.check(s->getCurrentState()).is_null(),"LOADED_PASSIVE_EDGE_PENETRATION_REJECTED");
  out["physical_passive_lip_contact_and_loaded_seal_guards"]="PASS";
  // A small observed patch may lie on a much larger uncertain payload bound.
  // Only the observed rectangle counts as a seal, including while attached.
  auto partial=req;
  partial["single_carton_contact"]["geometry_uncertainty"]={{"kind","BOUNDED_CUBOID_UNION"}};
  partial["single_carton_contact"]["size_m"]={.3,.2,.1};partial["world"][0]["size"]={.3,.2,.1};
  m710::SingleCartonContact patch(partial);require(patch.check(s->getCurrentState()).is_null(),"PARTIAL_OBSERVED_PATCH_ACCEPTED");
  state.setJointGroupPositions("manipulator",std::vector<double>{.08});state.update();
  require(!patch.check(state).is_null(),"INFERRED_ENVELOPE_IS_NOT_SEAL_SUPPORT");
  auto partial_loaded=partial;partial_loaded["attachment"]={{"id","target"},{"size",{.3,.2,.1}},{"pose",pose},{"touch_links",{"tool"}}};
  m710::SingleCartonContact held_patch(partial_loaded);require(held_patch.check(s->getCurrentState()).is_null(),"PARTIAL_LOADED_PATCH_VALID");
  bad=partial_loaded;bad["attachment"]["pose"][0][3]=.08;
  m710::SingleCartonContact shifted_patch(bad);require(!shifted_patch.check(s->getCurrentState()).is_null(),"PARTIAL_LOADED_SLIDE_REJECTED");
  bad=partial;bad["single_carton_contact"]["face"]["center_m"][1]=-.001;rejects(bad);
  bad=partial;bad["single_carton_contact"].erase("geometry_uncertainty");rejects(bad);
  partial["site_workspace_bounds_m"]={{-1.,-1.,-1.},{1.,1.,1.}};
  m710::SingleCartonContact site_inside(partial);require(site_inside.check(s->getCurrentState()).is_null(),"SITE_WORKSPACE_ACCEPTED");
  partial["site_workspace_bounds_m"]={{.5,.5,.5},{1.,1.,1.}};
  m710::SingleCartonContact site_outside(partial);require(!site_outside.check(s->getCurrentState()).is_null(),"UNOBSERVED_OUTSIDE_WORKSPACE_REJECTED");
  out["partial_observed_patch_and_complete_uncertainty_envelope"]="PASS";
  m710::nativeFrames()=old;out["physical_single_carton_contact_and_loaded_support"]="PASS";
}

// Step 5: TWO_LEVEL must take exactly the EXACT decisions (same validity,
// same first failing pair and distance) and its proxies must be conservative.
planning_scene::PlanningScenePtr twoJointScene() {
  const char* urdf_text=R"(<robot name="two"><link name="world"/>
    <link name="base"><collision><geometry><box size="0.2 0.2 0.05"/></geometry></collision></link>
    <joint name="fixed_base" type="fixed"><parent link="world"/><child link="base"/><origin xyz="0 0 -0.3"/></joint>
    <link name="mast"><collision><geometry><box size="0.04 0.04 0.4"/></geometry></collision></link>
    <joint name="fixed_mast" type="fixed"><parent link="base"/><child link="mast"/><origin xyz="-0.3 0 0.2"/></joint>
    <link name="arm"><collision><geometry><box size="0.3 0.04 0.04"/></geometry></collision></link>
    <joint name="J1" type="prismatic"><parent link="world"/><child link="arm"/><axis xyz="1 0 0"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
    <link name="hand"><collision><geometry><box size="0.04 0.08 0.04"/></geometry></collision></link>
    <joint name="fixed_hand" type="fixed"><parent link="arm"/><child link="hand"/><origin xyz="0.17 0 0"/></joint>
    <link name="finger"><collision><geometry><box size="0.02 0.02 0.06"/></geometry></collision></link>
    <joint name="fixed_finger" type="fixed"><parent link="hand"/><child link="finger"/><origin xyz="0.0 0.03 -0.05"/></joint>
    <link name="tip"><collision><geometry><box size="0.12 0.02 0.02"/></geometry></collision></link>
    <joint name="J2" type="revolute"><parent link="hand"/><child link="tip"/><axis xyz="0 0 1"/><origin xyz="0.08 0 0"/><limit lower="-3.1" upper="3.1" effort="1" velocity="1"/></joint>
    <link name="tip2"><collision><geometry><box size="0.02 0.05 0.02"/></geometry></collision></link>
    <joint name="fixed_tip2" type="fixed"><parent link="tip"/><child link="tip2"/><origin xyz="0.07 0 0"/></joint>
  </robot>)";
  auto urdf=urdf::parseURDF(urdf_text);auto srdf=std::make_shared<srdf::Model>();
  require(bool(urdf),"TWO_URDF");
  require(srdf->initString(*urdf,R"(<robot name="two"><group name="manipulator"><joint name="J1"/><joint name="J2"/></group>
    <disable_collisions link1="arm" link2="hand" reason="Adjacent"/><disable_collisions link1="hand" link2="finger" reason="Adjacent"/>
    <disable_collisions link1="hand" link2="tip" reason="Adjacent"/><disable_collisions link1="tip" link2="tip2" reason="Adjacent"/>
    <disable_collisions link1="base" link2="mast" reason="Adjacent"/></robot>)"),"TWO_SRDF");
  auto model=std::make_shared<moveit::core::RobotModel>(urdf,srdf);
  auto result=std::make_shared<planning_scene::PlanningScene>(model);
  result->getCurrentStateNonConst().setToDefaultValues();result->getCurrentStateNonConst().update();return result;
}
moveit_msgs::msg::CollisionObject obstacle(const std::string& name,double x,double y,double z,double sx,double sy,double sz) {
  moveit_msgs::msg::CollisionObject b;b.id=name;b.header.frame_id="world";b.operation=b.ADD;
  shape_msgs::msg::SolidPrimitive shape;shape.type=shape.BOX;shape.dimensions={sx,sy,sz};b.primitives.push_back(shape);
  geometry_msgs::msg::Pose pose;pose.orientation.w=1;pose.position.x=x;pose.position.y=y;pose.position.z=z;b.primitive_poses.push_back(pose);return b;
}
J twoLevelCases() {
  J out;out["mode"]="two_level";out["scope"]="synthetic implementation checks only";
  // Conservative sphere: lower bound <= exact FCL distance for 300 poses.
  {
    auto convex_vertices=std::make_shared<std::vector<fcl::Vector3d>>(std::vector<fcl::Vector3d>{
      {0,0,0},{.1,0,0},{0,.07,0},{0,0,.05},{.06,.05,.04}});
    auto faces=std::make_shared<std::vector<int>>(std::vector<int>{3,0,1,2, 3,0,1,3, 3,0,2,3, 3,1,2,4, 3,1,3,4, 3,2,3,4});
    auto convex=std::make_shared<fcl::Convexd>(convex_vertices,6,faces);
    auto box=std::make_shared<fcl::Boxd>(.08,.03,.05);
    size_t checked=0;
    for(int k=0;k<300;++k) {
      Eigen::Isometry3d A=Eigen::Isometry3d::Identity(),B=Eigen::Isometry3d::Identity();
      A.linear()=(Eigen::AngleAxisd(k*.37,Eigen::Vector3d::UnitX())*Eigen::AngleAxisd(k*.21,Eigen::Vector3d::UnitZ())).toRotationMatrix();
      B.linear()=Eigen::AngleAxisd(k*.53,Eigen::Vector3d::UnitY()).toRotationMatrix();
      A.translation()=Eigen::Vector3d(.001*k,-.0005*k,.0003*k);B.translation()=Eigen::Vector3d(.2-.0009*k,.0004*k,-.0002*k);
      fcl::CollisionObjectd a(convex,A),b(box,B);
      const auto sa=m710::ClearanceWorkspace::localSphere(a),sb=m710::ClearanceWorkspace::localSphere(b);
      const double lower=(A*sa.first-B*sb.first).norm()-sa.second-sb.second;
      fcl::DistanceResultd result;const double d=fcl::distance(&a,&b,fcl::DistanceRequestd(false),result);
      if(d>0) {require(lower<=d+1e-12,"SPHERE_PROXY_NOT_CONSERVATIVE");++checked;}
    }
    out["sphere_lower_bound_conservative"]={{"separated_poses_checked",checked},{"status","PASS"}};
  }
  // Decisions: EXACT and TWO_LEVEL workspaces on the same scene and states.
  size_t states=0,valid=0,invalid=0,constant_cleared=0,group_cleared=0,sphere_cleared=0,unvisited_cleared=0,exact_two=0,exact_one=0;
  for(int layout=0;layout<3;++layout) {
    auto s=twoJointScene();
    s->processCollisionObjectMsg(obstacle("wall",.9+.1*layout,0,0,.05,.6,.4));
    s->processCollisionObjectMsg(obstacle("post",0,.12-.03*layout,0,.04,.04,.3));
    s->processCollisionObjectMsg(obstacle("shelf",.2,-.2,.06*layout,.5,.05,.02));
    const auto* fcl=dynamic_cast<const collision_detection::CollisionEnvFCL*>(s->getCollisionEnvUnpadded().get());
    require(fcl,"FCL_ENV");
    auto required=[](const std::string&,collision_detection::BodyType,const std::string&,collision_detection::BodyType){return .005;};
    m710::ClearanceWorkspace exact(*fcl,s,required,true,0.,{},true,false),two(*fcl,s,required,true,0.,{},true,true);
    require(two.twoLevel() && !exact.twoLevel(),"TWO_LEVEL_FLAG");
    auto state=s->getCurrentState();
    for(int k=0;k<1500;++k) {
      const double j1=-1.+2.*((k*7919)%1500)/1499.,j2=-3.1+6.2*((k*104729)%1499)/1498.;
      state.setJointGroupPositions("manipulator",std::vector<double>{j1,j2});state.update();
      m710::ClearanceWorkspace::Failure fe,fw;
      const bool e=exact.check(state,fe,k%2?"output":"search"),w=two.check(state,fw,k%2?"output":"search");
      require(e==w,"TWO_LEVEL_DECISION_DIFFERS");
      if(!e)require(fe.a==fw.a&&fe.b==fw.b&&fe.distance==fw.distance&&fe.limit==fw.limit,"TWO_LEVEL_FAILURE_DIFFERS");
      ++states;(e?valid:invalid)++;
    }
    for(const auto& entry:two.phases) {
      constant_cleared+=entry.second.proxy_constant_cleared;group_cleared+=entry.second.proxy_group_cleared;
      sphere_cleared+=entry.second.proxy_sphere_cleared;unvisited_cleared+=entry.second.proxy_unvisited_cleared;
      exact_two+=entry.second.narrow_calls;
    }
    for(const auto& entry:exact.phases)exact_one+=entry.second.narrow_calls;
  }
  if(!(valid>100&&invalid>100))throw std::runtime_error("TWO_LEVEL_CASES_MUST_COVER_BOTH_OUTCOMES:valid="+std::to_string(valid)+",invalid="+std::to_string(invalid));
  require(sphere_cleared+group_cleared+unvisited_cleared+constant_cleared>0,"TWO_LEVEL_PROXIES_NOT_EXERCISED");
  require(exact_two<=exact_one,"TWO_LEVEL_MUST_NOT_ADD_EXACT_QUERIES");
  out["decision_equivalence"]={{"states",states},{"valid",valid},{"invalid",invalid},{"exact_fcl_calls_exact_mode",exact_one},
    {"exact_fcl_calls_two_level",exact_two},{"proxy_constant_cleared",constant_cleared},{"proxy_group_cleared",group_cleared},
    {"proxy_sphere_cleared",sphere_cleared},{"proxy_unvisited_cleared",unvisited_cleared},{"status","PASS"}};
  // Scope: TWO_LEVEL is refused outside the physical single-carton schema.
  {
    auto s=scene(.02);J req={{"collision_check_mode","TWO_LEVEL"},{"attachment",nullptr}};bool refused=false;
    try {m710::Clearance c(s,policy(),"optimized",nullptr,req);} catch(const std::exception& e) {refused=std::string(e.what())=="COLLISION_CHECK_MODE_SCOPE_INVALID";}
    require(refused,"TWO_LEVEL_SCOPE_NOT_ENFORCED");
    J bad={{"collision_check_mode","FAST"},{"attachment",nullptr}};refused=false;
    try {m710::Clearance c(s,policy(),"optimized",nullptr,bad);} catch(const std::exception& e) {refused=std::string(e.what())=="COLLISION_CHECK_MODE_INVALID";}
    require(refused,"UNKNOWN_CHECK_MODE_NOT_REFUSED");
    out["scope"]="PASS";
  }
  out["status"]="PASS";return out;
}

int main(int argc,char** argv) {
  const std::string mode=argc>1?argv[1]:"optimized";rclcpp::init(argc,argv);
  if(mode=="two_level") {auto result=twoLevelCases();std::cout<<result.dump(2)<<std::endl;rclcpp::shutdown();return 0;}
  J out;out["mode"]=mode;out["scope"]="synthetic implementation checks only";out["threshold_cases"]=J::array();
  controlledRepeatedPathCases(out);
  conservativeBoundsCases(out);
  physicalSingleCartonCases(out);
  J pairPolicy={{"schema","eco65_pair_collision_policy_v5"},{"robot_tool_exemption",{
    {"schema","eco65_named_robot_tool_exemption_v1"},{"scope","EXACT_LINK_3_VS_PART_026_ONLY"},
    {"exempt_pairs",J::array({J::array({"link_3","part_026"})})},{"tool_semantic","conservative_rigid"},
    {"tool_mesh_sha256","e40dd92e8d06fba77b65e5b078ad7f0ab21bb9b1ad29e564ab4e3675be18e9f4"},
    {"geometry_retained",true},{"allow_intersection",true},{"physical_execution","NOT_RUN"},{"execution_ready",false}}}};
  require(m710::eco65RobotToolExempt(pairPolicy,"link_3","part_026"),"EXACT_PAIR_REQUIRED");
  require(m710::eco65RobotToolExempt(pairPolicy,"part_026","link_3"),"EXACT_PAIR_SYMMETRIC");
  for(const auto& pair:J::array({J::array({"link_2","part_026"}),J::array({"link_3","part_027"}),
      J::array({"link_6","part_041_region_2"}),J::array({"part_026","robot_mount_plate"})}))
    require(!m710::eco65RobotToolExempt(pairPolicy,pair[0],pair[1]),"OTHER_PAIRS_RETAINED");
  auto forged=pairPolicy;forged["robot_tool_exemption"]["exempt_pairs"].push_back({"link_3","part_027"});
  bool scopeRejected=false;try{m710::eco65RobotToolPairs(forged);}catch(const std::exception&){scopeRejected=true;}
  require(scopeRejected,"WIDER_ROBOT_TOOL_PERMISSION_REJECTED");out["named_robot_tool_exemption"]="PASS";
  rigidCartonCases(out);
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
