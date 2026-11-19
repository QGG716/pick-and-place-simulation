// Synthetic geometry tests: implementation evidence, never scene success rates.
#include "clearance.h"
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <random>
#include <cmath>
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
  // Same 0.4 mm loaded-support guard in signed plane distance. The named
  // supporting carton is bound to the exact source object, not a blanket ACM.
  auto inclined=loaded;const double angle=5.*M_PI/180.;
  const Eigen::Vector3d sn(std::sin(angle),0.,std::cos(angle));
  const double sd=-.075*sn.x()-.05*sn.z();
  J plane={{"normal_world",{sn.x(),sn.y(),sn.z()}},{"offset_m",sd}};
  auto sp=I;sp[0][0]=std::cos(angle);sp[0][2]=std::sin(angle);sp[2][0]=-std::sin(angle);sp[2][2]=std::cos(angle);
  for(int k=0;k<3;++k)sp[k][3]=sn[k]*(sd-.005);
  inclined["world"][1]={{"id","support"},{"size",{1.,1.,.01}},{"pose",sp},
    {"category","confirmed_stack_support"},{"supports_target_id","target"},
    {"support_basis","SYNTHETIC_STACK_TEST_NOT_FIELD_EVIDENCE"},{"support_plane",plane}};
  inclined["single_carton_contact"]["supports"]=J::array({{{"id","support"},{"plane",plane}}});
  m710::SingleCartonContact inclined_support(inclined);
  require(inclined_support.check(s->getCurrentState()).is_null(),"INCLINED_SUPPORT_CONTACT");
  auto near_limit=s->getCurrentState();
  near_limit.setJointGroupPositions("manipulator",std::vector<double>{-.00039/sn.x()});near_limit.update();
  require(inclined_support.check(near_limit).is_null(),"INCLINED_SUPPORT_039MM_WITHIN_GUARD");
  near_limit.setJointGroupPositions("manipulator",std::vector<double>{-.00041/sn.x()});near_limit.update();
  require(!inclined_support.check(near_limit).is_null(),"INCLINED_SUPPORT_041MM_EXCEEDS_GUARD");
  auto below=s->getCurrentState();below.setJointGroupPositions("manipulator",std::vector<double>{-.01});below.update();
  require(!inclined_support.check(below).is_null(),"INCLINED_SUPPORT_04MM_PENETRATION_GUARD");
  bad=inclined;bad["world"][1]["support_basis"]="";rejects(bad);
  bad=inclined;bad["world"][1]["supports_target_id"]="other-carton";rejects(bad);
  bad=inclined;bad["world"][1]["pose"][2][3]=.1;rejects(bad);
  bad=inclined;bad["single_carton_contact"]["supports"][0]["plane"]["normal_world"]={0.,0.,2.};rejects(bad);
  bad=inclined;bad["single_carton_contact"]["supports"][0]["plane"]["offset_m"]=sd+.001;rejects(bad);
  bad=inclined;bad["attachment"]=nullptr;rejects(bad);
  bad=inclined;bad["allowed_pairs"].push_back({"robot","support"});rejects(bad);
  out["inclined_confirmed_carton_support_and_04mm_guard"]="PASS";
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
  // A measured side may extend beyond the front rectangle. Bind the larger
  // full payload to that unchanged observed seal patch arithmetically.
  auto observed_side=req;auto side_pose=pose;side_pose[0][3]=.004;
  observed_side["world"][0]["pose"]=side_pose;observed_side["world"][0]["size"]={.158,.1,.1};
  auto& os=observed_side["single_carton_contact"];
  os["target_pose"]=side_pose;os["size_m"]={.158,.1,.1};
  os["observed_side_envelope"]={{"source","ALL_ACCEPTED_SIDE_PLANE_SAMPLES_NO_SIZE_PRIOR"},
    {"front_dimensions_m",{.15,.1,.1}},{"expansion_m",{0.,.008}}};
  m710::SingleCartonContact measured_side(observed_side);
  require(measured_side.check(s->getCurrentState()).is_null(),"OBSERVED_SIDE_BOUND_WITH_UNCHANGED_PATCH");
  observed_side["attachment"]={{"id","target"},{"size",{.158,.1,.1}},{"pose",side_pose},{"touch_links",{"tool"}}};
  m710::SingleCartonContact measured_side_loaded(observed_side);
  require(measured_side_loaded.check(s->getCurrentState()).is_null(),"OBSERVED_SIDE_LOADED_PATCH_VALID");
  bad=observed_side;bad["attachment"]["pose"][0][3]=.073;
  m710::SingleCartonContact side_no_seal_gain(bad);
  require(!side_no_seal_gain.check(s->getCurrentState()).is_null(),"OBSERVED_SIDE_EXPANSION_GAINS_NO_SEAL_AREA");
  bad=observed_side;bad["single_carton_contact"]["observed_side_envelope"]["expansion_m"]={0.,.007};rejects(bad);
  bad=observed_side;bad["single_carton_contact"]["observed_side_envelope"]["expansion_m"]={-.001,.009};rejects(bad);
  bad=observed_side;bad["single_carton_contact"].erase("observed_side_envelope");rejects(bad);
  bad=observed_side;bad["single_carton_contact"]["observed_side_envelope"]["source"]="UNBOUND";rejects(bad);
  out["observed_side_envelope_preserves_contact_and_attachment"]="PASS";
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
  // Measured contact plane stays at the original CAD seal plane while the
  // collision box expands in all directions. It is not moved to the bound.
  auto measured=req;
  measured["single_carton_contact"]["geometry_uncertainty"]={{"kind","MEASURED_BOUNDARY_INTERVALS"}};
  measured["single_carton_contact"]["measured_box"]={{"pose",pose},{"size_m",{.15,.1,.1}}};
  measured["single_carton_contact"]["size_m"]={.17,.12,.12};measured["world"][0]["size"]={.17,.12,.12};
  m710::SingleCartonContact measured_contact(measured);
  require(measured_contact.check(s->getCurrentState()).is_null(),"MEASURED_PLANE_SEPARATE_FROM_ENVELOPE");
  measured["attachment"]={{"id","target"},{"size",{.17,.12,.12}},{"pose",pose},{"touch_links",{"tool"}}};
  m710::SingleCartonContact measured_loaded(measured);
  require(measured_loaded.check(s->getCurrentState()).is_null(),"MEASURED_LOADED_SEAL_PLANE");
  bad=measured;bad["attachment"]["pose"][1][3]=-.049;
  m710::SingleCartonContact measured_under(bad);
  require(!measured_under.check(s->getCurrentState()).is_null(),"MEASURED_LOADED_PENETRATION_REJECTED");
  bad=measured;bad["single_carton_contact"]["measured_box"]["size_m"][0]=.3;rejects(bad);
  bad=measured;bad["single_carton_contact"]["face"]["center_m"][1]=.01;rejects(bad);
  out["measured_contact_and_conservative_payload_envelope"]="PASS";
  // Step 6 scope: ADAPTIVE only without contact permission and without support.
  require(!c.adaptiveEligible() && !supporting.adaptiveEligible() && held.adaptiveEligible(),"ADAPTIVE_SCOPE_ELIGIBILITY");
  {
    std::vector<std::tuple<const moveit::core::LinkModel*,double,double>> inside,outside,none;
    site_inside.workspaceMargins(s->getCurrentState(),inside);site_outside.workspaceMargins(s->getCurrentState(),outside);
    c.workspaceMargins(s->getCurrentState(),none);
    require(!inside.empty() && none.empty(),"ADAPTIVE_WORKSPACE_MARGINS_PRESENT");
    for(const auto& m:inside)require(std::get<1>(m)>=0 && std::get<2>(m)>0,"ADAPTIVE_WORKSPACE_MARGIN_INSIDE");
    bool negative=false;for(const auto& m:outside)negative|=std::get<1>(m)<0;require(negative,"ADAPTIVE_WORKSPACE_MARGIN_OUTSIDE");
  }
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

// Step 6: adaptive sent-path audit. The lever bound must contain every body
// displacement; a skipped sample must pass the exact check; on every path the
// adaptive walk takes the same decision at the same first failing sample.
planning_scene::PlanningScenePtr armScene(double post_gap=std::nan("")) {
  const char* urdf_text=R"(<robot name="arm"><link name="world"/>
    <link name="base"><collision><origin xyz="0 0 -0.05"/><geometry><box size="0.3 0.3 0.1"/></geometry></collision></link>
    <joint name="fixed_base" type="fixed"><parent link="world"/><child link="base"/></joint>
    <link name="link1"><collision><origin xyz="0 0 0.15"/><geometry><box size="0.1 0.1 0.3"/></geometry></collision></link>
    <joint name="J1" type="revolute"><parent link="base"/><child link="link1"/><axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="1" velocity="1"/></joint>
    <link name="link2"><collision><origin xyz="0.25 0 0"/><geometry><box size="0.5 0.06 0.06"/></geometry></collision></link>
    <joint name="J2" type="revolute"><parent link="link1"/><child link="link2"/><origin xyz="0 0 0.3"/><axis xyz="0 1 0"/><limit lower="-2" upper="2" effort="1" velocity="1"/></joint>
    <link name="link3"><collision><origin xyz="0.175 0 0"/><geometry><box size="0.35 0.05 0.05"/></geometry></collision></link>
    <joint name="J3" type="revolute"><parent link="link2"/><child link="link3"/><origin xyz="0.5 0 0"/><axis xyz="0 1 0"/><limit lower="-2.5" upper="2.5" effort="1" velocity="1"/></joint>
    <link name="tool"><collision><origin xyz="0.03 0 0"/><geometry><box size="0.06 0.12 0.03"/></geometry></collision></link>
    <joint name="fixed_tool" type="fixed"><parent link="link3"/><child link="tool"/><origin xyz="0.35 0 0"/></joint>
  </robot>)";
  auto urdf=urdf::parseURDF(urdf_text);auto srdf=std::make_shared<srdf::Model>();
  require(bool(urdf),"ARM_URDF");
  require(srdf->initString(*urdf,R"(<robot name="arm"><group name="manipulator"><joint name="J1"/><joint name="J2"/><joint name="J3"/></group>
    <disable_collisions link1="base" link2="link1" reason="Adjacent"/><disable_collisions link1="link1" link2="link2" reason="Adjacent"/>
    <disable_collisions link1="link2" link2="link3" reason="Adjacent"/><disable_collisions link1="link3" link2="tool" reason="Adjacent"/></robot>)"),"ARM_SRDF");
  auto model=std::make_shared<moveit::core::RobotModel>(urdf,srdf);
  auto result=std::make_shared<planning_scene::PlanningScene>(model);
  result->getCurrentStateNonConst().setToDefaultValues();result->getCurrentStateNonConst().update();
  if(std::isnan(post_gap)) {
    result->processCollisionObjectMsg(obstacle("wall",.95,0,.3,.05,.8,.6));
    result->processCollisionObjectMsg(obstacle("pillar",.35,.45,.2,.08,.08,.5));
    result->processCollisionObjectMsg(obstacle("table",.5,-.3,.05,.4,.3,.1));
  } else {
    // A post under link2 at J1=0: vertical gap post_gap at its closest (link2 bottom at z=0.27).
    result->processCollisionObjectMsg(obstacle("post",.3,0,.27-post_gap-.1,.04,.04,.2));
  }
  return result;
}
std::vector<std::vector<double>> sampledStates(const std::vector<std::vector<double>>& path,double resolution) {
  // The same edge sampling as Clearance::checkPath.
  std::vector<std::vector<double>> out;
  for(size_t edge=0;edge+1<path.size();++edge) {
    const auto& a=path[edge];const auto& b=path[edge+1];double maximum=0,sum=0;
    for(size_t i=0;i<a.size();++i){maximum=std::max(maximum,std::abs(b[i]-a[i]));sum+=std::abs(b[i]-a[i]);}
    const size_t samples=2*std::max({size_t(1),size_t(std::ceil(maximum/resolution)),size_t(std::ceil(4.*sum/.0025))});
    for(size_t k=0;k<=samples;++k){auto q=a;for(size_t i=0;i<q.size();++i)q[i]+=double(k)/samples*(b[i]-a[i]);out.push_back(q);}
  }
  return out;
}
struct Walk {bool valid=true;long first=-1;std::string a,b;double distance=0;size_t checked=0;};
Walk walkPath(const planning_scene::PlanningScenePtr& s,m710::ClearanceWorkspace& w,const std::vector<std::vector<double>>& samples,bool adaptive) {
  Walk out;std::vector<m710::ClearanceWorkspace::Constraint> cs;std::vector<double> anchor;bool anchored=false;
  w.recording(adaptive);auto state=s->getCurrentState();
  for(size_t k=0;k<samples.size();++k) {
    state.setJointGroupPositions("manipulator",samples[k]);
    if(adaptive && anchored && state.satisfiesBounds() && m710::ClearanceWorkspace::provenClear(cs,samples[k],anchor,1e-6))continue;
    state.update();++out.checked;m710::ClearanceWorkspace::Failure f;
    if(!state.satisfiesBounds() || !w.check(state,f,"output")) {out.valid=false;out.first=long(k);out.a=f.a;out.b=f.b;out.distance=f.distance;break;}
    if(adaptive) {double lower;w.adaptiveConstraints(cs,lower);anchor=samples[k];anchored=true;}
  }
  w.recording(false);return out;
}
J adaptiveCases() {
  J out;out["mode"]="adaptive";out["scope"]="synthetic implementation checks only";
  auto required=[](const std::string&,collision_detection::BodyType,const std::string&,collision_detection::BodyType){return .005;};
  auto fclOf=[](const planning_scene::PlanningScenePtr& s) {
    const auto* f=dynamic_cast<const collision_detection::CollisionEnvFCL*>(s->getCollisionEnvUnpadded().get());require(f,"FCL_ENV");return f;};
  std::mt19937 rng(6518);std::uniform_real_distribution<double> unit(0.,1.);
  // 1. Lever bound: every box corner of every moving body moves no more than
  //    the bound, between two arbitrary configurations (prismatic + revolute
  //    chain, and a 3-revolute arm).
  {
    size_t pairs=0;double worst=0.;
    for(int scene_kind=0;scene_kind<2;++scene_kind) {
      auto s=scene_kind?armScene():twoJointScene();
      m710::ClearanceWorkspace w(*fclOf(s),s,required,true,0.,{},true,false,"manipulator");
      require(w.adaptive() && w.movingBodies()>0,"ADAPTIVE_PREPARED");
      const auto* group=s->getRobotModel()->getJointModelGroup("manipulator");
      auto state=s->getCurrentState();
      auto random_q=[&]() {std::vector<double> q;for(const auto* j:group->getActiveJointModels()) {
        const auto& b=j->getVariableBounds()[0];q.push_back(b.min_position_+unit(rng)*(b.max_position_-b.min_position_));}return q;};
      auto corners=[&](std::vector<std::vector<Eigen::Vector3d>>& out_corners) {
        out_corners.assign(w.movingBodies(),{});
        for(size_t i=0;i<w.movingBodies();++i) {
          const auto& o=w.object(i);const auto* box=dynamic_cast<const fcl::Boxd*>(o.collisionGeometry().get());
          require(box,"ADAPTIVE_TEST_EXPECTS_BOXES");
          for(int x:{-1,1})for(int y:{-1,1})for(int z:{-1,1})
            out_corners[i].push_back(o.getTransform()*Eigen::Vector3d(x*box->side[0]/2,y*box->side[1]/2,z*box->side[2]/2));
        }
      };
      for(int k=0;k<1000;++k) {
        auto q0=random_q(),q1=random_q();
        if(k%2) for(size_t v=0;v<q1.size();++v) q1[v]=q0[v]+(q1[v]-q0[v])*.02; // also small steps
        m710::ClearanceWorkspace::Failure f;std::vector<std::vector<Eigen::Vector3d>> c0,c1;
        state.setJointGroupPositions("manipulator",q0);state.update();w.check(state,f,"output");corners(c0);
        state.setJointGroupPositions("manipulator",q1);state.update();w.check(state,f,"output");corners(c1);
        std::vector<double> dq(q0.size());for(size_t v=0;v<dq.size();++v)dq[v]=q1[v]-q0[v];
        for(size_t i=0;i<c0.size();++i) {
          double moved=0.;for(size_t m=0;m<c0[i].size();++m)moved=std::max(moved,(c1[i][m]-c0[i][m]).norm());
          const double bound=w.displacementBound(i,dq);
          require(moved<=bound+1e-12,"ADAPTIVE_LEVER_BOUND_VIOLATED");
          if(bound>1e-9)worst=std::max(worst,moved/bound);++pairs;
        }
      }
    }
    out["lever_bound"]={{"body_displacements_checked",pairs},{"largest_displacement_to_bound_ratio",worst},{"status","PASS"}};
  }
  // 2. Proof soundness: a sample proven clear from an anchor passes the exact check.
  // 3. Paths: ADAPTIVE and SAMPLED take the same decision at the same first failing sample.
  {
    size_t proven=0,proof_checks=0,paths=0,valid_paths=0,invalid_paths=0,sampled=0,adaptive_checked=0;
    for(int two_level=0;two_level<2;++two_level) {
      auto s=armScene();
      m710::ClearanceWorkspace w(*fclOf(s),s,required,true,0.,{},true,bool(two_level),"manipulator"),
                               ref(*fclOf(s),s,required,true,0.,{},true,false);
      auto state=s->getCurrentState();
      for(int k=0;k<1500;++k) {
        std::vector<double> q0={-1.2+2.4*unit(rng),-1.2+1.6*unit(rng),-1.8+2.6*unit(rng)};
        state.setJointGroupPositions("manipulator",q0);state.update();m710::ClearanceWorkspace::Failure f;
        w.recording(true);const bool ok=w.check(state,f,"output");
        if(!ok){w.recording(false);continue;}
        std::vector<m710::ClearanceWorkspace::Constraint> cs;double lower;w.adaptiveConstraints(cs,lower);w.recording(false);
        for(int m=0;m<6;++m) {
          const double scale=std::pow(10.,-3.+2.5*unit(rng));auto q1=q0;
          for(auto& x:q1)x+=scale*(2.*unit(rng)-1.);
          if(!m710::ClearanceWorkspace::provenClear(cs,q1,q0,1e-6))continue;
          ++proven;state.setJointGroupPositions("manipulator",q1);state.update();
          require(ref.check(state,f,"output"),"ADAPTIVE_PROVEN_SAMPLE_FAILS_EXACT_CHECK");
        }
        ++proof_checks;
      }
      for(int k=0;k<300;++k) {
        std::vector<std::vector<double>> path;
        std::vector<double> q={-1.2+2.4*unit(rng),-1.+1.4*unit(rng),-1.5+2.3*unit(rng)};path.push_back(q);
        for(int e=0;e<3;++e){for(auto& x:q)x+=.6*(2.*unit(rng)-1.);path.push_back(q);}
        const auto samples=sampledStates(path,.02);
        const auto a=walkPath(s,ref,samples,false),b=walkPath(s,w,samples,true);
        require(a.valid==b.valid && a.first==b.first && a.a==b.a && a.b==b.b && a.distance==b.distance,"ADAPTIVE_PATH_DECISION_DIFFERS");
        require(b.checked<=a.checked,"ADAPTIVE_CHECKS_MORE");
        ++paths;(a.valid?valid_paths:invalid_paths)++;sampled+=a.checked;adaptive_checked+=b.checked;
      }
    }
    require(proven>200 && valid_paths>20 && invalid_paths>20,"ADAPTIVE_CASES_MUST_COVER_BOTH_OUTCOMES");
    require(adaptive_checked<sampled,"ADAPTIVE_NEVER_SKIPPED");
    out["proof_soundness"]={{"anchors",proof_checks},{"proven_samples_rechecked_exactly",proven},{"status","PASS"}};
    out["path_equivalence"]={{"paths",paths},{"valid",valid_paths},{"invalid",invalid_paths},
      {"sampled_checks",sampled},{"adaptive_checks",adaptive_checked},{"status","PASS"}};
  }
  // 4. A grazing (gap below the 5 mm limit) and a penetrating post: both
  //    methods reject at the same sample; a clear post passes with fewer checks.
  {
    J cases=J::array();
    for(double gap:{.008,.003,.0005,-.001}) {
      auto s=armScene(gap);
      m710::ClearanceWorkspace w(*fclOf(s),s,required,true,0.,{},true,false,"manipulator"),ref(*fclOf(s),s,required,true,0.,{},true,false);
      const auto samples=sampledStates({{-.6,0.,0.},{.6,0.,0.}},.02);
      const auto a=walkPath(s,ref,samples,false),b=walkPath(s,w,samples,true);
      require(a.valid==b.valid && a.first==b.first && a.a==b.a && a.b==b.b && a.distance==b.distance,"ADAPTIVE_GRAZING_DECISION_DIFFERS");
      require(a.valid==(gap>.005),gap>.005?"CLEAR_POST_MUST_PASS":"GRAZING_OR_PENETRATING_POST_MUST_REJECT");
      cases.push_back({{"gap_m",gap},{"valid",a.valid},{"first_failing_sample",a.first},{"sampled_checks",a.checked},{"adaptive_checks",b.checked},
        {"pair",{a.a,a.b}},{"distance_m",a.valid?J(nullptr):J(a.distance)}});
    }
    out["grazing_and_penetration"]={{"cases",cases},{"status","PASS"}};
  }
  // Scope: ADAPTIVE refused outside the physical single-carton schema and for unknown values.
  {
    auto s=scene(.02);bool refused=false;
    try {m710::Clearance c(s,policy(),"optimized",nullptr,J{{"sent_path_audit","ADAPTIVE"},{"attachment",nullptr}});}
    catch(const std::exception& e) {refused=std::string(e.what())=="SENT_PATH_AUDIT_SCOPE_INVALID";}
    require(refused,"ADAPTIVE_SCOPE_NOT_ENFORCED");refused=false;
    try {m710::Clearance c(s,policy(),"optimized",nullptr,J{{"sent_path_audit","FAST"},{"attachment",nullptr}});}
    catch(const std::exception& e) {refused=std::string(e.what())=="SENT_PATH_AUDIT_INVALID";}
    require(refused,"UNKNOWN_AUDIT_NOT_REFUSED");
    out["scope"]="PASS";
  }
  out["status"]="PASS";return out;
}

int main(int argc,char** argv) {
  const std::string mode=argc>1?argv[1]:"optimized";rclcpp::init(argc,argv);
  if(mode=="two_level") {auto result=twoLevelCases();std::cout<<result.dump(2)<<std::endl;rclcpp::shutdown();return 0;}
  if(mode=="adaptive") {auto result=adaptiveCases();std::cout<<result.dump(2)<<std::endl;rclcpp::shutdown();return 0;}
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
