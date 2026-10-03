// Current-scene edge checks and binding tests, using synthetic kinematics.
#include "static_prior.h"
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <iostream>
#include <sstream>
using J=nlohmann::json;
void require(bool ok,const char* what) {if(!ok)throw std::runtime_error(what);}
J pose() {return {{1.,0.,0.,0.},{0.,1.,0.,0.},{0.,0.,1.,0.},{0.,0.,0.,1.}};}
J context() {
  return {{"schema","m710_static_prior_context_v1"},{"identity",{{"robot","synthetic"}}},
    {"flange_from_task_tcp",pose()},{"tool_links",J::array()},{"compliant_tool_links",J::array()},
    {"collision_policy",{{"schema","m710_poc_pair_collision_policy_v4"}}},
    {"static_world",J::array()},{"allowed_pairs",J::array()},
    {"joint_limits_rad",J::array({{-1.,1.},{-1.,1.},{-1.,1.},{-1.,1.},{-1.,1.},{-1.,1.}})},
    {"interpolation",{{"edge_resolution_rad",.01},{"joint_step_l1_rad",.01},{"distance_norm","L1"},
      {"waypoint_semantics","joint_linear_between_nodes"}}},
    {"receiver_reserve_m",0.},{"placement_policy_sha256","synthetic-policy"},
    {"attachment",{{"mode","empty"}}}};
}
J graph() {
  return {{"status","SUCCESS"},{"prior_id","synthetic-two-portal"},{"context",context()},
    {"nodes",J::array({{-.2,0.,0.,0.,0.,0.},{.2,0.,0.,0.,0.,0.}})},
    {"edges",J::array({{0,1}})},{"interpolation_l1_step_rad",.01}};
}
J policy() {
  return {{"schema","m710_native_free_clearance_v1"},{"stage","pregrasp"},
    {"source_policy",{{"schema","m710_poc_pair_collision_policy_v4"},
      {"required_pair_clearance_m",.005},{"self_collision_clearance_m",0.}}},
    {"numerical_gap_tolerance_m",1e-9},{"tool_links",J::array()},{"conveyor_ids",J::array()},
    {"payload_id",""},{"receiver_reserve_m",0.},{"edge_resolution_rad",.01}};
}
planning_scene::PlanningScenePtr scene(bool blocked) {
  std::ostringstream xml;xml<<"<robot name='graph_test'><link name='world'/>";
  for(int i=1;i<=6;++i) {
    xml<<"<link name='L"<<i<<"'>";
    if(i==6)xml<<"<collision><geometry><box size='0.01 0.01 0.01'/></geometry></collision>";
    xml<<"</link><joint name='J"<<i<<"' type='prismatic'><parent link='"<<(i==1?"world":"L"+std::to_string(i-1))<<"'/>";
    xml<<"<child link='L"<<i<<"'/><axis xyz='"<<(i==2?"0 1 0":"1 0 0")<<"'/><limit lower='-1' upper='1' velocity='1' effort='1'/></joint>";
  }
  xml<<"</robot>";auto u=urdf::parseURDF(xml.str());auto srdf=std::make_shared<srdf::Model>();
  require(bool(u),"URDF");require(srdf->initString(*u,R"(<robot name="graph_test"><group name="manipulator"><chain base_link="world" tip_link="L6"/></group></robot>)"),"SRDF");
  auto model=std::make_shared<moveit::core::RobotModel>(u,srdf);auto s=std::make_shared<planning_scene::PlanningScene>(model);
  s->getCurrentStateNonConst().setToDefaultValues();s->getCurrentStateNonConst().setJointGroupPositions("manipulator",std::vector<double>{-.1,0.,0.,0.,0.,0.});
  s->getCurrentStateNonConst().update();
  if(blocked) {
    moveit_msgs::msg::CollisionObject body;body.id="new_obstacle";body.header.frame_id="world";body.operation=body.ADD;
    shape_msgs::msg::SolidPrimitive shape;shape.type=shape.BOX;shape.dimensions={.02,.2,.2};
    geometry_msgs::msg::Pose p;p.orientation.w=1.;body.primitives.push_back(shape);body.primitive_poses.push_back(p);
    require(s->processCollisionObjectMsg(body),"OBSTACLE");
  }
  return s;
}
void checkPortalBuildInputs() {
  auto clear=scene(false);m710::Clearance clearance(clear,policy());
  J first={{"portal_id","left"},{"q",{-.2,0.,0.,0.,0.,0.}},{"pose",pose()}};
  J duplicate=first;duplicate["portal_id"]="left_near_duplicate";duplicate["q"][0]=-.2+5e-10;
  J second=first;second["portal_id"]="right";second["q"][0]=.2;
  J request={{"prior_context",context()},{"clearance_policy",policy()},{"allowed_planning_time_s",5.},
    {"prior_build",{{"node_limit",4},{"attempt_limit",0},{"neighbors",1},
      {"portal_candidates",J::array({first,duplicate,second})}}}};
  const auto built=m710::StaticPriorGraph::build(request,clear,clearance);
  require(built.at("status")=="SUCCESS" && built.at("nodes").size()==2 &&
    built.at("nodes").at(0)==first.at("q") && built.at("nodes").at(1)==second.at("q"),
    "PORTAL_NODES_RETAIN_ORIGINAL_VALUES");
  require(built.at("accepted_portals").size()==3 && built.at("accepted_portals").at(1).at("node_index")==0 &&
    built.at("portal_duplicates")==1,"PORTAL_DEDUPLICATION_EXPLICIT");
  auto reject=[&](const J& bad,const char* label) {
    bool failed=false;try{m710::StaticPriorGraph::build(bad,clear,clearance);}catch(const std::exception&){failed=true;}
    require(failed,label);
  };
  auto bad=request;bad["prior_build"]["node_limit"]=2;reject(bad,"PORTALS_CANNOT_EXCEED_NODE_LIMIT");
  bad=request;bad["prior_build"]["node_limit"]=160;bad["prior_build"]["portal_candidates"]=J::array();
  for(size_t i=0;i<129;++i) {auto portal=first;portal["portal_id"]="p"+std::to_string(i);bad["prior_build"]["portal_candidates"].push_back(portal);}
  reject(bad,"PORTAL_INPUT_CAP_128");
  bad=request;bad["prior_build"]["portal_candidates"][0]["q"]={0.,0.,0.,0.,0.};reject(bad,"PORTAL_DIMENSION_REJECTED");
  bad=request;bad["prior_build"]["portal_candidates"][0]["q"][0]=2.;reject(bad,"PORTAL_LIMITS_REJECTED");
  bad=request;bad["prior_build"]["portal_candidates"][0]["q"][0]=std::numeric_limits<double>::infinity();reject(bad,"PORTAL_NONFINITE_REJECTED");
  auto blocked=scene(true);m710::Clearance obstruction(blocked,policy());
  auto collision=request;auto portal=first;portal["q"]={0.,0.,0.,0.,0.,0.};
  collision["prior_build"]["portal_candidates"]=J::array({portal});
  const auto refused=m710::StaticPriorGraph::build(collision,blocked,obstruction);
  require(refused.at("accepted_portals").empty() && refused.at("nodes").empty(),"PORTAL_FULL_GEOMETRY_CHECKED");
}
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);checkPortalBuildInputs();auto g=graph();m710::StaticPriorGraph::validate(g);
  auto c=context();require(m710::StaticPriorGraph::applicability(c,c).at("valid"),"SAME_CONTEXT");
  auto changed=c;changed["identity"]["robot"]="other";
  require(!m710::StaticPriorGraph::applicability(c,changed).at("valid").get<bool>(),"MODEL_REJECTED");
  changed=c;changed["flange_from_task_tcp"][0][3]=.001;
  require(!m710::StaticPriorGraph::applicability(c,changed).at("valid").get<bool>(),"TCP_REJECTED");
  changed=c;changed["attachment"]={{"mode","loaded"},{"size",{.6,.4,.3}},{"pose",pose()},{"touch_links",J::array()}};
  require(!m710::StaticPriorGraph::applicability(c,changed).at("valid").get<bool>(),"EMPTY_NOT_LOADED");
  auto loaded=changed;loaded["attachment"]["pose"][0][3]=.03;
  require(!m710::StaticPriorGraph::applicability(changed,loaded).at("valid").get<bool>(),"ATTACHMENT_OUTSIDE_SCOPE");
  loaded=changed;loaded["attachment"]["size"][0]=.7;
  require(!m710::StaticPriorGraph::applicability(changed,loaded).at("valid").get<bool>(),"PAYLOAD_SIZE_REJECTED");
  bool bad=false;auto invalid=g;invalid["nodes"][0][0]=2.;
  try {m710::StaticPriorGraph::validate(invalid);}catch(const std::exception&){bad=true;}require(bad,"LIMITS_REJECTED");
  J request={{"q_start",{-.1,0.,0.,0.,0.,0.}},{"q_goal",{.1,0.,0.,0.,0.,0.}},
    {"prior_context",c},{"clearance_policy",policy()},{"allowed_planning_time_s",5.}};
  auto clear=scene(false);m710::Clearance clearance(clear,policy());
  auto solved=m710::StaticPriorGraph::query(g,request,clear,clearance,nullptr);
  require(solved.at("status")=="SUCCESS" && solved.at("path").front()==request.at("q_start") &&
    solved.at("path").back()==request.at("q_goal"),"DIRECT_CONNECTOR_ACTUAL_ENDPOINTS");
  require(solved.at("prior_usage").at("prior_edges_reused")==0 && solved.at("prior_usage").at("prior_hit")==false,
    "DIRECT_CONNECTION_NOT_PRIOR_HIT");
  auto blocked=scene(true);m710::Clearance obstruction(blocked,policy());
  auto failed=m710::StaticPriorGraph::query(g,request,blocked,obstruction,nullptr);
  require(failed.at("status")=="PRIOR_NOT_CONNECTED" && failed.at("termination")=="GRAPH_DISCONNECTED" && failed.at("path").empty() &&
    failed.at("prior_usage").at("rejected_edges").get<size_t>()>0,"NEW_INTERIOR_OBSTACLE_NOT_ACCEPTED");
  const auto& failure_usage=failed.at("prior_usage");
  require(failure_usage.at("rejected_by_kind").at("direct_connector")==1 &&
    failure_usage.at("failed_edge_witnesses").size()>0 && failure_usage.at("failed_edge_witnesses").size()<=4 &&
    failure_usage.at("failed_edge_witnesses").front().at("kind")=="direct_connector" &&
    failure_usage.at("failed_edge_witnesses").front().contains("reason") &&
    failure_usage.at("failed_edge_witnesses").front().contains("fraction"),"FAILED_EDGE_WITNESS_IS_ACTUAL_AND_BOUNDED");
  size_t classified=0;for(const auto& value:failure_usage.at("rejected_by_kind"))classified+=value.get<size_t>();
  require(classified==failure_usage.at("rejected_edges").get<size_t>() &&
    failure_usage.at("timed_out_edges")==0,"GEOMETRIC_REJECTION_COUNTS_CONSISTENT");
  auto probe=clear->getCurrentState();size_t expired_checks=0;J expired;
  require(!m710::StaticPriorGraph::edge(probe,request.at("q_start").get<std::vector<double>>(),
    request.at("q_goal").get<std::vector<double>>(),clearance,nullptr,.01,std::chrono::steady_clock::now(),
    0.,expired_checks,nullptr,&expired) && expired.at("reason")=="TIMEOUT" && expired_checks==0 &&
    !expired.contains("pair"),"DEADLINE_WITNESS_IS_NOT_GEOMETRIC_FAILURE");
  auto one_portal=g;one_portal["nodes"]=J::array({{0.,.3,0.,0.,0.,0.}});one_portal["edges"]=J::array();
  auto via_portal=m710::StaticPriorGraph::query(one_portal,request,blocked,obstruction,nullptr);
  require(via_portal.at("status")=="SUCCESS" && via_portal.at("prior_usage").at("prior_nodes_reused")==1 &&
    via_portal.at("prior_usage").at("prior_edges_reused")==0 && via_portal.at("prior_usage").at("prior_hit")==true,
    "SINGLE_REUSED_PORTAL_IS_A_TRUE_PRIOR_HIT_WITHOUT_GRAPH_EDGE");
  auto dense=g;dense["nodes"]=J::array();dense["edges"]=J::array();
  for(size_t side=0;side<2;++side)for(size_t i=0;i<20;++i) {
    const double x=(side==0?-1.:1.)*(.11+.004*i);
    dense["nodes"].push_back({x,0.,0.,0.,0.,0.});
  }
  for(size_t a=0;a<20;++a)for(size_t b=20;b<40;++b)dense["edges"].push_back({a,b});
  auto limited_request=request;limited_request["q_start"]={-.3,0.,0.,0.,0.,0.};limited_request["q_goal"]={.3,0.,0.,0.,0.,0.};
  limited_request["allowed_planning_time_s"]=10.;
  auto limited=m710::StaticPriorGraph::query(dense,limited_request,blocked,obstruction,nullptr);
  require(limited.at("status")=="PRIOR_EDGE_CHECK_LIMIT" && limited.at("termination")=="EDGE_CHECK_LIMIT" &&
    limited.at("path").empty() && limited.at("prior_usage").at("edge_checks")==128,
    "FINITE_EDGE_LIMIT_IS_NOT_GRAPH_DISCONNECTED");
  auto portal_graph=dense;portal_graph["accepted_portals"]=J::array();
  for(size_t i=0;i<40;++i)portal_graph["accepted_portals"].push_back({{"portal_id","p"+std::to_string(i)},{"node_index",i},{"pose",pose()}});
  auto portal_solution=m710::StaticPriorGraph::query(portal_graph,limited_request,clear,clearance,nullptr);
  require(portal_solution.at("status")=="SUCCESS" &&
    portal_solution.at("prior_usage").at("portal_connections_added")==16,
    "EXTRA_PORTAL_CONNECTORS_CAPPED_AT_EIGHT_PER_ENDPOINT");
  request["allowed_planning_time_s"]=0.;
  require(m710::StaticPriorGraph::query(g,request,clear,clearance,nullptr).at("status")=="PRIOR_QUERY_TIMEOUT","SHARED_QUERY_BUDGET");
  std::cout<<"PASS: prior binding, direct-connection source, newly blocked edge and budget; synthetic checks only\n";
  rclcpp::shutdown();
}
