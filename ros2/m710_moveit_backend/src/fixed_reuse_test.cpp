// Focused candidate-local reuse differential tests; no physical qualification.
#define main original_clearance_main
#include "clearance_test.cpp"
#undef main
planning_scene::PlanningScenePtr fixedScene(double gap=.02,bool movable=false) {
  std::string xml=R"(<robot name="fixed"><link name="world"/><link name="robot"><collision><geometry><box size=".01 .01 .01"/></geometry></collision></link>
  <joint name="J1" type="revolute"><parent link="world"/><child link="robot"/><axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="1" velocity="1"/></joint>
  <link name="flange"/><joint name="J2" type="prismatic"><parent link="robot"/><child link="flange"/><origin xyz=".03 0 0"/><axis xyz="1 0 0"/><limit lower="-.5" upper=".5" effort="1" velocity="1"/></joint>
  <link name="tool"><collision><geometry><box size=".01 .01 .01"/></geometry></collision></link>)";
  xml+=movable?R"(<joint name="mount" type="prismatic"><parent link="flange"/><child link="tool"/><origin xyz=".02 0 0"/><axis xyz="1 0 0"/><limit lower="-.1" upper=".1" effort="1" velocity="1"/></joint></robot>)":
    R"(<joint name="mount" type="fixed"><parent link="flange"/><child link="tool"/><origin xyz=".02 0 0"/></joint></robot>)";
  auto u=urdf::parseURDF(xml);auto srdf=std::make_shared<srdf::Model>();
  require(bool(u)&&srdf->initString(*u,R"(<robot name="fixed"><group name="manipulator"><joint name="J1"/><joint name="J2"/></group></robot>)"),"MODEL");
  auto model=std::make_shared<moveit::core::RobotModel>(u,srdf);auto s=std::make_shared<planning_scene::PlanningScene>(model);
  s->getCurrentStateNonConst().setToDefaultValues();s->getCurrentStateNonConst().update();
  moveit_msgs::msg::AttachedCollisionObject a;a.link_name="flange";a.object=box("payload",.03+gap);a.object.header.frame_id="flange";
  require(s->processAttachedCollisionObjectMsg(a),"ATTACH");return s;
}
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);size_t compared=0;
  auto p=policy();p["payload_id"]="payload";
  for(double gap:{.004,.005-2e-9,.005-.5e-9,.005,.005+.5e-9,.005999999,.006000001,.02}) {
    auto s=fixedScene(gap);m710::Clearance ref(s,p,"optimized",false),opt(s,p,"optimized",true);
    auto q=s->getCurrentState();
    for(double angle:{0.,.3,-1.2,2.4}) {
      q.setJointGroupPositions("manipulator",std::vector<double>{angle,.1});q.update();
      const bool a=ref.check(q),b=opt.check(q);++compared;require(a==b,"BOUNDARY_DIFFERENTIAL");
      require(a==(gap+1e-9>=.005),"BOUNDARY_VERDICT");
      if(!a)require(ref.last_failure.at("pair")==opt.last_failure.at("pair"),"FIRST_PAIR_CHANGED");
    }
    if(gap<.006)require(opt.workspaceEvidence()["phases"]["endpoint"]["fixed_property_reuses"]==0,"BOUNDARY_REUSED");
  }
  auto s=fixedScene();m710::Clearance ref(s,p,"optimized",false),opt(s,p,"optimized",true);auto state=s->getCurrentState();
  for(double angle:{0.,.5,1.5}) {state.setJointGroupPositions("manipulator",std::vector<double>{angle,0.});state.update();require(ref.check(state)&&opt.check(state),"RIGID_PASS");}
  require(opt.workspaceEvidence()["phases"]["endpoint"]["fixed_property_reuses"].get<int>()>0,"NO_REUSE");
  state.setJointGroupPositions("manipulator",std::vector<double>{0.,-.05});state.update();
  require(!ref.check(state)&&!opt.check(state),"ACTIVE_LINK_NOT_CHECKED");require(ref.last_failure.at("pair")==opt.last_failure.at("pair"),"ACTIVE_PAIR");
  for(const std::string name:{"wall","neighbor"}) {
    auto world=fixedScene();world->processCollisionObjectMsg(box(name,.13));
    m710::Clearance a(world,p,"optimized",false),b(world,p,"optimized",true);auto q=world->getCurrentState();
    require(a.check(q)&&b.check(q),"WORLD_INITIAL");
    q.setJointGroupPositions("manipulator",std::vector<double>{0.,.05});q.update();
    require(!a.check(q)&&!b.check(q),"PAYLOAD_WORLD_CACHE_LEAK");require(a.last_failure.at("pair")==b.last_failure.at("pair"),"WORLD_FIRST_PAIR");
  }
  auto moving=fixedScene(.02,true);m710::Clearance mc(moving,p,"optimized",true);
  require(mc.check(moving->getCurrentState())&&mc.check(moving->getCurrentState()),"MOVABLE_INITIAL");
  require(mc.workspaceEvidence()["phases"]["endpoint"]["fixed_property_reuses"]==0,"MOVABLE_CACHED");
  auto mq=moving->getCurrentState();mq.setVariablePosition("mount",.03);mq.update();require(!mc.check(mq),"MOVABLE_COLLISION");
  // A new candidate always starts cold, including ownership, geometry and ACM changes.
  auto change=fixedScene(.004);m710::Clearance changed(change,p,"optimized",true);require(!changed.check(change->getCurrentState()),"NEW_GEOMETRY_REUSED");
  auto released=fixedScene();released->getCurrentStateNonConst().clearAttachedBody("payload");released->getCurrentStateNonConst().update();
  m710::Clearance unload(released,p,"optimized",true);require(unload.check(released->getCurrentState()),"RELEASED_INVALID");
  require(unload.workspaceEvidence()["phases"]["endpoint"]["fixed_property_reuses"]==0,"RELEASED_REUSED");
  auto stale=s->getCurrentState();stale.clearAttachedBody("payload");stale.update();require(!opt.check(stale),"STALE_OWNERSHIP_ACCEPTED");
  require(opt.last_failure.at("reason")=="DISTANCE_QUERY_FAILED","CONTEXT_ERROR_DISGUISED");
  auto mismatch=fixedScene();require(!opt.check(mismatch->getCurrentState()),"MODEL_MISMATCH_ACCEPTED");
  // Same model, contradictory attachment pose/touch/shape: the workspace rejects before reuse.
  auto mutation=s->diff();mutation->decoupleParent();mutation->getCurrentStateNonConst().clearAttachedBody("payload");
  moveit_msgs::msg::AttachedCollisionObject a;a.link_name="flange";a.object=box("payload",.09);a.object.header.frame_id="flange";
  require(mutation->processAttachedCollisionObjectMsg(a),"MUTATION_ATTACH");
  require(!opt.check(mutation->getCurrentState()),"ATTACHMENT_POSE_ACCEPTED");
  state=s->getCurrentState();state.setVariablePosition("J1",std::numeric_limits<double>::quiet_NaN());state.update();
  require(!opt.check(state),"NONFINITE_ACCEPTED");
  std::cout<<J({{"status","PASS"},{"boundary_comparisons",compared},{"scope","fixed_pair_reuse_geometry_context_dynamic_counterexamples"}}).dump()<<std::endl;
  rclcpp::shutdown();
}
