// Actual FCL reproduction and a single eligible pair: AABB cannot certify it.
#define main original_clearance_main
#include "clearance_test.cpp"
#undef main
planning_scene::PlanningScenePtr narrowScene(double gap=.04,bool movable=false) {
  std::string xml=R"(<robot name="narrow"><link name="world"/><link name="flange"/>
    <joint name="J1" type="revolute"><parent link="world"/><child link="flange"/><axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="1" velocity="1"/></joint>
    <link name="tool"><collision><geometry><box size="1 .01 .01"/></geometry></collision></link>)";
  xml+=movable?R"(<joint name="mount" type="prismatic"><parent link="flange"/><child link="tool"/><origin rpy="0 0 .7853981633974483"/><axis xyz="0 1 0"/><limit lower="-.2" upper=".2" effort="1" velocity="1"/></joint></robot>)":
    R"(<joint name="mount" type="fixed"><parent link="flange"/><child link="tool"/><origin rpy="0 0 .7853981633974483"/></joint></robot>)";
  auto u=urdf::parseURDF(xml);auto srdf=std::make_shared<srdf::Model>();
  require(bool(u)&&srdf->initString(*u,R"(<robot name="narrow"><group name="manipulator"><joint name="J1"/></group></robot>)"),"MODEL");
  auto s=std::make_shared<planning_scene::PlanningScene>(std::make_shared<moveit::core::RobotModel>(u,srdf));
  s->getCurrentStateNonConst().setToDefaultValues();s->getCurrentStateNonConst().update();
  moveit_msgs::msg::AttachedCollisionObject a;a.link_name="flange";a.object=box("payload",0);a.object.header.frame_id="flange";
  a.object.primitives[0].dimensions={1,.01,.01};
  auto& p=a.object.primitive_poses[0];const double angle=std::acos(-1.)/4;
  p.position.x=-std::sin(angle)*(.01+gap);p.position.y=std::cos(angle)*(.01+gap);
  p.orientation.z=std::sin(angle/2);p.orientation.w=std::cos(angle/2);
  require(s->processAttachedCollisionObjectMsg(a),"ATTACH");return s;
}
J phase(const ProfileComparedClearance& c) {return c.workspaceEvidence().at("phases").at("endpoint");}
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  auto shape=std::make_shared<fcl::Boxd>(1,.01,.01);
  fcl::Transform3d ta=fcl::Transform3d::Identity(),tb=ta;
  ta.linear()=Eigen::AngleAxisd(std::acos(-1.)/4,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  tb=ta;tb.translation()=ta.linear()*Eigen::Vector3d(0,.05,0);
  fcl::CollisionObjectd a(shape,ta),b(shape,tb);a.computeAABB();b.computeAABB();
  fcl::DistanceResultd limited;limited.min_distance=.006;
  const double returned=fcl::distance(&a,&b,fcl::DistanceRequestd(false),limited);
  fcl::DistanceResultd full;const double initial=full.min_distance;
  const double actual=fcl::distance(&a,&b,fcl::DistanceRequestd(false),full);
  J repro={{"initial",.006},{"returned",returned},{"final_min_distance",limited.min_distance},
    {"old_qualification",returned>.006&&limited.min_distance>.006},{"aabb_lower",a.getAABB().distance(b.getAABB())},
    {"untruncated_initial",initial},{"untruncated_returned",actual},{"untruncated_final",full.min_distance}};
  std::cout<<J({{"fcl_reproduction",repro}}).dump()<<std::endl;
  require(repro["aabb_lower"].get<double>()<=.006&&actual>.039&&actual<.041,"REAL_FCL_GEOMETRY");
  auto s=narrowScene();auto p=policy();p["payload_id"]="payload";
  ProfileComparedClearance ref(s,p,"optimized",false),opt(s,p,"optimized",true);
  require(opt.workspaceEvidence()["eligible_pairs"]==1,"NOT_SINGLE_PAIR");
  auto q=s->getCurrentState();require(ref.check(q)&&opt.check(q),"FIRST_VERDICT");
  auto first=phase(opt);
  require(first["aabb_skips"]==0&&first["narrow_calls"].get<size_t>()>=1,"FIRST_NOT_NARROW");
  for(double angle:{.2,-.5,1.1}) {q.setVariablePosition("J1",angle);q.update();require(q.satisfiesBounds(),"Q_BOUNDS");require(ref.check(q)&&opt.check(q),"VERDICT_DIFFERENCE");}
  auto last=phase(opt);
  std::cout<<J({{"first",first},{"last",last},{"reference",phase(ref)}}).dump()<<std::endl;
  // This assertion uses the old evidence API, so the same test exposes d33eabd.
  require(last["narrow_calls"]==first["narrow_calls"]&&last["fixed_property_reuses"]==3,"NARROW_QUALIFICATION_NOT_REUSED");
  require(first["fixed_aabb_qualifications"]==0&&first["fixed_narrow_qualifications"]==1&&first["fixed_qualification_queries"]==1,"WRONG_QUALIFICATION_SOURCE");
  require(last["fixed_aabb_reuses"]==0&&last["fixed_narrow_reuses"]==3,"WRONG_REUSE_SOURCE");
  require(first["narrow_calls"]==2&&phase(ref)["narrow_calls"]==4,"ACTUAL_QUERY_COUNT");
  ProfileComparedClearance baseline(s,p,"optimized",true,false);
  for(double angle:{0.,.2,-.5,1.1}) {
    auto bq=s->getCurrentState();bq.setVariablePosition("J1",angle);bq.update();require(baseline.check(bq),"BASELINE_VERDICT");
  }
  require(phase(baseline)["narrow_calls"]==4&&phase(baseline)["fixed_narrow_qualifications"]==0,"BASELINE_NOT_D33_BEHAVIOR");
  require(baseline.evidence()["native_check_reuse"]==true,"BASELINE_DISABLED_PREVIOUS_REUSE");
  size_t boundaries=0;
  for(double gap:{.004,.005-2e-9,.005-.5e-9,.005,.005+.5e-9,.006-2e-9,.006,.006+2e-9}) {
    auto bs=narrowScene(gap);ProfileComparedClearance br(bs,p,"optimized",false),bo(bs,p,"optimized",true);
    auto bq=bs->getCurrentState();
    for(double angle:{0.,.2,-.5}) {
      bq.setVariablePosition("J1",angle);bq.update();
      bool x=br.check(bq),y=bo.check(bq);++boundaries;
      require(x==y&&x==(gap+1e-9>=.005),"NARROW_BOUNDARY_VERDICT");
      if(!x)require(br.last_failure==bo.last_failure,"NARROW_BOUNDARY_WITNESS");
    }
    auto counts=phase(bo);
    require(counts["fixed_qualification_queries"].get<size_t>()<=1,"REPEATED_QUALIFICATION");
    if(gap<.006)require(counts["fixed_narrow_reuses"]==0,"BOUNDARY_CERTIFIED");
    if(gap>.006)require(counts["fixed_narrow_qualifications"]==1&&counts["fixed_narrow_reuses"]==2,"ABOVE_GUARD_NOT_REUSED");
  }
  for(const std::string name:{"wall","neighbor"}) {
    auto ws=narrowScene();auto obstacle=box(name,0);
    Eigen::Vector3d center=Eigen::AngleAxisd(.8+std::acos(-1.)/4,Eigen::Vector3d::UnitZ())*Eigen::Vector3d(.3,.05,0);
    obstacle.primitive_poses[0].position.x=center.x();obstacle.primitive_poses[0].position.y=center.y();
    ws->processCollisionObjectMsg(obstacle);
    ProfileComparedClearance wr(ws,p,"optimized",false),wo(ws,p,"optimized",true);auto wq=ws->getCurrentState();
    require(wr.check(wq)&&wo.check(wq)&&phase(wo)["fixed_narrow_qualifications"]==1,"WORLD_NARROW_NOT_ESTABLISHED");
    wq.setVariablePosition("J1",.8);wq.update();
    require(!wr.check(wq)&&!wo.check(wq)&&wr.last_failure==wo.last_failure,"NARROW_WORLD_CACHE_LEAK");
    auto pair=wo.last_failure.at("pair").get<std::vector<std::string>>();
    require(std::find(pair.begin(),pair.end(),"payload")!=pair.end()&&std::find(pair.begin(),pair.end(),name)!=pair.end(),"WRONG_WORLD_WITNESS");
    require(phase(wo)["fixed_narrow_reuses"]==1,"WORLD_DID_NOT_EXERCISE_REUSE");
  }
  auto ms=narrowScene(.04,true);ProfileComparedClearance movable(ms,p,"optimized",true);auto mq=ms->getCurrentState();
  require(movable.check(mq)&&movable.check(mq),"MOVABLE_INITIAL");
  require(phase(movable)["fixed_narrow_qualifications"]==0&&phase(movable)["fixed_qualification_queries"]==0,"MOVABLE_CERTIFIED");
  mq.setVariablePosition("mount",.05);mq.update();require(!movable.check(mq),"MOVABLE_COLLISION_MISSED");
  auto detached=s->getCurrentState();detached.clearAttachedBody("payload");detached.update();
  require(!opt.check(detached)&&opt.last_failure["detail"]=="WORKSPACE_ATTACHMENT_CHANGED","STALE_RELEASE");
  auto mutation=s->diff();mutation->decoupleParent();mutation->getCurrentStateNonConst().clearAttachedBody("payload");
  moveit_msgs::msg::AttachedCollisionObject attach;attach.link_name="flange";attach.object=box("payload",.1);attach.object.header.frame_id="flange";
  require(mutation->processAttachedCollisionObjectMsg(attach),"REATTACH");
  require(!opt.check(mutation->getCurrentState())&&opt.last_failure["detail"]=="WORKSPACE_ATTACHMENT_CHANGED","STALE_ATTACHMENT");
  auto other=narrowScene();require(!opt.check(other->getCurrentState())&&opt.last_failure["detail"]=="WORKSPACE_MODEL_CHANGED","STALE_MODEL");
  auto fresh=narrowScene(.004);ProfileComparedClearance new_candidate(fresh,p,"optimized",true);
  require(!new_candidate.check(fresh->getCurrentState()),"NEW_CANDIDATE_REUSED");
  auto released=s->diff();released->decoupleParent();released->getCurrentStateNonConst().clearAttachedBody("payload");
  released->getCurrentStateNonConst().update();ProfileComparedClearance release(released,p,"optimized",true);
  require(release.check(released->getCurrentState())&&phase(release)["fixed_property_reuses"]==0,"NEW_RELEASE_REUSED");
  // The same model/geometry with changed policy starts a new local workspace.
  auto altered=p;altered["tool_links"]=J::array();ProfileComparedClearance no_tool(s,altered,"optimized",true);
  require(no_tool.check(s->getCurrentState())&&no_tool.check(s->getCurrentState()),"POLICY_INITIAL");
  require(phase(no_tool)["fixed_narrow_qualifications"]==0&&phase(no_tool)["fixed_property_reuses"]==0,"STALE_POLICY");
  auto acm_scene=s->diff();acm_scene->decoupleParent();acm_scene->getAllowedCollisionMatrixNonConst().setEntry("tool","payload",true);
  ProfileComparedClearance exempt(acm_scene,p,"optimized",true);require(exempt.check(acm_scene->getCurrentState()),"ACM_INITIAL");
  require(exempt.workspaceEvidence()["eligible_pairs"]==0&&phase(exempt)["fixed_narrow_qualifications"]==0,"STALE_ACM");
  std::cout<<J({{"boundary_comparisons",boundaries},{"status","PASS"},{"scope","actual_FCL_narrow_source"}}).dump()<<std::endl;
  rclcpp::shutdown();
}
