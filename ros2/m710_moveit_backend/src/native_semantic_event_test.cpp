// Focused semantic-event and real MTC propagation regressions.
// Synthetic kinematics are implementation tests, never planning-run evidence.
#define M710_NATIVE_WORKER_NO_MAIN
#include "worker.cpp"
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>

void require(bool ok,const char* name) {if(!ok) throw std::runtime_error(name);}
planning_scene::PlanningScenePtr testScene() {
  const auto urdf=urdf::parseURDF(R"(<robot name="semantic_test"><link name="world"/>
    <link name="flange"/><joint name="J1" type="prismatic"><parent link="world"/>
    <child link="flange"/><axis xyz="1 0 0"/><limit lower="-1" upper="1" velocity="1" effort="1"/></joint></robot>)");
  auto srdf=std::make_shared<srdf::Model>();require(bool(urdf),"URDF");
  require(srdf->initString(*urdf,R"(<robot name="semantic_test"><group name="manipulator"><joint name="J1"/></group></robot>)"),"SRDF");
  auto model=std::make_shared<moveit::core::RobotModel>(urdf,srdf);
  auto scene=std::make_shared<planning_scene::PlanningScene>(model);
  scene->getCurrentStateNonConst().setToDefaultValues();scene->getCurrentStateNonConst().update();
  moveit_msgs::msg::AttachedCollisionObject a;a.link_name="flange";
  a.object.id="target";a.object.header.frame_id="flange";a.object.operation=a.object.ADD;
  shape_msgs::msg::SolidPrimitive shape;shape.type=shape.BOX;shape.dimensions={.1,.2,.3};
  geometry_msgs::msg::Pose shape_pose;shape_pose.orientation.w=1.;shape_pose.position.z=.2;
  a.object.primitives.push_back(shape);a.object.primitive_poses.push_back(shape_pose);
  require(scene->processAttachedCollisionObjectMsg(a),"ATTACH");return scene;
}
J eventRequest() {
  return {{"stage","place"},{"stage_id","task:place"},{"task_id","task"},{"parent_stage_id","task:transit"},
    {"q_start",J::array({0.})},{"pipeline_id","pilz_industrial_motion_planner"},{"planner_id","LIN"},
    {"goal_pose",serial(Eigen::Isometry3d::Identity())},{"attachment",{{"id","target"}}},
    {"clearance_policy",{{"target_id","target"}}}};
}
void checkScope() {
  const auto original=eventRequest();
  require(m710::semanticPlaceScope(true,"static_prior_fast",original,true,false),"STATIC_PLACE_ALLOWED");
  require(!m710::semanticPlaceScope(false,"static_prior_fast",original,true,false),"EXECUTION_FORBIDDEN");
  require(!m710::semanticPlaceScope(true,"cold_from_scratch",original,true,false),"COLD_UNCHANGED");
  require(!m710::semanticPlaceScope(true,"static_prior_fast",original,false,false),"ATTACHMENT_REQUIRED");
  require(!m710::semanticPlaceScope(true,"static_prior_fast",original,true,true),"DUPLICATE_WORLD_TARGET");
  auto req=original;req["attachment"]["id"]="other";
  require(!m710::semanticPlaceScope(true,"static_prior_fast",req,true,false),"TARGET_MISMATCH");
  req=original;req["parent_stage_id"]="";
  require(!m710::semanticPlaceScope(true,"static_prior_fast",req,true,false),"PARENT_REQUIRED");
  req=original;req["stage"]="contact";
  require(!m710::semanticPlaceScope(true,"static_prior_fast",req,true,false),"CONTACT_NOT_PLACE");
  req=original;req.erase("goal_pose");
  require(!m710::semanticPlaceScope(true,"static_prior_fast",req,true,false),"CARTESIAN_GOAL_REQUIRED");
}
void checkGoal() {
  auto scene=testScene();geometry_msgs::msg::Pose target;target.orientation.w=1.;
  require(m710::placeGoalSatisfied(scene,"flange",target),"EXACT_GOAL");
  target.position.x=.25e-6;
  require(m710::placeGoalSatisfied(scene,"flange",target),"EXISTING_POSITION_TOLERANCE");
  target.position.x=2e-6;
  require(!m710::placeGoalSatisfied(scene,"flange",target),"POSITION_OUTSIDE_TOLERANCE");
  target.position.x=0.;target.orientation.w=std::cos(1e-6);target.orientation.z=std::sin(1e-6);
  require(!m710::placeGoalSatisfied(scene,"flange",target),"ORIENTATION_OUTSIDE_TOLERANCE");
  target.orientation.w=1.;target.orientation.z=0.;
  require(!m710::placeGoalSatisfied(scene,"missing_tcp",target),"UNKNOWN_TCP");
  scene->getCurrentStateNonConst().setJointGroupPositions("manipulator",std::vector<double>{.1});
  scene->getCurrentStateNonConst().update();
  require(!m710::placeGoalSatisfied(scene,"flange",target),"CURRENT_STATE_MUST_REACH_GOAL");
}
void checkMtcEvent(bool valid_start,bool changed_terminal=false) {
  auto scene=testScene();auto record=std::make_shared<NativeStageRecord>();
  record->request=eventRequest();record->parent="task:transit";record->start=scene;
  if(!valid_start) record->request["q_start"]=J::array({.01});
  size_t generated=0,verified=0;
  auto generate=[&](NativeStageRecord& r) {
    ++generated;
    if(!m710::placeGoalSatisfied(r.start,"flange",pose(r.request.at("goal_pose")))) return;
    r.end=r.start->diff();r.end->decoupleParent();r.end->setCurrentState(r.start->getCurrentState());
    if(changed_terminal) {auto state=r.end->getCurrentState();
      state.setJointGroupPositions("manipulator",std::vector<double>{1e-12});state.update();r.end->setCurrentState(state);}
    r.result={{"generation_source","SEMANTIC_EVENT"}};r.ready=true;
  };
  auto verify=[&](const planning_scene::PlanningSceneConstPtr& before,const planning_scene::PlanningSceneConstPtr& after,const J&) {
    ++verified;require(before->getCurrentState().hasAttachedBody("target") &&
      after->getCurrentState().hasAttachedBody("target"),"EVENT_MUST_KEEP_ATTACHMENT");
  };
  mtc::Task task("",false);task.setRobotModel(scene->getRobotModel());task.setName("semantic_event_regression");
  auto initial=std::make_unique<mtc::stages::FixedState>("actual_parent");initial->setState(scene);task.add(std::move(initial));
  task.add(std::make_unique<NativeProcessStage>(record,generate,verify));
  if(!valid_start || changed_terminal) {
    bool rejected=false;
    try {task.plan(1);} catch(const std::exception& e) {rejected=std::string(e.what()).find(changed_terminal?"NATIVE_SEMANTIC_START_CHANGED":"NATIVE_TASK_UNPLANNED_CONNECTION")!=std::string::npos;}
    require(rejected && !record->ready && generated==(changed_terminal?1u:0u),"MTC_REJECTS_DIFFERENT_ACTUAL_STATE");return;
  }
  task.plan(1);
  require(task.solutions().size()==1 && record->ready && !record->trajectory,"MTC_ACCEPTS_NO_MOTION_STAGE");
  require(generated==1 && verified==1,"MTC_RUNS_GENERATION_AND_TRANSITION");
  std::vector<double> q;task.solutions().front()->end()->scene()->getCurrentState().copyJointGroupPositions("manipulator",q);
  require(q==std::vector<double>{0.} && task.solutions().front()->end()->scene()->getCurrentState().hasAttachedBody("target"),
    "MTC_PRESERVES_ACTUAL_Q_AND_ATTACHMENT");
  task.reset();task.plan(1);
  require(task.solutions().size()==1 && generated==1 && record->cache_hits==1 && verified==2,
    "MTC_REUSES_CHECKED_EVENT_WITH_TRANSITION");
}
void checkReplyCompression() {
  const J original={{"clearance",{{"policy",{{"static_geometry",{1,2,3}}}},
    {"counts",{{"queries",7}}},{"profile",{{"seconds",.1}}},
    {"search_gap_witness",{{"pair",{"link","obstacle"}}}},{"rejected_examples",J::array({{{"reason","COLLISION"}}})}}},
    {"points",J::array()}};
  auto cold=original;compactPlanningOnlyReply(cold,true,"cold_from_scratch");
  require(cold==original,"COLD_REPLY_UNCHANGED");
  auto ordinary=original;compactPlanningOnlyReply(ordinary,false,"static_prior_fast");
  require(ordinary==original,"EXECUTION_REPLY_UNCHANGED");
  auto fast=original;compactPlanningOnlyReply(fast,true,"static_prior_fast");
  auto expected=original;expected["clearance"].erase("policy");
  require(fast==expected,"FAST_COMPRESSION_PRESERVES_CHECKS_PROFILE_AND_WITNESS");
}
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);checkReplyCompression();checkScope();checkGoal();checkMtcEvent(true);checkMtcEvent(false);checkMtcEvent(true,true);
  std::cout<<"PASS: semantic place scope, original MoveIt goal tolerance and actual MTC no-motion propagation; synthetic tests only\n";
  rclcpp::shutdown();
}
