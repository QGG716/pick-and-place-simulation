// JSONL is the sole application IPC. ROS logging goes to stderr, never stdout.
#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/fixed_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_state/conversions.h>
#include <ompl/util/RandomNumbers.h>
#include <nlohmann/json.hpp>
#include <chrono>
#include <iostream>
#include <map>
#include <set>
#include "clearance.h"

using J = nlohmann::json;
namespace mtc = moveit::task_constructor;
using Clock = std::chrono::steady_clock;
double seconds(Clock::time_point t) { return std::chrono::duration<double>(Clock::now()-t).count(); }
Eigen::Isometry3d matrix(const J& j) {
  if (j.size()!=4) throw std::runtime_error("INVALID_TRANSFORM");
  Eigen::Isometry3d t=Eigen::Isometry3d::Identity();
  for(int r=0;r<4;++r) for(int c=0;c<4;++c) t.matrix()(r,c)=j.at(r).at(c).get<double>();
  if(!t.matrix().allFinite() || (t.linear().transpose()*t.linear()-Eigen::Matrix3d::Identity()).norm()>1e-8
     || std::abs(t.linear().determinant()-1)>1e-8) throw std::runtime_error("INVALID_TRANSFORM");
  return t;
}
J serial(const Eigen::Isometry3d& t) {
  J j=J::array(); for(int r=0;r<4;++r) { J row=J::array(); for(int c=0;c<4;++c) row.push_back(t.matrix()(r,c)); j.push_back(row); } return j;
}
geometry_msgs::msg::Pose pose(const J& value) {
  auto t=matrix(value); Eigen::Quaterniond q(t.linear()); geometry_msgs::msg::Pose p;
  p.position.x=t.translation().x(); p.position.y=t.translation().y(); p.position.z=t.translation().z();
  p.orientation.x=q.x();p.orientation.y=q.y();p.orientation.z=q.z();p.orientation.w=q.w(); return p;
}
moveit_msgs::msg::CollisionObject object(const J& b, const std::string& frame="world") {
  moveit_msgs::msg::CollisionObject o; o.header.frame_id=frame; o.id=b.at("id"); o.operation=o.ADD;
  shape_msgs::msg::SolidPrimitive s; s.type=s.BOX; auto dimensions=b.at("size").get<std::vector<double>>(); s.dimensions.assign(dimensions.begin(),dimensions.end());
  if(s.dimensions.size()!=3) throw std::runtime_error("INVALID_BOX");
  for(auto d:s.dimensions) if(!std::isfinite(d)||d<=0) throw std::runtime_error("INVALID_BOX");
  o.primitives.push_back(s); o.primitive_poses.push_back(pose(b.at("pose"))); return o;
}


// This stage imports an already authoritative-checked process path. It does
// not claim native coverage of compression/history constraints and never plans.
class CheckedProcessStage : public mtc::PropagatingForward {
  J spec;
public:
  explicit CheckedProcessStage(J value): mtc::PropagatingForward(value.at("name").get<std::string>()), spec(std::move(value)) {}
  void computeForward(const mtc::InterfaceState& from) override {
    auto next=from.scene()->diff();next->decoupleParent();
    auto state=next->getCurrentState(); std::vector<double> incoming;
    state.copyJointGroupPositions("manipulator",incoming);
    auto first=spec.at("path").at(0).get<std::vector<double>>();
    if(first.size()!=incoming.size()) throw std::runtime_error("STAGE_JOINT_COUNT_MISMATCH");
    for(size_t i=0;i<first.size();++i) if(std::abs(first[i]-incoming[i])>1e-9) throw std::runtime_error("STAGE_DISCONTINUITY");
    auto trajectory=std::make_shared<robot_trajectory::RobotTrajectory>(state.getRobotModel(),"manipulator");
    size_t index=0;
    for(const auto& row:spec.at("path")) {
      auto q=row.get<std::vector<double>>();state.setJointGroupPositions("manipulator",q);state.update();
      if(!state.satisfiesBounds()) throw std::runtime_error("STAGE_BOUNDS");
      trajectory->addSuffixWayPoint(state,index==0?0.:spec.at("durations").at(index-1).get<double>());++index;
    }
    next->setCurrentState(state);
    if(spec.contains("attach")) {
      const auto& b=spec.at("attach");std::string id=b.at("id");
      if(!next->getWorld()->hasObject(id)) throw std::runtime_error("ATTACH_WORLD_OBJECT_MISSING");
      next->getWorldNonConst()->removeObject(id);
      moveit_msgs::msg::AttachedCollisionObject a;a.link_name="flange";a.object=object(b,"flange");
      a.touch_links=b.at("touch_links").get<std::vector<std::string>>();
      if(!next->processAttachedCollisionObjectMsg(a)) throw std::runtime_error("ATTACH_FAILED");
    }
    if(spec.contains("release")) {
      const auto& b=spec.at("release");std::string id=b.at("id");
      if(!next->getCurrentState().hasAttachedBody(id)) throw std::runtime_error("RELEASE_ATTACHED_OBJECT_MISSING");
      moveit_msgs::msg::AttachedCollisionObject a;a.link_name="flange";a.object.id=id;a.object.operation=a.object.REMOVE;
      if(!next->processAttachedCollisionObjectMsg(a) || !next->processCollisionObjectMsg(object(b))) throw std::runtime_error("RELEASE_FAILED");
      if(next->getCurrentState().hasAttachedBody(id) || !next->getWorld()->hasObject(id)) throw std::runtime_error("RELEASE_OBJECT_STATE_INVALID");
    }
    mtc::SubTrajectory solution(trajectory);solution.setComment(spec.at("generator"));
    sendForward(from,mtc::InterfaceState(next),std::move(solution));
  }
};


class CountedPlanner : public mtc::solvers::PipelinePlanner {
  size_t& count;
public:
  CountedPlanner(const planning_pipeline::PlanningPipelinePtr& pipeline, size_t& calls): PipelinePlanner(pipeline), count(calls) {}
  Result plan(const planning_scene::PlanningSceneConstPtr& from, const planning_scene::PlanningSceneConstPtr& to,
    const moveit::core::JointModelGroup* group, double timeout, robot_trajectory::RobotTrajectoryPtr& result,
    const moveit_msgs::msg::Constraints& constraints) override {
    ++count; return PipelinePlanner::plan(from,to,group,timeout,result,constraints);
  }
  Result plan(const planning_scene::PlanningSceneConstPtr& from, const moveit::core::LinkModel& link,
    const Eigen::Isometry3d& offset, const Eigen::Isometry3d& target, const moveit::core::JointModelGroup* group,
    double timeout, robot_trajectory::RobotTrajectoryPtr& result, const moveit_msgs::msg::Constraints& constraints) override {
    ++count; return PipelinePlanner::plan(from,link,offset,target,group,timeout,result,constraints);
  }
};
class Worker {
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<robot_model_loader::RobotModelLoader> loader;
  moveit::core::RobotModelPtr model;
  planning_scene::PlanningScenePtr base;
  std::map<std::string,planning_pipeline::PlanningPipelinePtr> pipelines;
  std::vector<std::string> names;
  J identity, scene_content, bound_policy, bound_tools; std::string scene_key;
  uint32_t seed=0; size_t requests=0; std::map<std::string,size_t> calls;
public:
  J run(const J& req) {
    auto begin=Clock::now(); auto op=req.at("op").get<std::string>();
    if(op=="init") {
      if(model) throw std::runtime_error("ALREADY_INITIALIZED");
      seed=req.at("seed"); ompl::RNG::setSeed(seed);
      std::vector<rclcpp::Parameter> params;
      for(auto it=req.at("parameters").begin(); it!=req.at("parameters").end(); ++it) {
        const auto& v=it.value();
        if(v.is_string()) params.emplace_back(it.key(),v.get<std::string>());
        else if(v.is_boolean()) params.emplace_back(it.key(),v.get<bool>());
        else if(v.is_number_integer()) params.emplace_back(it.key(),v.get<int64_t>());
        else if(v.is_number()) params.emplace_back(it.key(),v.get<double>());
        else if(v.is_array()) params.emplace_back(it.key(),v.get<std::vector<std::string>>());
        else throw std::runtime_error("INVALID_PARAMETER_TYPE");
      }
      node=std::make_shared<rclcpp::Node>("m710_moveit_worker", rclcpp::NodeOptions().parameter_overrides(params).automatically_declare_parameters_from_overrides(true));
      loader=std::make_shared<robot_model_loader::RobotModelLoader>(node,"robot_description");
      model=loader->getModel(); if(!model) throw std::runtime_error("MODEL_UNAVAILABLE");
      const auto* group=model->getJointModelGroup("manipulator");
      if(!group) throw std::runtime_error("GROUP_UNAVAILABLE");
      names=group->getVariableNames(); if(names!=req.at("joint_names").get<std::vector<std::string>>()) throw std::runtime_error("JOINT_ORDER_MISMATCH");
      for(const auto& name:{"pilz_industrial_motion_planner","ompl"}) {
        auto p=std::make_shared<planning_pipeline::PlanningPipeline>(model,node,name,"planning_plugin","request_adapters");
        if(!p->getPlannerManager()) throw std::runtime_error(std::string("PIPELINE_UNAVAILABLE:")+name);
        p->displayComputedMotionPlans(false); p->publishReceivedRequests(false);
        pipelines[name]=p;
      }
      for(auto it=req.at("expected_collision_shapes").begin();it!=req.at("expected_collision_shapes").end();++it) {
        const auto* link=model->getLinkModel(it.key());
        if(!link || link->getShapes().size()!=it.value().get<size_t>()) throw std::runtime_error("COLLISION_GEOMETRY_MISSING:"+it.key());
        for(const auto& shape:link->getShapes()) {
          if(shape->type==shapes::MESH && static_cast<const shapes::Mesh*>(shape.get())->triangle_count==0) throw std::runtime_error("EMPTY_COLLISION_MESH");
        }
      }
      bound_policy=req.at("collision_policy");bound_tools=req.at("tool_links");
      base=std::make_shared<planning_scene::PlanningScene>(model);
      identity=req.at("identity");
      return {{"status","READY"},{"identity",identity},{"joint_names",names},{"cold_start_s",seconds(begin)},
              {"seed",seed},{"versions",{{"moveit",M710_MOVEIT_VERSION},{"mtc",M710_MTC_VERSION},{"pilz",M710_PILZ_VERSION}}},{"pipelines",{"pilz_industrial_motion_planner","ompl"}}};
    }
    if(!model) throw std::runtime_error("NOT_INITIALIZED");
    if(op!="fk" && op!="plan" && op!="compose" && op!="inspect" && op!="validate") throw std::runtime_error("UNKNOWN_OPERATION");
    if(req.at("identity")!=identity) throw std::runtime_error("MODEL_OR_POLICY_MISMATCH");
    auto state=base->getCurrentState();
    auto q=req.at("q_start").get<std::vector<double>>();
    if(q.size()!=names.size()) throw std::runtime_error("INVALID_JOINT_VECTOR");
    for(double x:q) if(!std::isfinite(x)) throw std::runtime_error("INVALID_JOINT_VECTOR");
    state.setJointGroupPositions("manipulator",q); state.setVariableVelocities(std::vector<double>(model->getVariableCount(),0)); state.update();
    if(!state.satisfiesBounds()) throw std::runtime_error("INVALID_START_BOUNDS");
    if(op=="fk") {
      J frames=J::object(); for(auto& l:req.at("links")) frames[l.get<std::string>()]=serial(state.getGlobalLinkTransform(l.get<std::string>()));
      return {{"status","SUCCESS"},{"frames",frames}};
    }
    if(req.value("cancelled",false)) throw std::runtime_error("CANCELLED");
    if(req.at("start_velocity").size()!=names.size()) throw std::runtime_error("INVALID_START_VELOCITY");
    for(double v:req.at("start_velocity")) if(v!=0.0) throw std::runtime_error("UNSUPPORTED_NONZERO_START_VELOCITY");
    if(!req.at("path_constraints").empty()) throw std::runtime_error("UNSUPPORTED_PATH_CONSTRAINTS");
    auto imported=Clock::now();
    std::string key=req.at("scene_fingerprint"); bool changed=key!=scene_key;
    if(!changed && req.at("world")!=scene_content) throw std::runtime_error("SCENE_FINGERPRINT_REUSED_WITH_DIFFERENT_CONTENT");
    if(changed) {
      auto next=std::make_shared<planning_scene::PlanningScene>(model); std::set<std::string> ids;
      for(const auto& b:req.at("world")) {
        if(!ids.insert(b.at("id").get<std::string>()).second) throw std::runtime_error("DUPLICATE_OBJECT");
        if(!next->processCollisionObjectMsg(object(b))) throw std::runtime_error("SCENE_IMPORT_FAILED");
      }
      base=next;scene_key=key;scene_content=req.at("world");
    }
    auto scene=base->diff(); scene->decoupleParent(); scene->setCurrentState(state);
    auto& acm=scene->getAllowedCollisionMatrixNonConst();
    for(const auto& pair:req.at("allowed_pairs")) acm.setEntry(pair.at(0).get<std::string>(),pair.at(1).get<std::string>(),true);
    std::string attached_id;
    if(!req.at("attachment").is_null()) {
      auto b=req.at("attachment"); attached_id=b.at("id");
      // A single object identity moves from world to attached; never duplicates.
      scene->getWorldNonConst()->removeObject(attached_id);
      moveit_msgs::msg::AttachedCollisionObject a; a.link_name="flange"; a.object=object(b,"flange");
      a.touch_links=b.at("touch_links").get<std::vector<std::string>>();
      if(!scene->processAttachedCollisionObjectMsg(a)) throw std::runtime_error("ATTACH_FAILED");
      if(scene->getWorld()->hasObject(attached_id)) throw std::runtime_error("DUPLICATE_ATTACHMENT");
    }
    double import_s=seconds(imported);
    if(op=="inspect") {
      J permissions=J::array();
      for(const auto& pair:req.at("inspect_pairs")) {
        collision_detection::AllowedCollision::Type allowed;
        bool found=acm.getEntry(pair.at(0).get<std::string>(),pair.at(1).get<std::string>(),allowed);
        permissions.push_back(found && allowed==collision_detection::AllowedCollision::ALWAYS);
      }
      std::vector<const moveit::core::AttachedBody*> bodies, base_bodies;scene->getCurrentState().getAttachedBodies(bodies);base->getCurrentState().getAttachedBodies(base_bodies);
      return {{"status","SUCCESS"},{"world_count",scene->getWorld()->size()},{"attached_count",bodies.size()},
        {"target_in_world",!attached_id.empty() && scene->getWorld()->hasObject(attached_id)},
        {"permissions",permissions},{"base_attached_count",base_bodies.size()}};
    }
    std::shared_ptr<m710::Clearance> clearance;
    if(op=="plan" || op=="validate") {
      if(!req.contains("clearance_policy")) throw std::runtime_error("CLEARANCE_POLICY_REQUIRED");
      if(req.at("clearance_policy").at("source_policy")!=bound_policy || req.at("clearance_policy").at("tool_links")!=bound_tools)
        throw std::runtime_error("EXECUTABLE_POLICY_MISMATCH");
      clearance=std::make_shared<m710::Clearance>(scene,req.at("clearance_policy"));
      // Environment + ACM belong to this immutable candidate. The predicate
      // always checks OMPL's supplied state, including its real attached body.
      scene->setStateFeasibilityPredicate([clearance](const moveit::core::RobotState& current,bool verbose){return clearance->check(current,verbose);});
    }
    auto path_check=[&](const J& path) {return clearance->checkPath(scene->getCurrentState(),path,req.at("clearance_policy").at("edge_resolution_rad"));};
    if(op=="validate") {
      collision_detection::CollisionRequest old_request;old_request.contacts=true;old_request.max_contacts=20;
      collision_detection::CollisionResult old_result;scene->checkCollision(old_request,old_result);
      clearance->phase="diagnostic";
      const bool valid=clearance->check(scene->getCurrentState());const J failure=clearance->last_failure;
      J path_failure=nullptr;if(req.contains("probe_path")) {clearance->phase="output";path_failure=path_check(req.at("probe_path"));}
      return {{"status","SUCCESS"},{"legacy_intersection_valid",!old_result.collision},{"native_valid",valid},
        {"failure",failure},{"path_failure",path_failure},{"clearance",clearance->evidence()},
        {"world_count",scene->getWorld()->size()},{"attached_id",attached_id},{"pipeline_calls",calls}};
    }
    if(op=="plan") {
      if(!clearance->check(scene->getCurrentState())) return {{"status","INVALID_START_CLEARANCE"},{"failure",clearance->last_failure},{"clearance",clearance->evidence()},{"pipeline_calls",calls}};
      if(req.contains("q_goal")) {
        auto goal=req.at("q_goal").get<std::vector<double>>();
        if(goal.size()!=names.size()) throw std::runtime_error("INVALID_GOAL");
        for(double x:goal) if(!std::isfinite(x)) throw std::runtime_error("INVALID_GOAL");
        auto end=scene->getCurrentState();end.setJointGroupPositions("manipulator",goal);end.update();
        if(!clearance->check(end)) return {{"status","INVALID_GOAL_CLEARANCE"},{"failure",clearance->last_failure},{"clearance",clearance->evidence()},{"pipeline_calls",calls}};
      }
    }
    collision_detection::CollisionRequest cr; cr.contacts=true; cr.max_contacts=20;
    collision_detection::CollisionResult collision; scene->checkCollision(cr,collision);
    if(collision.collision) {
      J pairs=J::array();for(auto& p:collision.contacts) pairs.push_back({p.first.first,p.first.second});
      return {{"status","INVALID_START_COLLISION"},{"collision_pairs",pairs},{"scene_import_s",import_s}};
    }
    if(op=="compose") {
      mtc::Task task("",false);task.setRobotModel(model);task.setName("checked_complete_cycle");
      auto initial=std::make_unique<mtc::stages::FixedState>("frozen_start");initial->setState(scene);task.add(std::move(initial));
      for(const auto& spec:req.at("stages")) task.add(std::make_unique<CheckedProcessStage>(spec));
      task.plan(1);if(task.solutions().empty()) throw std::runtime_error("MTC_COMPOSITION_FAILED");
      const auto& last=task.solutions().front()->end()->scene();std::vector<std::string> attached;
      std::vector<const moveit::core::AttachedBody*> bodies; last->getCurrentState().getAttachedBodies(bodies); for(const auto* body:bodies) attached.push_back(body->getName());
      return {{"status","SUCCESS"},{"stage_count",req.at("stages").size()}, {"world_count",last->getWorld()->size()},
        {"attached_objects",attached},{"mtc_composition_s",seconds(begin)},{"physical_execution","NOT_RUN"}};
    }
    const auto pipeline=req.at("pipeline_id").get<std::string>(); const auto planner=req.at("planner_id").get<std::string>();
    if(!pipelines.count(pipeline) || !((pipeline=="ompl"&&planner=="RRTConnectkConfigDefault") ||
       (pipeline=="pilz_industrial_motion_planner"&&(planner=="PTP"||planner=="LIN")))) throw std::runtime_error("PLANNER_UNAVAILABLE");
    auto solver=std::make_shared<CountedPlanner>(pipelines.at(pipeline),calls[pipeline+"/"+planner]);
    solver->setProperty("goal_joint_tolerance",1e-12);
    solver->setProperty("goal_position_tolerance",1e-6);
    solver->setProperty("goal_orientation_tolerance",1e-6);
    solver->setPlannerId(planner);solver->setProperty("max_velocity_scaling_factor",req.at("velocity_scale").get<double>());
    solver->setProperty("max_acceleration_scaling_factor",req.at("acceleration_scale").get<double>());
    mtc::Task task("",false);task.setRobotModel(model);task.setName(req.at("stage"));
    auto initial=std::make_unique<mtc::stages::FixedState>("frozen_start"); initial->setState(scene);task.add(std::move(initial));
    auto motion=std::make_unique<mtc::stages::MoveTo>(req.at("stage"),solver);motion->setGroup("manipulator");
    motion->setTimeout(req.at("allowed_planning_time_s").get<double>());
    if(req.contains("goal_pose")) {
      motion->setIKFrame(matrix(req.at("flange_from_task_tcp")),"flange");
      geometry_msgs::msg::PoseStamped target;target.header.frame_id="world";target.pose=pose(req.at("goal_pose"));motion->setGoal(target);
    } else {
      auto goal=req.at("q_goal").get<std::vector<double>>();if(goal.size()!=names.size()) throw std::runtime_error("INVALID_GOAL");
      std::map<std::string,double> joints;for(size_t i=0;i<names.size();++i) joints[names[i]]=goal[i];motion->setGoal(joints);
    }
    auto* motion_stage=motion.get();
    task.add(std::move(motion)); auto planning=Clock::now(); ++requests; clearance->phase="search"; task.plan(1);double plan_s=seconds(planning);
    J out={{"status","SEARCH_EXHAUSTED"},{"pipeline_id",pipeline},{"planner_id",planner},{"mtc_attempt_index",requests},{"pipeline_calls",calls},
      {"scene_import_s",import_s},{"scene_updated",changed},{"mtc_plan_s",plan_s},{"world_count",scene->getWorld()->size()},
      {"attached_id",attached_id},{"resident_seed",seed},{"request_seed",req.at("seed")},{"authoritative_status","NOT_RUN"},{"clearance",clearance->evidence()}};
    if(task.solutions().empty()) {std::ostringstream why; task.explainFailure(why);out["detail"]=why.str(); J failures=J::array();for(const auto& failure:motion_stage->failures()) failures.push_back(failure->comment());out["failure_comments"]=failures;return out;}
    moveit_task_constructor_msgs::msg::Solution msg;task.solutions().front()->toMsg(msg);
    J points=J::array();
    for(auto& sub:msg.sub_trajectory) {
      auto& jt=sub.trajectory.joint_trajectory; if(jt.points.empty()) continue;
      if(jt.joint_names!=names) throw std::runtime_error("OUTPUT_JOINT_ORDER_MISMATCH");
      if(pipeline=="ompl") {
        robot_trajectory::RobotTrajectory rt(model,"manipulator");rt.setRobotTrajectoryMsg(scene->getCurrentState(),sub.trajectory);
        trajectory_processing::IterativeParabolicTimeParameterization iptp;
        if(!iptp.computeTimeStamps(rt,req.at("velocity_scale").get<double>(),req.at("acceleration_scale").get<double>())) throw std::runtime_error("TIME_PARAMETERIZATION_FAILED");
        rt.getRobotTrajectoryMsg(sub.trajectory);
      }
      for(auto& p:sub.trajectory.joint_trajectory.points) points.push_back({{"q",p.positions},{"v",p.velocities},{"a",p.accelerations},
        {"t",p.time_from_start.sec+p.time_from_start.nanosec*1e-9}});
    }
    clearance->phase="output"; J path=J::array();for(const auto& p:points) path.push_back(p.at("q"));
    auto output_check=Clock::now();J native_failure=path_check(path);
    out["native_output_check_s"]=seconds(output_check);out["clearance"]=clearance->evidence();
    if(!native_failure.is_null()) {out["status"]="NATIVE_PATH_REJECTED";out["failure"]=native_failure;out["points"]=points;out["total_s"]=seconds(begin);return out;}
    out["native_output_status"]="PASS";
    out["status"]="SUCCESS";out["points"]=points;out["joint_names"]=names;out["total_s"]=seconds(begin);
    out["time_parameterization"]=pipeline=="ompl"?"IPTP_preserves_waypoints":"Pilz_original";return out;
  }
};
int main(int argc,char**argv) {
  setenv("RCUTILS_LOGGING_USE_STDOUT","0",1);rclcpp::init(argc,argv);Worker worker;std::string line;
  while(std::getline(std::cin,line)) {
    J req,answer;try {req=J::parse(line); auto* protocol=std::cout.rdbuf(std::cerr.rdbuf()); try {answer=worker.run(req);} catch(...) {std::cout.rdbuf(protocol);throw;} std::cout.rdbuf(protocol);} catch(const std::exception& e) {answer={{"status","ERROR"},{"detail",e.what()}};}
    answer["request_id"]=req.value("request_id",std::string());std::cout<<answer.dump()<<std::endl;
  }
  rclcpp::shutdown();return 0;
}
