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
#include <moveit/utils/moveit_error_code.h>
#include <ompl/util/RandomNumbers.h>
#include <nlohmann/json.hpp>
#include <chrono>
#include <iostream>
#include <map>
#include <set>
#include "clearance.h"
#include "process_policy.h"
#include "scene_geometry.h"

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

// Cache is native-owned and scoped to this resident task. It can only be
// populated by a solver invocation inside computeForward(), never JSON paths.
struct NativeStageRecord {
  J request,result;
  planning_scene::PlanningScenePtr start,end;
  robot_trajectory::RobotTrajectoryPtr trajectory;
  std::shared_ptr<m710::Clearance> clearance;
  std::shared_ptr<m710::ProcessPolicy> process;
  std::string parent;
  size_t cache_hits=0;
};
class NativeProcessStage : public mtc::PropagatingForward {
  std::shared_ptr<NativeStageRecord> record_;
  std::function<void(NativeStageRecord&)> generate_;
  std::function<void(const planning_scene::PlanningSceneConstPtr&,const planning_scene::PlanningSceneConstPtr&,const J&)> verify_scene_;
public:
  NativeProcessStage(std::shared_ptr<NativeStageRecord> record,std::function<void(NativeStageRecord&)> generate,
    std::function<void(const planning_scene::PlanningSceneConstPtr&,const planning_scene::PlanningSceneConstPtr&,const J&)> verify_scene)
    :mtc::PropagatingForward(record->request.at("stage_id").get<std::string>()),record_(std::move(record)),generate_(std::move(generate)),verify_scene_(std::move(verify_scene)) {}
  void computeForward(const mtc::InterfaceState& from) override {
    auto& r=*record_;std::vector<double> q;
    from.scene()->getCurrentState().copyJointGroupPositions("manipulator",q);
    const auto expected=r.request.at("q_start").get<std::vector<double>>();
    if(q.size()!=expected.size()) throw std::runtime_error("NATIVE_TASK_JOINT_COUNT_MISMATCH");
    for(size_t i=0;i<q.size();++i) if(std::abs(q[i]-expected[i])>1e-9) throw std::runtime_error("NATIVE_TASK_UNPLANNED_CONNECTION");
    verify_scene_(from.scene(),r.start,r.request);
    if(r.trajectory) ++r.cache_hits;else generate_(r);
    if(!r.trajectory) return;
    mtc::SubTrajectory solution(r.trajectory);solution.setComment("native_task_generated:"+r.request.at("stage_id").get<std::string>());
    sendForward(from,mtc::InterfaceState(r.end),std::move(solution));
  }
};
struct NativeTaskSession {
  std::unique_ptr<mtc::Task> task;
  std::map<std::string,std::shared_ptr<NativeStageRecord>> records;
  std::vector<std::string> active;
  J initial_q,initial_world;
  std::string target_id;
  size_t solve_calls=0,backtracks=0;
};
class Worker {
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<robot_model_loader::RobotModelLoader> loader;
  moveit::core::RobotModelPtr model;
  planning_scene::PlanningScenePtr base;
  std::map<std::string,planning_pipeline::PlanningPipelinePtr> pipelines;
  std::vector<std::string> names;
  J identity, scene_content, bound_policy, bound_tools, bound_compliant, bound_process_geometry; std::string scene_key;
  uint32_t seed=0; size_t requests=0; std::map<std::string,size_t> calls;
  std::map<std::string,std::unique_ptr<NativeTaskSession>> tasks;
  size_t ik_calls=0;
  bool planning_only=false;  // Bound at process initialization, never per-stage downgrade.
  static bool samePose(const Eigen::Isometry3d& a,const Eigen::Isometry3d& b) {return (a.matrix()-b.matrix()).cwiseAbs().maxCoeff()<=1e-7;}
  // Between adjacent motions only the selected target may change ownership.
  // Its actual pose must be identical on both sides of attach/release.
  void checkTransition(const planning_scene::PlanningSceneConstPtr& before,const planning_scene::PlanningSceneConstPtr& after,const J& req,
      const std::string& location="mtc_compute") {
    const std::string target=req.at("clearance_policy").at("target_id");
    auto ids=before->getWorld()->getObjectIds(),next=after->getWorld()->getObjectIds();
    std::set<std::string> a(ids.begin(),ids.end()),b(next.begin(),next.end());a.erase(target);b.erase(target);
    if(a!=b) throw std::runtime_error("NATIVE_TASK_WORLD_IDENTITY_CHANGED");
    for(const auto& id:a) {
      const auto x=before->getWorld()->getObject(id),y=after->getWorld()->getObject(id);
      if(x->shapes_.size()!=y->shapes_.size() || x->global_shape_poses_.size()!=x->shapes_.size() ||
         y->global_shape_poses_.size()!=y->shapes_.size()) throw std::runtime_error("NATIVE_TASK_WORLD_GEOMETRY_CHANGED");
      for(size_t i=0;i<x->shapes_.size();++i) {
        if(x->shapes_[i]->type!=shapes::BOX || y->shapes_[i]->type!=shapes::BOX) throw std::runtime_error("NATIVE_TASK_WORLD_GEOMETRY_UNSUPPORTED");
        const auto* sx=static_cast<const shapes::Box*>(x->shapes_[i].get());const auto* sy=static_cast<const shapes::Box*>(y->shapes_[i].get());
        for(int k=0;k<3;++k) if(sx->size[k]!=sy->size[k]) throw std::runtime_error("NATIVE_TASK_WORLD_GEOMETRY_CHANGED");
        if(!samePose(m710::worldShapePose(*x,i),m710::worldShapePose(*y,i))) throw std::runtime_error("NATIVE_TASK_WORLD_POSE_CHANGED");
      }
    }
    auto targetPose=[&](const planning_scene::PlanningSceneConstPtr& s) {
      const auto* attached=s->getCurrentState().getAttachedBody(target);const auto world=s->getWorld()->getObject(target);
      if(bool(attached)==bool(world)) throw std::runtime_error("NATIVE_TASK_TARGET_OWNERSHIP_INVALID");
      return attached?attached->getGlobalCollisionBodyTransforms().at(0):m710::worldShapePose(*world,0);
    };
    const auto before_pose=targetPose(before),after_pose=targetPose(after);
    if(!samePose(before_pose,after_pose)) {
      auto describe=[&](const planning_scene::PlanningSceneConstPtr& scene,const Eigen::Isometry3d& cached) {
        auto fresh=scene->getCurrentState();fresh.update();fresh.updateCollisionBodyTransforms();
        std::vector<double> q;fresh.copyJointGroupPositions("manipulator",q);
        const auto* body=fresh.getAttachedBody(target);
        J detail={{"q",q},{"in_world",scene->getWorld()->hasObject(target)},{"attached",bool(body)},
          {"cached_world_pose",serial(cached)},{"flange_world_pose",serial(fresh.getGlobalLinkTransform("flange"))}};
        if(body) {detail["updated_world_pose"]=serial(body->getGlobalCollisionBodyTransforms().at(0));
          detail["shape_pose_in_link"]=serial(body->getShapePosesInLinkFrame().at(0));}
        return detail;
      };
      std::cerr<<J({{"diagnostic","NATIVE_TASK_ATTACHMENT_TELEPORT"},{"location",location},{"task_id",req.value("task_id",std::string())},
        {"stage_id",req.value("stage_id",std::string())},{"parent_stage_id",req.value("parent_stage_id",std::string())},{"target_id",target},
        {"maximum_matrix_difference",(before_pose.matrix()-after_pose.matrix()).cwiseAbs().maxCoeff()},
        {"before",describe(before,before_pose)},{"after",describe(after,after_pose)}}).dump()<<std::endl;
      throw std::runtime_error("NATIVE_TASK_ATTACHMENT_TELEPORT");
    }
    auto targetSize=[&](const planning_scene::PlanningSceneConstPtr& s) {
      const auto* attached=s->getCurrentState().getAttachedBody(target);const auto world=s->getWorld()->getObject(target);
      const auto& shapes=attached?attached->getShapes():world->shapes_;
      if(shapes.size()!=1 || shapes[0]->type!=shapes::BOX) throw std::runtime_error("NATIVE_TASK_TARGET_SHAPE_INVALID");
      const auto* box=static_cast<const shapes::Box*>(shapes[0].get());return std::vector<double>{box->size[0],box->size[1],box->size[2]};
    };
    if(targetSize(before)!=targetSize(after)) throw std::runtime_error("NATIVE_TASK_TARGET_GEOMETRY_CHANGED");
  }
  J nativeTaskPlan(const J& req,const planning_scene::PlanningScenePtr& scene,
      const std::shared_ptr<m710::Clearance>& clearance,const std::shared_ptr<m710::ProcessPolicy>& process) {
    if(req.contains("path") || req.contains("stages")) throw std::runtime_error("NATIVE_COLD_EXTERNAL_PATH_FORBIDDEN");
    m710::validateNativeMotionScope(req);
    const std::string task_id=req.at("task_id"),stage_id=req.at("stage_id");
    if(!req.contains("parent_stage_id") || !req.at("parent_stage_id").is_string())
      throw std::runtime_error("NATIVE_TASK_EXPLICIT_PARENT_REQUIRED");
    const std::string parent=req.at("parent_stage_id");
    if(task_id.empty() || stage_id.empty()) throw std::runtime_error("NATIVE_TASK_ID_REQUIRED");
    if(parent.empty() && !req.at("attachment").is_null()) throw std::runtime_error("NATIVE_COLD_INITIAL_ATTACHMENT_FORBIDDEN");
    if(!tasks.count(task_id)) {
      if(!parent.empty()) throw std::runtime_error("NATIVE_TASK_PARENT_MISSING");
      if(tasks.size()>=8) throw std::runtime_error("NATIVE_TASK_SESSION_CAPACITY");
      auto session=std::make_unique<NativeTaskSession>();session->task=std::make_unique<mtc::Task>("",false);
      session->task->setRobotModel(model);session->task->setName(task_id);session->initial_q=req.at("q_start");session->initial_world=req.at("world");
      session->target_id=req.at("clearance_policy").at("target_id");
      auto initial=std::make_unique<mtc::stages::FixedState>("frozen_actual_initial_state");initial->setState(scene);session->task->add(std::move(initial));
      tasks[task_id]=std::move(session);
    }
    auto& session=*tasks.at(task_id);
    if(req.at("clearance_policy").at("target_id")!=session.target_id) throw std::runtime_error("NATIVE_TASK_TARGET_ID_CHANGED");
    if(session.records.count(stage_id)) throw std::runtime_error("NATIVE_STAGE_ID_REUSED");
    if(session.records.size()>=4096) throw std::runtime_error("NATIVE_TASK_STAGE_CAPACITY");
    std::vector<std::string> ancestors;std::string cursor=parent;std::set<std::string> seen;
    while(!cursor.empty()) {
      if(!seen.insert(cursor).second || !session.records.count(cursor) || !session.records.at(cursor)->trajectory) throw std::runtime_error("NATIVE_TASK_PARENT_INVALID");
      ancestors.push_back(cursor);cursor=session.records.at(cursor)->parent;
    }
    std::reverse(ancestors.begin(),ancestors.end());
    if(ancestors.empty()) {
      if(req.at("q_start")!=session.initial_q || req.at("world")!=session.initial_world) throw std::runtime_error("NATIVE_TASK_INITIAL_STATE_CHANGED");
    } else {
      const auto& previous=*session.records.at(parent);checkTransition(previous.end,scene,req,"stage_registration");
      if(previous.process && req.contains("process_policy") && !req.at("process_policy").at("initial_proximity").is_null()) {
        const auto terminal=previous.process->terminalProximity();const auto& next=req.at("process_policy").at("initial_proximity");
        if(!terminal.is_null() && terminal.value("fully_released",false) && !next.value("fully_released",false))
          throw std::runtime_error("NATIVE_TASK_PROXIMITY_HISTORY_RESET");
        if(!terminal.is_null() && terminal.contains("pairs") && next.contains("pairs")) {
          std::map<std::string,J> by_name;for(const auto& pair:next.at("pairs")) by_name[pair.at("obstacle").get<std::string>()]=pair;
          for(const auto& pair:terminal.at("pairs")) {
            const std::string name=pair.at("obstacle");
            if(!by_name.count(name) || std::abs(pair.at("last_signed_distance_m").get<double>()-by_name.at(name).at("last_signed_distance_m").get<double>())>1e-7 ||
               (pair.at("released").get<bool>()&&!by_name.at(name).at("released").get<bool>())) throw std::runtime_error("NATIVE_TASK_PROXIMITY_HISTORY_CHANGED");
          }
        }
      }
    }
    size_t common=0;while(common<ancestors.size()&&common<session.active.size()&&ancestors[common]==session.active[common]) ++common;
    session.task->reset();
    if(common<session.active.size()) ++session.backtracks;
    while(session.active.size()>common) {session.task->stages()->remove(-1);session.active.pop_back();}
    auto generate=[this](NativeStageRecord& r) {
      const auto& req=r.request;const std::string pipeline=req.at("pipeline_id"),planner=req.at("planner_id");
      if(!pipelines.count(pipeline)||!((pipeline=="ompl"&&planner=="RRTConnectkConfigDefault") ||
          (pipeline=="pilz_industrial_motion_planner"&&(planner=="PTP"||planner=="LIN")))) throw std::runtime_error("PLANNER_UNAVAILABLE");
      const std::string stage=req.at("stage");
      if(pipeline=="ompl" && (req.contains("goal_pose") || !(stage=="pregrasp" || stage=="transit" || stage=="residence")))
        throw std::runtime_error("CONSTRAINED_PROCESS_OMPL_FORBIDDEN");
      auto solver=std::make_shared<CountedPlanner>(pipelines.at(pipeline),calls[pipeline+"/"+planner]);
      solver->setPlannerId(planner);solver->setProperty("goal_joint_tolerance",1e-12);solver->setProperty("goal_position_tolerance",1e-6);
      solver->setProperty("goal_orientation_tolerance",1e-6);solver->setProperty("max_velocity_scaling_factor",req.at("velocity_scale").get<double>());
      solver->setProperty("max_acceleration_scaling_factor",req.at("acceleration_scale").get<double>());
      solver->setTimeout(req.at("allowed_planning_time_s").get<double>());solver->init(model);
      const auto* group=model->getJointModelGroup("manipulator");robot_trajectory::RobotTrajectoryPtr trajectory;
      r.clearance->phase="search";const auto begin=Clock::now();
      mtc::solvers::PlannerInterface::Result result;
      if(req.contains("goal_pose")) {
        if(req.at("flange_from_task_tcp")!=identity.at("flange_from_task_tcp")) throw std::runtime_error("TASK_TCP_CONTEXT_MISMATCH");
        const auto* tcp=model->getLinkModel(identity.at("task_tcp_link").get<std::string>());
        if(!tcp) throw std::runtime_error("TASK_TCP_UNAVAILABLE");
        result=solver->plan(r.start,*tcp,Eigen::Isometry3d::Identity(),matrix(req.at("goal_pose")),group,req.at("allowed_planning_time_s").get<double>(),trajectory,moveit_msgs::msg::Constraints{});
      } else {
        auto goal=r.start->diff();auto state=goal->getCurrentState();state.setJointGroupPositions(group,req.at("q_goal").get<std::vector<double>>());state.update();goal->setCurrentState(state);
        result=solver->plan(r.start,goal,group,req.at("allowed_planning_time_s").get<double>(),trajectory,moveit_msgs::msg::Constraints{});
      }
      r.result={{"status","NATIVE_PLANNING_FAILED"},{"pipeline_id",pipeline},{"planner_id",planner},{"mtc_plan_s",seconds(begin)},
        {"mtc_generation",true},{"generated_during_task",true},{"task_id",req.at("task_id")},{"stage_id",req.at("stage_id")},
        {"parent_stage_id",r.parent},{"native_solver_calls",{{pipeline=="ompl"?"OMPL":planner,1}}},{"pipeline_calls",calls},{"request_id",req.at("request_id")}};
      r.result["solver_returned_success"]=result.success;
      r.result["solver_message"]=result.message;
      // MTC exposes the original error name on failure, but drops the numeric
      // response and the successful response message. Never invent that code.
      r.result["moveit_error_code"]=nullptr;
      r.result["moveit_error_code_source"]="MTC_result_message_only";
      if(!result.success) for(int code : {-1,-2,-3,-4,-5,-6,-7,-10,-11,-12,-13,-14,-15,-16,-17,-18,-19,-21,-22,-23,-24,-25,-26,-27,-28,-29,-30,-31,99999}) {
        if(result.message==moveit::core::error_code_to_string(code)) {
          r.result["moveit_error_code"]=code;
          r.result["moveit_error_code_source"]="recovered_from_original_MoveIt_error_name";
          break;
        }
      }
      r.result["returned_waypoint_count"]=trajectory?trajectory->getWayPointCount():0;
      r.result["trajectory_present"]=bool(trajectory);
      r.result["processing_branch"]=!result?"SOLVER_RETURNED_FAILURE":!trajectory?
          "NULL_TRAJECTORY":trajectory->getWayPointCount()<2?"INSUFFICIENT_WAYPOINTS":"TRAJECTORY_CONVERSION";
      if(req.contains("goal_pose")) {
        const auto start_pose=r.start->getCurrentState().getGlobalLinkTransform(identity.at("task_tcp_link").get<std::string>());
        const auto goal_pose=matrix(req.at("goal_pose"));
        r.result["goal_translation_delta_m"]=(start_pose.translation()-goal_pose.translation()).norm();
        r.result["goal_rotation_delta_rad"]=Eigen::AngleAxisd(start_pose.linear().transpose()*goal_pose.linear()).angle();
      }
      if(!result || !trajectory || trajectory->getWayPointCount()<2) return;
      try {
      if(pipeline=="ompl") {r.result["processing_branch"]="TIME_PARAMETERIZATION";trajectory_processing::IterativeParabolicTimeParameterization iptp;
        if(!iptp.computeTimeStamps(*trajectory,req.at("velocity_scale").get<double>(),req.at("acceleration_scale").get<double>())) throw std::runtime_error("TIME_PARAMETERIZATION_FAILED");}
      r.result["processing_branch"]="TRAJECTORY_CONVERSION";
      moveit_msgs::msg::RobotTrajectory msg;trajectory->getRobotTrajectoryMsg(msg);
      if(msg.joint_trajectory.joint_names!=names) throw std::runtime_error("OUTPUT_JOINT_ORDER_MISMATCH");
      J points=J::array(),path=J::array();
      for(const auto& p:msg.joint_trajectory.points) {points.push_back({{"q",p.positions},{"v",p.velocities},{"a",p.accelerations},{"t",p.time_from_start.sec+p.time_from_start.nanosec*1e-9}});path.push_back(p.positions);}
      J failure=nullptr;
      if(!planning_only) {
        const auto checking=Clock::now();r.clearance->phase="output";const double resolution=req.at("clearance_policy").at("edge_resolution_rad");
        failure=r.clearance->checkPath(r.start->getCurrentState(),path,resolution);
        if(failure.is_null() && r.process) failure=r.process->checkPath(r.start->getCurrentState(),path,resolution);
        r.result["native_output_check_s"]=seconds(checking);
      } else {
        r.result["native_output_check_s"]=nullptr;
        r.result["planning_only_status"]="PLANNING_ONLY_NOT_EXECUTABLE";
      }
      r.result["clearance"]=r.clearance->evidence();
      if(r.process) r.result["process_policy"]=r.process->evidence();
      r.result["points"]=points;r.result["joint_names"]=names;
      if(!failure.is_null()) {r.result["status"]="NATIVE_PATH_REJECTED";r.result["failure"]=failure;return;}
      r.trajectory=trajectory;r.end=r.start->diff();r.end->decoupleParent();r.end->setCurrentState(trajectory->getLastWayPoint());
      r.result["status"]="SUCCESS";r.result["native_output_status"]=planning_only?"SKIPPED_PLANNING_ONLY":"PASS";
      r.result["process_semantics_checked"]=!planning_only && bool(r.process);
      r.result["planner_internal_process_checks"]=bool(r.process);
      r.result["time_parameterization"]=pipeline=="ompl"?"IPTP_preserves_waypoints":"Pilz_original";
      r.result["task_tcp_identity"]=identity.at("task_tcp_fingerprint");r.result["interpolated_link"]=req.contains("goal_pose")?identity.at("task_tcp_link"):J(nullptr);
      r.result["processing_branch"]="ACCEPTED";
      } catch(const std::exception& e) {
        r.result["status"]="NATIVE_PROCESSING_FAILED";
        r.result["failure"]={{"reason",e.what()},{"branch",r.result.at("processing_branch")}};
        r.trajectory.reset();r.end.reset();
      }
    };
    auto verify_scene=[this](const planning_scene::PlanningSceneConstPtr& before,const planning_scene::PlanningSceneConstPtr& after,const J& request) {
      checkTransition(before,after,request);
    };
    for(size_t i=common;i<ancestors.size();++i) {session.task->add(std::make_unique<NativeProcessStage>(session.records.at(ancestors[i]),generate,verify_scene));session.active.push_back(ancestors[i]);}
    auto record=std::make_shared<NativeStageRecord>();record->request=req;record->start=scene;record->clearance=clearance;record->process=process;record->parent=parent;
    session.records[stage_id]=record;session.task->add(std::make_unique<NativeProcessStage>(record,generate,verify_scene));session.active.push_back(stage_id);
    ++session.solve_calls;++requests;session.task->plan(1);
    if(record->result.is_null()) record->result={{"status","MTC_GENERATION_NOT_REACHED"}};
    record->result["mtc_task_solution_count"]=session.task->solutions().size();
    if(record->trajectory && session.task->solutions().empty()) {
      record->result["status"]="NATIVE_MTC_CONNECTION_FAILED";record->result["failure"]={{"reason","MTC_FULL_PREFIX_HAS_NO_SOLUTION"}};
      record->trajectory.reset();record->end.reset();
    }
    record->result["mtc_attempt_index"]=requests;record->result["task_solve_calls"]=session.solve_calls;record->result["task_backtracks"]=session.backtracks;
    record->result["retained_native_subsolutions"]=session.records.size();record->result["task_stage_ids"]=session.active;
    return record->result;
  }
public:
  J run(const J& req) {
    auto begin=Clock::now(); auto op=req.at("op").get<std::string>();
    if(op=="init") {
      if(model) throw std::runtime_error("ALREADY_INITIALIZED");
      planning_only=req.value("planning_only",false);
      seed=req.at("seed");
      static bool seeded=false;static uint32_t process_seed=0;
      if(!seeded) {ompl::RNG::setSeed(seed);process_seed=seed;seeded=true;}
      else if(seed!=process_seed) throw std::runtime_error("RESIDENT_SEED_MISMATCH");
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
      node=std::make_shared<rclcpp::Node>("m710_moveit_worker_"+req.at("identity").at("model_tool_fingerprint").get<std::string>().substr(0,12), rclcpp::NodeOptions().parameter_overrides(params).automatically_declare_parameters_from_overrides(true));
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
      bound_compliant=req.value("compliant_tool_links",J::array());bound_process_geometry=req.value("process_geometry",J(nullptr));
      base=std::make_shared<planning_scene::PlanningScene>(model);
      identity=req.at("identity");
      return {{"status","READY"},{"identity",identity},{"joint_names",names},{"cold_start_s",seconds(begin)},{"planning_only",planning_only},
              {"seed",seed},{"versions",{{"moveit",M710_MOVEIT_VERSION},{"mtc",M710_MTC_VERSION},{"pilz",M710_PILZ_VERSION}}},{"pipelines",{"pilz_industrial_motion_planner","ompl"}},
              {"capabilities",{{"native_ik",true},{"native_task_session",true},{"process_policy_schema","m710_native_process_v1"},
                {"process_geometry_bound",!bound_process_geometry.is_null()&&!bound_compliant.empty()},
                {"max_task_sessions",8},{"max_task_stages_per_session",4096}}}};
    }
    if(!model) throw std::runtime_error("NOT_INITIALIZED");
    if((op=="plan" || op=="ik") && req.value("planning_only",false)!=planning_only)
      throw std::runtime_error("PLANNING_ONLY_MODE_MISMATCH");
    if(planning_only && (op=="task_audit" || op=="compose" || op=="validate"))
      throw std::runtime_error("PLANNING_ONLY_NOT_EXECUTABLE");
    if(planning_only && op=="plan" && !req.value("require_native_motion",false))
      throw std::runtime_error("PLANNING_ONLY_REQUIRES_NATIVE_TASK");
    if(op!="fk" && op!="ik" && op!="task_audit" && op!="plan" && op!="compose" && op!="inspect" && op!="validate") throw std::runtime_error("UNKNOWN_OPERATION");
    if(req.at("identity")!=identity) throw std::runtime_error("MODEL_OR_POLICY_MISMATCH");
    if(op=="task_audit") {
      const std::string id=req.at("task_id");if(!tasks.count(id)) throw std::runtime_error("NATIVE_TASK_UNKNOWN");
      const auto& session=*tasks.at(id);const auto ids=req.at("stage_ids").get<std::vector<std::string>>();
      if(req.contains("target_id") && req.at("target_id")!=session.target_id) throw std::runtime_error("NATIVE_TASK_TARGET_ID_CHANGED");
      if(ids.empty()) throw std::runtime_error("NATIVE_TASK_EMPTY");
      std::string previous;size_t edges=0,attach=0,release=0;J stages=J::array(),terminal_transition=nullptr;
      bool was_attached=false;std::set<std::string> phase_names;
      for(const auto& stage_id:ids) {
        if(!session.records.count(stage_id)) throw std::runtime_error("NATIVE_TASK_STAGE_UNKNOWN");
        const auto& r=*session.records.at(stage_id);
        if(r.parent!=previous || !r.trajectory || r.result.at("status")!="SUCCESS") throw std::runtime_error("NATIVE_TASK_SOURCE_CHAIN_INVALID");
        const std::string target=r.request.at("clearance_policy").at("target_id");const bool attached=r.start->getCurrentState().hasAttachedBody(target);
        if(attached&&!was_attached) ++attach;if(!attached&&was_attached) ++release;was_attached=attached;
        phase_names.insert(r.request.at("stage").get<std::string>());
        size_t motion_edges=0;for(size_t i=1;i<r.trajectory->getWayPointCount();++i)
          if(r.trajectory->getWayPoint(i-1).distance(r.trajectory->getWayPoint(i))>1e-12) ++motion_edges;
        edges+=motion_edges;stages.push_back({{"stage_id",stage_id},{"parent_stage_id",r.parent},{"request_id",r.request.at("request_id")},
          {"native_solver_calls",r.result.at("native_solver_calls")},{"nonzero_motion_edges",motion_edges},{"native_cache_reuses",r.cache_hits}});previous=stage_id;
      }
      if(req.contains("terminal_state_request")) {
        auto terminal=req.at("terminal_state_request");
        for(const auto* key:{"path","probe_path","probe_states","stages","q_goal","goal_pose","pipeline_id","planner_id"})
          if(terminal.contains(key)) throw std::runtime_error("NATIVE_TASK_TERMINAL_MOTION_INPUT_FORBIDDEN");
        if(terminal.at("identity")!=identity || terminal.at("task_id")!=id || terminal.at("parent_stage_id")!=previous ||
           terminal.at("clearance_policy").at("target_id")!=session.target_id)
          throw std::runtime_error("NATIVE_TASK_TERMINAL_IDENTITY_MISMATCH");
        const std::string terminal_stage=terminal.at("stage");
        if((terminal_stage!="withdrawal" && terminal_stage!="residence") || !terminal.at("attachment").is_null() ||
           !terminal.contains("process_policy") || terminal.at("clearance_policy").at("schema")!="m710_native_process_clearance_v1")
          throw std::runtime_error("NATIVE_TASK_TERMINAL_RELEASE_CONTEXT_REQUIRED");
        const auto& last=*session.records.at(previous);std::vector<double> last_q;
        last.end->getCurrentState().copyJointGroupPositions("manipulator",last_q);
        if(terminal.at("q_start").get<std::vector<double>>()!=last_q) throw std::runtime_error("NATIVE_TASK_TERMINAL_STATE_CHANGED");
        // Use the same bound-model, scene import, clearance and process gates
        // as any native state validation. This operation cannot call a solver.
        terminal["op"]="validate";const auto before_calls=calls;const J checked=run(terminal);
        if(calls!=before_calls) throw std::runtime_error("NATIVE_TASK_TERMINAL_UNEXPECTED_SOLVER_CALL");
        if(checked.at("status")!="SUCCESS" || !checked.at("native_valid").get<bool>() || !checked.at("process_valid").get<bool>())
          return {{"status","NATIVE_TASK_TERMINAL_STATE_REJECTED"},{"task_id",id},{"complete_task",false},{"terminal_validation",checked}};
        // run(validate) left base bound to exactly the validated world. Its
        // current state is immutable import state; copy the native endpoint,
        // remove only this target's attachment, then compare both scenes.
        auto final_scene=base->diff();final_scene->decoupleParent();auto final_state=last.end->getCurrentState();
        final_state.clearAttachedBody(session.target_id);final_state.update();final_scene->setCurrentState(final_state);
        checkTransition(last.end,final_scene,terminal,"terminal_release");
        if(final_scene->getCurrentState().hasAttachedBody(session.target_id) || !final_scene->getWorld()->hasObject(session.target_id))
          throw std::runtime_error("NATIVE_TASK_TERMINAL_TARGET_OWNERSHIP_INVALID");
        const bool released=last.end->getCurrentState().hasAttachedBody(session.target_id);
        if(released) ++release;was_attached=false;
        terminal_transition={{"event",released?"RELEASE":"NONE"},{"zero_motion",true},{"native_solver_calls",0},
          {"parent_stage_id",previous},{"target_id",session.target_id},{"q_rad",last_q},{"native_state_validation",checked}};
      }
      if(attach!=1 || release!=1 || was_attached || !phase_names.count("contact") || !phase_names.count("extraction"))
        throw std::runtime_error("NATIVE_TASK_CYCLE_EVENTS_INCOMPLETE");
      return {{"status","SUCCESS"},{"task_id",id},{"generated_during_task",true},{"stage_ids",ids},{"complete_task",true},
        {"stage_sources",stages},{"nonzero_native_motion_edges",edges},{"attach_transitions",attach},{"release_transitions",release},
        {"terminal_transition",terminal_transition},
        {"task_solve_calls",session.solve_calls},{"task_backtracks",session.backtracks},{"external_path_imports",0}};
    }
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
    if(op=="ik") {
      if(req.contains("history") || req.contains("path") || req.contains("q_goal")) throw std::runtime_error("NATIVE_IK_FORBIDDEN_INPUT");
      const auto* group=model->getJointModelGroup("manipulator");const auto& solver=group->getSolverInstance();
      if(!solver || solver->getJointNames()!=names || solver->getTipFrame()!=identity.at("task_tcp_link").get<std::string>()) throw std::runtime_error("NATIVE_IK_SOLVER_CONTEXT_MISMATCH");
      const auto target=matrix(req.at("goal_pose"));const auto base_frame=solver->getBaseFrame();
      const auto local=state.getGlobalLinkTransform(base_frame).inverse()*target;
      std::vector<double> solution;moveit_msgs::msg::MoveItErrorCodes error;const auto started=Clock::now();++ik_calls;
      // getPositionIK performs one seed-directed solve; do not call the
      // timeout-based random-restart searchPositionIK API in native-cold.
      const bool success=solver->getPositionIK(pose(serial(local)),q,solution,error);
      J result={{"status","NATIVE_IK_FAILED"},{"native_ik_calls",1},{"resident_native_ik_calls",ik_calls},
        {"solver","MoveIt_KDL_getPositionIK"},{"ik_collision_checked",false},{"ik_s",seconds(started)},{"error_code",error.val},
        {"seed",req.at("seed")},{"random_restarts",0}};
      if(!success || solution.size()!=names.size()) return result;
      for(double x:solution) if(!std::isfinite(x)) return result;
      state.setJointGroupPositions(group,solution);state.update();if(!state.satisfiesBounds()) return result;
      const auto actual=state.getGlobalLinkTransform(identity.at("task_tcp_link").get<std::string>());
      const double position=(actual.translation()-target.translation()).norm();
      const double rotation=Eigen::AngleAxisd(actual.linear().transpose()*target.linear()).angle();
      result["position_error_m"]=position;result["orientation_error_rad"]=rotation;
      if(position>req.value("position_tolerance_m",1e-6) || rotation>req.value("orientation_tolerance_rad",1e-6)) return result;
      result["status"]="SUCCESS";result["q"]=solution;return result;
    }
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
      const std::string target=req.contains("clearance_policy") ? req.at("clearance_policy").value("target_id",attached_id) : attached_id;
      const auto world_target=scene->getWorld()->getObject(target);const auto* attached_target=scene->getCurrentState().getAttachedBody(target);
      J target_pose=nullptr;
      if(world_target && !world_target->global_shape_poses_.empty()) target_pose=serial(m710::worldShapePose(*world_target,0));
      if(attached_target && !attached_target->getGlobalCollisionBodyTransforms().empty()) target_pose=serial(attached_target->getGlobalCollisionBodyTransforms().at(0));
      return {{"status","SUCCESS"},{"world_count",scene->getWorld()->size()},{"attached_count",bodies.size()},
        {"target_id",target},{"target_in_world",bool(world_target)},{"target_attached",bool(attached_target)},{"target_world_pose",target_pose},
        {"permissions",permissions},{"base_attached_count",base_bodies.size()}};
    }
    std::shared_ptr<m710::Clearance> clearance;
    std::shared_ptr<m710::ProcessPolicy> process;
    if(op=="plan" || op=="validate") {
      if(!req.contains("clearance_policy")) throw std::runtime_error("CLEARANCE_POLICY_REQUIRED");
      if(req.at("clearance_policy").at("source_policy")!=bound_policy || req.at("clearance_policy").at("tool_links")!=bound_tools)
        throw std::runtime_error("EXECUTABLE_POLICY_MISMATCH");
      if(req.contains("process_policy")) {
        if(bound_compliant.empty() || req.at("clearance_policy").at("compliant_tool_links")!=bound_compliant)
          throw std::runtime_error("PROCESS_COLLIDER_OWNERSHIP_MISMATCH");
        if(bound_process_geometry.is_null()) throw std::runtime_error("PROCESS_CUP_GEOMETRY_BINDING_REQUIRED");
        if(req.at("process_policy").at("cups")!=bound_process_geometry.at("cups") ||
           req.at("process_policy").at("flange_from_physical_contact")!=bound_process_geometry.at("flange_from_physical_contact"))
          throw std::runtime_error("PROCESS_CUP_GEOMETRY_BINDING_MISMATCH");
        process=std::make_shared<m710::ProcessPolicy>(scene,req);
      }
      if(req.at("clearance_policy").at("schema")=="m710_native_process_clearance_v1" && !process)
        throw std::runtime_error("PROCESS_POLICY_REQUIRED");
      if(req.value("require_native_motion",false) && op=="plan" && !process) throw std::runtime_error("NATIVE_COLD_PROCESS_CONTEXT_REQUIRED");
      clearance=std::make_shared<m710::Clearance>(scene,req.at("clearance_policy"),req.value("clearance_mode",std::string("optimized")));
      // Environment + ACM belong to this immutable candidate. The predicate
      // always checks OMPL's supplied state, including its real attached body.
      scene->setStateFeasibilityPredicate([clearance,process](const moveit::core::RobotState& current,bool verbose){return clearance->check(current,verbose) && (!process || process->check(current));});
    }
    auto path_check=[&](const J& path) {return clearance->checkPath(scene->getCurrentState(),path,req.at("clearance_policy").at("edge_resolution_rad"));};
    if(op=="validate") {
      if(req.contains("probe_states")) {
        J results=J::array();auto probe=scene->getCurrentState();clearance->phase="fixed_set";
        const auto t=Clock::now();double legacy_s=0;size_t legacy_queries=0;
        for(const auto& row:req.at("probe_states")) {
          auto values=row.get<std::vector<double>>();
          if(values.size()!=names.size()) throw std::runtime_error("INVALID_PROBE_STATE");
          for(double x:values) if(!std::isfinite(x)) throw std::runtime_error("INVALID_PROBE_STATE");
          probe.setJointGroupPositions("manipulator",values);probe.update();
          auto one=Clock::now();const bool clearance_valid=clearance->check(probe);
          const bool process_valid=!process || process->check(probe,true);const bool valid=clearance_valid&&process_valid;double check_s=seconds(one);
          const J failure=clearance_valid&&process?process->last_failure:clearance->last_failure;
          collision_detection::CollisionRequest cr;collision_detection::CollisionResult collision;
          auto lt=Clock::now();scene->checkCollision(cr,collision,probe);legacy_s+=seconds(lt);++legacy_queries;
          results.push_back({{"valid",valid},{"failure",failure},{"process_valid",process_valid},{"check_s",check_s},{"legacy_intersection_valid",!collision.collision}});
        }
        return {{"status","SUCCESS"},{"states",results},{"clearance",clearance->evidence()},{"process_policy_evidence",process?process->evidence():J(nullptr)},
          {"legacy_intersection_s",legacy_s},{"legacy_intersection_queries",legacy_queries},{"batch_s",seconds(t)}};
      }
      collision_detection::CollisionRequest old_request;old_request.contacts=true;old_request.max_contacts=20;
      collision_detection::CollisionResult old_result;scene->checkCollision(old_request,old_result);
      clearance->phase="diagnostic";
      const bool clearance_valid=clearance->check(scene->getCurrentState());const J failure=clearance->last_failure;
      const bool process_valid=!process || process->check(scene->getCurrentState(),true);
      const J process_failure=process?process->last_failure:J(nullptr);
      J path_failure=nullptr,process_path_failure=nullptr;if(req.contains("probe_path")) {clearance->phase="output";path_failure=path_check(req.at("probe_path"));
        if(process) process_path_failure=process->checkPath(scene->getCurrentState(),req.at("probe_path"),req.at("clearance_policy").at("edge_resolution_rad"));}
      return {{"status","SUCCESS"},{"legacy_intersection_valid",!old_result.collision},{"native_valid",clearance_valid&&process_valid},
        {"failure",failure},{"path_failure",path_failure},{"clearance",clearance->evidence()},
        {"process_valid",process_valid},{"process_failure",process_failure},{"process_path_failure",process_path_failure},{"process_policy_evidence",process?process->evidence():J(nullptr)},
        {"world_count",scene->getWorld()->size()},{"attached_id",attached_id},{"pipeline_calls",calls}};
    }
    if(op=="plan") {
      if(!clearance->check(scene->getCurrentState())) return {{"status","INVALID_START_CLEARANCE"},{"failure",clearance->last_failure},{"clearance",clearance->evidence()},{"pipeline_calls",calls}};
      if(process && !process->check(scene->getCurrentState())) return {{"status","INVALID_START_PROCESS"},{"failure",process->last_failure},{"process_policy",process->evidence()},{"pipeline_calls",calls}};
      if(req.contains("q_goal")) {
        auto goal=req.at("q_goal").get<std::vector<double>>();
        if(goal.size()!=names.size()) throw std::runtime_error("INVALID_GOAL");
        for(double x:goal) if(!std::isfinite(x)) throw std::runtime_error("INVALID_GOAL");
        auto end=scene->getCurrentState();end.setJointGroupPositions("manipulator",goal);end.update();
        if(!clearance->check(end)) return {{"status","INVALID_GOAL_CLEARANCE"},{"failure",clearance->last_failure},{"clearance",clearance->evidence()},{"pipeline_calls",calls}};
        if(process && !process->check(end,true)) return {{"status","INVALID_GOAL_PROCESS"},{"failure",process->last_failure},{"process_policy",process->evidence()},{"pipeline_calls",calls}};
      }
    }
    collision_detection::CollisionRequest cr; cr.contacts=true; cr.max_contacts=20;
    collision_detection::CollisionResult collision; scene->checkCollision(cr,collision);
    if(collision.collision) {
      J pairs=J::array();for(auto& p:collision.contacts) pairs.push_back({p.first.first,p.first.second});
      return {{"status","INVALID_START_COLLISION"},{"collision_pairs",pairs},{"scene_import_s",import_s}};
    }
    if(op=="compose") {
      if(req.value("require_native_motion",false)) throw std::runtime_error("NATIVE_COLD_COMPOSITION_FORBIDDEN");
      mtc::Task task("",false);task.setRobotModel(model);task.setName("checked_complete_cycle");
      auto initial=std::make_unique<mtc::stages::FixedState>("frozen_start");initial->setState(scene);task.add(std::move(initial));
      for(const auto& spec:req.at("stages")) task.add(std::make_unique<CheckedProcessStage>(spec));
      task.plan(1);if(task.solutions().empty()) throw std::runtime_error("MTC_COMPOSITION_FAILED");
      const auto& last=task.solutions().front()->end()->scene();std::vector<std::string> attached;
      std::vector<const moveit::core::AttachedBody*> bodies; last->getCurrentState().getAttachedBodies(bodies); for(const auto* body:bodies) attached.push_back(body->getName());
      return {{"status","SUCCESS"},{"stage_count",req.at("stages").size()}, {"world_count",last->getWorld()->size()},
        {"attached_objects",attached},{"mtc_composition_s",seconds(begin)},{"physical_execution","NOT_RUN"}};
    }
    if(req.value("require_native_motion",false)) return nativeTaskPlan(req,scene,clearance,process);
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
      if(req.at("flange_from_task_tcp")!=identity.at("flange_from_task_tcp")) throw std::runtime_error("TASK_TCP_CONTEXT_MISMATCH");
      const std::string tcp=identity.at("task_tcp_link");
      if(!model->getLinkModel(tcp) || !model->getJointModelGroup("manipulator")->canSetStateFromIK(tcp)) throw std::runtime_error("TASK_TCP_IK_UNAVAILABLE");
      motion->setIKFrame(Eigen::Isometry3d::Identity(),tcp);
      geometry_msgs::msg::PoseStamped target;target.header.frame_id="world";target.pose=pose(req.at("goal_pose"));motion->setGoal(target);
    } else {
      auto goal=req.at("q_goal").get<std::vector<double>>();if(goal.size()!=names.size()) throw std::runtime_error("INVALID_GOAL");
      std::map<std::string,double> joints;for(size_t i=0;i<names.size();++i) joints[names[i]]=goal[i];motion->setGoal(joints);
    }
    auto* motion_stage=motion.get();
    task.add(std::move(motion)); auto planning=Clock::now(); ++requests; clearance->phase="search"; task.plan(1);double plan_s=seconds(planning);
    J out={{"status","SEARCH_EXHAUSTED"},{"pipeline_id",pipeline},{"planner_id",planner},{"mtc_attempt_index",requests},{"pipeline_calls",calls},
      {"scene_import_s",import_s},{"scene_updated",changed},{"mtc_plan_s",plan_s},{"world_count",scene->getWorld()->size()},
      {"attached_id",attached_id},{"task_tcp_identity",identity.at("task_tcp_fingerprint")},{"interpolated_link",req.contains("goal_pose")?identity.at("task_tcp_link"):J(nullptr)},
      {"resident_seed",seed},{"request_seed",req.at("seed")},{"authoritative_status","NOT_RUN"},{"clearance",clearance->evidence()}};
    if(task.solutions().empty()) {
      if(pipeline=="pilz_industrial_motion_planner") out["status"]="NATIVE_PLANNING_FAILED";
      std::ostringstream why; task.explainFailure(why);out["detail"]=why.str(); J failures=J::array();for(const auto& failure:motion_stage->failures()) failures.push_back(failure->comment());out["failure_comments"]=failures;
      for(const auto& item:failures) {
        const auto message=item.get<std::string>();
        if(message.find("NO_IK_SOLUTION")!=std::string::npos) out["status"]="NATIVE_IK_FAILED";
        else if(message.find("INVALID_MOTION_PLAN")!=std::string::npos) out["status"]="NATIVE_INVALID_MOTION_PLAN";
      }return out;}
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
  setenv("RCUTILS_LOGGING_USE_STDOUT","0",1);rclcpp::init(argc,argv);std::map<std::string,std::unique_ptr<Worker>> workers;std::map<std::string,J> startups,init_requests;std::string line;
  while(std::getline(std::cin,line)) {
    J req,answer;try {req=J::parse(line); auto* protocol=std::cout.rdbuf(std::cerr.rdbuf()); try {const auto key=req.at("identity").dump();
      J init_content=req;init_content.erase("request_id");
      if(req.at("op")=="init" && startups.count(key)) {
        if(init_content!=init_requests.at(key)) throw std::runtime_error("CONTEXT_IDENTITY_REUSED_WITH_DIFFERENT_CONTENT");
        answer=startups.at(key);answer["context_reused"]=true;
      }
      else {
        if(req.at("op")=="init") {
          if(workers.size()>=4) throw std::runtime_error("UNSUPPORTED_RESIDENT_CONTEXT_CAPACITY");
          workers[key]=std::make_unique<Worker>();
        }
        if(!workers.count(key)) throw std::runtime_error("UNKNOWN_MODEL_TCP_CONTEXT");
        answer=workers.at(key)->run(req);
        if(req.at("op")=="init") {answer["context_reused"]=false;startups[key]=answer;init_requests[key]=init_content;}
      }} catch(...) {std::cout.rdbuf(protocol);throw;} std::cout.rdbuf(protocol);} catch(const std::exception& e) {answer={{"status","ERROR"},{"detail",e.what()}};}
    answer["request_id"]=req.value("request_id",std::string());std::cout<<answer.dump()<<std::endl;
  }
  rclcpp::shutdown();return 0;
}
