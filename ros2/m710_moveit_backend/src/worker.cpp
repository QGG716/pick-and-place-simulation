#include "native_robot.h"
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
#include <condition_variable>
#include <cstring>
#include <deque>
#include <functional>
#include <iostream>
#include <map>
#include <mutex>
#include <set>
#include <thread>
#include "clearance.h"
#include "process_policy.h"
#include "basic_sampled_path.h"
#include "search_trace.h"
#include "scene_geometry.h"
#include "eco65_cycle.h"
#include "native_zero_motion.h"
#include "native_semantic_event.h"
#include "static_prior.h"
#include "native_timing.h"
#include "native_prepare.h"
#include "binary_frames.h"
#include "sc_composite.h"
#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <random_numbers/random_numbers.h>
#include <moveit/kinematic_constraints/utils.h>

using J = nlohmann::json;
namespace mtc = moveit::task_constructor;
using Clock = std::chrono::steady_clock;
double seconds(Clock::time_point t) { return std::chrono::duration<double>(Clock::now()-t).count(); }
// OMPL's process-global seed generator: seeded once at the first init, and
// re-seeded with the same value by an ECO65 task reset.
static bool g_ompl_seeded=false;static uint32_t g_process_seed=0;
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
  if(b.contains("convex_mesh")) {
    const auto& value=b.at("convex_mesh");shape_msgs::msg::Mesh mesh;
    for(const auto& v:value.at("vertices")) {
      if(v.size()!=3)throw std::runtime_error("INVALID_CONVEX_VERTEX");
      geometry_msgs::msg::Point p;p.x=v.at(0);p.y=v.at(1);p.z=v.at(2);
      if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))throw std::runtime_error("INVALID_CONVEX_VERTEX");
      mesh.vertices.push_back(p);
    }
    for(const auto& face:value.at("triangles")) {
      if(face.size()!=3)throw std::runtime_error("INVALID_CONVEX_FACE");
      shape_msgs::msg::MeshTriangle t;
      for(size_t i=0;i<3;++i){const int index=face.at(i);if(index<0||size_t(index)>=mesh.vertices.size())throw std::runtime_error("INVALID_CONVEX_INDEX");t.vertex_indices[i]=index;}
      mesh.triangles.push_back(t);
    }
    if(mesh.vertices.size()<4||mesh.triangles.size()<4)throw std::runtime_error("EMPTY_CONVEX_WORLD_MESH");
    o.meshes.push_back(mesh);o.mesh_poses.push_back(pose(b.at("pose")));return o;
  }
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
    state.copyJointGroupPositions(m710::nativeFrames().planning_group,incoming);
    auto first=spec.at("path").at(0).get<std::vector<double>>();
    if(first.size()!=incoming.size()) throw std::runtime_error("STAGE_JOINT_COUNT_MISMATCH");
    for(size_t i=0;i<first.size();++i) if(std::abs(first[i]-incoming[i])>1e-9) throw std::runtime_error("STAGE_DISCONTINUITY");
    auto trajectory=std::make_shared<robot_trajectory::RobotTrajectory>(state.getRobotModel(),m710::nativeFrames().planning_group);
    size_t index=0;
    for(const auto& row:spec.at("path")) {
      auto q=row.get<std::vector<double>>();state.setJointGroupPositions(m710::nativeFrames().planning_group,q);state.update();
      if(!state.satisfiesBounds()) throw std::runtime_error("STAGE_BOUNDS");
      trajectory->addSuffixWayPoint(state,index==0?0.:spec.at("durations").at(index-1).get<double>());++index;
    }
    next->setCurrentState(state);
    if(spec.contains("attach")) {
      const auto& b=spec.at("attach");std::string id=b.at("id");
      if(!next->getWorld()->hasObject(id)) throw std::runtime_error("ATTACH_WORLD_OBJECT_MISSING");
      next->getWorldNonConst()->removeObject(id);
      moveit_msgs::msg::AttachedCollisionObject a;a.link_name=m710::nativeFrames().flange_link;a.object=object(b,m710::nativeFrames().flange_link);
      a.touch_links=b.at("touch_links").get<std::vector<std::string>>();
      if(!next->processAttachedCollisionObjectMsg(a)) throw std::runtime_error("ATTACH_FAILED");
    }
    if(spec.contains("release")) {
      const auto& b=spec.at("release");std::string id=b.at("id");
      if(!next->getCurrentState().hasAttachedBody(id)) throw std::runtime_error("RELEASE_ATTACHED_OBJECT_MISSING");
      moveit_msgs::msg::AttachedCollisionObject a;a.link_name=m710::nativeFrames().flange_link;a.object.id=id;a.object.operation=a.object.REMOVE;
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
// populated inside computeForward(), never JSON paths. A checked semantic
// event has an end scene and no motion trajectory.
struct NativeStageRecord {
  J request,result;
  planning_scene::PlanningScenePtr start,end;
  robot_trajectory::RobotTrajectoryPtr trajectory;
  std::shared_ptr<m710::Clearance> clearance;
  std::shared_ptr<m710::ProcessPolicy> process;
  std::string parent;
  size_t cache_hits=0;
  bool ready=false;
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
    from.scene()->getCurrentState().copyJointGroupPositions(m710::nativeFrames().planning_group,q);
    const auto expected=r.request.at("q_start").get<std::vector<double>>();
    if(q.size()!=expected.size()) throw std::runtime_error("NATIVE_TASK_JOINT_COUNT_MISMATCH");
    for(size_t i=0;i<q.size();++i) if(std::abs(q[i]-expected[i])>1e-9) throw std::runtime_error("NATIVE_TASK_UNPLANNED_CONNECTION");
    verify_scene_(from.scene(),r.start,r.request);
    if(r.ready) ++r.cache_hits;else generate_(r);
    if(!r.ready) return;
    if(!r.end) throw std::runtime_error("NATIVE_TASK_READY_WITHOUT_END");
    if(r.result.value("generation_source",std::string())=="SEMANTIC_EVENT" ||
       r.result.value("generation_source",std::string())=="EXTERNAL_AXIS") {
      std::vector<double> terminal;r.end->getCurrentState().copyJointGroupPositions(m710::nativeFrames().planning_group,terminal);
      // No-motion semantics cannot use the motion connection tolerance to
      // change the real predecessor, even by a sub-nanoradian amount.
      if(q!=terminal) {r.ready=false;throw std::runtime_error("NATIVE_SEMANTIC_START_CHANGED");}
    }
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
  bool diagnostic=false;
};
// Reply compression only. The request, bound predicates and native record
// context retain the complete policy; cold/build outputs are unaffected.
void compactPlanningOnlyReply(J& reply,bool planning_only,const std::string& mode) {
  if(planning_only && mode=="static_prior_fast" && reply.contains("clearance") && reply.at("clearance").is_object())
    reply["clearance"].erase("policy");
}
class Worker {
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<robot_model_loader::RobotModelLoader> loader;
  moveit::core::RobotModelPtr model;
  planning_scene::PlanningScenePtr base;
  std::map<std::string,planning_pipeline::PlanningPipelinePtr> pipelines;
  planning_interface::PlannerConfigurationMap original_ompl_configurations;
  std::vector<std::string> names;
  J identity, scene_content, bound_policy, bound_tools, bound_compliant, bound_process_geometry; std::string scene_key;
  // process_geometry_ref (physical single carton, 2026-11-22): the caller's
  // digest of the process geometry it sent in init, stored verbatim. A request
  // may name that geometry by this digest instead of carrying it again.
  std::string bound_geometry_digest;
  uint32_t seed=0; size_t requests=0; std::map<std::string,size_t> calls;
  std::map<std::string,std::unique_ptr<NativeTaskSession>> tasks;
  // Uploaded ECO65 scenes (step 4): handle -> fingerprint, world, imported scene.
  struct SceneHandle {std::string fingerprint;J world;planning_scene::PlanningScenePtr scene;};
  std::map<std::string,SceneHandle> scene_handles;size_t scene_version=0;bool world_from_handle=false;
  size_t ik_calls=0;
  bool planning_only=false;  // Bound at process initialization, never per-stage downgrade.
  std::string planning_mode="cold_from_scratch";
  std::string evaluation_profile="strict";
  bool directed_j1=false;
  std::unique_ptr<m710::EcoCycleBinding> cycle_binding;
  m710::NativeDeadlinePtr native_deadline;
  Clock::time_point request_deadline=Clock::time_point::max();
  double remainingBudget() const {return std::chrono::duration<double>(request_deadline-Clock::now()).count();}
  J static_prior_database=nullptr;
  std::string prior_placement_policy_sha256;
  void checkPriorContext(const J& req) const {
    const auto& context=req.at("prior_context");const auto& policy=req.at("clearance_policy");
    auto equal=[&](const char* key,const J& value) {
      if(!context.contains(key)||context.at(key)!=value) throw std::runtime_error(std::string("PRIOR_REQUEST_BINDING_MISMATCH:")+key);
    };
    auto sorted=[](J value){std::sort(value.begin(),value.end());return value;};
    equal("schema","m710_static_prior_context_v1");equal("identity",identity);
    equal("flange_from_task_tcp",identity.at("flange_from_task_tcp"));
    if(req.at("flange_from_task_tcp")!=identity.at("flange_from_task_tcp")) throw std::runtime_error("TASK_TCP_CONTEXT_MISMATCH");
    equal("tool_links",sorted(bound_tools));equal("compliant_tool_links",sorted(bound_compliant));
    equal("collision_policy",bound_policy);
    J fixed=J::array();std::set<std::string> dynamic;
    for(const auto& body:req.at("world")) {
      const auto category=body.value("category",std::string());
      if(category=="carton"||category=="payload") dynamic.insert(body.at("id").get<std::string>());
      else fixed.push_back(body);
    }
    std::sort(fixed.begin(),fixed.end(),[](const J& a,const J& b){return a.at("id").get<std::string>()<b.at("id").get<std::string>();});
    equal("static_world",fixed);
    const auto& attached=req.at("attachment");
    if(!attached.is_null()) dynamic.insert(attached.at("id").get<std::string>());
    J pairs=J::array();
    for(const auto& pair:req.at("allowed_pairs")) {
      if(pair.size()!=2) throw std::runtime_error("PRIOR_ACM_PAIR_INVALID");
      if(!dynamic.count(pair.at(0).get<std::string>())&&!dynamic.count(pair.at(1).get<std::string>()))pairs.push_back(sorted(pair));
    }
    equal("allowed_pairs",sorted(pairs));
    const double resolution=policy.at("edge_resolution_rad");
    if(!std::isfinite(resolution)||resolution<=0) throw std::runtime_error("PRIOR_RESOLUTION_INVALID");
    equal("interpolation",{{"edge_resolution_rad",resolution},{"joint_step_l1_rad",std::min(resolution,.01)},
      {"distance_norm","L1"},{"waypoint_semantics","joint_linear_between_nodes"}});
    equal("receiver_reserve_m",policy.value("receiver_reserve_m",0.));
    J attachment={{"mode","empty"}};
    if(!attached.is_null()) {
      m710::processTransform(attached.at("pose"));m710::processBox(attached);
      attachment={{"mode","loaded"},{"size",attached.at("size")},{"pose",attached.at("pose")},
        {"touch_links",sorted(attached.at("touch_links"))}};
    }
    equal("attachment",attachment);
    const auto& limits=context.at("joint_limits_rad");
    if(limits.size()!=names.size())throw std::runtime_error("PRIOR_JOINT_LIMITS_INVALID");
    for(size_t i=0;i<names.size();++i) {
      const auto& bounds=model->getVariableBounds(names[i]);
      const double lo=limits.at(i).at(0),hi=limits.at(i).at(1);
      if(!std::isfinite(lo)||!std::isfinite(hi)||lo>=hi ||
        (bounds.position_bounded_ && (lo<bounds.min_position_-1e-9 || hi>bounds.max_position_+1e-9)))
        throw std::runtime_error("PRIOR_JOINT_LIMITS_EXCEED_MODEL");
    }
    const std::string placement=context.at("placement_policy_sha256");
    if(placement.empty()||(!prior_placement_policy_sha256.empty()&&placement!=prior_placement_policy_sha256))
      throw std::runtime_error("PRIOR_PLACEMENT_POLICY_MISMATCH");
  }
  static bool samePose(const Eigen::Isometry3d& a,const Eigen::Isometry3d& b) {return (a.matrix()-b.matrix()).cwiseAbs().maxCoeff()<=1e-7;}
  // Between adjacent motions only the selected target may change ownership.
  // Its actual pose must be identical on both sides of attach/release.
  void checkTransition(const planning_scene::PlanningSceneConstPtr& before,const planning_scene::PlanningSceneConstPtr& after,const J& req,
      const std::string& location="mtc_compute") {
    const std::string target=req.at("clearance_policy").at("target_id");
    auto ids=before->getWorld()->getObjectIds(),next=after->getWorld()->getObjectIds();
    const bool touch=!bound_process_geometry.is_null() && bound_process_geometry.value("geometry_type",std::string())=="known_tip_finite_datum_v1";
    std::set<std::string> a(ids.begin(),ids.end()),b(next.begin(),next.end());
    if(!touch && !cycle_binding){a.erase(target);b.erase(target);}
    if(a!=b) throw std::runtime_error("NATIVE_TASK_WORLD_IDENTITY_CHANGED");
    for(const auto& id:a) {
      const auto x=before->getWorld()->getObject(id),y=after->getWorld()->getObject(id);
      if(x->shapes_.size()!=y->shapes_.size() || x->global_shape_poses_.size()!=x->shapes_.size() ||
         y->global_shape_poses_.size()!=y->shapes_.size()) throw std::runtime_error("NATIVE_TASK_WORLD_GEOMETRY_CHANGED");
      for(size_t i=0;i<x->shapes_.size();++i) {
        if(x->shapes_[i]->type==shapes::MESH && y->shapes_[i]->type==shapes::MESH) {
          const auto* a=static_cast<const shapes::Mesh*>(x->shapes_[i].get());const auto* b=static_cast<const shapes::Mesh*>(y->shapes_[i].get());
          if(a->vertex_count!=b->vertex_count||a->triangle_count!=b->triangle_count ||
             !std::equal(a->vertices,a->vertices+3*a->vertex_count,b->vertices)||
             !std::equal(a->triangles,a->triangles+3*a->triangle_count,b->triangles) ||
             (x->global_shape_poses_[i].matrix()-y->global_shape_poses_[i].matrix()).cwiseAbs().maxCoeff()>1e-12)
            throw std::runtime_error("NATIVE_TASK_WORLD_MESH_CHANGED");
          continue;
        }
        if(x->shapes_[i]->type!=shapes::BOX || y->shapes_[i]->type!=shapes::BOX) throw std::runtime_error("NATIVE_TASK_WORLD_GEOMETRY_UNSUPPORTED");
        const auto* sx=static_cast<const shapes::Box*>(x->shapes_[i].get());const auto* sy=static_cast<const shapes::Box*>(y->shapes_[i].get());
        for(int k=0;k<3;++k) if(sx->size[k]!=sy->size[k]) throw std::runtime_error("NATIVE_TASK_WORLD_GEOMETRY_CHANGED");
        if(!samePose(m710::worldShapePose(*x,i),m710::worldShapePose(*y,i))) throw std::runtime_error("NATIVE_TASK_WORLD_POSE_CHANGED");
      }
    }
    if(cycle_binding) {
      std::vector<const moveit::core::AttachedBody*> old_attached,new_attached;
      before->getCurrentState().getAttachedBodies(old_attached);after->getCurrentState().getAttachedBodies(new_attached);
      if(old_attached.size()!=new_attached.size())throw std::runtime_error("ECO65_CYCLE_UNRECORDED_OWNERSHIP_CHANGE");
      for(const auto* old:old_attached) {
        const auto* next=after->getCurrentState().getAttachedBody(old->getName());
        if(!next || old->getAttachedLink()!=next->getAttachedLink() || old->getTouchLinks()!=next->getTouchLinks() ||
            old->getShapes().size()!=1 || next->getShapes().size()!=1 ||
            old->getShapes()[0]->type!=shapes::BOX || next->getShapes()[0]->type!=shapes::BOX ||
            (old->getShapePosesInLinkFrame()[0].matrix()-next->getShapePosesInLinkFrame()[0].matrix()).cwiseAbs().maxCoeff()>1e-12)
          throw std::runtime_error("ECO65_CYCLE_UNRECORDED_ATTACHMENT_CHANGE");
        const auto* x=static_cast<const shapes::Box*>(old->getShapes()[0].get());
        const auto* y=static_cast<const shapes::Box*>(next->getShapes()[0].get());
        for(int k=0;k<3;++k)if(x->size[k]!=y->size[k])throw std::runtime_error("ECO65_CYCLE_ATTACHMENT_SIZE_CHANGED");
      }
      return;
    }
    if(touch) {
      // The finite datum remains an immutable mesh in the world. Its vertices,
      // triangles and pose were checked above, including the target itself.
      if(!a.count(target)||before->getCurrentState().hasAttachedBody(target)||after->getCurrentState().hasAttachedBody(target))
        throw std::runtime_error("TOUCH_FIXED_DATUM_OWNERSHIP_CHANGED");
      return;
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
        std::vector<double> q;fresh.copyJointGroupPositions(m710::nativeFrames().planning_group,q);
        const auto* body=fresh.getAttachedBody(target);
        J detail={{"q",q},{"in_world",scene->getWorld()->hasObject(target)},{"attached",bool(body)},
          {"cached_world_pose",serial(cached)},{"flange_world_pose",serial(fresh.getGlobalLinkTransform(m710::nativeFrames().flange_link))}};
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
  struct CycleScene {
    planning_scene::PlanningScenePtr scene;
    std::shared_ptr<m710::ProcessPolicy> process;
    std::shared_ptr<m710::Clearance> clearance;
    J evidence;
  };
  void checkCycleGeometry(const J& req) const {
    if(!cycle_binding || req.at("identity")!=identity ||
       req.at("clearance_policy").at("source_policy")!=bound_policy ||
       req.at("clearance_policy").at("tool_links")!=bound_tools ||
       req.at("clearance_policy").at("compliant_tool_links")!=bound_compliant)
      throw std::runtime_error("ECO65_CYCLE_MODEL_POLICY_CHANGED");
    cycle_binding->checkRequest(req);
    for(auto it=bound_process_geometry.begin();it!=bound_process_geometry.end();++it)
      if(it.key()!="task_contract" && (!req.at("process_policy").contains(it.key()) ||
          req.at("process_policy").at(it.key())!=it.value()))
        throw std::runtime_error("ECO65_CYCLE_CONTOUR_GEOMETRY_CHANGED");
    auto original=bound_process_geometry.at("task_contract"),runtime=req.at("process_policy").at("task_contract");
    for(const std::string key:{"target_id","target_face","stack_carton_ids","original_support_names","receiving",
                              "continuous_task_mode","completed_ids"}) {original.erase(key);runtime.erase(key);}
    if(original!=runtime)throw std::runtime_error("ECO65_CYCLE_TASK_PERMISSION_CHANGED");
  }
  CycleScene cycleScene(const J& req,bool world_target=false) {
    checkCycleGeometry(req);
    CycleScene out;out.scene=std::make_shared<planning_scene::PlanningScene>(model);
    auto state=out.scene->getCurrentState();
    const auto q=req.at("q_start").get<std::vector<double>>();
    if(q.size()!=names.size())throw std::runtime_error("ECO65_CYCLE_JOINT_COUNT");
    for(double x:q)if(!std::isfinite(x))throw std::runtime_error("ECO65_CYCLE_INVALID_Q");
    state.setJointGroupPositions(m710::nativeFrames().planning_group,q);state.update();
    if(!state.satisfiesBounds())throw std::runtime_error("ECO65_CYCLE_Q_OUT_OF_BOUNDS");
    out.scene->setCurrentState(state);
    for(const auto& body:req.at("world"))
      if(!out.scene->processCollisionObjectMsg(object(body)))throw std::runtime_error("ECO65_CYCLE_SCENE_IMPORT_FAILED");
    for(const auto& pair:req.at("allowed_pairs"))
      out.scene->getAllowedCollisionMatrixNonConst().setEntry(pair.at(0).get<std::string>(),pair.at(1).get<std::string>(),true);
    if(!req.at("attachment").is_null()) {
      const auto& b=req.at("attachment");
      moveit_msgs::msg::AttachedCollisionObject a;a.link_name=m710::nativeFrames().flange_link;
      a.object=object(b,m710::nativeFrames().flange_link);a.touch_links=b.at("touch_links").get<std::vector<std::string>>();
      if(!out.scene->processAttachedCollisionObjectMsg(a))throw std::runtime_error("ECO65_CYCLE_ATTACH_IMPORT_FAILED");
    }
    out.process=std::make_shared<m710::ProcessPolicy>(out.scene,req,native_deadline);
    out.clearance=std::make_shared<m710::Clearance>(out.scene,req.at("clearance_policy"),"optimized",native_deadline,req);
    out.clearance->phase="initial";J failure=nullptr;
    if(!out.clearance->check(out.scene->getCurrentState()))failure=out.clearance->last_failure;
    if(failure.is_null() && !out.process->check(out.scene->getCurrentState(),req.value("endpoint",false)))
      failure=out.process->last_failure;
    if(failure.is_null()) {
      collision_detection::CollisionRequest cr;collision_detection::CollisionResult result;
      out.scene->checkCollision(cr,result);
      if(result.collision)failure={{"reason","ECO65_CYCLE_ORDINARY_COLLISION"}};
    }
    if(failure.is_null() && world_target)failure=m710::ecoCycleWorldTarget(out.scene,req,native_deadline);
    out.evidence={{"status",failure.is_null()?"SUCCESS":"STATE_REJECTED"},{"failure",failure},
      {"clearance",out.clearance->evidence()},{"process",out.process->evidence()}};
    // The complete immutable policy is already present in every archived
    // request. Avoid multiplying its exact pair inventory by lift samples.
    if(out.evidence.at("clearance").is_object())out.evidence["clearance"].erase("policy");
    out.evidence["policy_fingerprint"]=identity.at("policy_fingerprint");
    return out;
  }
  Eigen::Isometry3d cycleTargetPose(const planning_scene::PlanningSceneConstPtr& scene,const std::string& id) const {
    const auto& state=scene->getCurrentState();
    const auto* attached=state.hasAttachedBody(id)?state.getAttachedBody(id):nullptr;
    const auto world=scene->getWorld()->getObject(id);
    if(bool(attached)==bool(world))throw std::runtime_error("ECO65_CYCLE_TARGET_OWNER_REQUIRED");
    return attached?attached->getGlobalCollisionBodyTransforms().at(0):m710::worldShapePose(*world,0);
  }
  void generateCycleTransition(NativeStageRecord& r) {
    const auto begin=Clock::now();const auto& req=r.request;const auto& transition=req.at("transition");
    const std::string event=transition.at("event_type"),kind=transition.at("kind");
    const auto& after_req=transition.at("after_request");
    r.result={{"status","NATIVE_TRANSITION_REJECTED"},{"generation_source",kind=="external_axis"?"EXTERNAL_AXIS":"SEMANTIC_EVENT"},
      {"trajectory_present",false},{"returned_waypoint_count",0},{"native_solver_calls",J::object()},
      {"mtc_generation",true},{"generated_during_task",true},{"task_id",req.at("task_id")},
      {"stage_id",req.at("stage_id")},{"parent_stage_id",r.parent},{"request_id",req.at("request_id")},
      {"pipeline_id",req.at("pipeline_id")},{"planner_id",req.at("planner_id")},{"event_type",event},
      {"physical_execution","NOT_RUN"},{"execution_ready",false},{"sample_checks",J::array()}};
    try {
      if(kind!="event" && kind!="external_axis")throw std::runtime_error("ECO65_CYCLE_TRANSITION_KIND");
      if((kind=="external_axis")!=(event=="COUPLED_LIFT"))throw std::runtime_error("ECO65_CYCLE_AXIS_KIND_MISMATCH");
      const std::set<std::string> events={"ATTACH","RELEASE","NOMINAL_RECEIVE","IDEAL_OUTFEED","IDEAL_RECEIVER_DRAIN","NEXT_TARGET","COUPLED_LIFT"};
      if(!events.count(event))throw std::runtime_error("ECO65_CYCLE_EVENT_UNSUPPORTED");
      checkCycleGeometry(after_req);
      if(req.at("q_start")!=after_req.at("q_start") || req.at("task_id")!=after_req.at("task_id"))
        throw std::runtime_error("ECO65_CYCLE_TRANSITION_Q_CHANGED");
      const std::string target=req.at("process_policy").at("target_id"),next=after_req.at("process_policy").at("target_id");
      if((event!="NEXT_TARGET" && target!=next) || (event=="NEXT_TARGET" && target==next))
        throw std::runtime_error("ECO65_CYCLE_EVENT_TARGET_CHANGED");
      auto complete=req.at("completed_ids").get<std::set<std::string>>();
      auto next_complete=after_req.at("completed_ids").get<std::set<std::string>>();
      if(event=="IDEAL_RECEIVER_DRAIN") {
        if(complete.count(target))throw std::runtime_error("ECO65_CYCLE_DUPLICATE_DRAIN");
        complete.insert(target);
      }
      if(complete!=next_complete)throw std::runtime_error("ECO65_CYCLE_UNRECORDED_COMPLETION");
      if(event=="NEXT_TARGET" && (!complete.count(target)||complete.count(next)))
        throw std::runtime_error("ECO65_CYCLE_NEXT_TARGET_NOT_AVAILABLE");
      const auto& before_attachment=req.at("attachment");const auto& after_attachment=after_req.at("attachment");
      if(event=="ATTACH") {
        if(!before_attachment.is_null() || after_attachment.is_null() || req.at("stage")!="contact")
          throw std::runtime_error("ECO65_CYCLE_ATTACH_STATE");
      } else if(event=="RELEASE") {
        if(before_attachment.is_null() || !after_attachment.is_null())throw std::runtime_error("ECO65_CYCLE_RELEASE_STATE");
      } else if(before_attachment!=after_attachment)throw std::runtime_error("ECO65_CYCLE_UNRECORDED_ATTACHMENT");
      const std::string interval=req.at("business_interval"),next_interval=after_req.at("business_interval");
      if(req.value("completion_goal",std::string())!=after_req.value("completion_goal",std::string()))
        throw std::runtime_error("ECO65_CYCLE_COMPLETION_GOAL_CHANGED");
      const std::string expected_interval=m710::ecoCycleNextInterval(req,event);
      if(next_interval!=expected_interval || (event=="RELEASE"&&interval!="OUTBOUND"))
        throw std::runtime_error("ECO65_CYCLE_BUSINESS_INTERVAL_CHANGED");
      const double expected_reverse=next_interval==interval?req.at("j1_reverse_used_rad").get<double>():0.;
      if(std::abs(after_req.at("j1_reverse_used_rad").get<double>()-expected_reverse)>1e-14)
        throw std::runtime_error("ECO65_CYCLE_EVENT_REVERSE_BUDGET_CHANGED");
      if(event!="NEXT_TARGET" && req.at("process_policy").at("selection_binding")!=after_req.at("process_policy").at("selection_binding"))
        throw std::runtime_error("ECO65_CYCLE_SELECTION_CHANGED");
      if(event!="COUPLED_LIFT" && req.at("mechanism_state")!=after_req.at("mechanism_state"))
        throw std::runtime_error("ECO65_CYCLE_UNRECORDED_LIFT");
      auto before_world=m710::ecoCycleObjects(req.at("world")),after_world=m710::ecoCycleObjects(after_req.at("world"));
      if(event=="COUPLED_LIFT" && before_attachment.is_null() && !complete.count(target)) {
        Eigen::Isometry3d original=Eigen::Isometry3d::Identity();
        for(int k=0;k<3;++k)original.translation()[k]=cycle_binding->cartons.at(target).at("pose").at(k);
        if((m710::processTransform(before_world.at(target).at("pose")).matrix()-original.matrix()).cwiseAbs().maxCoeff()>1e-12)
          throw std::runtime_error("ECO65_CYCLE_LIFT_WITH_UNDRAINED_RECEIVER_FORBIDDEN");
      }
      auto a=before_world,b=after_world;
      if(event=="ATTACH"||event=="RELEASE"||event=="NOMINAL_RECEIVE"||event=="IDEAL_OUTFEED"||event=="IDEAL_RECEIVER_DRAIN") {a.erase(target);b.erase(target);}
      if(a.size()!=b.size())throw std::runtime_error("ECO65_CYCLE_EVENT_WORLD_CHANGED");
      for(const auto& item:a) {
        if(!b.count(item.first))throw std::runtime_error("ECO65_CYCLE_EVENT_WORLD_ID_CHANGED");
        const bool moving=event=="COUPLED_LIFT"&&cycle_binding->moving.count(item.first);
        if((moving&&!m710::ecoCycleSameShape(item.second,b.at(item.first))) ||
            (!moving&&item.second!=b.at(item.first)))throw std::runtime_error("ECO65_CYCLE_EVENT_UNRELATED_OBJECT_CHANGED");
      }
      if(event=="ATTACH") {
        bool native_contact=false;std::string cursor=r.parent;
        const auto& records=tasks.at(req.at("task_id").get<std::string>())->records;
        while(!cursor.empty()) {
          const auto& prior=*records.at(cursor);
          std::vector<double> end_q;
          prior.end->getCurrentState().copyJointGroupPositions(m710::nativeFrames().planning_group,end_q);
          if(!prior.ready || prior.result.at("status")!="SUCCESS" ||
              end_q!=req.at("q_start").get<std::vector<double>>())break;
          if(prior.request.at("op")=="plan" && prior.request.at("stage")=="contact" && prior.trajectory) {
            native_contact=true;r.result["native_contact_ancestor_stage_id"]=cursor;break;
          }
          if(!cycle_binding->binding.contains("transverse_process_policy") ||
              !m710::ecoCycleContactPreservingLift(prior.request,req))break;
          cursor=prior.parent;
        }
        if(!native_contact)throw std::runtime_error("ECO65_CYCLE_ATTACH_WITHOUT_NATIVE_CONTACT");
      }
      r.clearance->phase="initial";
      if(!r.clearance->check(r.start->getCurrentState()) || !r.process->check(r.start->getCurrentState(),event=="ATTACH")) {
        r.result["failure"]={{"reason","ECO65_CYCLE_BEFORE_STATE_REJECTED"},
          {"clearance",r.clearance->last_failure},{"process",r.process->last_failure}};return;
      }
      const bool dropped=event=="NOMINAL_RECEIVE";
      if(dropped||event=="IDEAL_OUTFEED")r.result["receiving_surface_check"]=
        "ACTUAL_CONVEX_SUPPORT_PLANE_BOUND_V1_OTHER_PAIRS_FCL";
      if(dropped) {
        if(r.parent.empty())throw std::runtime_error("ECO65_CYCLE_RECEIVE_WITHOUT_RELEASE");
        const auto& prior=tasks.at(req.at("task_id").get<std::string>())->records.at(r.parent)->request;
        if(prior.at("op")!="eco65_transition" || prior.at("transition").at("event_type")!="RELEASE")
          throw std::runtime_error("ECO65_CYCLE_RECEIVE_WITHOUT_RELEASE");
      }
      auto after=cycleScene(after_req,dropped);
      r.result["after_check"]=after.evidence;
      if(after.evidence.at("status")!="SUCCESS") {r.result["failure"]=after.evidence.at("failure");return;}
      if(event=="ATTACH"||event=="RELEASE") {
        if(!samePose(cycleTargetPose(r.start,target),cycleTargetPose(after.scene,target)))
          throw std::runtime_error("ECO65_CYCLE_OWNERSHIP_TELEPORT");
      }
      if(event=="RELEASE") {
        const auto pose=cycleTargetPose(r.start,target);const auto half=m710::processBox(req.at("process_policy").at("target")).half;
        const Eigen::Vector3d extent=pose.linear().cwiseAbs()*half;
        const auto& receiver=req.at("process_policy").at("task_contract").at("receiving");
        const double gap=pose.translation().z()-extent.z()-receiver.at("bounds_D_m").at(1).at(2).get<double>();
        const bool templates=cycle_binding->binding.contains("transverse_process_policy");
        const double expected_drop=templates?req.at("process_option").at("expected_drop_m").get<double>():
            cycle_binding->binding.at("mechanism_spec").at("config").at("desired_drop_m").get<double>();
        if(templates && (std::abs(pose.translation().z()-extent.z()-
            req.at("process_option").at("release_bottom_world_m").get<double>())>1e-6 ||
            (pose.linear()-Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff()>1e-6))
          throw std::runtime_error("ECO65_TEMPLATE_RELEASE_HEIGHT_OR_ORIENTATION_CHANGED");
        if(!m710::ecoCycleLevelBox(pose) || std::abs(gap-expected_drop)>1e-6)
          throw std::runtime_error("ECO65_CYCLE_RELEASE_DROP_INVALID");
        for(int k=0;k<2;++k)
          if(pose.translation()[k]-extent[k]<receiver.at("bounds_D_m").at(0).at(k).get<double>()+receiver.at("margin_m").get<double>()-1e-6 ||
             pose.translation()[k]+extent[k]>receiver.at("bounds_D_m").at(1).at(k).get<double>()-receiver.at("margin_m").get<double>()+1e-6)
            throw std::runtime_error("ECO65_CYCLE_RELEASE_PROJECTION_OUTSIDE");
      }
      const bool checked_outfeed=cycle_binding->binding.contains("prototype");
      if(event=="IDEAL_RECEIVER_DRAIN") {
        if(r.parent.empty())throw std::runtime_error("ECO65_CYCLE_DRAIN_WITHOUT_WITHDRAWAL");
        const auto& previous=*tasks.at(req.at("task_id").get<std::string>())->records.at(r.parent);
        if(previous.request.at("stage")!="withdrawal" ||
            (checked_outfeed ? previous.request.at("op")!="eco65_transition" ||
              previous.request.at("transition").at("event_type")!="IDEAL_OUTFEED" : !previous.trajectory))
          throw std::runtime_error("ECO65_CYCLE_DRAIN_WITHOUT_WITHDRAWAL_OR_CHECKED_OUTFEED");
        const auto carton=r.start->getWorld()->getObject(target);
        if(!carton || carton->shapes_.size()!=1)throw std::runtime_error("ECO65_CYCLE_DRAIN_WORLD_TARGET_REQUIRED");
        double minimum=INFINITY;std::string closest;
        for(const auto& item:bound_tools) {
          const auto* link=model->getLinkModel(item.get<std::string>());
          for(size_t k=0;k<link->getShapes().size();++k) {
            native_deadline->require();
            const double gap=m710::ecoCycleSolidDistance(link->getShapes()[k],r.start->getCurrentState().getCollisionBodyTransform(link,k),
              carton->shapes_[0],m710::worldShapePose(*carton,0));
            native_deadline->require();
            if(gap<minimum){minimum=gap;closest=item.get<std::string>();}
          }
        }
        r.result["tool_target_separation"]={{"minimum_m",minimum},{"tool_link",closest},{"required_m",.01}};
        if(minimum<.01-1e-9)throw std::runtime_error("ECO65_CYCLE_DRAIN_TOOL_NOT_SEPARATED");
        const auto received=checked_outfeed?m710::ecoCycleOutfeedWorld(r.start,req,cycle_binding->binding,bound_tools,native_deadline):
          m710::ecoCycleWorldTarget(r.start,req,native_deadline);
        if(!received.is_null()){r.result["failure"]=received;return;}
        const auto box=m710::processBox(req.at("process_policy").at("target"));
        const double bottom=box.pose.translation().z()-(box.pose.linear().cwiseAbs()*box.half).z();
        if(checked_outfeed) {
          const double exit=cycle_binding->binding.at("prototype").at("configuration").at("outfeed").at("output_plane_x_m");
          if(box.pose.translation().x()+box.half.x()>=exit)throw std::runtime_error("ECO65_DRAIN_BEFORE_FULL_ENVELOPE_EXIT");
        } else if(std::abs(bottom-req.at("process_policy").at("task_contract").at("receiving").at("bounds_D_m").at(1).at(2).get<double>())>1e-6)
          throw std::runtime_error("ECO65_CYCLE_DRAIN_BEFORE_RECEIVING");
      }
      const auto& samples=transition.at("sample_requests");
      const auto sample_contract=transition.value("sample_contract",std::string("FULL_REQUESTS_V1"));
      const bool compact_outfeed=sample_contract=="HELD_WORLD_TARGET_POSES_V2";
      if(sample_contract!="FULL_REQUESTS_V1" && !compact_outfeed)
        throw std::runtime_error("ECO65_CYCLE_SAMPLE_CONTRACT_UNKNOWN");
      if(compact_outfeed && event!="IDEAL_OUTFEED")
        throw std::runtime_error("ECO65_OUTFEED_COMPACT_EVENT_SCOPE");
      if(event=="COUPLED_LIFT" || dropped) {
        if(samples.size()<2 || samples.size()>100)throw std::runtime_error("ECO65_CYCLE_FINITE_SAMPLES_REQUIRED");
        double last=0;bool first=true;
        const double start=event=="COUPLED_LIFT"?req.at("mechanism_state").at("height_m").get<double>():
          m710::processTransform(before_world.at(target).at("pose")).translation().z();
        const double end=event=="COUPLED_LIFT"?after_req.at("mechanism_state").at("height_m").get<double>():
          m710::processTransform(after_world.at(target).at("pose")).translation().z();
        if(start==end || (dropped && end>=start))throw std::runtime_error("ECO65_CYCLE_EMPTY_OR_REVERSED_AXIS");
        for(size_t index=0;index<samples.size();++index) {
          if(remainingBudget()<=1.)throw std::runtime_error("ECO65_CYCLE_SAMPLE_BUDGET_EXPIRED");
          const auto& sample=samples.at(index);
          if(sample.at("q_start")!=req.at("q_start") || sample.at("attachment")!=before_attachment ||
              sample.at("completed_ids")!=req.at("completed_ids") ||
              sample.at("process_policy").at("selection_binding")!=req.at("process_policy").at("selection_binding"))
            throw std::runtime_error("ECO65_CYCLE_SAMPLE_STATE_CHANGED");
          auto rows=m710::ecoCycleObjects(sample.at("world"));
          if((index==0 && rows!=before_world) || (index+1==samples.size() && rows!=after_world))
            throw std::runtime_error("ECO65_CYCLE_SAMPLE_ENDPOINT_WORLD_CHANGED");
          const double value=event=="COUPLED_LIFT"?sample.at("mechanism_state").at("height_m").get<double>():
            m710::processTransform(rows.at(target).at("pose")).translation().z();
          if((first&&std::abs(value-start)>1e-12) || (!first&&(std::abs(value-last)>.010000001 ||
                (end-start)*(value-last)<-1e-12)) || (index+1==samples.size()&&std::abs(value-end)>1e-12))
            throw std::runtime_error("ECO65_CYCLE_SAMPLE_COVERAGE_INVALID");
          if(rows.size()!=before_world.size())throw std::runtime_error("ECO65_CYCLE_SAMPLE_WORLD_CHANGED");
          for(const auto& item:before_world) {
            if(!rows.count(item.first))throw std::runtime_error("ECO65_CYCLE_SAMPLE_WORLD_ID_CHANGED");
            if(dropped && item.first==target) {
              auto original=m710::processTransform(item.second.at("pose")),actual=m710::processTransform(rows.at(item.first).at("pose"));
              actual.translation().z()=original.translation().z();
              if(!m710::ecoCycleSameShape(item.second,rows.at(item.first)) ||
                  (original.matrix()-actual.matrix()).cwiseAbs().maxCoeff()>1e-12)
                throw std::runtime_error("ECO65_CYCLE_DROP_MUST_BE_TRANSLATION_Z");
            } else if(event=="COUPLED_LIFT"&&cycle_binding->moving.count(item.first)) {
              if(!m710::ecoCycleSameShape(item.second,rows.at(item.first)))throw std::runtime_error("ECO65_CYCLE_LIFT_SHAPE_CHANGED");
            } else if(item.second!=rows.at(item.first))throw std::runtime_error("ECO65_CYCLE_SAMPLE_UNRELATED_OBJECT_CHANGED");
          }
          auto checked=cycleScene(sample,dropped);
          r.result["sample_checks"].push_back({{"index",index},{"axis_value_m",value},{"check",checked.evidence}});
          if(checked.evidence.at("status")!="SUCCESS"){r.result["failure"]=checked.evidence.at("failure");return;}
          first=false;last=value;
        }
        if(dropped) {
          const auto box=m710::processBox(after_req.at("process_policy").at("target"));
          const double bottom=box.pose.translation().z()-(box.pose.linear().cwiseAbs()*box.half).z();
          if(std::abs(bottom-after_req.at("process_policy").at("task_contract").at("receiving").at("bounds_D_m").at(1).at(2).get<double>())>1e-6)
            throw std::runtime_error("ECO65_CYCLE_RECEIVE_NOT_ON_SURFACE");
        }
      } else if(event=="IDEAL_OUTFEED") {
        std::unique_ptr<m710::EcoHeldOutfeedSamples> held_samples;
        if(compact_outfeed)held_samples=std::make_unique<m710::EcoHeldOutfeedSamples>(req);
        const size_t sample_count=compact_outfeed?held_samples->size():samples.size();
        r.result["outfeed_sample_contract"]=sample_contract;
        r.result["outfeed_sample_count"]=sample_count;
        if(!checked_outfeed||!before_attachment.is_null()||sample_count<2||sample_count>256||r.parent.empty())
          throw std::runtime_error("ECO65_OUTFEED_BINDING_OR_SAMPLE_COUNT");
        const auto& previous=*tasks.at(req.at("task_id").get<std::string>())->records.at(r.parent);
        if(previous.request.at("stage")!="withdrawal"||!previous.trajectory)
          throw std::runtime_error("ECO65_OUTFEED_WITHOUT_NATIVE_WITHDRAWAL");
        Eigen::Vector3d last;bool x_started=false;
        for(size_t index=0;index<sample_count;++index) {
          const auto& sample=compact_outfeed?held_samples->sample(index):samples.at(index);
          // The compact form cannot change held fields: it is constructed from
          // this checked request, with only the target pose substituted.
          if(!compact_outfeed)checkCycleGeometry(sample);
          if(sample.at("q_start")!=req.at("q_start")||!sample.at("attachment").is_null()||
              sample.at("completed_ids")!=req.at("completed_ids")||sample.at("mechanism_state")!=req.at("mechanism_state")||
              sample.at("process_policy").at("selection_binding")!=req.at("process_policy").at("selection_binding"))
            throw std::runtime_error("ECO65_OUTFEED_HELD_STATE_CHANGED");
          auto rows=m710::ecoCycleObjects(sample.at("world"));
          if((index==0&&rows!=before_world)||(index+1==sample_count&&rows!=after_world)||rows.size()!=before_world.size())
            throw std::runtime_error("ECO65_OUTFEED_SAMPLE_ENDPOINT_WORLD_CHANGED");
          for(const auto& item:before_world)if(item.first!=target) {
            if(!rows.count(item.first)||rows.at(item.first)!=item.second)throw std::runtime_error("ECO65_OUTFEED_UNRELATED_OBJECT_CHANGED");
          }
          if(!m710::ecoCycleSameShape(rows.at(target),before_world.at(target)))throw std::runtime_error("ECO65_OUTFEED_SHAPE_CHANGED");
          const auto pose=m710::processTransform(rows.at(target).at("pose"));
          if(index) {
            const Eigen::Vector3d delta=pose.translation()-last;
            if(delta.norm()>.01000001 || delta.x()>1e-9 || delta.y()>1e-9 ||
                (std::abs(delta.x())>1e-9&&std::abs(delta.y())>1e-9) || (x_started&&std::abs(delta.y())>1e-9))
              throw std::runtime_error("ECO65_OUTFEED_DIRECTION_OR_STEP_CHANGED");
            if(delta.x()<-1e-9)x_started=true;
          }
          // q, the mechanism and every non-target object are identical above.
          // Reuse their checked scene; update/check only the moving world box.
          if(!after.scene->processCollisionObjectMsg(object(rows.at(target))))
            throw std::runtime_error("ECO65_OUTFEED_TARGET_UPDATE_FAILED");
          const auto failure=m710::ecoCycleOutfeedWorld(after.scene,sample,cycle_binding->binding,bound_tools,native_deadline);
          r.result["sample_checks"].push_back({{"index",index},{"check",{{"status",failure.is_null()?"SUCCESS":"STATE_REJECTED"},
            {"failure",failure},{"held_world_reused",true},{"policy_fingerprint",identity.at("policy_fingerprint")}}},{"outfeed_failure",failure}});
          if(!failure.is_null()){r.result["failure"]=failure;return;}
          last=pose.translation();
        }
        const auto box=m710::processBox(after_req.at("process_policy").at("target"));
        if(last.x()+box.half.x()>=cycle_binding->binding.at("prototype").at("configuration").at("outfeed").at("output_plane_x_m").get<double>())
          throw std::runtime_error("ECO65_OUTFEED_FULL_ENVELOPE_NOT_OUTSIDE");
      } else if(!samples.empty())throw std::runtime_error("ECO65_CYCLE_UNEXPECTED_EVENT_SAMPLES");
      r.end=after.scene;r.trajectory.reset();r.ready=true;r.result["status"]="SUCCESS";
      r.result["event_wall_s"]=seconds(begin);r.result["q_unchanged"]=true;
      r.result["after_runtime_contract"]=after_req.at("process_policy").at("task_contract");
      r.result["planning_only_status"]="PLANNING_ONLY_NOT_EXECUTABLE";
    } catch(const std::exception& error) {
      r.ready=false;r.end.reset();r.trajectory.reset();r.result["failure"]={{"reason",error.what()}};
      r.result["event_wall_s"]=seconds(begin);
    }
  }
  J nativeTaskPlan(const J& req,const planning_scene::PlanningScenePtr& scene,
      const std::shared_ptr<m710::Clearance>& clearance,const std::shared_ptr<m710::ProcessPolicy>& process,bool diagnostic=false) {
    if(req.contains("path") || req.contains("stages")) throw std::runtime_error("NATIVE_COLD_EXTERNAL_PATH_FORBIDDEN");
    const bool transition=req.at("op")=="eco65_transition";
    if(transition) {if(!cycle_binding || diagnostic)throw std::runtime_error("ECO65_TRANSITION_SCOPE_REQUIRED");}
    else m710::validateNativeMotionScope(req);
    const std::string task_id=req.at("task_id"),stage_id=req.at("stage_id");
    if(diagnostic) {
      const std::string stage=req.at("stage");
      const bool eco_lin=identity.value("robot_variant",std::string())=="ECO65-B" &&
        stage=="extraction" && req.contains("goal_pose") && req.value("planner_id",std::string())=="LIN";
      if(!planning_only || task_id.rfind("diagnostic-",0)!=0 || (!eco_lin &&
        (!req.contains("q_goal") || req.contains("goal_pose") || !(stage=="pregrasp"||stage=="transit"||stage=="residence"))))
        throw std::runtime_error("DIAGNOSTIC_CONNECTION_SCOPE_REQUIRED");
    } else if(task_id.rfind("diagnostic-",0)==0)throw std::runtime_error("DIAGNOSTIC_TASK_NOT_FORMAL");

    if(!req.contains("parent_stage_id") || !req.at("parent_stage_id").is_string())
      throw std::runtime_error("NATIVE_TASK_EXPLICIT_PARENT_REQUIRED");
    const std::string parent=req.at("parent_stage_id");
    if(task_id.empty() || stage_id.empty()) throw std::runtime_error("NATIVE_TASK_ID_REQUIRED");
    if(!diagnostic && parent.empty() && !req.at("attachment").is_null()) throw std::runtime_error("NATIVE_COLD_INITIAL_ATTACHMENT_FORBIDDEN");
    if(!tasks.count(task_id)) {
      if(!parent.empty()) throw std::runtime_error("NATIVE_TASK_PARENT_MISSING");
      if(tasks.size()>=8) throw std::runtime_error("NATIVE_TASK_SESSION_CAPACITY");
      auto session=std::make_unique<NativeTaskSession>();session->task=std::make_unique<mtc::Task>("",false);
      session->task->setRobotModel(model);session->task->setName(task_id);session->initial_q=req.at("q_start");session->initial_world=req.at("world");
      session->target_id=req.at("clearance_policy").at("target_id");session->diagnostic=diagnostic;
      auto initial=std::make_unique<mtc::stages::FixedState>("frozen_actual_initial_state");initial->setState(scene);session->task->add(std::move(initial));
      tasks[task_id]=std::move(session);
    }
    auto& session=*tasks.at(task_id);
    if(session.diagnostic!=diagnostic)throw std::runtime_error("DIAGNOSTIC_TASK_SCOPE_CHANGED");
    if(!cycle_binding && req.at("clearance_policy").at("target_id")!=session.target_id) throw std::runtime_error("NATIVE_TASK_TARGET_ID_CHANGED");
    if(session.records.count(stage_id)) throw std::runtime_error("NATIVE_STAGE_ID_REUSED");
    if(session.records.size()>=4096) throw std::runtime_error("NATIVE_TASK_STAGE_CAPACITY");
    std::vector<std::string> ancestors;std::string cursor=parent;std::set<std::string> seen;
    while(!cursor.empty()) {
      if(!seen.insert(cursor).second || !session.records.count(cursor) || !session.records.at(cursor)->ready) throw std::runtime_error("NATIVE_TASK_PARENT_INVALID");
      ancestors.push_back(cursor);cursor=session.records.at(cursor)->parent;
    }
    std::reverse(ancestors.begin(),ancestors.end());
    if(ancestors.empty()) {
      if(req.at("q_start")!=session.initial_q || req.at("world")!=session.initial_world) throw std::runtime_error("NATIVE_TASK_INITIAL_STATE_CHANGED");
      if(cycle_binding && (req.at("business_interval")!="FIRST_APPROACH" ||
          req.at("j1_reverse_used_rad")!=0. || req.at("process_policy").at("target_id")!=cycle_binding->binding.at("initial_target_id") ||
          !req.at("completed_ids").empty()))
        throw std::runtime_error("ECO65_CYCLE_INITIAL_RUNTIME_INVALID");
      if(cycle_binding) {
        const std::string target=req.at("process_policy").at("target_id");
        Eigen::Isometry3d expected=Eigen::Isometry3d::Identity();
        for(int k=0;k<3;++k)expected.translation()[k]=cycle_binding->cartons.at(target).at("pose").at(k);
        if((cycleTargetPose(scene,target).matrix()-expected.matrix()).cwiseAbs().maxCoeff()>1e-12)
          throw std::runtime_error("ECO65_CYCLE_INITIAL_CARTON_MOVED");
      }
    } else {
      const auto& previous=*session.records.at(parent);checkTransition(previous.end,scene,req,"stage_registration");
      const auto& previous_context=cycle_binding&&previous.request.at("op")=="eco65_transition"?
        previous.request.at("transition").at("after_request"):previous.request;
      if(cycle_binding && (previous_context.at("process_policy").at("target_id")!=req.at("process_policy").at("target_id") ||
          previous_context.at("mechanism_state")!=req.at("mechanism_state") ||
          previous_context.at("completed_ids")!=req.at("completed_ids") ||
          previous_context.at("j1_direction_policy")!=req.at("j1_direction_policy") ||
          previous_context.at("business_interval")!=req.at("business_interval")))
        throw std::runtime_error("ECO65_CYCLE_PARENT_RUNTIME_CHANGED");
      if(cycle_binding) {
        const double expected_reverse=previous.result.contains("basic_path_check")?
          previous.result.at("basic_path_check").at("j1_total_reverse_after_rad").get<double>():
          previous_context.at("j1_reverse_used_rad").get<double>();
        if(std::abs(req.at("j1_reverse_used_rad").get<double>()-expected_reverse)>1e-14)
          throw std::runtime_error("ECO65_CYCLE_PARENT_REVERSE_BUDGET_CHANGED");
      }
      if(req.contains("process_policy") && req.at("process_policy").contains("selection_binding")) {
        if(!previous_context.at("process_policy").contains("selection_binding") ||
           previous_context.at("process_policy").at("selection_binding")!=req.at("process_policy").at("selection_binding"))
          throw std::runtime_error("ECO65_ACTIVE_SELECTION_CHANGED_WITHIN_BRANCH");
      }
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
      if(r.request.at("op")=="eco65_transition") {generateCycleTransition(r);return;}
      const auto& req=r.request;const std::string pipeline=req.at("pipeline_id"),planner=req.at("planner_id");
      if(pipeline=="static_prior") {
        const auto begin=Clock::now();const std::string stage=req.at("stage");
        if(!planning_only || planning_mode!="static_prior_fast" || planner!="SPARSE_PRM" ||
          !req.contains("q_goal") || req.contains("goal_pose") ||
          !(stage=="pregrasp" || stage=="transit" || stage=="residence"))
          throw std::runtime_error("PRIOR_FREE_MOTION_SCOPE_REQUIRED");
        checkPriorContext(req);
        const std::string mode=req.at("attachment").is_null()?"empty":"loaded";
        r.result={{"status","PRIOR_NOT_COVERED"},{"pipeline_id",pipeline},{"planner_id",planner},
          {"generation_source","STATIC_PRIOR_QUERY"},{"mtc_generation",true},{"generated_during_task",true},
          {"task_id",req.at("task_id")},{"stage_id",req.at("stage_id")},{"parent_stage_id",r.parent},
          {"request_id",req.at("request_id")},{"native_solver_calls",J::object()},{"native_solver_s",0.},
          {"mtc_plan_s",0.},{"pipeline_calls",calls},{"returned_waypoint_count",0},{"trajectory_present",false}};
        if(static_prior_database.is_null() || !static_prior_database.at("modes").contains(mode)) return;
        auto result=m710::StaticPriorGraph::query(static_prior_database.at("modes").at(mode),req,r.start,*r.clearance,r.process.get());
        r.result["status"]=result.at("status");r.result["prior_query_s"]=seconds(begin);
        if(result.contains("termination"))r.result["termination"]=result.at("termination");
        if(result.contains("prior_usage")) {r.result["prior_usage"]=result.at("prior_usage");r.result["prior_usage"]["context_bound_by_worker"]=true;}
        if(result.at("status")!="SUCCESS") {if(result.contains("failure"))r.result["failure"]=result.at("failure");return;}
        r.result["generation_source"]=result.at("prior_usage").at("prior_hit").get<bool>()?
          "STATIC_PRIOR_REUSE":"NATIVE_DIRECT_CONNECTION";
        const double velocity=req.at("velocity_scale"),acceleration=req.at("acceleration_scale");
        if(!std::isfinite(velocity)||!std::isfinite(acceleration)||velocity<=0||velocity>1||acceleration<=0||acceleration>1)
          throw std::runtime_error("PRIOR_TIME_SCALE_INVALID");
        r.result["processing_branch"]="PRIOR_TIME_PARAMETERIZATION";
        auto trajectory=std::make_shared<robot_trajectory::RobotTrajectory>(model,m710::nativeFrames().planning_group);
        auto state=r.start->getCurrentState();
        for(const auto& point:result.at("path")) {
          state.setJointGroupPositions(m710::nativeFrames().planning_group,point.get<std::vector<double>>());state.update();
          trajectory->addSuffixWayPoint(state,0.);
        }
        if(trajectory->getWayPointCount()<2) {r.result["status"]="PRIOR_EMPTY_MOTION";return;}
        trajectory_processing::IterativeParabolicTimeParameterization iptp;
        if(!iptp.computeTimeStamps(*trajectory,velocity,acceleration)) {r.result["status"]="PRIOR_TIME_PARAMETERIZATION_FAILED";return;}
        moveit_msgs::msg::RobotTrajectory msg;trajectory->getRobotTrajectoryMsg(msg);
        if(msg.joint_trajectory.joint_names!=names)throw std::runtime_error("OUTPUT_JOINT_ORDER_MISMATCH");
        J points=J::array();for(const auto& point:msg.joint_trajectory.points)
          points.push_back({{"q",point.positions},{"v",point.velocities},{"a",point.accelerations},
            {"t",point.time_from_start.sec+point.time_from_start.nanosec*1e-9}});
        r.trajectory=trajectory;r.end=r.start->diff();r.end->decoupleParent();r.end->setCurrentState(trajectory->getLastWayPoint());
        r.result["points"]=points;r.result["joint_names"]=names;r.result["returned_waypoint_count"]=trajectory->getWayPointCount();
        r.result["trajectory_present"]=true;r.result["native_output_status"]="SKIPPED_PLANNING_ONLY";
        r.result["planning_only_status"]="PLANNING_ONLY_NOT_EXECUTABLE";r.result["native_output_check_s"]=nullptr;
        r.result["time_parameterization"]="IPTP_preserves_waypoints";r.result["clearance"]=r.clearance->evidence();
        r.result["process_policy"]=r.process?r.process->evidence():J(nullptr);
        r.result["process_semantics_checked"]=false;r.result["planner_internal_process_checks"]=bool(r.process);
        r.result["task_tcp_identity"]=identity.at("task_tcp_fingerprint");r.result["interpolated_link"]=nullptr;
        r.result["processing_branch"]="ACCEPTED_CURRENT_CONTEXT_PRIOR";r.result["prior_query_s"]=seconds(begin);
        r.ready=true;return;
      }
      if(!pipelines.count(pipeline)||!((pipeline=="ompl"&&planner=="RRTConnectkConfigDefault") ||
          (pipeline=="pilz_industrial_motion_planner"&&(planner=="PTP"||planner=="LIN")))) throw std::runtime_error("PLANNER_UNAVAILABLE");
      const std::string stage=req.at("stage");
      if(pipeline=="ompl" && (req.contains("goal_pose") || !(stage=="pregrasp" || stage=="transit" || stage=="residence")))
        throw std::runtime_error("CONSTRAINED_PROCESS_OMPL_FORBIDDEN");
      bool ompl_interpolate=true;
      if(req.contains("ompl_interpolate_output")) {
        if(!req.at("ompl_interpolate_output").is_boolean() || req.at("ompl_interpolate_output")!=false ||
            pipeline!="ompl" || !planning_only || planning_mode!="cold_from_scratch" ||
            identity.value("robot_variant",std::string())!="ECO65-B" ||
            bound_process_geometry.value("cup_selection_mode",std::string())!="SIMULATED_INDEPENDENT_CUPS")
          throw std::runtime_error("INVALID_OMPL_OUTPUT_INTERPOLATION_SCOPE");
        ompl_interpolate=false;
      }
      if(pipeline=="ompl") {
        // Reapply the immutable original map on every request. Only native
        // output interpolation changes; search resolution and dense FCL
        // output/process checks below retain their original policy.
        auto configurations=original_ompl_configurations;
        size_t configured=0;
        for(auto& entry:configurations) if(entry.second.group==m710::nativeFrames().planning_group) {
          if(!ompl_interpolate)entry.second.config["interpolate"]="0";
          ++configured;
        }
        if(configured==0)throw std::runtime_error("OMPL_GROUP_CONFIGURATION_MISSING");
        auto manager=pipelines.at(pipeline)->getPlannerManager();manager->setPlannerConfigurations(configurations);
        if(!ompl_interpolate)for(const auto& entry:manager->getPlannerConfigurations())
          if(entry.second.group==m710::nativeFrames().planning_group && entry.second.config.at("interpolate")!="0")
            throw std::runtime_error("OMPL_OUTPUT_INTERPOLATION_READBACK_MISMATCH");
      }
      const auto event_begin=Clock::now();
      const std::string target_id=req.at("clearance_policy").at("target_id");
      const auto& current=r.start->getCurrentState();
      if(m710::semanticPlaceScope(planning_only,planning_mode,req,
          current.hasAttachedBody(target_id),r.start->getWorld()->hasObject(target_id))) {
        if(req.at("flange_from_task_tcp")!=identity.at("flange_from_task_tcp"))
          throw std::runtime_error("TASK_TCP_CONTEXT_MISMATCH");
        const std::string tcp=identity.at("task_tcp_link");
        if(!model->getLinkModel(tcp)) throw std::runtime_error("TASK_TCP_UNAVAILABLE");
        if(m710::placeGoalSatisfied(r.start,tcp,pose(req.at("goal_pose")))) {
          // The common plan entry checked this exact start's bounds, clearance
          // and ordinary collision. Complete the same endpoint process gate;
          // preserve attachment until the subsequent withdrawal transition.
          if(!r.process || !r.process->check(current,true)) {
            r.result={{"status","NATIVE_SEMANTIC_PROCESS_REJECTED"},
              {"failure",r.process?r.process->last_failure:J("PROCESS_POLICY_REQUIRED")}};
            return;
          }
          std::vector<double> terminal;current.copyJointGroupPositions(m710::nativeFrames().planning_group,terminal);
          r.end=r.start->diff();r.end->decoupleParent();r.end->setCurrentState(current);
          r.result={{"status","SUCCESS"},{"pipeline_id","semantic_event"},{"planner_id","NONE"},
            {"generation_source","SEMANTIC_EVENT"},{"mtc_plan_s",0.},{"native_solver_s",0.},
            {"event_check_s",seconds(event_begin)},{"mtc_generation",true},{"generated_during_task",true},
            {"task_id",req.at("task_id")},{"stage_id",req.at("stage_id")},{"parent_stage_id",r.parent},
            {"request_id",req.at("request_id")},{"native_solver_calls",J::object()},{"pipeline_calls",calls},
            {"points",J::array()},{"terminal_q",terminal},{"joint_names",names},
            {"returned_waypoint_count",0},{"trajectory_present",false},
            {"solver_returned_success",nullptr},{"moveit_error_code",nullptr},
            {"processing_branch","SEMANTIC_PLACE_AT_GOAL"},{"goal_constraints_satisfied",true},
            {"native_output_status","SKIPPED_PLANNING_ONLY"},{"native_output_check_s",nullptr},
            {"planning_only_status","PLANNING_ONLY_NOT_EXECUTABLE"},{"process_semantics_checked",false},
            {"planner_internal_process_checks",true},{"time_parameterization","NOT_APPLICABLE_SEMANTIC_EVENT"},
            {"task_tcp_identity",identity.at("task_tcp_fingerprint")},{"interpolated_link",nullptr},
            {"clearance",r.clearance->evidence()},{"process_policy",r.process->evidence()},
            {"semantic_event",{{"event_type","PLACE_TARGET_REACHED"},{"task_id",req.at("task_id")},
              {"stage_id",req.at("stage_id")},{"parent_stage_id",r.parent},{"target_id",target_id},
              {"start_unchanged",true},{"goal_constraints_satisfied",true},{"release_pending",true}}}};
          r.ready=true;
          return;
        }
      }
      std::shared_ptr<m710::SearchTrace> trace;
      if(req.value("search_trace",false) && pipeline=="ompl") {
        if(pipelines.at(pipeline)->getPlannerPluginName()!="m710/RecordedOMPLPlanner")throw std::runtime_error("SEARCH_TRACE_PLUGIN_NOT_ENABLED");
        trace=std::make_shared<m710::SearchTrace>();trace->input=req;
        trace->input["joint_names"]=names;
        trace->event_limit=req.value("trace_event_limit",size_t(20000));
        trace->sample_limit=req.value("trace_sample_limit",size_t(40000));
        if(trace->event_limit>100000 || trace->sample_limit>250000)throw std::runtime_error("TRACE_RESOURCE_LIMIT");
      }
      m710::active_search_trace=trace;
      m710::active_j1_direction=req.value("j1_direction",0);
      m710::active_native_deadline=cycle_binding?native_deadline:nullptr;
      struct TraceReset {~TraceReset(){m710::active_search_trace.reset();m710::active_j1_direction=0;
        m710::active_native_deadline.reset();}} reset_trace;
      auto solver=std::make_shared<CountedPlanner>(pipelines.at(pipeline),calls[pipeline+"/"+planner]);
      solver->setPlannerId(planner);solver->setProperty("goal_joint_tolerance",1e-12);solver->setProperty("goal_position_tolerance",m710::kNativeGoalPositionToleranceM);
      solver->setProperty("goal_orientation_tolerance",m710::kNativeGoalOrientationToleranceRad);solver->setProperty("max_velocity_scaling_factor",req.at("velocity_scale").get<double>());
      solver->setProperty("max_acceleration_scaling_factor",req.at("acceleration_scale").get<double>());
      const double solve_budget=std::min(req.at("allowed_planning_time_s").get<double>(),remainingBudget()-req.value("return_reserve_s",5.));
      if(solve_budget<=0) {r.result={{"status","NATIVE_REQUEST_BUDGET_EXHAUSTED"}};return;}
      solver->setTimeout(solve_budget);solver->init(model);
      const auto* group=model->getJointModelGroup(m710::nativeFrames().planning_group);robot_trajectory::RobotTrajectoryPtr trajectory;
      r.clearance->phase="search";const auto begin=Clock::now();
      struct RestoreDeadline {
        m710::NativeDeadlinePtr deadline;Clock::time_point request_end;
        ~RestoreDeadline(){if(deadline)deadline->set(request_end);}
      } restore_deadline{cycle_binding?native_deadline:nullptr,request_deadline};
      if(cycle_binding)native_deadline->set(std::min(request_deadline,
        begin+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(solve_budget))));
      mtc::solvers::PlannerInterface::Result result;
      if(req.contains("goal_pose")) {
        if(req.at("flange_from_task_tcp")!=identity.at("flange_from_task_tcp")) throw std::runtime_error("TASK_TCP_CONTEXT_MISMATCH");
        const auto* tcp=model->getLinkModel(identity.at("task_tcp_link").get<std::string>());
        if(!tcp) throw std::runtime_error("TASK_TCP_UNAVAILABLE");
        result=solver->plan(r.start,*tcp,Eigen::Isometry3d::Identity(),matrix(req.at("goal_pose")),group,solve_budget,trajectory,moveit_msgs::msg::Constraints{});
      } else {
        auto goal=r.start->diff();auto state=goal->getCurrentState();state.setJointGroupPositions(group,req.at("q_goal").get<std::vector<double>>());state.update();goal->setCurrentState(state);
        result=solver->plan(r.start,goal,group,solve_budget,trajectory,moveit_msgs::msg::Constraints{});
      }
      const bool solver_deadline_expired=cycle_binding&&native_deadline->expired();
      if(cycle_binding)native_deadline->set(request_deadline);
      r.result={{"status","NATIVE_PLANNING_FAILED"},{"pipeline_id",pipeline},{"planner_id",planner},{"mtc_plan_s",seconds(begin)},
        {"mtc_generation",true},{"generated_during_task",true},{"task_id",req.at("task_id")},{"stage_id",req.at("stage_id")},
        {"parent_stage_id",r.parent},{"native_solver_calls",{{pipeline=="ompl"?"OMPL":planner,1}}},{"pipeline_calls",calls},{"request_id",req.at("request_id")}};
      if(trace) r.result["search_trace"]={{"schema","native_rrt_search_v1"},{"input",trace->input},{"planners",trace->planners},
        {"status",trace->planners.empty()?"NOT_STARTED_NO_TREE":"CAPTURED"},
        {"marker","PLANNING_ONLY_NOT_EXECUTABLE"},{"continuous_swept_proof",false}};
      r.result["solver_returned_success"]=result.success;
      r.result["ompl_output_interpolation"]=pipeline=="ompl"?J(ompl_interpolate):J(nullptr);
      r.result["solver_message"]=result.message;
      if(cycle_binding)r.result["absolute_solver_deadline"]={{"budget_s",solve_budget},
        {"expired_on_return",solver_deadline_expired},
        {"scope","SAME_REQUEST_CLOCK_VALIDITY_PAIR_LOOPS_AND_RRT_EXTENSIONS"},
        {"single_fcl_call_preemption","NOT_AVAILABLE_CHECKED_BEFORE_AND_AFTER"}};
      // MTC exposes the original error name on failure, but drops the numeric
      // response and the successful response message. Never invent that code.
      r.result["moveit_error_code"]=nullptr;
      r.result["moveit_error_code_source"]="MTC_result_message_only";
      if(!result.success) for(int code : {-1,-2,-3,-4,-5,-6,-7,-10,-11,-12,-13,-14,-15,-16,-17,-18,-19,-21,-22,-23,-24,-25,-26,-27,-28,-29,-30,-31,99999}) {
        if(result.message!=moveit::core::error_code_to_string(0) && result.message==moveit::core::error_code_to_string(code)) {
          r.result["moveit_error_code"]=code;
          r.result["moveit_error_code_source"]="recovered_from_original_MoveIt_error_name";
          break;
        }
      }
      r.result["returned_waypoint_count"]=trajectory?trajectory->getWayPointCount():0;
      r.result["trajectory_present"]=bool(trajectory);
      if(solver_deadline_expired) {
        r.result["status"]="NATIVE_SOLVER_DEADLINE_EXPIRED";
        r.result["failure"]={{"reason","NATIVE_ABSOLUTE_DEADLINE_EXPIRED"}};
        r.result["trajectory_present"]=false;trajectory.reset();return;
      }
      r.result["processing_branch"]=!result?"SOLVER_RETURNED_FAILURE":!trajectory?
          "NULL_TRAJECTORY":trajectory->getWayPointCount()<2?"INSUFFICIENT_WAYPOINTS":"TRAJECTORY_CONVERSION";
      if(req.contains("goal_pose")) {
        const auto start_pose=r.start->getCurrentState().getGlobalLinkTransform(identity.at("task_tcp_link").get<std::string>());
        const auto goal_pose=matrix(req.at("goal_pose"));
        r.result["goal_translation_delta_m"]=(start_pose.translation()-goal_pose.translation()).norm();
        r.result["goal_rotation_delta_rad"]=Eigen::AngleAxisd(start_pose.linear().transpose()*goal_pose.linear()).angle();
      }
      if(!result || !trajectory || trajectory->getWayPointCount()==0) return;
      const bool singleton=trajectory->getWayPointCount()==1;
      // Only this experimental place event accepts one native point. The Task
      // still creates its own stage and uses the real predecessor and scene.
      if(singleton && !(planning_only && stage=="place" && pipeline=="pilz_industrial_motion_planner" &&
          planner=="LIN" && req.contains("goal_pose") && !req.at("attachment").is_null())) return;
      try {
      if(pipeline=="ompl") {r.result["processing_branch"]="TIME_PARAMETERIZATION";trajectory_processing::IterativeParabolicTimeParameterization iptp;
        if(!iptp.computeTimeStamps(*trajectory,req.at("velocity_scale").get<double>(),req.at("acceleration_scale").get<double>())) throw std::runtime_error("TIME_PARAMETERIZATION_FAILED");}
      r.result["processing_branch"]="TRAJECTORY_CONVERSION";
      moveit_msgs::msg::RobotTrajectory msg;trajectory->getRobotTrajectoryMsg(msg);
      if(msg.joint_trajectory.joint_names!=names) throw std::runtime_error("OUTPUT_JOINT_ORDER_MISMATCH");
      if(singleton) {
        r.result["processing_branch"]="SINGLETON_GOAL_AND_STATE_CHECK";
        geometry_msgs::msg::PoseStamped target;target.header.frame_id=r.start->getPlanningFrame();
        target.pose=pose(req.at("goal_pose"));
        const auto constraints=kinematic_constraints::constructGoalConstraints(
          identity.at("task_tcp_link").get<std::string>(),target,
          solver->properties().get<double>("goal_position_tolerance"),
          solver->properties().get<double>("goal_orientation_tolerance"));
        kinematic_constraints::KinematicConstraintSet goal(model);
        const bool configured=goal.add(constraints,r.start->getTransforms());
        const bool satisfied=configured && goal.decide(trajectory->getFirstWayPoint()).satisfied;
        const auto rejection=m710::stationaryPointFailure(msg.joint_trajectory,
            req.at("q_start").get<std::vector<double>>(),satisfied);
        if(!rejection.empty()) throw std::runtime_error(rejection);
        // Start clearance was already checked under this exact stage context.
        // Complete the single endpoint's existing process predicates as well.
        if(!r.process || !r.process->check(trajectory->getFirstWayPoint(),true)) {
          r.result["failure"]=r.process?r.process->last_failure:J("PROCESS_POLICY_REQUIRED");
          r.result["status"]="NATIVE_SINGLETON_PROCESS_REJECTED";return;
        }
        r.result["zero_motion_event"]={{"type","NATIVE_STATIONARY_PLACE"},{"goal_constraints_satisfied",true},
          {"native_duration_s",msg.joint_trajectory.points.front().time_from_start.sec+msg.joint_trajectory.points.front().time_from_start.nanosec*1e-9},
          {"start_unchanged",true},{"task_id",req.at("task_id")},{"stage_id",req.at("stage_id")},{"parent_stage_id",r.parent}};
      }
      J points=J::array(),path=J::array();
      for(const auto& p:msg.joint_trajectory.points) {points.push_back({{"q",p.positions},{"v",p.velocities},{"a",p.accelerations},{"t",p.time_from_start.sec+p.time_from_start.nanosec*1e-9}});path.push_back(p.positions);}
      J failure=nullptr;
      if(!planning_only || identity.value("robot_variant",std::string())=="ECO65-B") {
        const auto checking=Clock::now();r.clearance->phase="output";const double resolution=req.at("clearance_policy").at("edge_resolution_rad");
        if(evaluation_profile=="chain_integration_v1") {
          if(!r.process)throw std::runtime_error("PROCESS_POLICY_REQUIRED");
          const auto endpoint=[&](const moveit::core::RobotState& last)->J {
            if(!last.satisfiesBounds())return {{"reason","OUTPUT_ENDPOINT_JOINT_BOUNDS"}};
            if(req.contains("goal_pose")) {
              const auto actual=last.getGlobalLinkTransform(identity.at("task_tcp_link").get<std::string>());
              const auto target=matrix(req.at("goal_pose"));
              const double position=(actual.translation()-target.translation()).norm();
              const double angle=Eigen::AngleAxisd(actual.linear().transpose()*target.linear()).angle();
              r.result["endpoint_fk"]={{"position_error_m",position},{"orientation_error_rad",angle},
                {"position_tolerance_m",m710::kNativeGoalPositionToleranceM},
                {"orientation_tolerance_rad",m710::kNativeGoalOrientationToleranceRad}};
              if(!std::isfinite(position)||!std::isfinite(angle)||position>m710::kNativeGoalPositionToleranceM||angle>m710::kNativeGoalOrientationToleranceRad)
                return {{"reason","OUTPUT_ENDPOINT_TCP_GOAL_MISMATCH"}};
            } else {
              std::vector<double> q;last.copyJointGroupPositions(group,q);
              const auto goal=req.at("q_goal").get<std::vector<double>>();
              for(size_t j=0;j<q.size();++j)if(std::abs(q[j]-goal[j])>1e-9)return {{"reason","OUTPUT_ENDPOINT_JOINT_GOAL_MISMATCH"}};
            }
            return nullptr;
          };
          r.result["basic_path_check"]=m710::basicSampledPath(r.start,*r.clearance,*r.process,path,resolution,
              std::min(req.at("output_check_timeout_s").get<double>(),remainingBudget()-req.value("return_reserve_s",5.)),endpoint,req.value("j1_direction",0),
              cycle_binding?req.at("j1_reverse_used_rad").get<double>():-1.,m710::kEcoCycleReverseLimit);
          failure=r.result["basic_path_check"]["failure"];
          r.result["dense_output_review"]="NOT_RUN_PROFILE";
          r.result["start_state_status"]="PASS";
          r.result["joint_goal_state_status"]=req.contains("q_goal")?"PASS":"NOT_APPLICABLE_POSE_GOAL";
        } else {
          failure=r.clearance->checkPath(r.start->getCurrentState(),path,resolution);
          if(failure.is_null() && r.process) failure=r.process->checkPath(r.start->getCurrentState(),path,resolution);
        }
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
      const bool checked=!planning_only || identity.value("robot_variant",std::string())=="ECO65-B";
      if(planning_only)r.result["planning_only_status"]="PLANNING_ONLY_NOT_EXECUTABLE";
      r.result["status"]="SUCCESS";r.result["native_output_status"]=evaluation_profile=="chain_integration_v1"?"BASIC_SAMPLED_PASS":checked?"PASS":"SKIPPED_PLANNING_ONLY";
      r.result["process_semantics_checked"]=checked && bool(r.process);
      r.result["planner_internal_process_checks"]=bool(r.process);
      r.result["time_parameterization"]=pipeline=="ompl"?"IPTP_preserves_waypoints":"Pilz_original";
      r.result["task_tcp_identity"]=identity.at("task_tcp_fingerprint");r.result["interpolated_link"]=req.contains("goal_pose")?identity.at("task_tcp_link"):J(nullptr);
      r.result["processing_branch"]="ACCEPTED";
      r.result["generation_source"]="NATIVE_GENERATED";
      r.ready=true;
      } catch(const std::exception& e) {
        r.result["status"]="NATIVE_PROCESSING_FAILED";
        r.result["failure"]={{"reason",e.what()},{"branch",r.result.at("processing_branch")}};
        r.ready=false;r.trajectory.reset();r.end.reset();
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
    if(record->ready && session.task->solutions().empty()) {
      record->result["status"]="NATIVE_MTC_CONNECTION_FAILED";record->result["failure"]={{"reason","MTC_FULL_PREFIX_HAS_NO_SOLUTION"}};
      record->ready=false;record->trajectory.reset();record->end.reset();
    }
    record->result["mtc_attempt_index"]=requests;record->result["task_solve_calls"]=session.solve_calls;record->result["task_backtracks"]=session.backtracks;
    record->result["retained_native_subsolutions"]=session.records.size();record->result["task_stage_ids"]=session.active;
    record->result["diagnostic_only"]=diagnostic;
    record->result["evaluation_profile"]=evaluation_profile;
    if(record->result.contains("mtc_plan_s"))record->result["native_solver_s"]=record->result["mtc_plan_s"];
    // Solver failures still need the actual search-state clearance/process
    // evidence. In particular, INVALID_MOTION_PLAN alone hides which inactive
    // cup or neighboring carton rejected the contact trajectory.
    if(!record->result.contains("clearance"))record->result["clearance"]=record->clearance->evidence();
    if(record->process && !record->result.contains("process_policy"))record->result["process_policy"]=record->process->evidence();
    if(diagnostic)record->result["planning_only_status"]="PLANNING_ONLY_NOT_EXECUTABLE";
    compactPlanningOnlyReply(record->result,planning_only,planning_mode);
    return record->result;
  }
  // -- two-level collision equivalence (step 5, development evidence) -----------
  // The same scene, policy and states through EXACT and TWO_LEVEL. Reports
  // every disagreement; a TWO_LEVEL pass where EXACT fails is a counterexample.
  J collisionEquivalence(const J& req,const planning_scene::PlanningScenePtr& scene) {
    if(identity.value("policy_scope",std::string())!="PHYSICAL_SINGLE_CARTON")throw std::runtime_error("EQUIVALENCE_SCOPE_INVALID");
    J exact_req=req;exact_req["collision_check_mode"]="EXACT";J two_req=req;two_req["collision_check_mode"]="TWO_LEVEL";
    auto built=Clock::now();m710::Clearance exact(scene,req.at("clearance_policy"),"optimized",native_deadline,exact_req);const double exact_build=seconds(built);
    built=Clock::now();m710::Clearance two(scene,req.at("clearance_policy"),"optimized",native_deadline,two_req);const double two_build=seconds(built);
    const auto* group=model->getJointModelGroup(m710::nativeFrames().planning_group);
    std::vector<std::vector<double>> states;std::vector<std::string> origin;
    for(const auto& row:req.value("probe_states",J::array())) {states.push_back(row.get<std::vector<double>>());origin.push_back("given");}
    random_numbers::RandomNumberGenerator rng(uint32_t(req.value("random_seed",6518)));
    auto probe=scene->getCurrentState();
    const size_t given=states.size();const double perturb=req.value("perturb_rad",0.);
    for(size_t k=0;perturb>0 && k<given*size_t(req.value("perturbed_per_state",0));++k) {
      auto q=states[k%given];for(auto& x:q)x+=rng.uniformReal(-perturb,perturb);
      probe.setJointGroupPositions(group,q);probe.enforceBounds(group);std::vector<double> b;probe.copyJointGroupPositions(group,b);
      states.push_back(b);origin.push_back("perturbed");
    }
    for(size_t k=0;k<size_t(req.value("random_states",0));++k) {
      probe.setToRandomPositions(group,rng);std::vector<double> q;probe.copyJointGroupPositions(group,q);states.push_back(q);origin.push_back("random");
    }
    J mismatches=J::array(),phases=J::object();size_t counterexamples=0,extra_rejects=0,reason_differences=0,checked=0;bool expired=false;
    for(const auto& phase:req.value("phases",std::vector<std::string>{"output"})) {
      exact.phase=phase;two.phase=phase;std::map<std::string,std::vector<double>> te,tw;size_t valid=0,invalid=0;
      for(size_t i=0;i<states.size();++i) {
        if(native_deadline&&native_deadline->expired()) {expired=true;break;}
        probe.setJointGroupPositions(group,states[i]);probe.update();
        auto a=Clock::now();const bool e=exact.check(probe);te[origin[i]].push_back(seconds(a));const J fe=exact.last_failure;
        a=Clock::now();const bool w=two.check(probe);tw[origin[i]].push_back(seconds(a));const J fw=two.last_failure;
        ++checked;(e?valid:invalid)++;
        auto note=[&](const char* kind){if(mismatches.size()<20)mismatches.push_back({{"kind",kind},{"phase",phase},{"origin",origin[i]},{"q",states[i]},{"exact",fe},{"two_level",fw}});};
        if(w&&!e) {++counterexamples;note("TWO_LEVEL_PASS_EXACT_FAIL");}
        else if(!w&&e) {++extra_rejects;note("TWO_LEVEL_FAIL_EXACT_PASS");}
        else if(!e&&fe!=fw) {++reason_differences;note("FAILURE_DIFFERS");}
      }
      auto summary=[](std::vector<double> v)->J {
        if(v.empty())return nullptr;std::sort(v.begin(),v.end());double sum=0;for(double x:v)sum+=x;
        return {{"count",v.size()},{"median_s",v[v.size()/2]},{"p90_s",v[std::min(v.size()-1,size_t(v.size()*.9))]},{"max_s",v.back()},{"mean_s",sum/v.size()}};
      };
      J times=J::object();
      for(const auto& entry:te) {
        times[entry.first]={{"exact",summary(entry.second)},{"two_level",summary(tw[entry.first])}};
        if(req.value("return_times",false)) {times[entry.first]["exact_raw_s"]=entry.second;times[entry.first]["two_level_raw_s"]=tw[entry.first];}
      }
      phases[phase]={{"valid",valid},{"invalid",invalid},{"timing",times}};
    }
    return {{"status",expired?"NATIVE_SOLVER_BUDGET_EXHAUSTED":"SUCCESS"},{"states",states.size()},{"checks",checked},
            {"counterexamples",counterexamples},{"extra_rejects",extra_rejects},{"failure_differences",reason_differences},
            {"mismatches",mismatches},{"phases",phases},{"build_s",{{"exact",exact_build},{"two_level",two_build}}},
            {"two_level_workspace",two.workspaceEvidence()},{"exact_workspace",exact.workspaceEvidence()},
            {"scope","DEVELOPMENT_EQUIVALENCE_EVIDENCE_NOT_A_PLAN"}};
  }
  // -- resident ECO65 single-carton tasks (step 4) --------------------------------
  // Step 7: controlled/trajectory.py prepare() in the worker (native_prepare.h).
  static J preparedJson(const m710::PreparedTrajectory& p) {
    auto rows=[](const std::vector<double>& flat) {J out=J::array();for(size_t i=0;i+5<flat.size();i+=6)out.push_back(std::vector<double>(flat.begin()+i,flat.begin()+i+6));return out;};
    return {{"period_s",p.period},{"time_scale",p.time_scale},{"source_duration_s",p.source_duration},{"scaled_duration_s",p.scaled_duration},
      {"send_duration_s",p.send_duration},{"terminal_hold_s",p.terminal_hold},{"reach_bound_m",p.reach_bound},
      {"subdivisions_per_send",p.subdivisions},{"analytic_speed_bounds_rad_s",p.speed_bounds},
      {"t",p.t},{"q",rows(p.q)},{"v",rows(p.v)},{"a",rows(p.a)},{"collision_path",rows(p.collision)}};
  }
  static m710::PreparedTrajectory prepareFrom(const J& points,const J& limits,double time_scale,double period,double reach) {
    m710::PrepareInput in;
    for(const auto& p:points) {
      in.t.push_back(p.at("t").get<double>());
      for(const char* key:{"q","v","a"}) {
        auto& target=key[0]=='q'?in.q:key[0]=='v'?in.v:in.a;
        for(const auto& x:p.at(key))target.push_back(x.get<double>());
      }
    }
    m710::PrepareLimits L{limits.at("lower").get<std::vector<double>>(),limits.at("upper").get<std::vector<double>>(),
                          limits.at("velocity").get<std::vector<double>>(),limits.at("acceleration").get<std::vector<double>>()};
    return m710::nativePrepare(in,L,time_scale,period,reach);
  }
  J prepareOp(const J& req) {
    requireSingleCartonContext(req);const auto t=Clock::now();
    try {
      const auto out=prepareFrom(req.at("points"),req.at("limits"),req.at("time_scale").get<double>(),req.at("period_s").get<double>(),req.at("reach_bound_m").get<double>());
      return {{"status","PREPARED"},{"prepared",preparedJson(out)},{"prepare_s",seconds(t)}};
    } catch(const std::runtime_error& e) {return {{"status","PREPARE_REJECTED"},{"error",e.what()},{"prepare_s",seconds(t)}};}
  }
  // Step 7: composite requests (sc_composite.h). Every sub-request goes through
  // dispatch() exactly as if Python had sent it; afterwards the composite's
  // own deadline is registered again, so a cancel always reaches it.
  J composite(const J& req) {
    requireSingleCartonContext(req);
    const auto own=native_deadline;const auto own_end=request_deadline;
    m710::CompositeRunner runner([this](const J& sub){return dispatch(sub);},
      [this,own,own_end]{
        // A cancel that reached the sub-request cancels the whole composite.
        if(native_deadline&&native_deadline!=own&&native_deadline->cancelled()&&own)own->cancel();
        native_deadline=own;request_deadline=own_end;if(on_deadline)on_deadline(own);},own,req);
    return runner.run();
  }
  void requireSingleCartonContext(const J& req) const {
    if(!model || identity.value("policy_scope",std::string())!="PHYSICAL_SINGLE_CARTON")throw std::runtime_error("TASK_RESET_SCOPE_INVALID");
    if(req.at("identity")!=identity)throw std::runtime_error("MODEL_OR_POLICY_MISMATCH");
  }
  // A new task starts as a fresh process would: no world object, scene,
  // counter, cached OMPL context/sampler/simplifier or RNG state of an
  // earlier task remains (AGENTS 2026-11-16 3c).
  J taskReset(const J& req) {
    const auto t=Clock::now();requireSingleCartonContext(req);
    if(req.at("seed")!=seed)throw std::runtime_error("RESIDENT_SEED_MISMATCH");
    base=std::make_shared<planning_scene::PlanningScene>(model);scene_key.clear();scene_content=nullptr;
    scene_handles.clear();scene_version=0;calls.clear();requests=0;ik_calls=0;tasks.clear();
    m710::ClearanceWorkspace::clearPairCache();
    auto p=std::make_shared<planning_pipeline::PlanningPipeline>(model,node,"ompl","planning_plugin","request_adapters");
    if(!p->getPlannerManager())throw std::runtime_error("PIPELINE_UNAVAILABLE:ompl");
    p->displayComputedMotionPlans(false);p->publishReceivedRequests(false);
    pipelines["ompl"]=p;original_ompl_configurations=p->getPlannerManager()->getPlannerConfigurations();
    ompl::RNG::setSeed(g_process_seed);
    return {{"status","RESET"},{"seed",seed},{"ompl_pipeline_rebuilt",true},{"ompl_seed_reapplied",g_process_seed},
            {"reset_s",seconds(t)},{"scope","PHYSICAL_SINGLE_CARTON_TASK_START"}};
  }
  // One upload per observation and worker; later requests carry the handle.
  J sceneUpload(const J& req) {
    const auto t=Clock::now();requireSingleCartonContext(req);
    const std::string fingerprint=req.at("scene_fingerprint");
    auto next=std::make_shared<planning_scene::PlanningScene>(model);std::set<std::string> ids;
    for(const auto& b:req.at("world")) {
      if(!ids.insert(b.at("id").get<std::string>()).second) throw std::runtime_error("DUPLICATE_OBJECT");
      if(!next->processCollisionObjectMsg(object(b))) throw std::runtime_error("SCENE_IMPORT_FAILED");
    }
    const std::string handle="scene-"+std::to_string(++scene_version);
    scene_handles[handle]=SceneHandle{fingerprint,req.at("world"),next};
    base=next;scene_key=fingerprint;scene_content=req.at("world");
    return {{"status","SCENE_READY"},{"scene_handle",handle},{"scene_version",scene_version},
            {"scene_fingerprint",fingerprint},{"objects",ids.size()},{"upload_s",seconds(t)}};
  }
  // The stored world re-enters the request, so every policy reading
  // req["world"] sees exactly the uploaded content.
  J expandSceneHandle(const J& input) {
    requireSingleCartonContext(input);
    if(input.contains("world"))throw std::runtime_error("SCENE_HANDLE_WITH_WORLD");
    const auto found=scene_handles.find(input.at("scene_handle").get<std::string>());
    if(found==scene_handles.end())throw std::runtime_error("SCENE_HANDLE_UNKNOWN");
    if(input.at("scene_fingerprint")!=found->second.fingerprint)throw std::runtime_error("SCENE_HANDLE_FINGERPRINT_MISMATCH");
    J expanded=input;expanded.erase("scene_handle");expanded["world"]=found->second.world;
    if(scene_key!=found->second.fingerprint) {base=found->second.scene;scene_key=found->second.fingerprint;scene_content=found->second.world;}
    return expanded;
  }
public:
  std::function<void(const m710::NativeDeadlinePtr&)> on_deadline;
  // process_geometry_ref: the request's single_carton_contact names the
  // geometry bound at init by the digest given with it; the bound geometry
  // re-enters the request, so every check reads exactly the bound content.
  void expandGeometryRef(J& r) {
    requireSingleCartonContext(r);
    auto& spec=r.at("single_carton_contact");
    if(spec.contains("geometry"))throw std::runtime_error("PROCESS_GEOMETRY_REF_WITH_GEOMETRY");
    if(bound_geometry_digest.empty())throw std::runtime_error("PROCESS_GEOMETRY_REF_NOT_BOUND");
    if(spec.at("geometry_ref")!=bound_geometry_digest)throw std::runtime_error("PROCESS_GEOMETRY_REF_MISMATCH");
    spec.erase("geometry_ref");spec["geometry"]=bound_process_geometry;
  }
  J dispatch(const J& input) {
    J expanded;world_from_handle=false;
    const bool handle=input.is_object() && input.contains("scene_handle") && input.value("op",std::string())!="init";
    const bool geometry_ref=input.is_object() && input.contains("single_carton_contact") &&
      input.at("single_carton_contact").is_object() && input.at("single_carton_contact").contains("geometry_ref");
    if(handle) {expanded=expandSceneHandle(input);world_from_handle=true;}
    else if(geometry_ref)expanded=input;
    if(geometry_ref)expandGeometryRef(expanded);
    const J& req=(handle||geometry_ref)?expanded:input;
    struct HandleFlagReset {bool& flag;~HandleFlagReset(){flag=false;}} handle_flag_reset{world_from_handle};
    const auto begin=Clock::now();const double budget=req.value("request_budget_s",86400.);
    if(!std::isfinite(budget)||budget<=0)throw std::runtime_error("INVALID_REQUEST_BUDGET");
    request_deadline=begin+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(budget));
    native_deadline=std::make_shared<m710::NativeDeadline>(request_deadline);
    if(on_deadline)on_deadline(native_deadline);
    auto result=run(req);result["evaluation_profile"]=evaluation_profile;
    result["budget"]={{"request_budget_s",budget},{"remaining_on_return_s",remainingBudget()},{"return_reserve_s",req.value("return_reserve_s",5.)}};
    if(req.value("search_trace",false) && !result.contains("search_trace")) {
      const std::string pipeline=req.value("pipeline_id",std::string());
      result["search_trace_status"]=pipeline=="ompl"?"NOT_STARTED_NO_TREE":
        pipeline=="external_axis"?"NOT_APPLICABLE_EXTERNAL_AXIS":
        pipeline=="semantic_event"?"NOT_APPLICABLE_SEMANTIC_EVENT":
        req.at("op")=="ik"?"NOT_APPLICABLE_IK":"NOT_APPLICABLE_PILZ";
    }
    result["request_wall_s"]=seconds(begin);return result;
  }
  J run(const J& req) {
    // JSONL dispatch is synchronous, but the resident process can retain four
    // model/TCP contexts. Restore this worker's bound frames on every dispatch.
    if(model) m710::bindNativeFrames(identity);
    auto begin=Clock::now(); auto op=req.at("op").get<std::string>();
    if(m710::NativeTiming::enabled())
      m710::NativeTiming::instance().stage(req.contains("stage")&&req.at("stage").is_string()?req.at("stage").get<std::string>():std::string());
    if(op=="init") {
      if(model) throw std::runtime_error("ALREADY_INITIALIZED");
      m710::bindNativeFrames(req.at("identity"));
      planning_only=req.value("planning_only",false);
      planning_mode=req.value("planning_mode",std::string("cold_from_scratch"));
      evaluation_profile=req.value("evaluation_profile",std::string("strict"));
      directed_j1=req.value("directed_j1_enabled",false);
      if(directed_j1 && (req.at("identity").value("robot_variant",std::string())!="ECO65-B" || !planning_only))
        throw std::runtime_error("DIRECTED_J1_SCOPE_INVALID");
      if(evaluation_profile!="strict" && evaluation_profile!="chain_integration_v1")throw std::runtime_error("UNKNOWN_EVALUATION_PROFILE");
      if(evaluation_profile=="chain_integration_v1" && (!planning_only || planning_mode!="cold_from_scratch" ||
          req.at("identity").value("robot_variant",std::string())!="ECO65-B" ||
          req.at("process_geometry").value("geometry_type",std::string())!="cad_seal_contours_v1"))
        throw std::runtime_error("CHAIN_INTEGRATION_REQUIRES_ECO65_COLD_PLANNING_ONLY");
      prior_placement_policy_sha256=req.value("placement_policy_sha256",std::string());
      if(planning_mode!="cold_from_scratch" && planning_mode!="static_prior_fast")
        throw std::runtime_error("UNKNOWN_PLANNING_MODE");
      if(planning_mode=="static_prior_fast" && !planning_only)
        throw std::runtime_error("STATIC_PRIOR_REQUIRES_PLANNING_ONLY");
      seed=req.at("seed");
      if(!g_ompl_seeded) {ompl::RNG::setSeed(seed);g_process_seed=seed;g_ompl_seeded=true;}
      else if(seed!=g_process_seed) throw std::runtime_error("RESIDENT_SEED_MISMATCH");
      const bool physical_single=req.at("identity").value("policy_scope",std::string())=="PHYSICAL_SINGLE_CARTON";
      if(physical_single && (planning_only || req.at("identity").value("robot_variant",std::string())!="ECO65-B"))
        throw std::runtime_error("PHYSICAL_SINGLE_CARTON_SCOPE_INVALID");
      std::vector<rclcpp::Parameter> params;
      for(auto it=req.at("parameters").begin(); it!=req.at("parameters").end(); ++it) {
        if(physical_single && it.key()=="ompl.planning_plugin") {
          // Same seeded RRTConnect, with the existing request deadline checked
          // inside long CONNECT loops. No trace, J1 constraint or prior path.
          params.emplace_back(it.key(),std::string("m710/RecordedOMPLPlanner"));continue;
        }
        if((req.value("search_trace_enabled",false)||directed_j1||
            !req.value("continuous_task_mode",std::string()).empty()) && it.key()=="ompl.planning_plugin") {
          if(!planning_only || planning_mode!="cold_from_scratch")throw std::runtime_error("SEARCH_TRACE_SCOPE_INVALID");
          params.emplace_back(it.key(),std::string("m710/RecordedOMPLPlanner"));continue;
        }
        const auto& v=it.value();
        if(v.is_string()) params.emplace_back(it.key(),v.get<std::string>());
        else if(v.is_boolean()) params.emplace_back(it.key(),v.get<bool>());
        else if(v.is_number_integer()) params.emplace_back(it.key(),v.get<int64_t>());
        else if(v.is_number()) params.emplace_back(it.key(),v.get<double>());
        else if(v.is_array()) params.emplace_back(it.key(),v.get<std::vector<std::string>>());
        else throw std::runtime_error("INVALID_PARAMETER_TYPE");
      }
      auto init_model=std::make_unique<m710::NativeSpan>("init.node_and_model_load");
      node=std::make_shared<rclcpp::Node>("m710_moveit_worker_"+req.at("identity").at("model_tool_fingerprint").get<std::string>().substr(0,12), rclcpp::NodeOptions().parameter_overrides(params).automatically_declare_parameters_from_overrides(true));
      loader=std::make_shared<robot_model_loader::RobotModelLoader>(node,"robot_description");
      model=loader->getModel(); if(!model) throw std::runtime_error("MODEL_UNAVAILABLE");
      const auto* group=model->getJointModelGroup(m710::nativeFrames().planning_group);
      if(!group) throw std::runtime_error("GROUP_UNAVAILABLE");
      for(const auto& link:{m710::nativeFrames().base_link,m710::nativeFrames().flange_link,
                            req.at("identity").at("task_tcp_link").get<std::string>()})
        if(!model->getLinkModel(link)) throw std::runtime_error("ROBOT_FRAME_UNAVAILABLE:"+link);
      names=group->getVariableNames(); if(names!=req.at("joint_names").get<std::vector<std::string>>()) throw std::runtime_error("JOINT_ORDER_MISMATCH");
      init_model.reset();auto init_pipelines=std::make_unique<m710::NativeSpan>("init.planning_pipelines");
      for(const auto& name:{"pilz_industrial_motion_planner","ompl"}) {
        auto p=std::make_shared<planning_pipeline::PlanningPipeline>(model,node,name,"planning_plugin","request_adapters");
        if(!p->getPlannerManager()) throw std::runtime_error(std::string("PIPELINE_UNAVAILABLE:")+name);
        p->displayComputedMotionPlans(false); p->publishReceivedRequests(false);
        pipelines[name]=p;
      }
      original_ompl_configurations=pipelines.at("ompl")->getPlannerManager()->getPlannerConfigurations();
      init_pipelines.reset();m710::NativeSpan init_policy("init.geometry_and_policy_validation");
      for(auto it=req.at("expected_collision_shapes").begin();it!=req.at("expected_collision_shapes").end();++it) {
        const auto* link=model->getLinkModel(it.key());
        if(!link || link->getShapes().size()!=it.value().get<size_t>()) throw std::runtime_error("COLLISION_GEOMETRY_MISSING:"+it.key());
        for(const auto& shape:link->getShapes()) {
          if(shape->type==shapes::MESH && static_cast<const shapes::Mesh*>(shape.get())->triangle_count==0) throw std::runtime_error("EMPTY_COLLISION_MESH");
        }
      }
      bound_policy=req.at("collision_policy");bound_tools=req.at("tool_links");
      bound_compliant=req.value("compliant_tool_links",J::array());bound_process_geometry=req.value("process_geometry",J(nullptr));
      bound_geometry_digest=req.value("process_geometry_digest",std::string());
      if(!bound_geometry_digest.empty() && (bound_process_geometry.is_null() || bound_geometry_digest.size()!=64))
        throw std::runtime_error("PROCESS_GEOMETRY_DIGEST_INVALID");
      if(req.at("identity").value("robot_variant",std::string())=="ECO65-B")
        m710::validateEco65MastExemption(bound_policy,model,bound_process_geometry,bound_tools);
      if(bound_policy.contains("extraction_carton_contact"))
        m710::validateEco65ExtractionContact(bound_policy,bound_process_geometry,
          req.at("coupled_cycle_binding").at("original_cartons"));
      m710::validateEco65CupNeighbors(bound_policy,bound_process_geometry,bound_compliant);
      m710::validateEco65RobotToolExemption(bound_policy,model,bound_process_geometry,bound_tools);
      m710::validateEco65RigidCartonPolicy(bound_policy,model,bound_process_geometry,bound_tools,bound_compliant);
      const auto continuous_mode=req.value("continuous_task_mode",std::string());
      if(!continuous_mode.empty()) {
        const bool free_joints=req.at("coupled_cycle_binding").contains("transverse_process_policy");
        if(continuous_mode!=m710::kEcoCycleMode || directed_j1==free_joints || !planning_only ||
            evaluation_profile!="chain_integration_v1" || planning_mode!="cold_from_scratch" ||
            req.at("identity").value("robot_variant",std::string())!="ECO65-B")
          throw std::runtime_error("ECO65_CONTINUOUS_MODE_SCOPE_INVALID");
        cycle_binding=std::make_unique<m710::EcoCycleBinding>(req.at("coupled_cycle_binding"),bound_process_geometry);
      }
      if(!bound_process_geometry.is_null() && bound_process_geometry.value("geometry_type",std::string())=="known_tip_finite_datum_v1") {
        if(!planning_only || planning_mode!="cold_from_scratch" || req.at("identity").at("robot_variant")!="ECO65-B" ||
           req.at("identity").at("task_tcp_link")!="calibration_probe_tcp" || bound_compliant!=J::array({"probe_tip"}))
          throw std::runtime_error("TOUCH_REQUIRES_BOUND_PROBE_TCP_COLD_SIMULATION");
      }
      if(!bound_process_geometry.is_null() && bound_process_geometry.value("geometry_type",std::string())=="cad_seal_contours_v1") {
        if(!planning_only || planning_mode!="cold_from_scratch" || req.at("identity").at("robot_variant")!="ECO65-B")
          throw std::runtime_error("ECO65_REQUIRES_COLD_PLANNING_ONLY");
        m710::validateSealContours(bound_process_geometry);
        m710::processTransform(bound_process_geometry.at("flange_from_contact_frame"));
      }
      base=std::make_shared<planning_scene::PlanningScene>(model);
      identity=req.at("identity");
      // Physical single carton (2-second planning, 2026-11-23): the process-level
      // robot/tool convex solids that every TWO_LEVEL workspace of this process
      // reuses (ClearanceWorkspace::cachedConvex, same key and same solids) are
      // built here instead of by this worker's first request, where they cost
      // 0.2-0.3 s on the critical path of the first task after a start.
      J convex_prewarm=nullptr;
      if(identity.value("policy_scope",std::string())=="PHYSICAL_SINGLE_CARTON") {
        const auto warming=Clock::now();size_t solids=0;
        for(const auto* link:model->getLinkModelsWithCollisionGeometry()) {
          const auto& shapes=link->getShapes();
          for(size_t i=0;i<shapes.size();++i)if(shapes[i] && shapes[i]->type==shapes::MESH) {
            m710::ClearanceWorkspace::robotConvex(link,i,static_cast<const shapes::Mesh*>(shapes[i].get()));++solids;
          }
        }
        convex_prewarm={{"solids",solids},{"seconds",seconds(warming)},{"scope","PROCESS_ROBOT_CONVEX_CACHE_SAME_KEY_AND_SOLIDS_AS_FIRST_TWO_LEVEL_REQUEST"}};
      }
      return {{"status","READY"},{"identity",identity},{"joint_names",names},{"cold_start_s",seconds(begin)},{"planning_only",planning_only},{"planning_mode",planning_mode},{"robot_convex_prewarm",convex_prewarm},
              {"seed",seed},{"versions",{{"moveit",M710_MOVEIT_VERSION},{"mtc",M710_MTC_VERSION},{"pilz",M710_PILZ_VERSION}}},{"pipelines",{"pilz_industrial_motion_planner","ompl"}},
              {"capabilities",{{"observed_side_envelope_v1",physical_single},{"inclined_carton_support_v1",physical_single},{"adjacent_contact_plane_v1",physical_single},{"neighbor_lip_contact_v1",physical_single},{"neighbor_lip_contact_v2",physical_single},{"single_carton_posture_preference",physical_single},{"partial_observed_contact_envelope",physical_single},{"bounded_single_carton_planning",physical_single},
                {"single_carton_output_check_deferral",physical_single},{"single_carton_tool_robot_clearance",physical_single},{"absolute_planning_deadline",physical_single},{"resident_task_reset",physical_single},{"scene_handles",physical_single},{"request_cancel",physical_single},{"path_endpoint_reuse",physical_single},{"two_level_collision",physical_single},{"adaptive_sent_path_audit",physical_single},{"clearance_predicate_solution_check",physical_single},{"pilz_solution_check_deferral",physical_single},{"workspace_pair_cache",physical_single},{"adaptive_per_body_constraints",physical_single},{"adaptive_rule_link_transforms",physical_single},{"adaptive_contact_sent_path_audit",physical_single},{"sc_ik_batch",physical_single},{"process_geometry_ref",physical_single},{"lin_retry_ladder",physical_single},{"native_prepare",physical_single},{"binary_protocol",physical_single?2:0},{"composite_requests",physical_single},{"attempt_journal",physical_single},{"native_ik",true},{"native_task_session",true},{"semantic_place_event",planning_only&&planning_mode=="static_prior_fast"},
                {"native_ik_batch",planning_only&&planning_mode=="static_prior_fast"},
                {"static_prior_graph",planning_only&&planning_mode=="static_prior_fast"},{"process_policy_schema","m710_native_process_v1"},
                {"process_geometry_bound",!bound_process_geometry.is_null()&&!bound_compliant.empty()},
                {"max_task_sessions",8},{"max_task_stages_per_session",4096}}}};
    }
    if(!model) throw std::runtime_error("NOT_INITIALIZED");
    if(op=="sc_prepare")return prepareOp(req);
    if(op=="sc_endpoint_screen"||op=="sc_plan_candidate")return composite(req);
    if(op=="sc_task_reset")return taskReset(req);
    if(op=="sc_scene_upload")return sceneUpload(req);
    if(op=="sc_scene_release") {
      requireSingleCartonContext(req);const size_t removed=scene_handles.erase(req.at("scene_handle").get<std::string>());
      return {{"status","RELEASED"},{"removed",removed}};
    }
    if(cycle_binding && req.value("continuous_task_mode",std::string())!=m710::kEcoCycleMode)
      throw std::runtime_error("ECO65_CONTINUOUS_MODE_REQUIRED");
    if(req.value("evaluation_profile",std::string("strict"))!=evaluation_profile)
      throw std::runtime_error("EVALUATION_PROFILE_MISMATCH");
    if((op=="plan" || op=="diagnostic_connection" || op=="ik" || op=="ik_batch" || op=="sc_ik_batch" || op=="eco65_transition") && req.value("planning_only",false)!=planning_only)
      throw std::runtime_error("PLANNING_ONLY_MODE_MISMATCH");
    if((op=="plan" || op=="diagnostic_connection" || op=="ik" || op=="ik_batch" || op=="sc_ik_batch" || op=="load_static_prior" || op=="build_static_prior" || op.rfind("prior_",0)==0) &&
        req.value("planning_mode",std::string("cold_from_scratch"))!=planning_mode)
      throw std::runtime_error("PLANNING_MODE_MISMATCH");
    if(planning_only && (op=="task_audit" || op=="compose" || op=="validate"))
      throw std::runtime_error("PLANNING_ONLY_NOT_EXECUTABLE");
    // Offline state checking has no execution/export authority. The formal
    // validate/compose/task_audit gates above stay closed for planning-only.
    if(op=="check_state_offline" || op=="check_path_offline") {
      if(!planning_only || identity.value("robot_variant",std::string())!="ECO65-B")
        throw std::runtime_error("OFFLINE_CHECK_SCOPE_INVALID");
      if(op=="check_path_offline" && evaluation_profile!="chain_integration_v1")
        throw std::runtime_error("BASIC_PATH_DIAGNOSTIC_REQUIRES_CHAIN_PROFILE");
      // Continue through the same bound model and process predicates below.
    }
    if(planning_only && (op=="plan" || op=="diagnostic_connection") && !req.value("require_native_motion",false))
      throw std::runtime_error("PLANNING_ONLY_REQUIRES_NATIVE_TASK");
    if(op!="fk" && op!="ik" && op!="ik_batch" && op!="sc_ik_batch" && op!="load_static_prior" && op!="build_static_prior" &&
      op!="task_audit" && op!="plan" && op!="diagnostic_connection" && op!="compose" && op!="inspect" && op!="validate" && op!="check_state_offline" && op!="check_path_offline" && op!="retire_touch_task" && op!="eco65_transition" && op!="sc_collision_equivalence") throw std::runtime_error("UNKNOWN_OPERATION");
    if(req.at("identity")!=identity) throw std::runtime_error("MODEL_OR_POLICY_MISMATCH");
    if(op=="eco65_transition" && !cycle_binding)throw std::runtime_error("ECO65_TRANSITION_MODE_REQUIRED");
    if(op=="retire_touch_task") {
      if(!planning_only || bound_process_geometry.value("geometry_type",std::string())!="known_tip_finite_datum_v1")throw std::runtime_error("TOUCH_TASK_RETIRE_SCOPE");
      const std::string task=req.at("task_id");const size_t count=tasks.erase(task);
      return {{"status","RETIRED"},{"removed",count},{"task_id",task},{"scope","Release cache before replan from measured state; does not authorize execution"}};
    }
    if(op=="load_static_prior" || op=="build_static_prior" || op=="ik_batch") {
      if(!planning_only || planning_mode!="static_prior_fast") throw std::runtime_error("STATIC_PRIOR_MODE_REQUIRED");
    }
    if(op=="sc_ik_batch") {
      // Physical single carton (2026-11-22): one request carrying the seeds of
      // one pose. Each seed runs the unchanged "ik" request (same seed field,
      // tolerances and budget fields, q_start = that seed); every reply is
      // returned in seed order, with no early stop, exactly as the caller would
      // have received them one by one.
      if(identity.value("policy_scope",std::string())!="PHYSICAL_SINGLE_CARTON") throw std::runtime_error("SC_IK_BATCH_SCOPE_INVALID");
      const auto& seeds=req.at("ik_seeds");
      if(!seeds.is_array() || seeds.empty() || seeds.size()>256) throw std::runtime_error("SC_IK_BATCH_LIMIT");
      J results=J::array();
      for(const auto& seed:seeds) {
        if(native_deadline && native_deadline->expired()) return {{"status","NATIVE_ABSOLUTE_DEADLINE_EXPIRED"},{"results",results}};
        auto one=req;one["op"]="ik";one["q_start"]=seed;one.erase("ik_seeds");
        results.push_back(run(one));
      }
      return {{"status","SUCCESS"},{"results",results},{"native_ik_calls",results.size()},{"batch_s",seconds(begin)}};
    }
    if(op=="load_static_prior") {
      const auto& database=req.at("database");
      if(database.at("schema")!="m710_static_prior_v1" || database.at("status")!="PLANNING_ONLY_NOT_EXECUTABLE" ||
          database.at("qualification_status")!="NOT_EVALUATED") throw std::runtime_error("PRIOR_DATABASE_MARKER_INVALID");
      if(database.at("modes").empty() || database.at("modes").size()>2) throw std::runtime_error("PRIOR_DATABASE_MODES_INVALID");
      for(auto it=database.at("modes").begin();it!=database.at("modes").end();++it) {
        if(it.key()!="empty" && it.key()!="loaded")throw std::runtime_error("PRIOR_DATABASE_MODE_INVALID");
        m710::StaticPriorGraph::validate(it.value());
        if(it.value().at("context").at("identity")!=identity ||
            it.value().at("context").at("attachment").at("mode")!=it.key())
          throw std::runtime_error("PRIOR_DATABASE_IDENTITY_MISMATCH");
      }
      static_prior_database=database;
      return {{"status","SUCCESS"},{"loaded_modes",database.at("modes").size()},{"load_s",seconds(begin)},
        {"planning_only_status","PLANNING_ONLY_NOT_EXECUTABLE"}};
    }
    if(op=="ik_batch") {
      const auto& seeds=req.at("ik_seeds");
      if(!seeds.is_array() || seeds.empty() || seeds.size()>8) throw std::runtime_error("NATIVE_IK_BATCH_LIMIT");
      const double budget=req.at("allowed_planning_time_s");
      if(!std::isfinite(budget)||budget<=0) return {{"status","NATIVE_IK_BATCH_TIMEOUT"},{"results",J::array()},{"consume_count",0},{"native_ik_calls",0},{"ik_s",0.}};
      J results=J::array();double solver_s=0.;bool timed_out=false;
      for(size_t i=0;i<seeds.size();++i) {
        if(req.value("cancelled",false)) return {{"status","CANCELLED"},{"results",results},{"consume_count",results.size()},{"native_ik_calls",results.size()},{"ik_s",solver_s}};
        if(seconds(begin)>=budget) {timed_out=true;break;}
        auto one=req;one["op"]="ik";one["q_start"]=seeds.at(i);one.erase("ik_seeds");
        one["seed"]=req.at("seed").get<uint32_t>()+i;
        one["allowed_planning_time_s"]=budget-seconds(begin);
        const auto before=Clock::now();auto result=run(one);result["seed_index"]=i;
        solver_s+=result.value("ik_s",seconds(before));results.push_back(result);
        if(result.at("status")=="SUCCESS" && req.value("stop_on_first_success",true))break;
      }
      return {{"status",timed_out?"NATIVE_IK_BATCH_TIMEOUT":"SUCCESS"},{"results",results},
        {"consume_count",results.size()},{"native_ik_calls",results.size()},{"ik_s",solver_s},{"batch_s",seconds(begin)}};
    }
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
        if(r.parent!=previous || !r.ready || r.result.at("status")!="SUCCESS") throw std::runtime_error("NATIVE_TASK_SOURCE_CHAIN_INVALID");
        const std::string target=r.request.at("clearance_policy").at("target_id");const bool attached=r.start->getCurrentState().hasAttachedBody(target);
        if(attached&&!was_attached) ++attach;if(!attached&&was_attached) ++release;was_attached=attached;
        phase_names.insert(r.request.at("stage").get<std::string>());
        size_t motion_edges=0;for(size_t i=1;r.trajectory && i<r.trajectory->getWayPointCount();++i)
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
        last.end->getCurrentState().copyJointGroupPositions(m710::nativeFrames().planning_group,last_q);
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
    state.setJointGroupPositions(m710::nativeFrames().planning_group,q); state.setVariableVelocities(std::vector<double>(model->getVariableCount(),0)); state.update();
    if(!state.satisfiesBounds()) throw std::runtime_error("INVALID_START_BOUNDS");
    if(op=="fk") {
      J frames=J::object(); for(auto& l:req.at("links")) frames[l.get<std::string>()]=serial(state.getGlobalLinkTransform(l.get<std::string>()));
      return {{"status","SUCCESS"},{"frames",frames}};
    }
    if(req.value("cancelled",false)) throw std::runtime_error("CANCELLED");
    if(op=="ik") {
      m710::NativeSpan ik_span("ik");
      if(req.contains("history") || req.contains("path") || req.contains("q_goal")) throw std::runtime_error("NATIVE_IK_FORBIDDEN_INPUT");
      const auto* group=model->getJointModelGroup(m710::nativeFrames().planning_group);const auto& solver=group->getSolverInstance();
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
    {m710::NativeSpan span("world.compare_same_fingerprint");
    if(!changed && !world_from_handle && req.at("world")!=scene_content) throw std::runtime_error("SCENE_FINGERPRINT_REUSED_WITH_DIFFERENT_CONTENT");}
    if(changed) {
      m710::NativeSpan span("world.import_new_scene");
      auto next=std::make_shared<planning_scene::PlanningScene>(model); std::set<std::string> ids;
      for(const auto& b:req.at("world")) {
        if(!ids.insert(b.at("id").get<std::string>()).second) throw std::runtime_error("DUPLICATE_OBJECT");
        if(!next->processCollisionObjectMsg(object(b))) throw std::runtime_error("SCENE_IMPORT_FAILED");
      }
      base=next;scene_key=key;scene_content=req.at("world");
    }
    auto scene_setup=std::make_unique<m710::NativeSpan>("scene.diff_acm_attachment");
    auto scene=base->diff(); scene->decoupleParent(); scene->setCurrentState(state);
    auto& acm=scene->getAllowedCollisionMatrixNonConst();
    for(const auto& pair:req.at("allowed_pairs")) acm.setEntry(pair.at(0).get<std::string>(),pair.at(1).get<std::string>(),true);
    std::string attached_id;
    if(!req.at("attachment").is_null()) {
      auto b=req.at("attachment"); attached_id=b.at("id");
      // A single object identity moves from world to attached; never duplicates.
      scene->getWorldNonConst()->removeObject(attached_id);
      moveit_msgs::msg::AttachedCollisionObject a; a.link_name=m710::nativeFrames().flange_link; a.object=object(b,m710::nativeFrames().flange_link);
      a.touch_links=b.at("touch_links").get<std::vector<std::string>>();
      if(!scene->processAttachedCollisionObjectMsg(a)) throw std::runtime_error("ATTACH_FAILED");
      if(scene->getWorld()->hasObject(attached_id)) throw std::runtime_error("DUPLICATE_ATTACHMENT");
    }
    scene_setup.reset();
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
    if(op=="plan" || op=="diagnostic_connection" || op=="validate" || op=="check_state_offline" || op=="check_path_offline" || op=="build_static_prior" || op=="eco65_transition" || op=="sc_collision_equivalence") {
      if(!req.contains("clearance_policy")) throw std::runtime_error("CLEARANCE_POLICY_REQUIRED");
      if(req.at("clearance_policy").at("schema")=="eco65_native_single_carton_clearance_v1" &&
          (req.at("single_carton_contact").at("geometry")!=bound_process_geometry ||
           identity.value("policy_scope",std::string())!="PHYSICAL_SINGLE_CARTON"))
        throw std::runtime_error("SINGLE_CARTON_CAD_TOPOLOGY_BINDING_MISMATCH");
      if(req.at("clearance_policy").at("source_policy")!=bound_policy || req.at("clearance_policy").at("tool_links")!=bound_tools)
        throw std::runtime_error("EXECUTABLE_POLICY_MISMATCH");
      if(req.contains("process_policy") && op!="build_static_prior") {
        if(bound_compliant.empty() || req.at("clearance_policy").at("compliant_tool_links")!=bound_compliant)
          throw std::runtime_error("PROCESS_COLLIDER_OWNERSHIP_MISMATCH");
        if(bound_process_geometry.is_null()) throw std::runtime_error("PROCESS_CUP_GEOMETRY_BINDING_REQUIRED");
        if(bound_process_geometry.value("geometry_type",std::string())=="cad_seal_contours_v1" ||
           bound_process_geometry.value("geometry_type",std::string())=="known_tip_finite_datum_v1") {
          if(cycle_binding)checkCycleGeometry(req);
          for(auto it=bound_process_geometry.begin();it!=bound_process_geometry.end();++it)
            if(!(cycle_binding&&it.key()=="task_contract") &&
                (!req.at("process_policy").contains(it.key()) || req.at("process_policy").at(it.key())!=it.value()))
              throw std::runtime_error("PROCESS_CONTOUR_GEOMETRY_BINDING_MISMATCH");
        } else if(req.at("process_policy").at("cups")!=bound_process_geometry.at("cups") ||
                  req.at("process_policy").at("flange_from_physical_contact")!=bound_process_geometry.at("flange_from_physical_contact"))
            throw std::runtime_error("PROCESS_CUP_GEOMETRY_BINDING_MISMATCH");
        process=std::make_shared<m710::ProcessPolicy>(scene,req,cycle_binding?native_deadline:nullptr);
      }
      if((req.at("clearance_policy").at("schema")=="m710_native_process_clearance_v1" ||
          req.at("clearance_policy").at("schema")=="eco65_native_process_clearance_v1") && !process && op!="build_static_prior")
        throw std::runtime_error("PROCESS_POLICY_REQUIRED");
      if(req.value("require_native_motion",false) && (op=="plan" || op=="diagnostic_connection") && !process) throw std::runtime_error("NATIVE_COLD_PROCESS_CONTEXT_REQUIRED");
      // sent_path_audit ADAPTIVE only for the final audit of a sent path (validate with probe_path).
      if(req.value("sent_path_audit",std::string("SAMPLED"))!="SAMPLED" && !(op=="validate" && req.contains("probe_path") && !req.contains("probe_states")))
        throw std::runtime_error("SENT_PATH_AUDIT_SCOPE_INVALID");
      {m710::NativeSpan span("clearance.construct_and_workspace");
      clearance=std::make_shared<m710::Clearance>(scene,req.at("clearance_policy"),req.value("clearance_mode",std::string("optimized")),
        (cycle_binding || identity.value("policy_scope",std::string())=="PHYSICAL_SINGLE_CARTON")?native_deadline:nullptr,req);}
      // Environment + ACM belong to this immutable candidate. The predicate
      // always checks OMPL's supplied state, including its real attached body.
      scene->setStateFeasibilityPredicate([clearance,process](const moveit::core::RobotState& current,bool verbose){
        bool valid=clearance->check(current,verbose);J failure=valid?J(nullptr):clearance->last_failure;
        const char* checker="Clearance";
        if(valid && process) {++process->planner_checks;valid=process->check(current);checker="ProcessPolicy";
          if(!valid)failure=process->last_failure;}
        if(m710::extension_recorder) {
          if(!failure.is_null())failure["phase"]=clearance->phase;
          std::vector<double> q;current.copyJointGroupPositions(m710::nativeFrames().planning_group,q);
          (*m710::extension_recorder)(q,valid,checker,failure);
        }
        return valid;});
    }
    if(op=="build_static_prior") {
      checkPriorContext(req);
      for(const auto& body:req.at("world")) {
        const auto category=body.value("category",std::string());
        if(category=="carton" || category=="payload") throw std::runtime_error("PRIOR_BUILD_REQUIRES_STATIC_WORLD");
      }
      auto result=m710::StaticPriorGraph::build(req,scene,*clearance);result["scene_import_s"]=import_s;return result;
    }
    if(op=="sc_collision_equivalence")return collisionEquivalence(req,scene);
    auto path_check=[&](const J& path) {return clearance->checkPath(scene->getCurrentState(),path,req.at("clearance_policy").at("edge_resolution_rad"));};
    if(op=="check_path_offline") {
      if(!process)throw std::runtime_error("PROCESS_POLICY_REQUIRED");
      const auto report=m710::basicSampledPath(scene,*clearance,*process,req.at("probe_path"),
        req.at("clearance_policy").at("edge_resolution_rad"),req.at("output_check_timeout_s"),
        [](const moveit::core::RobotState&)->J{return nullptr;},
        cycle_binding?m710::ecoCycleDirection(req):0,
        cycle_binding?req.at("j1_reverse_used_rad").get<double>():-1.,m710::kEcoCycleReverseLimit);
      return {{"status",report.at("status")},{"basic_path_check",report},
        {"clearance",clearance->evidence()},{"process_policy",process->evidence()},
        {"planning_only_status","PLANNING_ONLY_NOT_EXECUTABLE"},{"execution_authorized",false},
        {"generation_source","OFFLINE_DIAGNOSTIC_NOT_A_PLAN"}};
    }
    if(op=="check_state_offline") {
      const auto phase=req.value("check_phase",std::string("initial"));
      if(phase!="initial" && phase!="search")throw std::runtime_error("INVALID_CHECK_PHASE");
      clearance->phase=phase;
      const bool clear=clearance->check(scene->getCurrentState());
      const bool valid=process && process->check(scene->getCurrentState(),req.value("endpoint",false));
      return {{"status",clear&&valid?"SUCCESS":"STATE_REJECTED"},{"native_valid",clear},{"process_valid",valid},
        {"clearance",clearance->evidence()},{"process",process?process->evidence():J(nullptr)},
        {"planning_only_status","PLANNING_ONLY_NOT_EXECUTABLE"},{"execution_authorized",false}};
    }
    if(op=="validate") {
      if(req.contains("probe_states")) {
        J results=J::array();auto probe=scene->getCurrentState();clearance->phase="fixed_set";
        const auto t=Clock::now();double legacy_s=0;size_t legacy_queries=0;
        for(const auto& row:req.at("probe_states")) {
          auto values=row.get<std::vector<double>>();
          if(values.size()!=names.size()) throw std::runtime_error("INVALID_PROBE_STATE");
          for(double x:values) if(!std::isfinite(x)) throw std::runtime_error("INVALID_PROBE_STATE");
          probe.setJointGroupPositions(m710::nativeFrames().planning_group,values);probe.update();
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
      collision_detection::CollisionResult old_result;
      {m710::NativeSpan span("validate.legacy_collision");scene->checkCollision(old_request,old_result);}
      clearance->phase="diagnostic";
      auto endpoint_span=std::make_unique<m710::NativeSpan>("validate.endpoint_state");
      const bool clearance_valid=clearance->check(scene->getCurrentState());const J failure=clearance->last_failure;
      const bool process_valid=!process || process->check(scene->getCurrentState(),true);
      endpoint_span.reset();
      const J process_failure=process?process->last_failure:J(nullptr);
      J path_failure=nullptr,process_path_failure=nullptr;if(req.contains("probe_path")) {clearance->phase="output";
        {m710::NativeSpan span("validate.check_path");path_failure=path_check(req.at("probe_path"));}
        if(process) process_path_failure=process->checkPath(scene->getCurrentState(),req.at("probe_path"),req.at("clearance_policy").at("edge_resolution_rad"));}
      m710::NativeSpan evidence_span("validate.reply_evidence");
      return {{"status","SUCCESS"},{"legacy_intersection_valid",!old_result.collision},{"native_valid",clearance_valid&&process_valid},
        {"failure",failure},{"path_failure",path_failure},{"clearance",clearance->evidence()},
        {"process_valid",process_valid},{"process_failure",process_failure},{"process_path_failure",process_path_failure},{"process_policy_evidence",process?process->evidence():J(nullptr)},
        {"world_count",scene->getWorld()->size()},{"attached_id",attached_id},{"pipeline_calls",calls}};
    }
    if(op=="plan" || op=="diagnostic_connection") {
      m710::NativeSpan span("plan.start_goal_checks");
      const std::string phase=req.at("stage");
      const int expected=cycle_binding?m710::ecoCycleDirection(req):
        directed_j1?(phase=="contact"||phase=="extraction"||phase=="transit"||phase=="place"?-1:phase=="withdrawal"||phase=="residence"?1:0):0;
      if(req.value("j1_direction",0)!=expected)throw std::runtime_error("J1_DIRECTION_POLICY_MISMATCH");
      // Validate starts and explicit goals under the SAME reserve used by OMPL.
      if(req.value("pipeline_id",std::string())=="ompl")clearance->phase="search";
      if(req.contains("q_goal") && (cycle_binding?
          (expected && req.at("j1_reverse_used_rad").get<double>()+std::max(0.,-expected*(req.at("q_goal").at(0).get<double>()-req.at("q_start").at(0).get<double>()))>m710::kEcoCycleReverseLimit):
          !m710::directedJ1(req.at("q_start").at(0),req.at("q_goal").at(0),expected)))
        return {{"status","INVALID_GOAL_J1_DIRECTION"},{"j1_direction",expected}};
      if(!clearance->check(scene->getCurrentState())) return {{"status","INVALID_START_CLEARANCE"},{"failure",clearance->last_failure},{"clearance",clearance->evidence()},{"pipeline_calls",calls}};
      if(process && !process->check(scene->getCurrentState())) return {{"status","INVALID_START_PROCESS"},{"failure",process->last_failure},{"process_policy",process->evidence()},{"pipeline_calls",calls}};
      if(req.contains("q_goal")) {
        auto goal=req.at("q_goal").get<std::vector<double>>();
        if(goal.size()!=names.size()) throw std::runtime_error("INVALID_GOAL");
        for(double x:goal) if(!std::isfinite(x)) throw std::runtime_error("INVALID_GOAL");
        auto end=scene->getCurrentState();end.setJointGroupPositions(m710::nativeFrames().planning_group,goal);end.update();
        if(!clearance->check(end)) return {{"status","INVALID_GOAL_CLEARANCE"},{"failure",clearance->last_failure},{"clearance",clearance->evidence()},{"pipeline_calls",calls}};
        if(process && !process->check(end,true)) return {{"status","INVALID_GOAL_PROCESS"},{"failure",process->last_failure},{"process_policy",process->evidence()},{"pipeline_calls",calls}};
        if(evaluation_profile=="chain_integration_v1") {
          collision_detection::CollisionRequest cr;collision_detection::CollisionResult collision;scene->checkCollision(cr,collision,end);
          if(collision.collision)return {{"status","INVALID_GOAL_COLLISION"}};
        }
      }
    }
    collision_detection::CollisionRequest cr; cr.contacts=true; cr.max_contacts=20;
    collision_detection::CollisionResult collision;
    {m710::NativeSpan span("plan.start_legacy_collision");scene->checkCollision(cr,collision);}
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
    if(req.value("require_native_motion",false) || op=="eco65_transition") return nativeTaskPlan(req,scene,clearance,process,op=="diagnostic_connection");
    // One native plan attempt at the given Pilz/OMPL velocity and acceleration
    // scaling, on this request's scene, ACM, attachment and Clearance.
    auto attempt=[&](double velocity_scale,double acceleration_scale)->J {
      const auto pipeline=req.at("pipeline_id").get<std::string>(); const auto planner=req.at("planner_id").get<std::string>();
      if(!pipelines.count(pipeline) || !((pipeline=="ompl"&&planner=="RRTConnectkConfigDefault") ||
         (pipeline=="pilz_industrial_motion_planner"&&(planner=="PTP"||planner=="LIN")))) throw std::runtime_error("PLANNER_UNAVAILABLE");
      auto solver=std::make_shared<CountedPlanner>(pipelines.at(pipeline),calls[pipeline+"/"+planner]);
      solver->setProperty("goal_joint_tolerance",1e-12);
      solver->setProperty("goal_position_tolerance",m710::kNativeGoalPositionToleranceM);
      solver->setProperty("goal_orientation_tolerance",m710::kNativeGoalOrientationToleranceRad);
      solver->setPlannerId(planner);solver->setProperty("max_velocity_scaling_factor",velocity_scale);
      solver->setProperty("max_acceleration_scaling_factor",acceleration_scale);
      mtc::Task task("",false);task.setRobotModel(model);task.setName(req.at("stage"));
      auto initial=std::make_unique<mtc::stages::FixedState>("frozen_start"); initial->setState(scene);task.add(std::move(initial));
      auto motion=std::make_unique<mtc::stages::MoveTo>(req.at("stage"),solver);motion->setGroup(m710::nativeFrames().planning_group);
      motion->setTimeout(req.at("allowed_planning_time_s").get<double>());
      if(req.contains("goal_pose")) {
        if(req.at("flange_from_task_tcp")!=identity.at("flange_from_task_tcp")) throw std::runtime_error("TASK_TCP_CONTEXT_MISMATCH");
        const std::string tcp=identity.at("task_tcp_link");
        if(!model->getLinkModel(tcp) || !model->getJointModelGroup(m710::nativeFrames().planning_group)->canSetStateFromIK(tcp)) throw std::runtime_error("TASK_TCP_IK_UNAVAILABLE");
        motion->setIKFrame(Eigen::Isometry3d::Identity(),tcp);
        geometry_msgs::msg::PoseStamped target;target.header.frame_id="world";target.pose=pose(req.at("goal_pose"));motion->setGoal(target);
      } else {
        auto goal=req.at("q_goal").get<std::vector<double>>();if(goal.size()!=names.size()) throw std::runtime_error("INVALID_GOAL");
        std::map<std::string,double> joints;for(size_t i=0;i<names.size();++i) joints[names[i]]=goal[i];motion->setGoal(joints);
      }
      auto* motion_stage=motion.get();
      task.add(std::move(motion)); auto planning=Clock::now(); ++requests; clearance->phase="search";
      const bool bounded_single=identity.value("policy_scope",std::string())=="PHYSICAL_SINGLE_CARTON";
      const bool defer_output=req.value("defer_output_check_to_sent_path",false);
      if(defer_output && (!bounded_single || evaluation_profile!="strict" || planning_only))
        throw std::runtime_error("OUTPUT_CHECK_DEFERRAL_SCOPE_INVALID");
      const double solve_budget=req.at("allowed_planning_time_s");
      if(!std::isfinite(solve_budget)||solve_budget<=0)throw std::runtime_error("INVALID_SOLVE_BUDGET");
      std::shared_ptr<m710::SearchTrace> single_trace;
      if(bounded_single && pipeline=="ompl" && req.value("search_trace",false)) {
        single_trace=std::make_shared<m710::SearchTrace>();single_trace->event_limit=2000;single_trace->sample_limit=4000;
        m710::active_search_trace=single_trace;
      }
      struct SingleSolveReset {
        m710::NativeDeadlinePtr deadline;Clock::time_point end;
        ~SingleSolveReset(){if(deadline){deadline->set(end);m710::active_native_deadline.reset();m710::active_search_trace.reset();}}
      } single_reset{bounded_single?native_deadline:nullptr,request_deadline};
      if(bounded_single) {
        native_deadline->set(std::min(request_deadline,planning+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(solve_budget))));
        m710::active_native_deadline=native_deadline;
      }
      // solution_path_check CLEARANCE_PREDICATE (physical single carton, user
      // request 2026-11-22): the pipeline's isPathValid checks every returned
      // waypoint with (a) MoveIt's mesh intersection test and (b) this request's
      // feasibility predicate (Clearance + process policy). For ECO65 every
      // Clearance pair has a positive required gap (2.4 mm, tool/robot 5 mm), it
      // skips exactly the ACM-allowed and attached touch-link pairs that the
      // intersection test skips, and it measures convex solids that contain the
      // meshes, so (b) passing implies (a) passing. CLEARANCE_PREDICATE runs (b)
      // alone on the same waypoints, with the pipeline's start-state exception.
      // DEFERRED_TO_SENT_PATH_AUDIT (physical single carton, user decision B
      // 2026-11-23): a Pilz request returns its waypoints without this check.
      // Allowed only together with the output-check deferral: the caller must
      // run its mandatory sent-path audit, whose probe states include every
      // returned waypoint (a knot of the sent quintic) with this request's
      // Clearance and process predicate, before the segment can be registered.
      // OMPL requests keep PIPELINE or CLEARANCE_PREDICATE.
      const std::string solution_check=req.value("solution_path_check",std::string("PIPELINE"));
      if(solution_check!="PIPELINE" && solution_check!="CLEARANCE_PREDICATE" && solution_check!="DEFERRED_TO_SENT_PATH_AUDIT")
        throw std::runtime_error("INVALID_SOLUTION_PATH_CHECK");
      const bool predicate_only=solution_check=="CLEARANCE_PREDICATE";
      const bool deferred_solution=solution_check=="DEFERRED_TO_SENT_PATH_AUDIT";
      if(predicate_only && (!bounded_single || !clearance->eco65() || !(clearance->requiredGap()>0.)))throw std::runtime_error("SOLUTION_PATH_CHECK_SCOPE_INVALID");
      if(deferred_solution && (!bounded_single || !clearance->eco65() || !defer_output || pipeline!="pilz_industrial_motion_planner"))
        throw std::runtime_error("SOLUTION_PATH_CHECK_DEFERRAL_SCOPE_INVALID");
      auto& pipeline_ref=*pipelines.at(pipeline);const bool pipeline_check=pipeline_ref.getCheckSolutionPaths();
      struct PipelineCheckReset {planning_pipeline::PlanningPipeline& p;bool flag;~PipelineCheckReset(){p.checkSolutionPaths(flag);}} pipeline_reset{pipeline_ref,pipeline_check};
      if(predicate_only || deferred_solution)pipeline_ref.checkSolutionPaths(false);
      {m710::NativeSpan span(pipeline=="ompl"?"plan.solve_ompl":"plan.solve_pilz");task.plan(1);}
      J predicate_check=nullptr;bool predicate_rejected=false;
      if(predicate_only && !task.solutions().empty()) {
        m710::NativeSpan span("plan.solution_predicate_check");
        moveit_task_constructor_msgs::msg::Solution checked;task.solutions().front()->toMsg(checked);
        auto state=scene->getCurrentState();std::vector<size_t> invalid;size_t waypoints=0;J first=nullptr;bool expired=false;
        try {
          for(auto& sub:checked.sub_trajectory) for(auto& point:sub.trajectory.joint_trajectory.points) {
            if(sub.trajectory.joint_trajectory.joint_names!=names) throw std::runtime_error("OUTPUT_JOINT_ORDER_MISMATCH");
            state.setJointGroupPositions(m710::nativeFrames().planning_group,point.positions);state.update();
            if(!scene->isStateFeasible(state)) {invalid.push_back(waypoints);if(first.is_null())first=clearance->last_failure;}
            ++waypoints;
          }
        } catch(const std::runtime_error& e) {
          // The solve budget ran out inside a check: reported below as budget exhaustion.
          if(std::string(e.what())!="NATIVE_ABSOLUTE_DEADLINE_EXPIRED")throw;
          expired=true;
        }
        if(expired)invalid.push_back(waypoints);
        // Same exception as PlanningPipeline: an invalid start alone is accepted
        // (the start was already checked above).
        predicate_rejected=!invalid.empty() && !(invalid.size()==1 && invalid[0]==0);
        predicate_check={{"mode","CLEARANCE_PREDICATE"},{"waypoints",waypoints},{"invalid_waypoints",invalid.size()},
          {"first_invalid_waypoint",invalid.empty()?J(nullptr):J(invalid[0])},{"first_failure",first},
          {"intersection_test","IMPLIED_BY_POSITIVE_CLEARANCE_ON_CONTAINING_CONVEX_SOLIDS"}};
      }
      double plan_s=seconds(planning);
      const bool solve_expired=bounded_single&&native_deadline->expired();
      if(bounded_single){native_deadline->set(request_deadline);m710::active_native_deadline.reset();}
      J out={{"status","SEARCH_EXHAUSTED"},{"pipeline_id",pipeline},{"planner_id",planner},{"mtc_attempt_index",requests},{"pipeline_calls",calls},
        {"scene_import_s",import_s},{"scene_updated",changed},{"mtc_plan_s",plan_s},{"world_count",scene->getWorld()->size()},
        {"attached_id",attached_id},{"task_tcp_identity",identity.at("task_tcp_fingerprint")},{"interpolated_link",req.contains("goal_pose")?identity.at("task_tcp_link"):J(nullptr)},
        {"resident_seed",seed},{"request_seed",req.at("seed")},{"authoritative_status","NOT_RUN"},{"clearance",clearance->evidence()}};
      if(bounded_single)out["absolute_solver_deadline"]={{"budget_s",solve_budget},{"expired_on_return",solve_expired},
        {"scope","SAME_REQUEST_CLOCK_VALIDITY_PAIR_LOOPS_AND_RRT_EXTENSIONS"},{"single_fcl_call_preemption","NOT_AVAILABLE_CHECKED_BEFORE_AND_AFTER"}};
      if(single_trace)out["search_trace"]={{"scope","DIAGNOSTIC_NOT_EXECUTION_EVIDENCE"},{"planners",single_trace->planners}};
      if(predicate_only)out["solution_path_check"]=predicate_check.is_null()?J({{"mode","CLEARANCE_PREDICATE"},{"waypoints",0}}):predicate_check;
      if(deferred_solution)out["solution_path_check"]={{"mode","DEFERRED_TO_SENT_PATH_AUDIT"},{"waypoints",nullptr},
        {"required_caller_check","SENT_PATH_AUDIT_PROBE_STATES_INCLUDE_EVERY_RETURNED_WAYPOINT"}};
      if(solve_expired){out["status"]="NATIVE_SOLVER_BUDGET_EXHAUSTED";return out;}
      if(predicate_rejected) {
        // Mirrors a pipeline isPathValid rejection: no solution from this request.
        if(pipeline=="pilz_industrial_motion_planner") out["status"]="NATIVE_PLANNING_FAILED";
        out["detail"]="SOLUTION_PATH_REJECTED_BY_CLEARANCE_PREDICATE";out["failure_comments"]=J::array();return out;
      }
      if(task.solutions().empty()) {
        if(pipeline=="pilz_industrial_motion_planner") out["status"]="NATIVE_PLANNING_FAILED";
        std::ostringstream why; task.explainFailure(why);out["detail"]=why.str(); J failures=J::array();for(const auto& failure:motion_stage->failures()) failures.push_back(failure->comment());out["failure_comments"]=failures;
        for(const auto& item:failures) {
          const auto message=item.get<std::string>();
          if(message.find("NO_IK_SOLUTION")!=std::string::npos) out["status"]="NATIVE_IK_FAILED";
          else if(message.find("INVALID_MOTION_PLAN")!=std::string::npos) out["status"]="NATIVE_INVALID_MOTION_PLAN";
        }return out;}
      auto output_span=std::make_unique<m710::NativeSpan>("plan.solution_to_points");
      moveit_task_constructor_msgs::msg::Solution msg;task.solutions().front()->toMsg(msg);
      J points=J::array();
      for(auto& sub:msg.sub_trajectory) {
        auto& jt=sub.trajectory.joint_trajectory; if(jt.points.empty()) continue;
        if(jt.joint_names!=names) throw std::runtime_error("OUTPUT_JOINT_ORDER_MISMATCH");
        if(pipeline=="ompl") {
          robot_trajectory::RobotTrajectory rt(model,m710::nativeFrames().planning_group);rt.setRobotTrajectoryMsg(scene->getCurrentState(),sub.trajectory);
          trajectory_processing::IterativeParabolicTimeParameterization iptp;
          if(!iptp.computeTimeStamps(rt,velocity_scale,acceleration_scale)) throw std::runtime_error("TIME_PARAMETERIZATION_FAILED");
          rt.getRobotTrajectoryMsg(sub.trajectory);
        }
        for(auto& p:sub.trajectory.joint_trajectory.points) points.push_back({{"q",p.positions},{"v",p.velocities},{"a",p.accelerations},
          {"t",p.time_from_start.sec+p.time_from_start.nanosec*1e-9}});
      }
      // Physical single-carton callers may defer this dense check of the
      // planner's linear-joint waypoint path: they must run the identical
      // clearance->checkPath ("output" phase, same policy and resolution) on
      // the exact sent quintic path, which contains every returned waypoint,
      // before the segment can be registered. Every other scope keeps it.
      output_span.reset();
      if(deferred_solution)out["solution_path_check"]["waypoints"]=points.size();
      if(defer_output) {
        out["native_output_check_s"]=nullptr;out["clearance"]=clearance->evidence();
        out["native_output_status"]="DEFERRED_TO_CALLER_SENT_PATH_CHECK";
        out["output_check_deferral"]={{"scope","PHYSICAL_SINGLE_CARTON_STRICT"},
          {"required_caller_check","VALIDATE_PROBE_PATH_OUTPUT_PHASE_ON_EXACT_SENT_PATH_INCLUDING_ALL_WAYPOINTS"},
          {"returned_waypoints",points.size()}};
      } else {
      clearance->phase="output"; J path=J::array();for(const auto& p:points) path.push_back(p.at("q"));
      auto output_check=Clock::now();J native_failure;
      {m710::NativeSpan span("plan.output_check_path");native_failure=path_check(path);}
      out["native_output_check_s"]=seconds(output_check);out["clearance"]=clearance->evidence();
      if(!native_failure.is_null()) {out["status"]="NATIVE_PATH_REJECTED";out["failure"]=native_failure;out["points"]=points;out["total_s"]=seconds(begin);return out;}
      out["native_output_status"]="PASS";
      }
      out["status"]="SUCCESS";out["points"]=points;out["joint_names"]=names;out["total_s"]=seconds(begin);
      out["time_parameterization"]=pipeline=="ompl"?"IPTP_preserves_waypoints":"Pilz_original";return out;
    };
    J out=attempt(req.at("velocity_scale").get<double>(),req.at("acceleration_scale").get<double>());
    if(!req.contains("lin_retry_scales"))return out;
    // lin_retry_scales (physical single carton, 2026-11-22): the caller's
    // slower Pilz LIN profiles after a failed attempt, tried here in the given
    // order on the same scene, ACM, attachment and Clearance instead of one
    // request per profile. Retried exactly when the caller would retry (status
    // not SUCCESS); the reply is the last attempt's, with every attempt listed.
    // Pilz LIN is deterministic, so each attempt equals the separate request.
    if(identity.value("policy_scope",std::string())!="PHYSICAL_SINGLE_CARTON" ||
       req.at("pipeline_id")!="pilz_industrial_motion_planner" || req.at("planner_id")!="LIN" ||
       !req.at("lin_retry_scales").is_array() || req.at("lin_retry_scales").size()>8)
      throw std::runtime_error("LIN_RETRY_SCOPE_INVALID");
    auto summary=[](const J& r,double scale)->J {
      return {{"velocity_scale",scale},{"acceleration_scale",scale},{"status",r.value("status",std::string())},
        {"detail",r.value("detail",J(nullptr))},{"failure_comments",r.value("failure_comments",J(nullptr))},
        {"native_output_status",r.value("native_output_status",J(nullptr))},{"mtc_attempt_index",r.value("mtc_attempt_index",J(nullptr))},
        {"mtc_plan_s",r.value("mtc_plan_s",J(nullptr))},{"solution_path_check",r.value("solution_path_check",J(nullptr))},
        {"pipeline_calls",r.value("pipeline_calls",J(nullptr))}};
    };
    double scale=req.at("velocity_scale");
    if(req.at("acceleration_scale")!=req.at("velocity_scale"))throw std::runtime_error("LIN_RETRY_SCOPE_INVALID");
    J ladder=J::array({summary(out,scale)});
    for(const auto& next:req.at("lin_retry_scales")) {
      if(out.value("status",std::string())=="SUCCESS")break;
      const double factor=next.get<double>();
      if(!std::isfinite(factor)||factor<=0.||factor>1.)throw std::runtime_error("LIN_RETRY_SCALE_INVALID");
      if(native_deadline && native_deadline->expired())break;
      scale=factor;out=attempt(factor,factor);ladder.push_back(summary(out,factor));
    }
    out["lin_retry"]={{"attempts",ladder},{"final_velocity_scale",scale},{"final_acceleration_scale",scale},
      {"scope","SAME_SCENE_ACM_ATTACHMENT_CLEARANCE_PILZ_LIN_DETERMINISTIC"}};
    return out;
  }
};
#ifndef M710_NATIVE_WORKER_NO_MAIN
int main(int argc,char**argv) {
  setenv("RCUTILS_LOGGING_USE_STDOUT","0",1);rclcpp::init(argc,argv);std::map<std::string,std::unique_ptr<Worker>> workers;std::map<std::string,J> startups,init_requests;std::string line;
  // Measurement only (M710_NATIVE_TIMING=1). The serialization time of a
  // reply is known only after it is written, so it is reported with the next.
  const bool timing=m710::NativeTiming::enabled();double previous_dump_s=-1.,previous_reply_bytes=-1.;
  // Resident ECO65 workers (M710_WORKER_CANCEL=1): a reader thread takes
  // out-of-band {"op":"cancel","target_request_id":...} lines and cancels the
  // running request's NativeDeadline; every other line is queued and handled
  // serially exactly as before. Without the variable the loop is unchanged.
  const bool async_cancel=[]{const char* v=std::getenv("M710_WORKER_CANCEL");return v&&std::strcmp(v,"1")==0;}();
  // Step 7: stdin carries JSON lines and/or protocol-2 binary frames
  // (binary_frames.h, magic E65B); a reply uses the format of its request.
  struct Message {std::string line;bool binary=false;m710::Frame frame;};
  auto read_message=[&](Message& m)->bool {
    const int c=std::cin.peek();if(c==EOF)return false;
    if(c=='E') {m.binary=true;m.line.clear();return m710::readFrame(std::cin,m.frame);}
    m.binary=false;return bool(std::getline(std::cin,m.line));
  };
  std::mutex queue_mutex,running_mutex;std::condition_variable queue_cv;std::deque<Message> queued;bool input_closed=false;
  std::string running_id;m710::NativeDeadlinePtr running_deadline;bool running_cancelled=false;std::set<std::string> early_cancels,pending_ids;
  std::thread reader;
  if(async_cancel)reader=std::thread([&]{
    Message m;
    while(true) {
      try {if(!read_message(m))break;} catch(...) {break;}
      std::string target;bool cancel=false;
      if(m.binary&&m.frame.kind==m710::kFrameCancel) {cancel=true;target=m.frame.value.value("target_request_id",std::string());}
      else if(!m.binary) {
        try {
          const J parsed=J::parse(m.line);
          if(parsed.value("op",std::string())=="cancel") {
            cancel=true;
            if(parsed.contains("target_request_id")&&parsed["target_request_id"].is_string())
              target=parsed["target_request_id"].get<std::string>();
          }
        } catch(...) {} // malformed ordinary requests retain the normal error reply
      }
      if(cancel) {
        std::lock_guard<std::mutex> g(running_mutex);
        if(!target.empty()&&target==running_id) {if(running_deadline)running_deadline->cancel();running_cancelled=true;}
        else if(pending_ids.count(target))early_cancels.insert(target);
        continue;
      }
      try {
        const J parsed=m.binary?m.frame.value:J::parse(m.line);
        const auto id=parsed.value("request_id",std::string());
        if(!id.empty()) {std::lock_guard<std::mutex> g(running_mutex);pending_ids.insert(id);}
      } catch(...) {}
      {std::lock_guard<std::mutex> g(queue_mutex);queued.push_back(std::move(m));m=Message{};}
      queue_cv.notify_one();
    }
    {std::lock_guard<std::mutex> g(queue_mutex);input_closed=true;}
    queue_cv.notify_one();
  });
  Message message;
  auto next_message=[&](Message& out)->bool {
    if(!async_cancel) {try {return read_message(out);} catch(...) {return false;}}
    std::unique_lock<std::mutex> g(queue_mutex);queue_cv.wait(g,[&]{return input_closed||!queued.empty();});
    if(queued.empty())return false;
    out=std::move(queued.front());queued.pop_front();return true;
  };
  while(next_message(message)) {
    const auto received=Clock::now();double parse_s=-1.;line=message.binary?std::string():message.line;
    if(timing)m710::NativeTiming::instance().reset();
    J req,answer;try {req=message.binary?std::move(message.frame.value):J::parse(line);parse_s=seconds(received); auto* protocol=std::cout.rdbuf(std::cerr.rdbuf()); try {const auto key=req.at("identity").dump();
      // Only an init request is compared with the stored init content; other
      // requests are no longer deep-copied for it (2026-11-22, no behaviour change).
      const bool is_init=req.at("op")=="init";J init_content;
      if(is_init) {init_content=req;init_content.erase("request_id");}
      if(is_init && startups.count(key)) {
        if(init_content!=init_requests.at(key)) throw std::runtime_error("CONTEXT_IDENTITY_REUSED_WITH_DIFFERENT_CONTENT");
        answer=startups.at(key);answer["context_reused"]=true;
      }
      else {
        if(req.at("op")=="init") {
          if(workers.size()>=4) throw std::runtime_error("UNSUPPORTED_RESIDENT_CONTEXT_CAPACITY");
          workers[key]=std::make_unique<Worker>();
        }
        if(!workers.count(key)) throw std::runtime_error("UNKNOWN_MODEL_TCP_CONTEXT");
        if(async_cancel) {
          std::lock_guard<std::mutex> g(running_mutex);
          running_id=req.value("request_id",std::string());running_cancelled=false;running_deadline=nullptr;
          workers.at(key)->on_deadline=[&](const m710::NativeDeadlinePtr& d) {
            std::lock_guard<std::mutex> h(running_mutex);running_deadline=d;
            if(running_cancelled||early_cancels.erase(running_id)) {d->cancel();running_cancelled=true;}
          };
        }
        answer=workers.at(key)->dispatch(req);
        if(req.at("op")=="init") {answer["context_reused"]=false;startups[key]=answer;init_requests[key]=init_content;}
      }} catch(...) {std::cout.rdbuf(protocol);throw;} std::cout.rdbuf(protocol);} catch(const std::exception& e) {answer={{"status","ERROR"},{"detail",e.what()}};}
    if(!answer.contains("evaluation_profile"))answer["evaluation_profile"]=req.value("evaluation_profile",std::string("strict"));
    answer["request_id"]=req.value("request_id",std::string());
    if(async_cancel) {
      std::lock_guard<std::mutex> g(running_mutex);
      if(running_cancelled)answer["cancelled_by_request"]=true;
      pending_ids.erase(req.value("request_id",std::string()));early_cancels.erase(req.value("request_id",std::string()));
      running_id.clear();running_deadline=nullptr;running_cancelled=false;
    }
    if(timing) {
      J t=m710::NativeTiming::instance().json();
      t["request_line_bytes"]=message.binary?message.frame.bytes:line.size();t["request_parse_s"]=parse_s<0?J(nullptr):J(parse_s);
      t["request_format"]=message.binary?"BINARY_FRAME_V2":"JSON_LINE";
      t["handled_s"]=seconds(received);
      t["previous_reply_dump_s"]=previous_dump_s<0?J(nullptr):J(previous_dump_s);
      t["previous_reply_bytes"]=previous_reply_bytes<0?J(nullptr):J(previous_reply_bytes);
      answer["native_timing"]=t;
    }
    const auto dumping=Clock::now();
    if(message.binary) {
      const std::string bytes=m710::encodeFrame(m710::kFrameReply,message.frame.id,std::move(answer));
      std::cout.write(bytes.data(),std::streamsize(bytes.size()));std::cout.flush();previous_reply_bytes=double(bytes.size());
    } else {const std::string text=answer.dump();std::cout<<text<<std::endl;previous_reply_bytes=double(text.size());}
    previous_dump_s=seconds(dumping);
  }
  if(reader.joinable())reader.detach();
  rclcpp::shutdown();return 0;
}

#endif  // M710_NATIVE_WORKER_NO_MAIN
