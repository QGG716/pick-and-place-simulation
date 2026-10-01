#pragma once
#include <moveit/planning_scene/planning_scene.h>
#include <nlohmann/json.hpp>
#include <chrono>
#include <mutex>
#include <set>
#include "clearance_workspace.h"

namespace m710 {
using Json = nlohmann::json;
// One immutable world/ACM/policy context per candidate; never captures q_start.
class Clearance {
  collision_detection::CollisionEnvConstPtr env_;
  collision_detection::AllowedCollisionMatrix acm_;
  Json policy_;
  std::set<std::string> tools_, conveyors_;
  std::string payload_;
  double gap_, reserve_, query_range_;
  using Clock=std::chrono::steady_clock;
  std::map<std::string,double> profile_;
  static double elapsed(Clock::time_point t) {return std::chrono::duration<double>(Clock::now()-t).count();}
  std::mutex mutex_;
  std::unique_ptr<ClearanceWorkspace> workspace_;
  std::string mode_;
  struct Count {size_t queries=0,rejected=0,distance_api_calls=0,clearance_rejected=0,legacy_witness_checks=0;
    double seconds=0,bookkeeping_s=0,witness_s=0;};
  std::map<std::string,Count> counts_;
public:
  std::string phase = "endpoint";
  Json last_failure = nullptr, rejected_examples = Json::array(), search_gap_witness = nullptr;
  Clearance(const planning_scene::PlanningSceneConstPtr& scene, const Json& policy, const std::string& mode="optimized")
    : env_(scene->getCollisionEnvUnpadded()), acm_(scene->getAllowedCollisionMatrix()), policy_(policy), mode_(mode) {
    const bool process=policy.at("schema")=="m710_native_process_clearance_v1";
    if((policy.at("schema")!="m710_native_free_clearance_v1" && !process) ||
       policy.at("source_policy").at("schema")!="m710_poc_pair_collision_policy_v4" ||
       policy.at("source_policy").at("required_pair_clearance_m")!=.005 ||
       policy.at("source_policy").at("self_collision_clearance_m")!=0. ||
       policy.at("numerical_gap_tolerance_m")!=1e-9)
      throw std::runtime_error("UNSUPPORTED_CLEARANCE_POLICY");
    const std::string stage=policy.at("stage");
    if(!process && stage!="transit" && stage!="pregrasp" && stage!="residence")
      throw std::runtime_error("UNSUPPORTED_CLEARANCE_STAGE");
    tools_=policy.at("tool_links").get<std::set<std::string>>();
    conveyors_=policy.at("conveyor_ids").get<std::set<std::string>>();
    payload_=policy.at("payload_id").get<std::string>();
    gap_=policy.at("source_policy").at("required_pair_clearance_m");
    reserve_=stage=="transit" ? policy.at("receiver_reserve_m").get<double>() : 0.;
    if(!std::isfinite(reserve_) || reserve_<0) throw std::runtime_error("INVALID_RESERVE");
    // Query range is deliberately larger than every acceptance threshold.
    // It is not padding and not an acceptance tolerance.
    query_range_=gap_+reserve_+.001;
    if(scene->getCollisionDetectorName()!="FCL") throw std::runtime_error("FCL_REQUIRED");
    if(mode!="reference" && mode!="optimized") throw std::runtime_error("UNKNOWN_CLEARANCE_MODE");
    if(mode=="optimized") {
      const auto* fcl=dynamic_cast<const collision_detection::CollisionEnvFCL*>(env_.get());
      if(!fcl) throw std::runtime_error("FCL_REQUIRED");
      auto preparing=Clock::now();
      workspace_=std::make_unique<ClearanceWorkspace>(*fcl,scene,[this](const auto& a,auto ta,const auto& b,auto tb){return required(a,ta,b,tb);});
      profile_["workspace_allocation_s"]=elapsed(preparing);
    }
  }
  double required(const std::string& a, collision_detection::BodyType ta,
                  const std::string& b, collision_detection::BodyType tb) const {
    using namespace collision_detection;
    if(ta==BodyTypes::ROBOT_LINK && tb==BodyTypes::ROBOT_LINK && !tools_.count(a) && !tools_.count(b)) return 0.;
    if((a==payload_ && conveyors_.count(b)) || (b==payload_ && conveyors_.count(a))) return gap_+reserve_;
    return gap_;
  }
  bool check(const moveit::core::RobotState& input, bool = false) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto start=std::chrono::steady_clock::now();
    auto& c=counts_[phase];++c.queries;
    last_failure=nullptr;
    auto finish=[&](bool valid) {
      auto bookkeeping=Clock::now();
      if(!valid) {
        ++c.rejected;
        if(last_failure.value("reason",std::string())=="CLEARANCE_INSUFFICIENT") {
          ++c.clearance_rejected;
          if(phase=="search" && search_gap_witness.is_null()) {
            auto witness=Clock::now();auto probe=input;probe.updateCollisionBodyTransforms();
            collision_detection::CollisionRequest request;collision_detection::CollisionResult result;
            env_->checkSelfCollision(request,result,probe,acm_);env_->checkRobotCollision(request,result,probe,acm_);
            ++c.legacy_witness_checks;c.witness_s+=elapsed(witness);
            if(!result.collision) {search_gap_witness=last_failure;std::vector<double> q;probe.copyJointGroupPositions("manipulator",q);search_gap_witness["q_rad"]=q;search_gap_witness["legacy_intersection_valid"]=true;}
          }
        }

        if(rejected_examples.size()<8) { auto detail=last_failure; detail["phase"]=phase; std::vector<double> q;input.copyJointGroupPositions("manipulator",q);detail["q_rad"]=q;rejected_examples.push_back(detail); }
      }
      c.bookkeeping_s+=elapsed(bookkeeping);c.seconds+=elapsed(start);return valid;
    };
    if(!input.satisfiesBounds()) {last_failure={{"reason","JOINT_BOUNDS"}};return finish(false);}
    auto copying=Clock::now();
    auto state=input;state.updateCollisionBodyTransforms();
    profile_["state_copy_update_s"]+=elapsed(copying);
    if(workspace_) {
      ClearanceWorkspace::Failure f;
      try {
        if(workspace_->check(state,f,phase)) return finish(true);
        last_failure={{"reason",!std::isfinite(f.distance)?"DISTANCE_UNKNOWN":f.distance<=0.?"INTERSECTION_OR_CONTACT":"CLEARANCE_INSUFFICIENT"},
          {"pair",{f.a,f.b}},{"surface_distance_m",std::isfinite(f.distance)?Json(f.distance):Json(nullptr)},
          {"required_pair_clearance_m",f.limit},{"numerical_gap_tolerance_m",1e-9},{"query_range_m",f.limit+.001},
          {"query_scope",f.self?"self_including_robot_tool_and_payload":"robot_tool_payload_to_world"}};
      } catch(const std::exception& e) {last_failure={{"reason","DISTANCE_QUERY_FAILED"},{"detail",e.what()}};}
      return finish(false);
    }
    collision_detection::DistanceRequest req;
    req.type=collision_detection::DistanceRequestType::SINGLE;
    req.acm=&acm_;req.distance_threshold=query_range_;
    // SINGLE returns each pair's minimum below the query range. Ordinary self
    // pairs use zero clearance; robot/tool pairs retain the external gap.
    for(bool self:{true,false}) {
      collision_detection::DistanceResult result;
      auto querying=Clock::now();
      try {
        if(self) env_->distanceSelf(req,result,state); else env_->distanceRobot(req,result,state);
        ++c.distance_api_calls;
      } catch(const std::exception& e) {
        last_failure={{"reason","DISTANCE_QUERY_FAILED"},{"detail",e.what()}};return finish(false);
      }
      profile_[self?"self_distance_api_s":"world_distance_api_s"]+=elapsed(querying);
      for(const auto& entry:result.distances) for(const auto& d:entry.second) {
        const double limit=required(d.link_names[0],d.body_types[0],d.link_names[1],d.body_types[1]);
        if(!std::isfinite(d.distance) || d.distance<=0. || d.distance+1e-9<limit) {
          last_failure={{"reason",!std::isfinite(d.distance)?"DISTANCE_UNKNOWN":d.distance<=0.?"INTERSECTION_OR_CONTACT":"CLEARANCE_INSUFFICIENT"},
            {"pair",{d.link_names[0],d.link_names[1]}},{"surface_distance_m",std::isfinite(d.distance)?Json(d.distance):Json(nullptr)},
            {"required_pair_clearance_m",limit},{"numerical_gap_tolerance_m",1e-9},{"query_range_m",query_range_},
            {"query_scope",self?"self_including_robot_tool_and_payload":"robot_tool_payload_to_world"}};
          return finish(false);
        }
      }
      if(result.collision) {last_failure={{"reason","DISTANCE_COLLISION_WITHOUT_PAIR_EVIDENCE"}};return finish(false);}
      // Empty results mean no non-exempt pair within this finite query range,
      // NOT an infinite minimum clearance. Unsupported geometry is rejected at import.
    }
    return finish(true);
  }
  Json checkPath(moveit::core::RobotState probe, const Json& path, double resolution) {
    if(!std::isfinite(resolution) || resolution<=0 || path.empty()) throw std::runtime_error("INVALID_CHECK_PATH");
    const size_t count=probe.getJointModelGroup("manipulator")->getVariableCount();
    for(size_t edge=0;edge<std::max(size_t(1),path.size()-1);++edge) {
      auto a=path.at(edge).get<std::vector<double>>();auto b=path.at(std::min(edge+1,path.size()-1)).get<std::vector<double>>();
      if(a.size()!=count || b.size()!=count) throw std::runtime_error("INVALID_PATH_JOINT_COUNT");
      double maximum=0,sum=0;
      for(size_t i=0;i<a.size();++i) {if(!std::isfinite(a[i]) || !std::isfinite(b[i])) throw std::runtime_error("INVALID_PATH_VALUES");maximum=std::max(maximum,std::abs(b[i]-a[i]));sum+=std::abs(b[i]-a[i]);}
      const size_t samples=2*std::max({size_t(1),size_t(std::ceil(maximum/resolution)),size_t(std::ceil(4.*sum/.0025))});
      for(size_t k=0;k<=samples;++k) {
        const double fraction=double(k)/samples;auto q=a;for(size_t i=0;i<q.size();++i) q[i]+=fraction*(b[i]-a[i]);
        probe.setJointGroupPositions("manipulator",q);probe.update();
        if(!check(probe)) {Json failure=last_failure;failure["edge"]=edge;failure["fraction"]=fraction;failure["q_rad"]=q;return failure;}
      }
    }
    return nullptr;
  }
  Json workspaceEvidence() const {
    if(!workspace_) return nullptr;
    Json result={{"prepared_objects",workspace_->stats.prepared_objects},{"eligible_pairs",workspace_->stats.eligible_pairs},
      {"exempt_pairs",workspace_->stats.exempt_pairs},{"prepare_s",workspace_->stats.prepare_s},{"geometry_prepare_s",workspace_->stats.geometry_prepare_s},{"pair_filter_s",workspace_->stats.pair_filter_s},{"phases",Json::object()}};
    for(const auto& entry:workspace_->phases) {const auto& s=entry.second;
      result["phases"][entry.first]={{"queries",s.queries},{"aabb_tests",s.aabb_tests},{"aabb_skips",s.aabb_skips},
        {"narrow_calls",s.narrow_calls},{"self_narrow",s.self_narrow},{"world_narrow",s.world_narrow},
        {"transform_aabb_update_s",s.update_s},{"broad_phase_s",s.broad_s},{"narrow_s",s.narrow_s},
        {"self_narrow_s",s.self_s},{"world_narrow_s",s.world_s}};
    }return result;
  }
  Json evidence() const {
    const auto t=Clock::now();Json counts=Json::object();
    for(const auto& entry:counts_) {const auto& c=entry.second;
      counts[entry.first]={{"queries",c.queries},{"rejected",c.rejected},{"valid",c.queries-c.rejected},
        {"distance_api_calls",c.distance_api_calls},{"clearance_rejected",c.clearance_rejected},
        {"legacy_witness_checks",c.legacy_witness_checks},{"seconds",c.seconds},
        {"bookkeeping_s",c.bookkeeping_s},{"legacy_witness_s",c.witness_s}};
    }
    Json result={{"counts",counts},{"rejected_examples",rejected_examples},
    {"search_gap_witness",search_gap_witness},{"query_range_m",query_range_},{"policy",policy_},{"distance_method","MoveIt_FCL_unpadded_pair_minima"},
    {"mode",mode_},{"workspace",workspaceEvidence()},{"profile",profile_},{ "continuous_swept_proof",false}};result["evidence_materialization_s"]=elapsed(t);return result;}
};
}
