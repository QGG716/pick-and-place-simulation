#pragma once
#include <moveit/planning_scene/planning_scene.h>
#include <nlohmann/json.hpp>
#include <chrono>
#include <mutex>
#include <set>

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
  std::mutex mutex_;
public:
  std::string phase = "endpoint";
  Json counts = Json::object(), last_failure = nullptr, rejected_examples = Json::array(), search_gap_witness = nullptr;
  Clearance(const planning_scene::PlanningSceneConstPtr& scene, const Json& policy)
    : env_(scene->getCollisionEnvUnpadded()), acm_(scene->getAllowedCollisionMatrix()), policy_(policy) {
    if(policy.at("schema")!="m710_native_free_clearance_v1" ||
       policy.at("source_policy").at("schema")!="m710_poc_pair_collision_policy_v4" ||
       policy.at("source_policy").at("required_pair_clearance_m")!=.005 ||
       policy.at("source_policy").at("self_collision_clearance_m")!=0. ||
       policy.at("numerical_gap_tolerance_m")!=1e-9)
      throw std::runtime_error("UNSUPPORTED_CLEARANCE_POLICY");
    const std::string stage=policy.at("stage");
    if(stage!="transit" && stage!="pregrasp" && stage!="residence")
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
    auto& c=counts[phase];
    if(c.is_null()) c={{"queries",0},{"rejected",0},{"distance_api_calls",0},{"clearance_rejected",0},{"legacy_witness_checks",0},{"seconds",0.}};
    c["queries"]=c["queries"].get<size_t>()+1;
    last_failure=nullptr;
    auto finish=[&](bool valid) {
      c["seconds"]=c["seconds"].get<double>()+std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
      if(!valid) {
        c["rejected"]=c["rejected"].get<size_t>()+1;
        if(last_failure.value("reason",std::string())=="CLEARANCE_INSUFFICIENT") {
          c["clearance_rejected"]=c["clearance_rejected"].get<size_t>()+1;
          if(phase=="search" && search_gap_witness.is_null()) {
            auto probe=input;probe.updateCollisionBodyTransforms();
            collision_detection::CollisionRequest request;collision_detection::CollisionResult result;
            env_->checkSelfCollision(request,result,probe,acm_);env_->checkRobotCollision(request,result,probe,acm_);
            c["legacy_witness_checks"]=c["legacy_witness_checks"].get<size_t>()+1;
            if(!result.collision) {search_gap_witness=last_failure;std::vector<double> q;probe.copyJointGroupPositions("manipulator",q);search_gap_witness["q_rad"]=q;search_gap_witness["legacy_intersection_valid"]=true;}
          }
        }

        if(rejected_examples.size()<8) { auto detail=last_failure; detail["phase"]=phase; std::vector<double> q;input.copyJointGroupPositions("manipulator",q);detail["q_rad"]=q;rejected_examples.push_back(detail); }
      }
      return valid;
    };
    if(!input.satisfiesBounds()) {last_failure={{"reason","JOINT_BOUNDS"}};return finish(false);}
    auto state=input;state.updateCollisionBodyTransforms();
    collision_detection::DistanceRequest req;
    req.type=collision_detection::DistanceRequestType::SINGLE;
    req.acm=&acm_;req.distance_threshold=query_range_;
    // SINGLE returns each pair's minimum below the query range. Ordinary self
    // pairs use zero clearance; robot/tool pairs retain the external gap.
    for(bool self:{true,false}) {
      collision_detection::DistanceResult result;
      try {
        if(self) env_->distanceSelf(req,result,state); else env_->distanceRobot(req,result,state);
        c["distance_api_calls"]=c["distance_api_calls"].get<size_t>()+1;
      } catch(const std::exception& e) {
        last_failure={{"reason","DISTANCE_QUERY_FAILED"},{"detail",e.what()}};return finish(false);
      }
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
  Json evidence() const {return {{"counts",counts},{"rejected_examples",rejected_examples},
    {"search_gap_witness",search_gap_witness},{"query_range_m",query_range_},{"policy",policy_},{"distance_method","MoveIt_FCL_unpadded_pair_minima"},
    {"continuous_swept_proof",false}};}
};
}
