#include "native_robot.h"
#pragma once
#include <moveit/planning_scene/planning_scene.h>
#include <nlohmann/json.hpp>
#include <chrono>
#include <mutex>
#include <set>
#include <iostream>
#include <cstdlib>
#include <optional>
#include "clearance_workspace.h"
#include "eco65_collision_policy.h"
#include "eco65_rigid_carton_policy.h"
#include "eco65_single_carton.h"

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
  // Physical single carton only: larger clearance between owned tool solids
  // and non-tool robot links (incident 2026-11-14), with explicit exemptions
  // for motion-invariant pairs (fixed mount, J6-coaxial adapter).
  double tool_robot_gap_=0.;std::set<std::string> tool_robot_exempt_links_;std::set<std::pair<std::string,std::string>> tool_robot_exempt_pairs_;
  using Clock=std::chrono::steady_clock;
  std::map<std::string,double> profile_;
  static double elapsed(Clock::time_point t) {return std::chrono::duration<double>(Clock::now()-t).count();}
  std::mutex mutex_;
  std::unique_ptr<ClearanceWorkspace> workspace_;
  std::string mode_;
  bool eco65_=false;
  bool path_endpoint_reuse_=false;
  bool two_level_=false;
  // sent_path_audit ADAPTIVE: skip an output-phase path sample only when the
  // per-pair displacement bound from the last exactly checked sample proves it
  // passes; every other sample keeps the exact check.
  bool adaptive_=false;
  static constexpr double kAdaptiveProofEpsilon=1e-6;
  struct AdaptiveAudit {size_t sampled=0,checked=0,skipped=0,anchors=0;double minimum_lower=INFINITY,minimum_workspace=INFINITY,constraint_s=0,proof_s=0;} audit_;
  NativeDeadlinePtr deadline_;
  std::unique_ptr<SingleCartonContact> single_carton_;
  struct Count {size_t queries=0,rejected=0,distance_api_calls=0,clearance_rejected=0,legacy_witness_checks=0;
    double seconds=0,bookkeeping_s=0,witness_s=0;};
  std::map<std::string,Count> counts_;
public:
  std::string phase = "endpoint";
  Json last_failure = nullptr, rejected_examples = Json::array(), search_gap_witness = nullptr;
  Clearance(const planning_scene::PlanningSceneConstPtr& scene, const Json& policy, const std::string& mode="optimized",
            NativeDeadlinePtr deadline={},const Json& request=nullptr)
    : env_(scene->getCollisionEnvUnpadded()), acm_(scene->getAllowedCollisionMatrix()), policy_(policy), mode_(mode),deadline_(std::move(deadline)) {
    const bool process=policy.at("schema")=="m710_native_process_clearance_v1";
    eco65_=policy.at("schema")=="eco65_native_process_clearance_v1" ||
      policy.at("schema")=="eco65_native_empty_clearance_v1" ||
      policy.at("schema")=="eco65_native_single_carton_clearance_v1";
    if(policy.at("schema")=="eco65_native_single_carton_clearance_v1")
      single_carton_=std::make_unique<SingleCartonContact>(request,deadline_);
    // Optional (physical single carton only): a sample bitwise equal to the
    // immediately preceding, already passed sample (the endpoint shared by
    // two adjacent edges) is not checked twice. The checked state set and
    // every edge's sampling rule are unchanged.
    if(!request.is_null() && request.is_object() && request.value("path_endpoint_reuse",false)) {
      if(!single_carton_)throw std::runtime_error("PATH_ENDPOINT_REUSE_SCOPE_INVALID");
      path_endpoint_reuse_=true;
    }
    // collision_check_mode (physical single carton only): EXACT (default) or
    // TWO_LEVEL, where conservative proxies may only prove pairs clear and
    // every other pair keeps the exact FCL decision.
    const std::string check_mode=(!request.is_null()&&request.is_object())?request.value("collision_check_mode",std::string("EXACT")):std::string("EXACT");
    if(check_mode!="EXACT"&&check_mode!="TWO_LEVEL")throw std::runtime_error("COLLISION_CHECK_MODE_INVALID");
    if(check_mode=="TWO_LEVEL"&&!single_carton_)throw std::runtime_error("COLLISION_CHECK_MODE_SCOPE_INVALID");
    two_level_=check_mode=="TWO_LEVEL";
    // sent_path_audit (physical single carton, no contact, no support): SAMPLED (default) or ADAPTIVE.
    const std::string audit=(!request.is_null()&&request.is_object())?request.value("sent_path_audit",std::string("SAMPLED")):std::string("SAMPLED");
    if(audit!="SAMPLED"&&audit!="ADAPTIVE")throw std::runtime_error("SENT_PATH_AUDIT_INVALID");
    if(audit=="ADAPTIVE"&&(!single_carton_||!single_carton_->adaptiveEligible()))throw std::runtime_error("SENT_PATH_AUDIT_SCOPE_INVALID");
    adaptive_=audit=="ADAPTIVE";
    if(policy.at("schema")=="eco65_native_empty_clearance_v1" &&
       (policy.at("source_policy").at("schema")!="eco65_pair_collision_policy_v1" ||
        request.is_null() || request.contains("process_policy") ||
        !request.at("attachment").is_null() ||
        request.at("allowed_pairs")!=Json::array({Json::array({"base_link","robot_mount_plate"})}) ||
        request.at("identity").value("policy_scope",std::string())!="CONTROLLED_EMPTY_NO_CONTACT"))
      throw std::runtime_error("EMPTY_MOTION_SCOPE_INVALID");
    if(eco65_) {
      const auto& source=policy.at("source_policy");
      if((source.at("schema")!="eco65_pair_collision_policy_v1" && !eco65HasMastPolicy(source)) ||
         source.at("collision_margin_m")!=.002 || source.at("mesh_error_reserve_m")!=.0002 ||
         source.at("required_pair_clearance_m")!=.0024 || source.at("self_collision_clearance_m")!=.0024 ||
         source.at("contact_tolerance_m")!=.0004 || policy.at("numerical_gap_tolerance_m")!=1e-9 ||
         policy.at("receiver_reserve_m")!=0.)throw std::runtime_error("UNSUPPORTED_ECO65_CLEARANCE_POLICY");
      if(eco65HasMastPolicy(source)) {
        for(const auto& pair:source.at("robot_mast_exemption").at("exempt_pairs")) {
          collision_detection::AllowedCollision::Type allowed;
          if(!acm_.getEntry(pair.at(0).get<std::string>(),pair.at(1).get<std::string>(),allowed) ||
              allowed!=collision_detection::AllowedCollision::ALWAYS)
            throw std::runtime_error("ECO65_MAST_SRDF_ACM_MISMATCH");
        }
      }
      for(const auto& pair:eco65RobotToolPairs(source)) {
        collision_detection::AllowedCollision::Type allowed;
        if(!acm_.getEntry(pair.at(0).get<std::string>(),pair.at(1).get<std::string>(),allowed) ||
            allowed!=collision_detection::AllowedCollision::ALWAYS)
          throw std::runtime_error("ECO65_ROBOT_TOOL_SRDF_ACM_MISMATCH");
      }
      for(const auto& pair:eco65RigidCartonPairs(request)) {
        collision_detection::AllowedCollision::Type allowed;
        if(!acm_.getEntry(pair.at(0).get<std::string>(),pair.at(1).get<std::string>(),allowed) ||
            allowed!=collision_detection::AllowedCollision::ALWAYS)
          throw std::runtime_error("ECO65_RIGID_CARTON_ACM_MISMATCH");
      }
      if(source.at("schema")=="eco65_pair_collision_policy_v3" || eco65HasCupPolicy(source)) {
        if(request.is_null() || request.at("clearance_policy")!=policy)
          throw std::runtime_error("ECO65_EXTRACTION_RUNTIME_CONTEXT_REQUIRED");
        const auto expected=eco65ExtractionNeighbors(request);
        const std::string target=policy.at("target_id");
        for(const auto& id:expected) {
          collision_detection::AllowedCollision::Type allowed;
          if(!scene->getCurrentState().hasAttachedBody(target) || !scene->getWorld()->hasObject(id) ||
              !acm_.getEntry(target,id,allowed) || allowed!=collision_detection::AllowedCollision::ALWAYS)
            throw std::runtime_error("ECO65_EXTRACTION_ACM_MISMATCH");
        }
        if(request.value("continuous_task_mode",std::string())=="eco65_coupled_cycle_v1" &&
            policy.at("stage")!="extraction")for(const auto& id:eco65ExtractionOriginalIds(source))if(id!=target) {
          collision_detection::AllowedCollision::Type allowed;
          if(acm_.getEntry(target,id,allowed) && allowed==collision_detection::AllowedCollision::ALWAYS)
            throw std::runtime_error("ECO65_EXTRACTION_PERMISSION_LEAKED_TO_OTHER_STAGE");
        }
      }
      if(eco65HasCupPolicy(source)) {
        for(const auto& pair:eco65CupNeighborPairs(request)) {
          collision_detection::AllowedCollision::Type allowed;
          if(!acm_.getEntry(pair.at(0).get<std::string>(),pair.at(1).get<std::string>(),allowed) ||
              allowed!=collision_detection::AllowedCollision::ALWAYS)
            throw std::runtime_error("ECO65_CUP_NEIGHBOR_ACM_MISMATCH");
        }
      }
    } else if((policy.at("schema")!="m710_native_free_clearance_v1" && !process) ||
       policy.at("source_policy").at("schema")!="m710_poc_pair_collision_policy_v4" ||
       policy.at("source_policy").at("required_pair_clearance_m")!=.005 ||
       policy.at("source_policy").at("self_collision_clearance_m")!=0. ||
       policy.at("numerical_gap_tolerance_m")!=1e-9)
      throw std::runtime_error("UNSUPPORTED_CLEARANCE_POLICY");
    const std::string stage=policy.at("stage");
    if(!process && !eco65_ && stage!="transit" && stage!="pregrasp" && stage!="residence")
      throw std::runtime_error("UNSUPPORTED_CLEARANCE_STAGE");
    tools_=policy.at("tool_links").get<std::set<std::string>>();
    conveyors_=policy.at("conveyor_ids").get<std::set<std::string>>();
    payload_=policy.at("payload_id").get<std::string>();
    gap_=policy.at("source_policy").at("required_pair_clearance_m");
    if(policy.contains("tool_robot_clearance")) {
      const auto& t=policy.at("tool_robot_clearance");
      if(!eco65_ || !single_carton_) throw std::runtime_error("TOOL_ROBOT_CLEARANCE_SCOPE_INVALID");
      tool_robot_gap_=t.at("clearance_m");
      if(!std::isfinite(tool_robot_gap_) || tool_robot_gap_<gap_ || tool_robot_gap_>.1 || t.value("basis",std::string()).empty())
        throw std::runtime_error("TOOL_ROBOT_CLEARANCE_INVALID");
      tool_robot_exempt_links_=t.at("exempt_robot_links").get<std::set<std::string>>();
      for(const auto& pair:t.at("exempt_pairs")) {
        const std::string tool=pair.at(0),link=pair.at(1);
        if(!tools_.count(tool) || tools_.count(link)) throw std::runtime_error("TOOL_ROBOT_EXEMPT_PAIR_INVALID");
        tool_robot_exempt_pairs_.insert({tool,link});
      }
      for(const auto& link:tool_robot_exempt_links_) if(tools_.count(link)) throw std::runtime_error("TOOL_ROBOT_EXEMPT_LINK_INVALID");
    }
    reserve_=stage=="transit" ? policy.at("receiver_reserve_m").get<double>() : 0.;
    if(!std::isfinite(reserve_) || reserve_<0) throw std::runtime_error("INVALID_RESERVE");
    const double search_payload_reserve=policy.value("search_payload_reserve_m",0.);
    if(!std::isfinite(search_payload_reserve) || search_payload_reserve<0. || search_payload_reserve>.001 ||
       (search_payload_reserve>0. && (!eco65_ || stage!="transit")))
      throw std::runtime_error("INVALID_SEARCH_PAYLOAD_RESERVE");
    // Query range is deliberately larger than every acceptance threshold.
    // It is not padding and not an acceptance tolerance.
    query_range_=std::max(gap_,tool_robot_gap_)+reserve_+.001;
    if(scene->getCollisionDetectorName()!="FCL") throw std::runtime_error("FCL_REQUIRED");
    if(mode!="reference" && mode!="optimized") throw std::runtime_error("UNKNOWN_CLEARANCE_MODE");
    if(eco65_ && mode!="optimized") throw std::runtime_error("ECO65_EXPLICIT_CONVEX_WORKSPACE_REQUIRED");
    if(mode=="optimized") {
      const auto* fcl=dynamic_cast<const collision_detection::CollisionEnvFCL*>(env_.get());
      if(!fcl) throw std::runtime_error("FCL_REQUIRED");
      auto preparing=Clock::now();
      workspace_=std::make_unique<ClearanceWorkspace>(*fcl,scene,[this](const auto& a,auto ta,const auto& b,auto tb){return required(a,ta,b,tb);},eco65_,search_payload_reserve,deadline_,bool(single_carton_),two_level_,
        adaptive_?m710::nativeFrames().planning_group:std::string());
      profile_["workspace_allocation_s"]=elapsed(preparing);
    }
    if(adaptive_&&(!workspace_||!workspace_->adaptive()))throw std::runtime_error("SENT_PATH_AUDIT_SCOPE_INVALID");
  }
  double required(const std::string& a, collision_detection::BodyType ta,
                  const std::string& b, collision_detection::BodyType tb) const {
    using namespace collision_detection;
    if(eco65_ && tool_robot_gap_>0. && ta==BodyTypes::ROBOT_LINK && tb==BodyTypes::ROBOT_LINK && tools_.count(a)!=tools_.count(b)) {
      const std::string& tool=tools_.count(a)?a:b;const std::string& link=tools_.count(a)?b:a;
      if(!tool_robot_exempt_links_.count(link) && !tool_robot_exempt_pairs_.count({tool,link})) return tool_robot_gap_;
    }
    if(eco65_) return gap_; // Source applies margin + two body reserves once, including self pairs.
    if(ta==BodyTypes::ROBOT_LINK && tb==BodyTypes::ROBOT_LINK && !tools_.count(a) && !tools_.count(b)) return 0.;
    if((a==payload_ && conveyors_.count(b)) || (b==payload_ && conveyors_.count(a))) return gap_+reserve_;
    return gap_;
  }
  bool check(const moveit::core::RobotState& input, bool = false) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto start=std::chrono::steady_clock::now();
    auto& c=counts_[phase];++c.queries;
    last_failure=nullptr;
    // Measurement only (M710_NATIVE_TIMING=1): duration of this one complete
    // exact state check; every rejection path sets last_failure before return.
    struct StateTiming {
      bool on;Clock::time_point start;const std::string& phase;const Json& failure;
      ~StateTiming() {if(on)NativeTiming::instance().state(phase,failure.is_null(),Clearance::elapsed(start));}
    } state_timing{NativeTiming::enabled(),start,phase,last_failure};
    if(nativeExpired(deadline_)) {
      last_failure={{"reason","NATIVE_ABSOLUTE_DEADLINE_EXPIRED"},{"phase",phase}};
      ++c.rejected;return false;
    }
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
            if(!result.collision) {search_gap_witness=last_failure;std::vector<double> q;probe.copyJointGroupPositions(m710::nativeFrames().planning_group,q);search_gap_witness["q_rad"]=q;search_gap_witness["legacy_intersection_valid"]=true;}
          }
        }

        if(rejected_examples.size()<8) { auto detail=last_failure; detail["phase"]=phase; std::vector<double> q;input.copyJointGroupPositions(m710::nativeFrames().planning_group,q);detail["q_rad"]=q;rejected_examples.push_back(detail); }
      }
      c.bookkeeping_s+=elapsed(bookkeeping);c.seconds+=elapsed(start);
      if(eco65_ && std::getenv("ECO65_TRACE_CHECKS") && c.queries%100==0)
        std::cerr<<"ECO65_CLEARANCE_PROGRESS "<<phase<<" queries="<<c.queries<<" seconds="<<c.seconds<<std::endl;
      return valid;
    };
    if(!input.satisfiesBounds()) {last_failure={{"reason","JOINT_BOUNDS"}};return finish(false);}
    auto copying=Clock::now();
    // TWO_LEVEL: a state whose collision transforms are already current is
    // used as is (same transforms); otherwise copied and updated as before.
    std::optional<moveit::core::RobotState> updated;const moveit::core::RobotState* current=&input;
    if(!two_level_ || input.dirtyCollisionBodyTransforms()) {
      NativeSpan span("clearance.state_copy_update");updated.emplace(input);updated->updateCollisionBodyTransforms();current=&*updated;
    }
    const auto& state=*current;
    if(single_carton_) {
      {NativeSpan span("clearance.single_carton_contact");last_failure=single_carton_->check(state);}
      if(!last_failure.is_null())return finish(false);
    }
    profile_["state_copy_update_s"]+=elapsed(copying);
    if(workspace_) {
      ClearanceWorkspace::Failure f;
      try {
        if(workspace_->check(state,f,phase)) return finish(true);
        last_failure={{"reason",!f.reason.empty()?f.reason:!std::isfinite(f.distance)?"DISTANCE_UNKNOWN":f.distance<=0.?"INTERSECTION_OR_CONTACT":"CLEARANCE_INSUFFICIENT"},
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
  size_t dense_path_calls=0, exact_repeated_path_states_reused=0;
  Json checkPath(moveit::core::RobotState probe, const Json& path, double resolution) {
    ++dense_path_calls;
    if(!std::isfinite(resolution) || resolution<=0 || path.empty()) throw std::runtime_error("INVALID_CHECK_PATH");
    const size_t count=probe.getJointModelGroup(m710::nativeFrames().planning_group)->getVariableCount();
    // One immutable scene/policy per call. Reuse only the immediately preceding
    // successful check of an exactly equal joint vector, only in controlled empty
    // mode. Keep every distinct endpoint/interior state and all legacy sampling.
    const bool reuse=policy_.at("schema")=="eco65_native_empty_clearance_v1" || path_endpoint_reuse_;
    // ADAPTIVE walks exactly the same samples. A sample is skipped only when,
    // for every constraint of the last exactly checked sample (the anchor),
    // margin - sum_v c_v*|q_v - anchor_v| >= 1 µm: then every pair keeps at
    // least its limit and every site-workspace box stays inside, and the
    // remaining single-carton conditions do not depend on the state in this
    // scope. Joint bounds and the deadline are still tested per sample.
    const bool adaptive=adaptive_ && phase=="output";
    if(adaptive && probe.getJointModelGroup(m710::nativeFrames().planning_group)->getVariableNames()!=workspace_->adaptiveJoints())
      throw std::runtime_error("ADAPTIVE_AUDIT_GROUP_MISMATCH");
    struct Recording {ClearanceWorkspace* w;bool on;~Recording(){if(on)w->recording(false);}} recording{workspace_.get(),adaptive};
    if(adaptive)workspace_->recording(true);
    std::vector<ClearanceWorkspace::Constraint> constraints;std::vector<double> anchor;bool anchored=false;
    std::vector<double> previous;
    for(size_t edge=0;edge<std::max(size_t(1),path.size()-1);++edge) {
      auto a=path.at(edge).get<std::vector<double>>();auto b=path.at(std::min(edge+1,path.size()-1)).get<std::vector<double>>();
      if(a.size()!=count || b.size()!=count) throw std::runtime_error("INVALID_PATH_JOINT_COUNT");
      double maximum=0,sum=0;
      for(size_t i=0;i<a.size();++i) {if(!std::isfinite(a[i]) || !std::isfinite(b[i])) throw std::runtime_error("INVALID_PATH_VALUES");maximum=std::max(maximum,std::abs(b[i]-a[i]));sum+=std::abs(b[i]-a[i]);}
      const size_t samples=2*std::max({size_t(1),size_t(std::ceil(maximum/resolution)),size_t(std::ceil(4.*sum/.0025))});
      for(size_t k=0;k<=samples;++k) {
        const double fraction=double(k)/samples;auto q=a;for(size_t i=0;i<q.size();++i) q[i]+=fraction*(b[i]-a[i]);
        if(reuse && q==previous && !nativeExpired(deadline_)) {++exact_repeated_path_states_reused;continue;}
        if(adaptive) ++audit_.sampled;
        if(adaptive && anchored && !nativeExpired(deadline_)) {
          probe.setJointGroupPositions(m710::nativeFrames().planning_group,q);
          if(probe.satisfiesBounds() && proven(constraints,q,anchor)) {++audit_.skipped;previous=q;continue;}
        }
        probe.setJointGroupPositions(m710::nativeFrames().planning_group,q);probe.update();
        if(!check(probe)) {Json failure=last_failure;failure["edge"]=edge;failure["fraction"]=fraction;failure["q_rad"]=q;return failure;}
        previous=q;
        if(adaptive) {++audit_.checked;anchor=q;anchorConstraints(probe,constraints);anchored=true;}
      }
    }
    return nullptr;
  }
  bool proven(const std::vector<ClearanceWorkspace::Constraint>& constraints,const std::vector<double>& q,const std::vector<double>& anchor) {
    const auto t=Clock::now();const bool ok=ClearanceWorkspace::provenClear(constraints,q,anchor,kAdaptiveProofEpsilon);
    audit_.proof_s+=elapsed(t);return ok;
  }
  void anchorConstraints(const moveit::core::RobotState& state,std::vector<ClearanceWorkspace::Constraint>& constraints) {
    const auto t=Clock::now();double lower=INFINITY;
    workspace_->adaptiveConstraints(constraints,lower);
    audit_.minimum_lower=std::min(audit_.minimum_lower,lower);++audit_.anchors;
    std::vector<std::tuple<const moveit::core::LinkModel*,double,double>> margins;
    single_carton_->workspaceMargins(state,margins);
    for(const auto& [link,margin,rho]:margins) {
      audit_.minimum_workspace=std::min(audit_.minimum_workspace,margin);
      std::vector<double> lever;if(!workspace_->leverOf(link,rho,lever))continue; // fixed to the world
      constraints.push_back({std::min(margin,workspace_->marginCap()),lever});
    }
    audit_.constraint_s+=elapsed(t);
  }
  Json workspaceEvidence() const {
    if(!workspace_) return nullptr;
    Json result={{"tight_transformed_local_bounds",workspace_->tightBounds()},{"prepared_objects",workspace_->stats.prepared_objects},{"eligible_pairs",workspace_->stats.eligible_pairs},
      {"exempt_pairs",workspace_->stats.exempt_pairs},{"prepare_s",workspace_->stats.prepare_s},{"geometry_prepare_s",workspace_->stats.geometry_prepare_s},{"pair_filter_s",workspace_->stats.pair_filter_s},{"phases",Json::object()}};
    for(const auto& entry:workspace_->phases) {const auto& s=entry.second;
      result["phases"][entry.first]={{"queries",s.queries},{"aabb_tests",s.aabb_tests},{"aabb_skips",s.aabb_skips},
        {"hierarchy_nodes_visited",s.hierarchy_nodes_visited},
        {"narrow_calls",s.narrow_calls},{"self_narrow",s.self_narrow},{"world_narrow",s.world_narrow},
        {"proxy_constant_cleared",s.proxy_constant_cleared},{"proxy_group_cleared",s.proxy_group_cleared},
        {"proxy_sphere_cleared",s.proxy_sphere_cleared},{"proxy_unvisited_cleared",s.proxy_unvisited_cleared},
        {"exact_candidates",s.exact_candidates},{"states_with_exact",s.states_with_exact},
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
    Json result={{"dense_path_calls",dense_path_calls},{"exact_repeated_path_states_reused",exact_repeated_path_states_reused},{"counts",counts},{"rejected_examples",rejected_examples},
    {"search_gap_witness",search_gap_witness},{"query_range_m",query_range_},{"policy",policy_},{"distance_method","MoveIt_FCL_unpadded_pair_minima"},
    {"search_payload_query_range_m",query_range_+policy_.value("search_payload_reserve_m",0.)},
    {"search_reserve_scope","ATTACHED_PAYLOAD_PAIRS_DURING_SEARCH_ONLY_OUTPUT_POLICY_UNCHANGED"},
    {"mode",mode_},{"tool_robot_clearance_m",tool_robot_gap_},{"collision_check_mode",two_level_?"TWO_LEVEL":"EXACT"},{"workspace",workspaceEvidence()},{"profile",profile_},{ "continuous_swept_proof",false}};
    if(adaptive_)result["sent_path_audit"]={{"method","ADAPTIVE"},{"sampled_states",audit_.sampled},{"checked_states",audit_.checked},
      {"skipped_states",audit_.skipped},{"anchors",audit_.anchors},
      {"distance_evidence_scope","EXACT_ANCHORS_DYNAMIC_CONSTRAINTS_ONLY_EXCLUDES_SKIPPED_STATES_AND_FIXED_PAIRS"},
      {"whole_path_minimum_distance_status","NOT_COMPUTED"},
      {"anchor_dynamic_pair_distance_lower_bound_m",std::isfinite(audit_.minimum_lower)?Json(audit_.minimum_lower):Json(nullptr)},
      {"anchor_workspace_margin_m",std::isfinite(audit_.minimum_workspace)?Json(audit_.minimum_workspace):Json(nullptr)},
      {"margin_cap_m",workspace_->marginCap()},{"proof_epsilon_m",kAdaptiveProofEpsilon},
      {"constraint_s",audit_.constraint_s},{"proof_s",audit_.proof_s},
      {"skip_rule","PER_GROUP_PAIR_LEVER_BOUND_FROM_LAST_EXACT_SAMPLE"},{"samples","IDENTICAL_TO_SAMPLED"}};
    result["evidence_materialization_s"]=elapsed(t);return result;}
};
}
