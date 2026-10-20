#pragma once
#include "clearance.h"
#include "process_policy.h"
#include "joint_direction.h"
#include <limits>
#include <functional>

namespace m710 {
// ECO65 only: ProcessPolicy::check has the complete state-local process rules.
// FANUC's ordered proximity history remains in its strict checkPath implementation.
// Replay uses q=a+s(u)*(b-a), with monotonic common quintic s, so uniform s
// samples check the identical geometric curve, independently of its time law.
inline Json basicSampledPath(const planning_scene::PlanningScenePtr& scene,
                            Clearance& clearance, ProcessPolicy& process,
                            const Json& path, double resolution, double timeout_s,
                            const std::function<Json(const moveit::core::RobotState&)>& endpoint, int direction=0,
                            double reverse_used=-1.,double reverse_limit=1e-6) {
  const auto begin=std::chrono::steady_clock::now();
  auto elapsed=[&](){return std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();};
  Json report={{"status","BASIC_SAMPLED_INCOMPLETE"},{"sampling_rule","MAX_ABS_JOINT_DELTA_CEIL_V1"},
    {"geometric_interpolation","COMMON_PROGRESS_STRAIGHT_JOINT_EDGES"},{"resolution_rad",resolution},
    {"raw_nodes",path.size()},{"checked_states",0},{"clearance_checks",0},{"process_checks",0},
    {"ordinary_collision_checks",0},{"endpoint_process_checks",0},{"completed",false},
    {"actual_endpoint_status","NOT_RUN"},{"continuous_swept_proof",false},{"failure",nullptr},
    {"j1_direction",direction},{"j1_tolerance_rad",kJ1DirectionEpsilon},{"j1_edge_checks",0}};
  size_t checked=0,clear=0,proc=0,collision_count=0,endpoints=0,edge=0;double fraction=0;
  const bool total_reverse=reverse_used>=0.;double reverse=0.;
  if(total_reverse)report["j1_total_reverse_before_rad"]=reverse_used;
  try {
    if(path.size()<2 || !std::isfinite(resolution) || resolution<=0 || !std::isfinite(timeout_s) || timeout_s<=0)
      throw std::runtime_error("INVALID_BASIC_SAMPLED_INPUT");
    auto state=scene->getCurrentState();const auto group=nativeFrames().planning_group;
    const size_t joints=state.getJointModelGroup(group)->getVariableCount();
    std::vector<std::vector<double>> nodes;
    for(const auto& row:path) {
      auto q=row.get<std::vector<double>>();
      if(q.size()!=joints)throw std::runtime_error("INVALID_PATH_JOINT_COUNT");
      for(double x:q)if(!std::isfinite(x))throw std::runtime_error("INVALID_PATH_VALUES");
      nodes.push_back(q);
    }
    clearance.phase="output";
    for(edge=0;edge<std::max(size_t(1),nodes.size()-1);++edge) {
      const auto& a=nodes[edge];const auto& b=nodes[std::min(edge+1,nodes.size()-1)];double maximum=0;
      report["j1_edge_checks"]=edge+1;
      reverse+=j1ReverseTravel(a.at(0),b.at(0),direction);
      if(total_reverse)report["j1_total_reverse_after_rad"]=reverse_used+reverse;
      if((total_reverse && (!std::isfinite(reverse_limit) || reverse_limit!=1e-6 ||
           reverse_used+reverse>reverse_limit)) ||
          (!total_reverse&&!directedJ1(a.at(0),b.at(0),direction))) {
        report["status"]="BASIC_SAMPLED_REJECTED";
        report["failure"]={{"reason","J1_DIRECTION_VIOLATION"},{"edge",edge},{"from_q",a},{"to_q",b},{"direction",direction}};
        throw std::runtime_error("J1_DIRECTION_VIOLATION");
      }
      for(size_t j=0;j<joints;++j)maximum=std::max(maximum,std::abs(b[j]-a[j]));
      const double n=std::max(1.,std::ceil(maximum/resolution));
      if(!std::isfinite(n) || n>=double(std::numeric_limits<size_t>::max()))
        throw std::runtime_error("BASIC_SAMPLE_COUNT_OVERFLOW");
      const size_t count=size_t(n);
      for(size_t k=(edge?1:0);k<=count;++k) {
        if(elapsed()>timeout_s)throw std::runtime_error("BASIC_SAMPLED_TIMEOUT");
        fraction=double(k)/count;auto q=a;
        // Preserve native endpoints bit-for-bit, including the final LIN state.
        if(k==count)q=b;else for(size_t j=0;j<joints;++j)q[j]+=fraction*(b[j]-a[j]);
        state.setJointGroupPositions(group,q);state.update();++checked;
        const bool last=edge+2>=nodes.size() && k==count;
        Json failure=nullptr;
        ++clear;
        if(!clearance.check(state))failure=clearance.last_failure;
        if(failure.is_null()) {++proc;if(last)++endpoints;
          if(!process.check(state,last))failure=process.last_failure;}
        if(failure.is_null()) {
          collision_detection::CollisionRequest request;collision_detection::CollisionResult result;
          ++collision_count;scene->checkCollision(request,result,state);
          if(result.collision)failure={{"reason","BASIC_ORDINARY_COLLISION"}};
        }
        if(last && failure.is_null())failure=endpoint(state);
        if(last)report["actual_endpoint_status"]=failure.is_null()?"PASS":"FAIL";
        if(!failure.is_null()) {
          failure["edge"]=edge;failure["fraction"]=fraction;failure["q_rad"]=q;
          report["failure"]=failure;report["status"]="BASIC_SAMPLED_REJECTED";
          throw std::runtime_error("BASIC_STATE_REJECTED");
        }
      }
    }
    if(elapsed()>timeout_s)throw std::runtime_error("BASIC_SAMPLED_TIMEOUT");
    report["completed"]=true;report["status"]="BASIC_SAMPLED_PASS";
  } catch(const std::exception& error) {
    if(report["failure"].is_null())report["failure"]={{"reason",error.what()},{"edge",edge},{"fraction",fraction}};
  }
  report["checked_states"]=checked;report["clearance_checks"]=clear;report["process_checks"]=proc;
  report["ordinary_collision_checks"]=collision_count;report["endpoint_process_checks"]=endpoints;
  report["wall_s"]=elapsed();return report;
}
}
