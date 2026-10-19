#pragma once
#include <nlohmann/json.hpp>
#include <moveit/kinematics_base/kinematics_base.h>
#include <cmath>
#include <set>
#include <stdexcept>
namespace m710 {
using IkJson=nlohmann::json;
inline bool boundedIkScope(const IkJson& r,bool enabled,bool planning_only,const std::string& mode) {
  if(!enabled || !planning_only || mode!="cold_from_scratch" || r.value("op","")!="ik" ||
      r.value("stage","")!="transit" || r.value("purpose","")!="FREE_LOADED_TRANSFER" ||
      r.value("parent_stage_id","").empty() || r.value("task_id","").empty() ||
      !r.value("ownership_transition",IkJson(nullptr)).is_null()) return false;
  const auto& a=r.at("attachment");const auto& p=r.at("process_policy");
  if(a.is_null() || a.at("id")!=p.at("target_id") || !p.value("attached",false) ||
      !p.at("support_names").empty() || !p.at("initial_proximity").is_null()) return false;
  const auto& mask=p.at("actual_contact_mask");
  if(mask.size()!=72 || mask!=p.at("commanded_active_mask")) return false;
  bool any=false;for(const auto& x:mask) {if(!x.is_boolean())return false;any|=x.get<bool>();}
  std::set<std::string> ids;
  for(const auto& b:r.at("world")) if(b.at("id")==a.at("id") || !ids.insert(b.at("id").get<std::string>()).second)return false;
  return any;
}
// Only the no-prior single-target entry opts in. Current q stays the motion start.
inline bool boundedTargetIkScope(const IkJson& r,bool enabled,bool planning_only,const std::string& mode,bool coupled_parent=false) {
  if(!enabled || !planning_only || mode!="cold_from_scratch" || r.value("op","")!="ik" ||
      r.value("task_id","").empty())return false;
  if(r.value("stage","")=="transit")return boundedIkScope(r,true,planning_only,mode);
  if(r.value("stage","")!="pregrasp" || r.value("purpose","")!="FREE_APPROACH" ||
      !r.at("attachment").is_null() || r.at("process_policy").value("attached",true) ||
      (!coupled_parent && !r.value("parent_stage_id",std::string()).empty()) ||
      !r.value("ownership_transition",IkJson(nullptr)).is_null())return false;
  size_t targets=0;std::set<std::string> ids;
  for(const auto& b:r.at("world")) {if(!ids.insert(b.at("id").get<std::string>()).second)return false;
    if(b.at("id")==r.at("process_policy").at("target_id"))++targets;}
  return targets==1;
}
inline double boundedIkBudget(double requested,double remaining) {
  if(!std::isfinite(requested)||requested<=0||requested>.10 ||
      !std::isfinite(remaining)) throw std::runtime_error("INVALID_IK_SOLVER_BUDGET");
  return std::min(requested,std::max(0.,remaining));
}
template<class Solver>
bool invokeIk(Solver& solver,bool bounded,const geometry_msgs::msg::Pose& target,
              const std::vector<double>& seed,double budget,std::vector<double>& out,
              moveit_msgs::msg::MoveItErrorCodes& error) {
  kinematics::KinematicsQueryOptions exact;exact.return_approximate_solution=false;
  return bounded ? solver.searchPositionIK(target,seed,budget,out,error,exact)
                 : solver.getPositionIK(target,seed,out,error,exact);
}
}
