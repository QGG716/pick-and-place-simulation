#pragma once
#include "clearance.h"

// Test-only differential wrapper: every existing scenario exercises both modes.
// Production uses one checker; no test clock/counter enters the formal benchmark.
class ProfileComparedClearance : public m710::Clearance {
  m710::Clearance detailed_;
  static void expect(bool ok,const char* why) {if(!ok)throw std::runtime_error(why);}
  static nlohmann::json counts(nlohmann::json j) {
    if(!j.is_object())return j;
    for(auto it=j.begin();it!=j.end();) {
      const auto& key=it.key();
      if(key=="pair_profile_enabled"||key=="pair_profile_status"||
          (key.size()>=2&&key.substr(key.size()-2)=="_s"))it=j.erase(it);
      else {it.value()=counts(it.value());++it;}
    }
    return j;
  }
  void compare() {
    expect(last_failure==detailed_.last_failure,"PAIR_PROFILE_FAILURE_DRIFT");
    auto normal=workspaceEvidence(),detail=detailed_.workspaceEvidence();
    expect(counts(normal)==counts(detail),"PAIR_PROFILE_GEOMETRY_COUNTS_DRIFT");
    if(normal.is_null())return; // legacy reference backend has no workspace timers
    expect(normal["pair_profile_enabled"]==false&&detail["pair_profile_enabled"]==true,"PAIR_PROFILE_FLAG");
    expect(normal["pair_profile_status"]=="NOT_MEASURED"&&detail["pair_profile_status"]=="MEASURED","PAIR_PROFILE_STATUS");
    for(auto it=normal["phases"].begin();it!=normal["phases"].end();++it)
      for(const char* key:{"broad_phase_s","narrow_s","self_narrow_s","world_narrow_s"}) {
        expect(it.value().at(key).is_null(),"UNMEASURED_MUST_BE_NULL");
        expect(detail["phases"][it.key()].at(key).is_number(),"MEASURED_MUST_BE_NUMERIC");
      }
  }
public:
  ProfileComparedClearance(const planning_scene::PlanningSceneConstPtr& s,const m710::Json& p,
      const std::string& mode="optimized",bool reuse=true,bool qualify=true)
    :m710::Clearance(s,p,mode,reuse,qualify,false),detailed_(s,p,mode,reuse,qualify,true) {}
  bool check(const moveit::core::RobotState& state,bool flag=false) {
    detailed_.phase=phase;
    const bool a=m710::Clearance::check(state,flag),b=detailed_.check(state,flag);
    expect(a==b,"PAIR_PROFILE_VERDICT_DRIFT");compare();return a;
  }
  m710::Json checkPath(moveit::core::RobotState state,const m710::Json& path,double resolution) {
    detailed_.phase=phase;
    const auto a=m710::Clearance::checkPath(state,path,resolution),b=detailed_.checkPath(state,path,resolution);
    expect(a==b,"PAIR_PROFILE_PATH_WITNESS_DRIFT");compare();return a;
  }
};
