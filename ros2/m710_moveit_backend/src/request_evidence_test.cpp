#include "request_evidence.h"
#include <iostream>
#include <stdexcept>
using J=nlohmann::json;
void require(bool ok,const char* msg){if(!ok)throw std::runtime_error(msg);}
int main(){
 int c=0,p=0;
 auto read=[&]{++c;return J{{"counts",{{"search",{{"queries",3},{"seconds",.2}}}}},
   {"policy",{{"static","omit"}}},{"workspace",{{"pair_profile_enabled",false},{"narrow_s",nullptr}}}};};
 auto process=[&]{++p;return J{{"state_checks",2},{"last_failure",{{"reason","EDGE_REJECTED"}}}};};
 for(const auto& status:{"PRIOR_NOT_CONNECTED","PRIOR_QUERY_TIMEOUT","NATIVE_PLANNING_FAILED","CANCELLED"}){
  J out={{"status",status},{"solver_message","INVALID_MOTION_PLAN"},{"moveit_error_code",-2}};
  m710::completeRequestEvidence(out,true,true,read,process);
  require(out["status"]==status&&out["solver_message"]=="INVALID_MOTION_PLAN"&&out["moveit_error_code"]==-2,"ORIGINAL_STATUS");
  require(!out.contains("failure")&&!out["clearance"].contains("policy"),"NO_PROMOTED_EDGE_WITNESS_OR_POLICY");
  require(out["request_check_evidence"]["clearance_check_s"]==.2,"ACTUAL_COST");
  const int before=c; m710::completeRequestEvidence(out,true,true,read,process);require(c==before,"ONCE");
 }
 require(c==4&&p==4,"ONE_SNAPSHOT_PER_REQUEST");
 J success={{"status","SUCCESS"},{"clearance",read()},{"process_policy",process()}};
 const int before=c;
 m710::completeRequestEvidence(success,true,true,read,process);require(c==before,"SUCCESS_NOT_REMATERIALIZED");
 J missing={{"status","ERROR"}};m710::completeRequestEvidence(missing,false,false,read,process);
 require(missing["request_check_evidence"]["clearance"]=="NOT_CREATED"&&missing["request_check_evidence"]["clearance_check_s"].is_null(),"NOT_CREATED");
 J empty={{"status","ERROR"}};
 m710::completeRequestEvidence(empty,true,false,[]{return J{{"counts",J::object()}};},process);
 require(empty["request_check_evidence"]["clearance"]=="CREATED_NOT_CHECKED"&&empty["request_check_evidence"]["clearance_check_s"].is_null(),"NOT_CHECKED");
 J broken={{"status","ERROR"}};
 m710::completeRequestEvidence(broken,true,false,[]()->J{throw std::runtime_error("unavailable");},process);
 require(broken["request_check_evidence"]["clearance"]=="UNAVAILABLE"&&broken["request_check_evidence"]["clearance_check_s"].is_null(),"UNAVAILABLE");
 std::cout<<"PASS request-local evidence: failures, once-only success, no-check, unavailable; no geometry/solver callback\n";
}
