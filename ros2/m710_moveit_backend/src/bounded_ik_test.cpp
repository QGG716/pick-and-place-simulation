#include "bounded_ik.h"
#include <iostream>
using J=nlohmann::json;
void require(bool x,const char* msg){if(!x)throw std::runtime_error(msg);}
struct ObservedSolver {
 int get=0,search=0;double timeout=-1;std::vector<double> first;
 bool getPositionIK(const geometry_msgs::msg::Pose&,const std::vector<double>& q,std::vector<double>&,
   moveit_msgs::msg::MoveItErrorCodes&,const kinematics::KinematicsQueryOptions& o) {
   require(!o.return_approximate_solution,"EXACT");++get;first=q;return false;
 }
 bool searchPositionIK(const geometry_msgs::msg::Pose&,const std::vector<double>& q,double t,
   std::vector<double>&,moveit_msgs::msg::MoveItErrorCodes&,const kinematics::KinematicsQueryOptions& o) {
   require(!o.return_approximate_solution,"EXACT");++search;timeout=t;first=q;return true;
 }
};
int main(){
 ObservedSolver solver;geometry_msgs::msg::Pose p;std::vector<double> q={1,2,3,4,5,6},out;
 moveit_msgs::msg::MoveItErrorCodes error;
 require(!m710::invokeIk(solver,false,p,q,.1,out,error),"OLD_RESULT");
 require(solver.get==1 && solver.search==0 && solver.first==q,"OLD_ONE_API");
 require(m710::invokeIk(solver,true,p,q,.03,out,error),"BOUNDED_RESULT");
 require(solver.get==1 && solver.search==1 && solver.timeout==.03 && solver.first==q,"POSITIVE_BUDGET_EXACT_FIRST_SEED");
 require(m710::boundedIkBudget(.1,.03)==.03 && m710::boundedIkBudget(.1,90)==.1 &&
         m710::boundedIkBudget(.1,-.01)==0,"DEADLINE_CLAMP");
 for(double x:{0.,-.1,.10001,std::numeric_limits<double>::quiet_NaN()}) {
  bool threw=false;try{m710::boundedIkBudget(x,90);}catch(...){threw=true;}require(threw,"BAD_BUDGET");
 }
 J r={{"op","ik"},{"stage","transit"},{"purpose","FREE_LOADED_TRANSFER"},{"parent_stage_id","extraction"},
 {"task_id","current"},{"attachment",{{"id","target"}}},{"world",J::array({{{"id","neighbor"}}})},
 {"process_policy",{{"target_id","target"},{"attached",true},{"support_names",J::array()},{"initial_proximity",nullptr},
 {"actual_contact_mask",std::vector<bool>(72,true)},{"commanded_active_mask",std::vector<bool>(72,true)}}}};
 require(m710::boundedIkScope(r,true,true,"cold_from_scratch"),"SCOPED");
 require(!m710::boundedIkScope(r,false,true,"cold_from_scratch"),"OLD_ENTRY");
 require(!m710::boundedIkScope(r,true,false,"cold_from_scratch"),"STRICT_ENTRY");
 require(!m710::boundedIkScope(r,true,true,"static_prior_fast"),"PRIOR_ENTRY");
 for(const char* bad:{"stage","purpose","parent","target","mask","world","support","transition","task"}) {
  auto x=r;std::string b=bad;
  if(b=="stage")x["stage"]="pregrasp";
  if(b=="purpose")x["purpose"]="EXTRACTION_PROCESS";
  if(b=="parent")x["parent_stage_id"]="";
  if(b=="task")x["task_id"]="";
  if(b=="target")x["attachment"]["id"]="wrong";
  if(b=="mask")x["process_policy"]["actual_contact_mask"]=std::vector<bool>(72,false);
  if(b=="world")x["world"].push_back({{"id","target"}});
  if(b=="support")x["process_policy"]["support_names"]={"support"};
  if(b=="transition")x["ownership_transition"]="ATTACH";
  require(!m710::boundedIkScope(x,true,true,"cold_from_scratch"),bad);
 }
 std::cout<<"PASS API dispatch, scope, exact seed/options, positive/clamped budget\n";
}
