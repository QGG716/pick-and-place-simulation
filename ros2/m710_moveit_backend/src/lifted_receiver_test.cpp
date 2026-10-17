#include "lifted_receiver.h"
#include <cassert>
#include <iostream>
using J=nlohmann::json;
J box(std::string id,double z) {
  return {{"id",id},{"size",{.68,1.6,.12}},{"category","conveyor"},
    {"pose",{{1,0,0,-.55},{0,1,0,.35},{0,0,1,z},{0,0,0,1}}}};
}
template<class F> void rejected(F f) {bool bad=false;try {f();}catch(const std::exception&){bad=true;}assert(bad);}
int main() {
  auto low=box("conveyor_transverse",.54),high=box("conveyor_transverse",1.99);
  auto sweep=m710::receiverSweep(low,high);
  assert(std::abs(sweep["size"][2].get<double>()-1.57)<1e-12);
  auto obstacle=box("midway",1.2);
  assert(m710::receiverBoxDistance(low,obstacle)>.005);
  assert(m710::receiverBoxDistance(high,obstacle)>.005);
  assert(m710::receiverBoxDistance(sweep,obstacle)<=0.);
  auto target=box("target",2.25);target["size"]={.6,.4,.3};target["pose"][0][3]=.3;
  auto binding=J{{"target_id","target"},{"low",low},{"high",high}};
  auto req=J{{"planning_only",true},{"planning_mode","cold_from_scratch"},{"attachment",nullptr},
    {"parent_stage_id","this-task:1"},{"q_start",{0,0,0,0,0,0}},{"q_goal",{0,0,0,0,0,0}},
    {"stage","pregrasp"},{"pipeline_id","external_axis"},{"planner_id","LIFT_Z"},
    {"scene_fingerprint","low-scene"},{"receiver_end_fingerprint","high-scene"},
    {"ownership_transition",nullptr},{"process_policy",{{"target_id","target"},{"target",target}}},
    {"world",{low,target}},{"receiver_end_world",{high,target}}};
  assert(m710::receiverTransitionWorld(req,binding)==req["receiver_end_world"]);
  for(const auto key:{"q_goal","world","attachment","parent_stage_id","stage"}) {
    auto bad=req;
    if(std::string(key)=="q_goal")bad[key][0]=.01;
    else if(std::string(key)=="world")bad[key][1]["pose"][0][3]=.4;
    else if(std::string(key)=="attachment")bad[key]=target;
    else bad[key]="";
    rejected([&]{m710::receiverTransitionWorld(bad,binding);});
  }
  auto sideways=high;sideways["pose"][0][3]=-.54;
  rejected([&]{m710::receiverSweep(low,sideways);});
  auto resize=high;resize["size"][0]=.7;
  rejected([&]{m710::receiverSweep(low,resize);});
  auto chassis=box("chassis",.3);chassis["size"]={2.1,1.5,.6};chassis["pose"][0][3]=-1.94;
  // Low test belt spans [-.89,-.21]; chassis ends exactly at -.89.
  binding["designed_contacts"]=J::array({{"chassis","conveyor_transverse"}});
  assert(m710::receiverDesignedPlaneContact(low,chassis,binding));
  assert(m710::receiverBoxDistance(sweep,chassis)<=0.);
  assert(m710::receiverBoxDistance(high,chassis)>.005);
  auto penetration=chassis;penetration["pose"][0][3]=-1.939;
  assert(!m710::receiverDesignedPlaneContact(low,penetration,binding));
  auto undeclared=chassis;undeclared["id"]="wall";
  assert(!m710::receiverDesignedPlaneContact(low,undeclared,binding));
  assert(!m710::receiverDesignedPlaneContact(low,chassis,J::object()));
  std::cout<<"PASS exact Z union: endpoint clear / middle rejected; strict transition invariants\n";
}
