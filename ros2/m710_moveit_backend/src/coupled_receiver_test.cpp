#include "nominal_release.h"
#include <fstream>
#include <cassert>
#include <iostream>
using J=nlohmann::json;
template<class F> void rejects(F f){bool failed=false;try{f();}catch(const std::exception&){failed=true;}assert(failed);}
int main(int argc,char**argv){
  assert(argc==2);std::ifstream input(argv[1]);J data;input>>data;
  size_t checked=0;
  for(const auto& b:data.at("bindings")){
    m710::validateCoupledBinding(b);
    J request={{"planning_only",true},{"planning_mode","cold_from_scratch"},{"planner_id","COUPLED_LIFT"},
      {"q_start",{0,0,0,0,0,0}},{"q_goal",{0,0,0,0,0,0}},{"attachment",nullptr},{"ownership_transition",nullptr},
      {"parent_stage_id","actual-parent"},{"stage","pregrasp"},{"world",b.at("samples").front().at("world")},
      {"receiver_end_world",b.at("samples").back().at("world")},{"process_policy",{{"target_id",b.at("target_id")}}}};
    assert(m710::coupledTransitionWorld(request,b)==request.at("receiver_end_world"));
    auto mismatch=request;mismatch["q_goal"][0]=.01;rejects([&]{m710::coupledTransitionWorld(mismatch,b);});
    auto held=request;held["attachment"]={{"id",b.at("target_id")}};held["stage"]="extraction";
    held["process_policy"]["initial_proximity"]=nullptr;held["process_policy"]["support_names"]=J::array();
    for(const auto key:{"world","receiver_end_world"}){J filtered=J::array();for(const auto& box:held[key])if(box.at("id")!=b.at("target_id"))filtered.push_back(box);held[key]=filtered;}
    assert(m710::coupledTransitionWorld(held,b)==held.at("receiver_end_world"));
    held["process_policy"]["support_names"]={"neighbor"};rejects([&]{m710::coupledTransitionWorld(held,b);});
    for(const auto& s:b.at("samples")){assert(m710::coupledWorldFailure(s,b).is_null());++checked;}
    auto bad=b;bad["samples"][0]["world"][0]["size"][0]=0.;rejects([&]{m710::validateCoupledBinding(bad);});
  }
  const auto& binding=data.at("bindings")[0];
  assert(!m710::coupledWorldFailure(data.at("old_model"),binding).is_null());
  for(const auto& request:data.at("releases")){
    m710::validateNominalDrop(request,binding);
    auto obstacle=request.at("process_policy").at("target");obstacle["id"]="UNKNOWN_RIGID_OBSTACLE";
    assert(!m710::nominalDropWorldFailure(request.at("nominal_release").at("samples").front().at("box"),J::array({obstacle})).is_null());
    auto bad=request;bad["nominal_release"]["samples"][0]["box"]["pose"][0][3]=99.;
    rejects([&]{m710::validateNominalDrop(bad,binding);});
    bad=request;bad["nominal_release"]["initial_linear_velocity_m_s"][0]=1.;
    rejects([&]{m710::validateNominalDrop(bad,binding);});
    bad=request;for(auto& s:bad["nominal_release"]["samples"])s["box"]["pose"][1][3]=3.;
    bad["process_policy"]["target"]["pose"][1][3]=3.;
    rejects([&]{m710::validateNominalDrop(bad,binding);});
  }
  std::cout<<"PASS synchronized native world samples="<<checked<<" old geometry rejected; release pose, velocity, footprint guards\n";
}
