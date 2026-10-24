#include "nominal_release.h"
#include <fstream>
#include <cassert>
#include <iostream>
using J=nlohmann::json;
template<class F> void rejects(F f){bool failed=false;try{f();}catch(const std::exception&){failed=true;}assert(failed);}
int main(int argc,char**argv){
  assert(argc==2);std::ifstream input(argv[1]);J data;input>>data;
  if(data.contains("l01_hmax")) {
    assert(data.at("l01_hmax").size()==5);
    for(const auto& item:data.at("l01_hmax")) {
      const auto& b=item.at("binding");const auto& req=item.at("request");
      m710::validateCoupledBinding(b);m710::validateReceiveHeight(req,b);
      auto bad=b;bad["receive_contract"].erase("l01_hmax_permission");
      rejects([&]{m710::validateReceiveHeight(req,bad);});
      bad=b;bad["receive_contract"]["l01_hmax_permission"]["allowed_target_ids"].push_back("carton_l00_c01");
      rejects([&]{m710::validateReceiveHeight(req,bad);});
      bad=b;bad["receive_contract"]["l01_hmax_permission"]["original_cartons"].erase(0);
      rejects([&]{m710::validateReceiveHeight(req,bad);});
      bad=b;bad["receive_contract"]["initial_target"]["pose"][1][3]=9.;
      rejects([&]{m710::validateReceiveHeight(req,bad);});
      auto changed=req;changed["planning_only"]=false;
      rejects([&]{m710::validateReceiveHeight(changed,b);});
      changed=req;changed["process_policy"]["target"]["pose"][1][3]=9.;
      rejects([&]{m710::validateReceiveHeight(changed,b);});
      bad=b;bad["receive_contract"]["height_m"]=.3;
      rejects([&]{m710::validateReceiveHeight(req,bad);});
      bad=b;bad["samples"][0]["height_m"]=.3;
      rejects([&]{m710::validateReceiveHeight(req,bad);});
    }
    std::cout<<"PASS frozen five-carton Hmax scope; legacy Hmin, geometry, Y, height, stationary and planning-only guards\n";
    return 0;
  }
  size_t checked=0;
  for(const auto& b:data.at("bindings")){
    m710::validateCoupledBinding(b);
    J request={{"planning_only",true},{"planning_mode","cold_from_scratch"},{"planner_id","COUPLED_LIFT"},
      {"q_start",{0,0,0,0,0,0}},{"q_goal",{0,0,0,0,0,0}},{"attachment",nullptr},{"ownership_transition",nullptr},
      {"parent_stage_id","actual-parent"},{"stage","pregrasp"},{"world",b.at("samples").front().at("world")},
      {"receiver_end_world",b.at("samples").back().at("world")},{"process_policy",{{"target_id",b.at("target_id")}}}};
    assert(m710::coupledTransitionWorld(request,b)==request.at("receiver_end_world"));
    auto receive=b;receive["receive_contract"]={{"height_m",b.at("samples").back().at("height_m")},{"junction_front",false}};
    auto extracting=request;extracting["stage"]="extraction";extracting["world"]=b.at("samples").back().at("world");
    m710::validateReceiveHeight(extracting,receive);
    auto wrong=receive;wrong["receive_contract"]["height_m"]=99.;
    rejects([&]{m710::validateReceiveHeight(extracting,wrong);});
    auto missing=extracting;missing["world"].erase(missing["world"].begin());
    rejects([&]{m710::validateReceiveHeight(missing,receive);});
    auto mismatch=request;mismatch["q_goal"][0]=.01;rejects([&]{m710::coupledTransitionWorld(mismatch,b);});
    auto held=request;held["attachment"]={{"id",b.at("target_id")}};held["stage"]="extraction";
    held["process_policy"]["initial_proximity"]=nullptr;held["process_policy"]["support_names"]=J::array();
    for(const auto key:{"world","receiver_end_world"}){J filtered=J::array();for(const auto& box:held[key])if(box.at("id")!=b.at("target_id"))filtered.push_back(box);held[key]=filtered;}
    rejects([&]{m710::coupledTransitionWorld(held,b);});
    held["process_policy"]["support_names"]={"neighbor"};rejects([&]{m710::coupledTransitionWorld(held,b);});
    for(const auto& s:b.at("samples")){assert(m710::coupledWorldFailure(s,b).is_null());++checked;}
    auto bad=b;bad["samples"][0]["world"][0]["size"][0]=0.;rejects([&]{m710::validateCoupledBinding(bad);});
  }
  const auto& binding=data.at("bindings")[0];
  if(binding.value("geometry_revision","")=="upper_seam_lower_slider_v3") {
    auto sample=binding.at("samples").back();J trans,longitudinal;
    for(const auto& b:sample.at("world")) {
      if(b.at("id")=="conveyor_transverse"){trans=b;m710::validateSolid(b);}
      if(b.at("id")=="conveyor_longitudinal")longitudinal=b;
    }
    assert(trans.contains("solid"));assert(m710::solidSignedGap(trans,longitudinal)>=-1e-9);
    for(auto& b:sample["world"])if(b.at("id")=="conveyor_longitudinal")b["pose"][0][3]=b["pose"][0][3].get<double>()+.02;
    assert(!m710::coupledWorldFailure(sample,binding).is_null());
  }
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
