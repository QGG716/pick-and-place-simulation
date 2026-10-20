// Synthetic protocol/math regressions, never evidence of a planned carton.
#include "eco65_cycle.h"
#include <iostream>
#include <functional>
using J=nlohmann::json;
static void require(bool condition,const char* reason) {
  if(!condition)throw std::runtime_error(reason);
}
static void rejects(const std::function<void()>& f) {
  bool rejected=false;try {f();}catch(const std::exception&){rejected=true;}
  require(rejected,"EXPECTED_PROTOCOL_REJECTION");
}
static J pose(double x=0.,double y=0.,double z=0.) {
  return {{1.,0.,0.,x},{0.,1.,0.,y},{0.,0.,1.,z},{0.,0.,0.,1.}};
}
int main() {
  try {
    J req={{"continuous_task_mode",m710::kEcoCycleMode},{"j1_direction_policy",m710::kEcoCycleJ1Policy},
      {"j1_reverse_used_rad",0.},{"j1_reverse_limit_rad",1e-6},{"business_interval","FIRST_APPROACH"},{"j1_direction",-1}};
    require(m710::ecoCycleDirection(req)==-1,"FIRST_APPROACH_DIRECTION");
    require(m710::directedJ1(1.,.8,m710::ecoCycleDirection(req)),"FIRST_APPROACH_CLOCKWISE_ACCEPT");
    require(!m710::directedJ1(.8,1.,m710::ecoCycleDirection(req)),"FIRST_APPROACH_COUNTERCLOCKWISE_REJECT");
    auto old=req;old["j1_direction"]=0;rejects([&]{m710::ecoCycleDirection(old);});
    req["business_interval"]="OUTBOUND";req["j1_direction"]=-1;
    require(m710::ecoCycleDirection(req)==-1,"OUTBOUND_DIRECTION");
    req["business_interval"]="RETURN";req["j1_direction"]=1;req["stage"]="contact";
    require(m710::ecoCycleDirection(req)==1,"NEXT_CONTACT_MUST_REMAIN_RETURN");
    auto bad=req;bad.erase("business_interval");rejects([&]{m710::ecoCycleDirection(bad);});
    bad=req;bad["j1_direction"]=-1;rejects([&]{m710::ecoCycleDirection(bad);});
    bad=req;bad["j1_reverse_used_rad"]=1.1e-6;rejects([&]{m710::ecoCycleDirection(bad);});
    double reverse=0.,q=0.;
    for(int i=0;i<20;++i){const double next=q-6e-8;require(m710::directedJ1(q,next,1),"PER_EDGE_NUMERICAL_TOLERANCE");
      reverse+=m710::j1ReverseTravel(q,next,1);q=next;}
    require(reverse>m710::kEcoCycleReverseLimit,"SMALL_REVERSE_EDGES_MUST_ACCUMULATE");
    require(m710::j1ReverseTravel(3.,-3.,1)==6.,"RAW_ANGLE_MUST_NOT_WRAP");
    J edges={{"edge_alignment",{{"schema","rigid_tool_face_edge_alignment_v1"},
      {"axes_contact_frame",{{1.,0.,0.},{0.,0.,1.}}},{"tolerance_rad",std::acos(-1.)/180.}}}};
    Eigen::Isometry3d contact=Eigen::Isometry3d::Identity();
    contact.linear()<<1,0,0,0,0,-1,0,1,0;
    require(m710::checkSealEdgeAlignment(edges,contact,2).at("status")=="PASS","CAD_EDGE_DIRECT");
    contact.linear()=Eigen::AngleAxisd(std::acos(-1.)/2,Eigen::Vector3d::UnitZ())*contact.linear();
    require(m710::checkSealEdgeAlignment(edges,contact,2).at("status")=="PASS","CAD_EDGE_SWAPPED");
    contact.linear()=Eigen::AngleAxisd(std::acos(-1.)/4,Eigen::Vector3d::UnitZ())*contact.linear();
    require(m710::checkSealEdgeAlignment(edges,contact,2).at("status")=="FAIL","CAD_EDGE_DIAGONAL_REJECTED");
    auto level=Eigen::Isometry3d::Identity();
    level.linear()=Eigen::AngleAxisd(.7,Eigen::Vector3d::UnitZ()).toRotationMatrix();
    require(m710::ecoCycleLevelBox(level),"PURE_YAW_IS_LEVEL");
    level.linear()=Eigen::AngleAxisd(.01,Eigen::Vector3d::UnitX()).toRotationMatrix();
    require(!m710::ecoCycleLevelBox(level),"TILTED_BOX_IS_NOT_LEVEL");
    level.linear()=Eigen::AngleAxisd(std::acos(-1.),Eigen::Vector3d::UnitX()).toRotationMatrix();
    require(!m710::ecoCycleLevelBox(level),"INVERTED_BOX_IS_NOT_UPRIGHT");
    J world=J::array({{{"id","beltT"},{"size",{.2,.6,.01}},{"pose",pose(0,0,.18)}},
      {{"id","beltL"},{"size",{.5,.2,.01}},{"pose",pose(-.35,-.2,.18)}},
      {{"id","foot"},{"size",{.03,.03,.01}},{"pose",pose(0,0,.005)}}});
    J cartons=J::array();
    for(int i=0;i<12;++i)cartons.push_back({{"id","carton_"+std::to_string(i)},{"pose",{1.+i*.2,0.,.5}},
      {"size_m",{.15,.1,.1}},{"supported_by",{"floor"}}});
    J receiving={{"region_id","conveyor_transverse"},{"support_ids",{"beltT"}},{"margin_m",.01},
      {"bounds_D_m",{{-.1,-.3,.17},{.1,.3,.18}}}};
    J binding={{"schema","eco65_coupled_cycle_binding_v1"},{"original_cartons",cartons},
      {"initial_target_id","carton_0"},{"initial_world",world},
      {"mechanism_spec",{{"geometry_revision","upper_seam_lower_slider_v3"},{"fingerprint","synthetic"},
        {"low_height_m",.18},{"high_height_m",.45},{"longitudinal_length_m",.5},
        {"transverse_center_xy_m",{0.,0.}},{"transverse_xmin_m",-.1},{"longitudinal_center_y_m",-.2},
        {"belt_templates",J::array({{{"id","beltT"},{"owner","conveyor_transverse"}},
                                  {{"id","beltL"},{"owner","conveyor_longitudinal"}}})}}},
      {"mechanism_initial_state",{{"height_m",.18},{"receiving",receiving},
        {"mechanism",{{"moving_object_ids",{"beltT","beltL"}},{"fixed_structure_ids",{"foot"}}}}}}};
    m710::EcoCycleBinding contract(binding,J::object());
    J ids=J::array();
    for(const auto& box:cartons) {ids.push_back(box.at("id"));world.push_back({{"id",box.at("id")},
      {"size",box.at("size_m")},{"pose",pose(box.at("pose").at(0),0,.5)}});}
    req["world"]=world;req["attachment"]=nullptr;req["completed_ids"]=J::array();
    req["mechanism_state"]={{"height_m",.18},{"spec_fingerprint","synthetic"}};
    req["process_policy"]={{"target_id","carton_0"},{"stack_carton_ids",ids},
      {"task_contract",{{"target_id","carton_0"},{"continuous_task_mode",m710::kEcoCycleMode},
        {"original_support_names",{"floor"}},{"completed_ids",J::array()},{"stack_carton_ids",ids},{"receiving",receiving}}}};
    contract.checkRequest(req);
    bad=req;bad["world"][2]["pose"][0][3]=.001;rejects([&]{contract.checkRequest(bad);});
    bad=req;bad["world"][0]["size"][0]=.21;rejects([&]{contract.checkRequest(bad);});
    bad=req;bad["world"][4]["pose"][0][3]=9.;rejects([&]{contract.checkRequest(bad);});
    bad=req;bad["mechanism_state"]["height_m"]=.5;rejects([&]{contract.checkRequest(bad);});
    bad=req;bad["world"][1]["pose"][2][3]=.2;rejects([&]{contract.checkRequest(bad);});
    bad=req;bad["world"].push_back({{"id","unknown"},{"size",{1,1,1}},{"pose",pose()}});
    rejects([&]{contract.checkRequest(bad);});
    std::cout<<J({{"status","PASS"},{"scope","SYNTHETIC_DIRECTION_AND_SCENE_PROTOCOL_ONLY"},
      {"cases",25},{"accumulated_reverse_rad",reverse},{"physical_execution","NOT_RUN"}}).dump()<<std::endl;
    return 0;
  } catch(const std::exception& error) {std::cerr<<error.what()<<std::endl;return 1;}
}
