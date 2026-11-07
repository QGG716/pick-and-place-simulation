#include "coupled_receiver.h"
#include "ideal_outfeed.h"
#include "process_policy.h"
#include <cassert>
#include <fstream>
#include <iostream>
using J=nlohmann::json;
template<class F> void rejects(F f){bool failed=false;try{f();}catch(const std::exception&){failed=true;}assert(failed);}
int main(int argc,char**argv) {
  assert(argc==2);std::ifstream f(argv[1]);J data;f>>data;
  m710::validateGapLayout(data.at("layout"));
  for(const auto& item:data.at("receive")) {
    const auto& req=item.at("request");const auto& binding=item.at("binding");
    m710::validateReceiveHeight(req,binding);m710::validateCoupledBinding(binding);
    for(int i=0;i<5;++i){auto bad=binding;
      if(i==0)bad["receive_contract"]["joint_search_contract"]["release_bottom_m"]=.1;
      if(i==1)bad["receive_contract"]["joint_search_contract"]["mode"]="Hmax";
      if(i==2)bad["joint_search_layout"]["base_xyz_a_m"][0]=.625;
      if(i==3)bad["joint_search_layout"]["maximum_surface_height_m"]=2.1;
      if(i==4)bad["receive_contract"]["joint_search_contract"]["original_bottom_m"]=9.;
      rejects([&]{m710::validateReceiveHeight(req,bad);});
    }
    auto bad=binding;bad["samples"][0]["height_m"]=.54;
    rejects([&]{m710::validateCoupledBinding(bad);});
  }
  for(const auto& req:data.at("outfeeds")) {
    m710::validateWaitOutfeed(req);
    auto bad=req;bad["outfeed"]["finite_support"]["longitudinal_top_x_bounds_m"][0]=-99.;
    rejects([&]{m710::validateWaitOutfeed(bad);});
  }
  Eigen::Isometry3d base=Eigen::Isometry3d::Identity();base.translation()=Eigen::Vector3d(-1.35,.15,1.);
  J req={{"planning_only",true},{"planning_mode","cold_from_scratch"},{"world",J::array({data.at("plinth")})}};
  assert(m710::qualifiedSearchPlinthPair(req,data.at("layout"),base,"base_link","mount_plinth"));
  for(auto link:{"J1_link","J6_link","tool_rigid_0"})assert(!m710::qualifiedSearchPlinthPair(req,data.at("layout"),base,link,"mount_plinth"));
  req["world"][0]["pose"][2][3]=.801;
  assert(!m710::qualifiedSearchPlinthPair(req,data.at("layout"),base,"base_link","mount_plinth"));
  J policy={{"fixed_mount_contact_mode","qualified_planes_only"},{"robot_mount_contact_exemption",false}};
  assert(!m710::authorizedRobotMountPair(policy,"J1_link","chassis"));
  std::cout<<"PASS gap receive/transition range, direct drop, lower raise, finite outfeed and plinth tamper checks\n";
}
