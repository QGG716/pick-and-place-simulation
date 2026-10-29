#include "ideal_outfeed.h"
#include "process_policy.h"
#include <fstream>
#include <cassert>
#include <iostream>
using J=nlohmann::json;
template<class F> void rejects(F f){bool failed=false;try{f();}catch(const std::exception&){failed=true;}assert(failed);}
int main(int argc,char**argv){
  assert(argc==2);std::ifstream f(argv[1]);J data;f>>data;
  Eigen::Isometry3d base=Eigen::Isometry3d::Identity();base.translation()=Eigen::Vector3d(-1.225,0.,.7);
  J mount={{"schema","fixed40_joint_search_20261028_v1"},{"mount_design","SOLID_PLINTH"},{"base_xyz_world_m",{-1.225,0.,.7}}};
  J box={{"id","mount_plinth"},{"category","mount_structure"},{"size",{.5635,.55,.1}},
    {"pose",{{1.,0.,0.,-1.28175},{0.,1.,0.,0.},{0.,0.,1.,.65},{0.,0.,0.,1.}}}};
  J request={{"planning_only",true},{"planning_mode","cold_from_scratch"},{"world",J::array({box})}};
  assert(m710::qualifiedSearchPlinthPair(request,mount,base,"base_link","mount_plinth"));
  for(int i=0;i<7;++i){auto bad=request;auto bad_mount=mount;auto pose=base;
    if(i==0)bad["world"][0]["size"][0]=.6;
    if(i==1)bad["world"][0]["pose"][2][3]=.651;
    if(i==2)bad["world"][0]["category"]="chassis";
    if(i==3)bad["planning_only"]=false;
    if(i==4)bad_mount["schema"]="legacy";
    if(i==5)pose.translation().x()+=.001;
    if(i==6){bad["world"][0]["pose"][0][0]=0.;bad["world"][0]["pose"][0][1]=-1.;
      bad["world"][0]["pose"][1][0]=1.;bad["world"][0]["pose"][1][1]=0.;}
    assert(!m710::qualifiedSearchPlinthPair(bad,bad_mount,pose,"base_link","mount_plinth"));
  }
  for(const auto& link:{"J1_link","J2_link","tool_rigid_0","tool_compliant_bellows_0"})
    assert(!m710::qualifiedSearchPlinthPair(request,mount,base,link,"mount_plinth"));
  for(const auto& request:data.at("outfeeds")) {
    m710::validateWaitOutfeed(request);
    J drop={{"planner_id","NOMINAL_DROP_RELEASE"},{"pipeline_id","semantic_event"},{"stage","extraction"},{"stage_id","drop"}};
    m710::validateWaitOutfeedParent(request,drop,nullptr);
    auto alternative=request;alternative["outfeed"]["tool_timing"]="WITHDRAW_THEN_WAIT_FULL_OUTFEED";
    J withdrawal={{"planner_id","LIN"},{"pipeline_id","pilz_industrial_motion_planner"},
      {"stage","withdrawal"},{"attachment",nullptr},{"parent_stage_id","drop"},{"stage_id","withdraw"}};
    m710::validateWaitOutfeed(alternative);m710::validateWaitOutfeedParent(alternative,withdrawal,drop);
    rejects([&]{m710::validateWaitOutfeedParent(alternative,drop,nullptr);});
    for(int i=0;i<4;++i){auto bad=withdrawal;
      if(i==0)bad["planner_id"]="PTP";
      if(i==1)bad["stage"]="pregrasp";
      if(i==2)bad["attachment"]=J::object();
      if(i==3)bad["parent_stage_id"]="unrelated";
      rejects([&]{m710::validateWaitOutfeedParent(alternative,bad,drop);});
    }
    J binding={{"outfeed_park_q_rad",{0.,0.,0.,0.,0.,0.}},
      {"joint_search_layout",{{"schema","fixed40_joint_search_20261028_v1"}}}};
    J park={{"purpose","FREE_UNLOADED_REPOSITION"},{"stage","residence"},{"planning_only",true},
      {"planning_mode","cold_from_scratch"},{"attachment",nullptr},{"q_goal",{0.,0.,0.,0.,0.,0.}},
      {"pipeline_id","ompl"},{"planner_id","RRTConnectkConfigDefault"},{"parent_stage_id","withdraw"},
      {"process_policy",{{"support_names",J::array()},{"initial_proximity",nullptr}}}};
    m710::validateOutfeedPark(park,withdrawal,drop,binding);
    alternative["outfeed"]["tool_timing"]="WITHDRAW_PARK_THEN_WAIT_FULL_OUTFEED";
    m710::validateWaitOutfeed(alternative);m710::validateWaitOutfeedParent(alternative,park,withdrawal,drop);
    rejects([&]{m710::validateWaitOutfeedParent(alternative,withdrawal,drop);});
    for(int i=0;i<6;++i){auto bad=park;
      if(i==0)bad["q_goal"][0]=.1;
      if(i==1)bad["stage"]="withdrawal";
      if(i==2)bad["attachment"]=J::object();
      if(i==3)bad["process_policy"]["initial_proximity"]=J::object();
      if(i==4)bad["process_policy"]["support_names"]=J::array({"chassis"});
      if(i==5)bad["planning_only"]=false;
      rejects([&]{m710::validateOutfeedPark(bad,withdrawal,drop,binding);});
    }
    auto bad=request;bad["q_goal"][0]=.01;rejects([&]{m710::validateWaitOutfeed(bad);});
    bad=request;bad["outfeed"]["samples"][1]["box"]["pose"][0][3]=9.;rejects([&]{m710::validateWaitOutfeed(bad);});
    bad=request;bad["outfeed"]["samples"][1]["t"]=0.;rejects([&]{m710::validateWaitOutfeed(bad);});
    bad=request;bad["outfeed"]["samples"].erase(bad["outfeed"]["samples"].begin()+2,bad["outfeed"]["samples"].end());rejects([&]{m710::validateWaitOutfeed(bad);});
    bad=request;bad["outfeed"]["tool_timing"]="WITHDRAW_SIMULTANEOUSLY";rejects([&]{m710::validateWaitOutfeed(bad);});
  }
  for(const auto& item:data.at("receive")) {
    const auto& request=item.at("request");const auto& binding=item.at("binding");
    m710::validateReceiveHeight(request,binding);
    auto bad=binding;bad["receive_contract"]["height_m"]=.1;rejects([&]{m710::validateReceiveHeight(request,bad);});
    bad=binding;bad["receive_contract"]["joint_search_contract"]["minimum_m"]=.1;rejects([&]{m710::validateReceiveHeight(request,bad);});
    bad=binding;bad["receive_contract"]["joint_search_contract"]["mode"]="CLAMPED";rejects([&]{m710::validateReceiveHeight(request,bad);});
    if(binding.at("receive_contract").at("joint_search_contract").at("mode")=="LIFTED_TOP_EXPLICIT") {
      for(int i=0;i<5;++i){bad=binding;auto& p=bad["receive_contract"]["joint_search_contract"];
        if(i==0)p["face"]="x-";
        if(i==1)p["explicit_receive_height_m"]=.7;
        if(i==2)p.erase("explicit_receive_height_m");
        if(i==3)p["mode"]="Hmax";
        if(i==4)p["release_bottom_m"]=.05;
        rejects([&]{m710::validateReceiveHeight(request,bad);});
      }
    }
  }
  std::cout<<"PASS native joint-search receive bounds and full outfeed timing tamper checks\n";
}
