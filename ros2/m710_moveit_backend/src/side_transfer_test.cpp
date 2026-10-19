#include "side_transfer.h"
#include <cassert>
#include <iostream>
using namespace m710;
int main(){
  ProcessBox target;target.id="held";target.half={.3,.2,.15};target.pose.translation()=Eigen::Vector3d(.3,-.8,2.25);
  ProcessBox support;support.id="under";support.half=target.half;support.pose.translation()=Eigen::Vector3d(.3,-.8,1.95);
  ProcessBox next=support;next.id="next";next.pose.translation().y()=-.38;
  std::map<std::string,ProcessBox> world{{"under",support},{"next",next}};
  std::set<std::string> stack{"held","under","next"};
  ProcessJson history={{"schema","right_inner_planar_history_v1"},{"planar_support_ids",{"next","under"}},
    {"touched_ids",{"under"}},{"released_ids",ProcessJson::array()}};
  assert(sideState(target,world,stack,history,.0052,.0002,true).is_null());
  target.pose.translation().y()=-.38;
  assert(sideState(target,world,stack,history,.0052,.0002,true).is_null());
  assert(history.at("released_ids")==ProcessJson::array({"under"}));
  auto bad=target;bad.pose.translation().y()=-.8;
  assert(sideState(bad,world,stack,history,.0052,.0002,false).at("reason")=="SIDE_TRANSFER_RELEASED_PAIR_REENTRY");
  bad=target;bad.pose.translation().z()-=.002;
  assert(sideState(bad,world,stack,history,.0052,.0002,false).at("reason")=="SIDE_TRANSFER_SUPPORT_PENETRATION");
  auto obstruction=target;obstruction.id="neighbor";world["neighbor"]=obstruction;stack.insert("neighbor");
  assert(sideState(target,world,stack,history,.0052,.0002,false).at("reason")=="SIDE_TRANSFER_NONPLANAR_CLEARANCE");
  world.erase("neighbor");stack.erase("neighbor");target.pose.translation().x()=-.51;
  assert(sideState(target,world,stack,history,.0052,.0002,true).is_null());
  assert(history.at("released_ids").size()==2);
  std::cout<<"side_transfer_test PASS (synthetic geometry, real FCL)\n";
}
