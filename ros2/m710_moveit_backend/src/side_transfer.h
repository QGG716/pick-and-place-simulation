#pragma once
#include "process_geometry.h"
#include <fcl/fcl.h>

namespace m710 {
inline double sideSurfaceGap(const ProcessBox& a,const ProcessBox& b) {
  auto sa=std::make_shared<fcl::Boxd>(2*a.half.x(),2*a.half.y(),2*a.half.z());
  auto sb=std::make_shared<fcl::Boxd>(2*b.half.x(),2*b.half.y(),2*b.half.z());
  fcl::CollisionObjectd oa(sa,a.pose),ob(sb,b.pose);fcl::DistanceResultd result;
  const double d=fcl::distance(&oa,&ob,fcl::DistanceRequestd(false),result);
  if(!std::isfinite(d))throw std::runtime_error("SIDE_TRANSFER_DISTANCE_UNKNOWN");return d;
}
inline double sideBottom(const ProcessBox& b) {return b.pose.translation().z()-b.pose.linear().row(2).cwiseAbs().dot(b.half);}
inline double sideTop(const ProcessBox& b) {return b.pose.translation().z()+b.pose.linear().row(2).cwiseAbs().dot(b.half);}
inline void validateSideHistory(const ProcessJson& history,const std::map<std::string,ProcessBox>& world,
                                const std::set<std::string>& stack,const std::string& target) {
  if(history.at("schema")!="right_inner_planar_history_v1")throw std::runtime_error("SIDE_TRANSFER_SCHEMA");
  const auto original=processBox(history.at("original_target"));
  if(original.id!=target)throw std::runtime_error("SIDE_TRANSFER_TARGET_IDENTITY");
  std::set<std::string> expected;
  for(const auto& n:stack)if(n!=target) {
    const auto& b=world.at(n);
    if(std::abs(sideTop(b)-sideBottom(original))<=1e-9 &&
        (b.pose.linear().col(2)-Eigen::Vector3d::UnitZ()).cwiseAbs().maxCoeff()<=1e-12)expected.insert(n);
  }
  if(expected!=history.at("planar_support_ids").get<std::set<std::string>>())
    throw std::runtime_error("SIDE_TRANSFER_PLANAR_GEOMETRY_MISMATCH");
  const auto touched=history.at("touched_ids").get<std::set<std::string>>();
  for(const auto& n:touched)if(n==target||!stack.count(n))throw std::runtime_error("SIDE_TRANSFER_HISTORY_IDENTITY");
  for(const auto& n:history.at("released_ids").get<std::set<std::string>>())
    if(!touched.count(n))throw std::runtime_error("SIDE_TRANSFER_RELEASE_WITHOUT_CONTACT");
}
inline ProcessJson sideState(const ProcessBox& target,const std::map<std::string,ProcessBox>& world,
    const std::set<std::string>& stack,ProcessJson& history,double gap,double tolerance,bool advance,size_t* queries=nullptr) {
  const auto planar=history.at("planar_support_ids").get<std::set<std::string>>();
  auto touched=history.at("touched_ids").get<std::set<std::string>>();
  auto released=history.at("released_ids").get<std::set<std::string>>();
  for(const auto& name:stack)if(name!=target.id) {
    const auto& other=world.at(name);if(queries)++*queries;
    const double d=sideSurfaceGap(target,other),signed_gap=processSignedGap(target,other);
    if(d+1e-9>=gap) {if(touched.count(name))released.insert(name);continue;}
    std::string reason;
    if(released.count(name))reason="SIDE_TRANSFER_RELEASED_PAIR_REENTRY";
    else if(!planar.count(name))reason="SIDE_TRANSFER_NONPLANAR_CLEARANCE";
    else if(signed_gap < -tolerance || sideBottom(target)<sideTop(other)-tolerance)
      reason="SIDE_TRANSFER_SUPPORT_PENETRATION";
    if(!reason.empty())return {{"reason",reason},{"pair",{target.id,name}},{"signed_distance_m",signed_gap},{"surface_distance_m",d}};
    touched.insert(name);
  }
  if(advance){history["touched_ids"]=touched;history["released_ids"]=released;}
  return nullptr;
}
}
