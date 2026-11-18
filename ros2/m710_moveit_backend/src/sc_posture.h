#pragma once
// Source-bound soft IK ordering for the real-site entry only. No state gate.
#include <nlohmann/json.hpp>
#include <Eigen/Geometry>
#include <array>
#include <algorithm>
#include <map>
#include <string>
#include <stdexcept>
#include <vector>
#include <cmath>

namespace m710 {
inline std::array<double,2> singleCartonPostureKey(const nlohmann::json& policy,
    const std::vector<double>& q,const std::vector<double>& reference) {
  double travel=0.;for(size_t i=0;i<q.size();++i)travel+=(q[i]-reference.at(i))*(q[i]-reference.at(i));
  travel=std::sqrt(travel);if(policy.is_null())return {0.,travel};
  if(policy.at("mode")!="SOFT_LINK_DIRECTIONS"||q.size()!=6||policy.at("chain").size()!=6)
    throw std::runtime_error("POSTURE_PREFERENCE_INVALID");
  auto matrix=[](const nlohmann::json& j){Eigen::Matrix4d t;for(int r=0;r<4;++r)for(int c=0;c<4;++c)t(r,c)=j.at(r).at(c).get<double>();return t;};
  Eigen::Matrix4d t=matrix(policy.at("base"));std::map<std::string,Eigen::Vector3d> points;
  for(size_t i=0;i<6;++i) {
    const auto& j=policy.at("chain").at(i);
    if(j.at("type")!="revolute"||j.at("name")!="joint_"+std::to_string(i+1))throw std::runtime_error("POSTURE_CHAIN_INVALID");
    Eigen::Vector3d axis(j.at("axis").at(0),j.at("axis").at(1),j.at("axis").at(2));
    Eigen::Matrix4d rotation=Eigen::Matrix4d::Identity();rotation.block<3,3>(0,0)=Eigen::AngleAxisd(q[i],axis.normalized()).toRotationMatrix();
    t=(t*matrix(j.at("origin"))*rotation).eval();points[j.at("child").get<std::string>()]=t.block<3,1>(0,3);
  }
  const auto links=policy.at("metric_links").get<std::vector<std::string>>();
  const Eigen::Vector3d upper=(points.at(links.at(1))-points.at(links.at(0))).normalized();
  const Eigen::Vector3d fore=(points.at(links.at(2))-points.at(links.at(1))).normalized();
  const double up=std::clamp(upper.z(),-1.,1.),horizontal=std::clamp(fore.z(),-1.,1.);
  const double score=policy.at("upper_up_weight").get<double>()*(1.-up)+
    policy.at("forearm_horizontal_weight").get<double>()*horizontal*horizontal+
    policy.at("joint_travel_weight").get<double>()*travel;
  if(!std::isfinite(score))throw std::runtime_error("POSTURE_SCORE_INVALID");
  return {std::floor(score*1e9+.5),travel};
}
}
