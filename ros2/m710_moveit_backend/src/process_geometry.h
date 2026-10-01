#pragma once
#include <Eigen/Geometry>
#include <nlohmann/json.hpp>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>
#include <set>
#include <vector>

namespace m710 {
using ProcessJson = nlohmann::json;
inline void validateNativeMotionScope(const ProcessJson& request) {
  const std::string stage=request.at("stage");
  const std::set<std::string> constrained={"contact","contact_endpoint","next_contact","support-release","extraction","place","placement","withdrawal"};
  if(constrained.count(stage) && (request.at("pipeline_id")!="pilz_industrial_motion_planner" ||
      request.at("planner_id")!="LIN" || !request.contains("goal_pose")))
    throw std::runtime_error("NATIVE_PROCESS_REQUIRES_TASK_TCP_LIN_GOAL");
}
struct ProcessBox {
  std::string id;
  Eigen::Isometry3d pose=Eigen::Isometry3d::Identity();
  Eigen::Vector3d half=Eigen::Vector3d::Zero();
};
inline Eigen::Isometry3d processTransform(const ProcessJson& j) {
  if(!j.is_array() || j.size()!=4) throw std::runtime_error("INVALID_PROCESS_TRANSFORM");
  Eigen::Isometry3d t=Eigen::Isometry3d::Identity();
  for(int r=0;r<4;++r) {if(!j.at(r).is_array() || j.at(r).size()!=4) throw std::runtime_error("INVALID_PROCESS_TRANSFORM");
    for(int c=0;c<4;++c) t.matrix()(r,c)=j.at(r).at(c).get<double>();}
  if(!t.matrix().allFinite() || (t.matrix().row(3)-Eigen::RowVector4d(0,0,0,1)).norm()>1e-12 ||
     (t.linear().transpose()*t.linear()-Eigen::Matrix3d::Identity()).norm()>1e-8 ||
     std::abs(t.linear().determinant()-1)>1e-8) throw std::runtime_error("INVALID_PROCESS_TRANSFORM");
  return t;
}
inline ProcessBox processBox(const ProcessJson& j) {
  ProcessBox b; b.id=j.at("id");b.pose=processTransform(j.at("pose"));
  const auto size=j.at("size").get<std::vector<double>>();
  if(size.size()!=3) throw std::runtime_error("INVALID_PROCESS_BOX");
  for(int i=0;i<3;++i) {if(!std::isfinite(size[i]) || size[i]<=0) throw std::runtime_error("INVALID_PROCESS_BOX");b.half[i]=size[i]/2;}
  return b;
}
// The same uninflated, normalized 15-axis SAT measure used by the core's
// path-dependent compression/stack monitor. It is not a Euclidean clearance.
inline double processSignedGap(const ProcessBox& a,const ProcessBox& b) {
  double gap=-std::numeric_limits<double>::infinity();
  const auto delta=b.pose.translation()-a.pose.translation();
  auto axisGap=[&](Eigen::Vector3d axis) {
    const double norm=axis.norm();if(norm<=1e-12) return;axis/=norm;
    const double radiusA=(a.pose.linear().transpose()*axis).cwiseAbs().dot(a.half);
    const double radiusB=(b.pose.linear().transpose()*axis).cwiseAbs().dot(b.half);
    gap=std::max(gap,std::abs(axis.dot(delta))-radiusA-radiusB);
  };
  for(int i=0;i<3;++i) {axisGap(a.pose.linear().col(i));axisGap(b.pose.linear().col(i));}
  for(int i=0;i<3;++i) for(int k=0;k<3;++k) axisGap(a.pose.linear().col(i).cross(b.pose.linear().col(k)));
  return gap;
}
inline bool processSupportSeparated(const ProcessBox& box,const ProcessBox& support,double tolerance) {
  const Eigen::Isometry3d local=support.pose.inverse()*box.pose;
  const double lowest=local.translation().z()-local.linear().row(2).cwiseAbs().dot(box.half);
  return lowest>=support.half.z()-tolerance;
}
inline ProcessJson processSealRings(const ProcessJson& policy,const ProcessBox& target,const Eigen::Isometry3d& flange) {
  const auto& cups=policy.at("cups");
  const auto& active=policy.at("commanded_active_mask");
  if(cups.size()!=72 || active.size()!=72) throw std::runtime_error("PROCESS_REQUIRES_72_CUPS");
  for(const auto key:{"geometrically_eligible_mask","actual_contact_mask"}) {
    if(policy.at(key).size()!=72) throw std::runtime_error("PROCESS_REQUIRES_72_MASK_BITS");
    for(const auto& bit:policy.at(key)) if(!bit.is_boolean()) throw std::runtime_error("PROCESS_MASK_NOT_BOOLEAN");
  }
  std::map<std::string,std::pair<int,double>> faces={ {"front",{0,-1}}, {"left",{1,1}}, {"right",{1,-1}}, {"top",{2,1}} };
  const auto face=policy.at("target_face").get<std::string>();
  if(!faces.count(face)) throw std::runtime_error("PROCESS_TARGET_FACE_INVALID");
  const auto [axis,sign]=faces.at(face);
  const Eigen::Isometry3d local=target.pose.inverse()*flange*processTransform(policy.at("flange_from_physical_contact"));
  const Eigen::Vector3d ray=local.linear().col(2);
  const double denom=ray[axis], alignment=-sign*denom;
  double gap=policy.at("max_attachment_gap_m"),penetration=policy.at("maximum_penetration_m"),
    angle=policy.at("max_normal_misalignment_rad"),margin=policy.at("suction_edge_margin_m");
  for(double x:{gap,penetration,angle,margin}) if(!std::isfinite(x)||x<0) throw std::runtime_error("PROCESS_SEAL_TOLERANCE_INVALID");
  if(angle>=std::acos(-1.)/2) throw std::runtime_error("PROCESS_SEAL_ANGLE_INVALID");
  ProcessJson eligible=ProcessJson::array(),contact=ProcessJson::array();size_t commanded=0;std::set<std::string> ids;
  for(size_t i=0;i<72;++i) {
    const auto& cup=cups.at(i);if(cup.at("index").get<size_t>()!=i || !ids.insert(cup.at("cup_id").get<std::string>()).second)
      throw std::runtime_error("PROCESS_CUP_IDENTITY_INVALID");
    if(!active.at(i).is_boolean()) throw std::runtime_error("PROCESS_MASK_NOT_BOOLEAN");
    const auto p=cup.at("center_contact_frame_m").get<std::vector<double>>();double radius=cup.at("seal_radius_m");
    if(p.size()!=3 || !std::isfinite(radius)||radius<=0) throw std::runtime_error("PROCESS_CUP_GEOMETRY_INVALID");
    const Eigen::Vector3d center=local*Eigen::Vector3d(p[0],p[1],p[2]);
    if(!center.allFinite()) throw std::runtime_error("PROCESS_CUP_GEOMETRY_INVALID");
    bool valid=alignment>1e-12 && alignment>=std::cos(angle);
    if(valid) {
      const double distance=(sign*target.half[axis]-center[axis])/denom;
      const double amplitude=radius*std::hypot(local.linear()(axis,0),local.linear()(axis,1))/std::abs(denom);
      valid=distance-amplitude>=-penetration-1e-12 && distance+amplitude<=gap+1e-12;
      const Eigen::Vector3d projected=center+distance*ray;
      for(int k=0;k<3;++k) if(k!=axis) {
        const double u=local.linear()(k,0)-local.linear()(axis,0)*ray[k]/denom;
        const double v=local.linear()(k,1)-local.linear()(axis,1)*ray[k]/denom;
        valid=valid && std::abs(projected[k])+radius*std::hypot(u,v)+margin<=target.half[k]+1e-12;
      }
    }
    const bool on=active.at(i).get<bool>();commanded+=on;eligible.push_back(valid);contact.push_back(on&&valid);
    if(on && (!policy.at("geometrically_eligible_mask").at(i).get<bool>() || !valid))
      return {{"reason","COMMANDED_CUP_SEAL_INVALID"},{"cup_index",i},{"cup_id",cup.at("cup_id")},{"computed_eligible",eligible}};
  }
  if(commanded==0) return {{"reason","NO_COMMANDED_CUP"}};
  return {{"status","PASS"},{"computed_geometrically_eligible_mask",eligible},{"computed_actual_contact_mask",contact},{"commanded_count",commanded},
    {"contact_observation","predicted_robot_state_geometry_not_physical_contact"}};
}
}
