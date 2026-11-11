#pragma once
#include <moveit/planning_scene/planning_scene.h>
#include <nlohmann/json.hpp>
#include "native_robot.h"
namespace m710 {
// Physical contact permission is confined to source-bound terminal skirts and
// a named box. No robot, rigid tool, neighboring carton or unknown exemption.
class SingleCartonContact {
  using J=nlohmann::json;
  J spec_; Eigen::Isometry3d target_, flange_cad_;
  std::vector<Eigen::Vector3d> rings_, sealing_rings_, corners_;
  Eigen::Vector3d normal_,u_,v_,center_,half_;
  bool loaded_,contact_;
  static Eigen::Vector3d vec(const J& j) {
    if(j.size()!=3)throw std::runtime_error("SINGLE_CARTON_INVALID_VECTOR");
    Eigen::Vector3d v(j[0],j[1],j[2]);if(!v.allFinite())throw std::runtime_error("SINGLE_CARTON_NONFINITE");return v;
  }
  static Eigen::Isometry3d tf(const J& j) {
    Eigen::Isometry3d t=Eigen::Isometry3d::Identity();
    for(int i=0;i<4;++i)for(int k=0;k<4;++k)t.matrix()(i,k)=j.at(i).at(k);
    if(!t.matrix().allFinite() || (t.linear().transpose()*t.linear()-Eigen::Matrix3d::Identity()).norm()>1e-7 ||
       std::abs(t.linear().determinant()-1)>1e-7)throw std::runtime_error("SINGLE_CARTON_INVALID_TRANSFORM");
    return t;
  }
 public:
  explicit SingleCartonContact(const J& req):spec_(req.at("single_carton_contact")) {
    if(req.at("identity").value("policy_scope",std::string())!="PHYSICAL_SINGLE_CARTON" || req.contains("process_policy"))
      throw std::runtime_error("SINGLE_CARTON_SCOPE_INVALID");
    const auto& g=spec_.at("geometry");const std::string target=spec_.at("target_id");
    loaded_=!req.at("attachment").is_null();contact_=spec_.at("allow_contact");
    target_=tf(spec_.at("target_pose"));flange_cad_=tf(g.at("flange_from_contact_frame"));
    half_=vec(spec_.at("size_m"))/2;
    if((half_.array()<=0).any())throw std::runtime_error("SINGLE_CARTON_SIZE_INVALID");
    center_=vec(spec_.at("face").at("center_m"));normal_=vec(spec_.at("face").at("normal"));
    u_=vec(spec_.at("face").at("u"));v_=vec(spec_.at("face").at("v"));
    if((center_-(target_*Eigen::Vector3d(0,0,half_.z()))).norm()>1e-7 ||
       (normal_-target_.linear().col(2)).norm()>1e-7 || (u_-target_.linear().col(0)).norm()>1e-7 ||
       (v_-target_.linear().col(1)).norm()>1e-7 ||
       std::abs(double(spec_.at("face").at("size_m")[0])-2*half_.x())>1e-9 ||
       std::abs(double(spec_.at("face").at("size_m")[1])-2*half_.y())>1e-9)
      throw std::runtime_error("SINGLE_CARTON_FACE_NOT_BOUND_TO_BOX");
    bool target_found=false;
    for(const auto& b:req.at("world"))if(b.at("id")==target) {
      target_found=b.at("pose")==spec_.at("target_pose") && b.at("size")==spec_.at("size_m");
    }
    if(!target_found)throw std::runtime_error("SINGLE_CARTON_TARGET_WORLD_MISMATCH");
    J allowed=J::array({J::array({"base_link","robot_mount_plate"})});J lips=J::array();
    const auto ids=spec_.at("active_ids");
    const auto connected=g.at("connected_cup_ids");
    // Legacy requests retain their exact sealing-only permission. The new
    // passive policy explicitly binds every permitted lip to connected CAD.
    const auto contact_ids=spec_.value("target_lip_contact_ids",ids);
    std::set<std::string> unique;
    for(const auto& id:ids)if(!unique.insert(id.get<std::string>()).second)throw std::runtime_error("SINGLE_CARTON_DUPLICATE_CUP");
    if(!g.at("topology").value("uncovered_cups_self_close",false) && ids!=connected)throw std::runtime_error("SINGLE_CARTON_UNVERIFIED_INDEPENDENT_CUPS");
    for(const auto& id:ids)if(std::find(connected.begin(),connected.end(),id)==connected.end())throw std::runtime_error("SINGLE_CARTON_UNKNOWN_CONNECTED_CUP");
    if(ids.empty())throw std::runtime_error("SINGLE_CARTON_EMPTY_CUPS");
    unique.clear();
    for(const auto& id:contact_ids) {
      if(!unique.insert(id.get<std::string>()).second)throw std::runtime_error("SINGLE_CARTON_DUPLICATE_CONTACT_CUP");
      if(std::find(connected.begin(),connected.end(),id)==connected.end())throw std::runtime_error("SINGLE_CARTON_UNKNOWN_CONTACT_CUP");
    }
    for(const auto& id:ids)if(std::find(contact_ids.begin(),contact_ids.end(),id)==contact_ids.end())throw std::runtime_error("SINGLE_CARTON_SEAL_WITHOUT_CONTACT");
    if(!g.at("topology").value("uncovered_cups_self_close",false) && contact_ids!=ids)
      throw std::runtime_error("SINGLE_CARTON_UNVERIFIED_PASSIVE_CONTACT");
    for(const auto& id:contact_ids) {
      const auto lip=g.at("terminal_band_by_contour").at(id.get<std::string>());lips.push_back(lip);
      if(contact_||loaded_)allowed.push_back(J::array({target,lip}));
      bool found=false;
      for(const auto& c:g.at("contours"))if(c.at("id")==id) {
        if(c.at("points_m").size()<3)throw std::runtime_error("SINGLE_CARTON_EMPTY_CAD_SEAL");
        bool sealing=std::find(ids.begin(),ids.end(),id)!=ids.end();
        for(const auto& p:c.at("points_m")) {rings_.push_back(vec(p));if(sealing)sealing_rings_.push_back(vec(p));}found=true;
      }
      if(!found)throw std::runtime_error("SINGLE_CARTON_UNKNOWN_CAD_SEAL");
    }
    if(loaded_) {
      const auto& a=req.at("attachment");
      if(a.at("id")!=target || a.at("size")!=spec_.at("size_m") || a.at("touch_links")!=lips)
        throw std::runtime_error("SINGLE_CARTON_ATTACHMENT_CHANGED");
      target_=tf(a.at("pose"));
      for(int i:{-1,1})for(int j:{-1,1})for(int k:{-1,1})corners_.push_back(Eigen::Vector3d(i*half_[0],j*half_[1],k*half_[2]));
      for(const auto& support:spec_.at("supports")) {
        const std::string id=support.at("id");double top=support.at("top_world_m");bool found=false;
        for(const auto& object:req.at("world"))if(object.at("id")==id) {
          auto T=tf(object.at("pose"));double highest=-INFINITY;
          if(object.contains("convex_mesh"))for(const auto& p:object.at("convex_mesh").at("vertices"))highest=std::max(highest,(T*vec(p)).z());
          else {auto size=vec(object.at("size"))/2;highest=T.translation().z()+T.linear().row(2).cwiseAbs().dot(size);}
          if(std::abs(highest-top)>1e-8)throw std::runtime_error("SINGLE_CARTON_SUPPORT_PLANE_UNBOUND");found=true;
        }
        if(!found)throw std::runtime_error("SINGLE_CARTON_SUPPORT_UNKNOWN");
        allowed.push_back(J::array({target,id}));
      }
    } else if(!spec_.at("supports").empty())throw std::runtime_error("SINGLE_CARTON_UNLOADED_SUPPORT_EXCEPTION");
    if(req.at("allowed_pairs")!=allowed)throw std::runtime_error("SINGLE_CARTON_EXTRA_COLLISION_PERMISSION");
  }
  nlohmann::json check(const moveit::core::RobotState& s) const {
    const auto flange=s.getGlobalLinkTransform(nativeFrames().flange_link);
    if(loaded_) {
      auto box=flange*target_;
      // Fixed flange->payload attachment must keep the anticipated seals on
      // the original +Z box face at every loaded state, not merely at grasp.
      Eigen::Isometry3d T=target_.inverse()*flange_cad_;
      if((T.linear()*vec(spec_.at("geometry").at("normal_cad"))).z()>-std::cos(.01))
        return {{"reason","SINGLE_CARTON_CONTACT_NORMAL_MISALIGNED"}};
      for(const auto& p:rings_)if((T*p).z()<half_.z()-.0004)
        return {{"reason","SINGLE_CARTON_SEAL_PLANE_PENETRATION"}};
      for(const auto& p:sealing_rings_) {
        Eigen::Vector3d d=T*p;
        if(std::abs(d.z()-half_.z())>.0004)return {{"reason","SINGLE_CARTON_LOADED_SEAL_GAP"}};
        if(std::abs(d.x())>half_.x()-.003 || std::abs(d.y())>half_.y()-.003)
          return {{"reason","SINGLE_CARTON_SEAL_EDGE_COVERAGE"}};
      }
      for(const auto& support:spec_.at("supports"))for(const auto& p:corners_)
        if((box*p).z()<double(support.at("top_world_m"))-.0004)
          return {{"reason","SINGLE_CARTON_PAYLOAD_SUPPORT_PENETRATION"},{"support",support.at("id")}};
    } else if(contact_) {
      auto T=flange*flange_cad_;auto size=spec_.at("face").at("size_m");
      if((T.linear()*vec(spec_.at("geometry").at("normal_cad"))).dot(normal_)>-std::cos(.01))
        return {{"reason","SINGLE_CARTON_CONTACT_NORMAL_MISALIGNED"}};
      for(const auto& p:rings_) {
        Eigen::Vector3d d=T*p-center_;double gap=d.dot(normal_);
        if(gap<-.0004)return {{"reason","SINGLE_CARTON_SEAL_PLANE_PENETRATION"}};
      }
      for(const auto& p:sealing_rings_) {
        Eigen::Vector3d d=T*p-center_;double gap=d.dot(normal_);
        if(gap<.0024 && (std::abs(d.dot(u_))>double(size[0])/2-.003 || std::abs(d.dot(v_))>double(size[1])/2-.003))
          return {{"reason","SINGLE_CARTON_SEAL_EDGE_COVERAGE"}};
      }
    }
    return nullptr;
  }
};
}
