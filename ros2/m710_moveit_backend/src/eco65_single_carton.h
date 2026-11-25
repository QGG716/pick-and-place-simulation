#pragma once
#include <set>
#include <moveit/planning_scene/planning_scene.h>
#include <Eigen/Eigenvalues>
#include <nlohmann/json.hpp>
#include <tuple>
#include <vector>
#include "native_robot.h"
#include "native_deadline.h"
namespace m710 {
// Physical contact permission is confined to source-bound terminal skirts and
// a named box. Loaded support permission is source-bound with a penetration
// guard; no robot, rigid tool, unnamed neighbour or unknown exemption.
class SingleCartonContact {
  using J=nlohmann::json;
  J spec_; Eigen::Isometry3d target_, flange_cad_;
  std::vector<Eigen::Vector3d> rings_, sealing_rings_, corners_;
  std::vector<std::pair<Eigen::Vector3d,double>> support_planes_;
  Eigen::Vector3d normal_,u_,v_,center_,half_,patch_center_;
  Eigen::Vector2d patch_half_;
  bool loaded_,contact_;
  bool bounded_workspace_=false;
  NativeDeadlinePtr deadline_;
  // Contour bounds (2026-11-23): per CAD contour of rings_ / sealing_rings_,
  // its index range, centre c, unit axis a, in-plane radius r and off-plane
  // height h (every point p: p-c = x + y*a, |x| <= r, |y| <= h). Used only to
  // skip a contour that cannot fail a test by at least kContourBoundMargin;
  // any other contour runs the unchanged per-point test, in the original order.
  struct ContourBound {size_t begin,end;Eigen::Vector3d c,a;double r,h;};
  std::vector<ContourBound> ring_bounds_,sealing_bounds_;
  static constexpr double kContourBoundMargin=1e-9;
  Eigen::Vector3d normal_cad_=Eigen::Vector3d::Zero();double face_size_[2]={0.,0.};
  // Loaded: the seal conditions use target^-1 * flange_cad, which is fixed;
  // their result is computed once with the unchanged code.
  J loaded_seal_;
  Eigen::Vector3d workspace_lo_,workspace_hi_;
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
  static ContourBound bound(const std::vector<Eigen::Vector3d>& points,size_t begin,size_t end) {
    ContourBound b{begin,end,Eigen::Vector3d::Zero(),Eigen::Vector3d::UnitZ(),0.,0.};
    for(size_t i=begin;i<end;++i)b.c+=points[i];b.c/=double(end-begin);
    Eigen::Matrix3d cov=Eigen::Matrix3d::Zero();for(size_t i=begin;i<end;++i)cov+=(points[i]-b.c)*(points[i]-b.c).transpose();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
    if(solver.info()==Eigen::Success)b.a=solver.eigenvectors().col(0).normalized();
    if(!b.a.allFinite()||std::abs(b.a.norm()-1.)>1e-12)b.a=Eigen::Vector3d::UnitZ();
    for(size_t i=begin;i<end;++i) {
      const Eigen::Vector3d q=points[i]-b.c;const double y=q.dot(b.a);
      b.h=std::max(b.h,std::abs(y));b.r=std::max(b.r,(q-y*b.a).norm());
    }
    // Rounding of the bound itself is covered by the 1 nm decision margin.
    return b;
  }
  // Largest |w . (T p - o)| change from the contour centre over the contour:
  // w . R (p - c) with R = T.linear(); wl = R^T w split along the axis.
  static double spread(const ContourBound& b,const Eigen::Vector3d& wl) {
    const double along=wl.dot(b.a);return b.r*(wl-along*b.a).norm()+b.h*std::abs(along);
  }
 public:
  explicit SingleCartonContact(const J& req,NativeDeadlinePtr deadline={}):spec_(req.at("single_carton_contact")),deadline_(std::move(deadline)) {
    if(req.at("identity").value("policy_scope",std::string())!="PHYSICAL_SINGLE_CARTON" || req.contains("process_policy"))
      throw std::runtime_error("SINGLE_CARTON_SCOPE_INVALID");
    const auto& g=spec_.at("geometry");const std::string target=spec_.at("target_id");
    loaded_=!req.at("attachment").is_null();contact_=spec_.at("allow_contact");
    if(req.contains("site_workspace_bounds_m")) {
      workspace_lo_=vec(req.at("site_workspace_bounds_m")[0]);workspace_hi_=vec(req.at("site_workspace_bounds_m")[1]);
      if((workspace_hi_-workspace_lo_).minCoeff()<=0)throw std::runtime_error("INVALID_SITE_WORKSPACE");
      bounded_workspace_=true;
    }
    target_=tf(spec_.at("target_pose"));flange_cad_=tf(g.at("flange_from_contact_frame"));
    half_=vec(spec_.at("size_m"))/2;
    if((half_.array()<=0).any())throw std::runtime_error("SINGLE_CARTON_SIZE_INVALID");
    center_=vec(spec_.at("face").at("center_m"));normal_=vec(spec_.at("face").at("normal"));
    u_=vec(spec_.at("face").at("u"));v_=vec(spec_.at("face").at("v"));
    patch_center_=target_.inverse()*center_;
    patch_half_=Eigen::Vector2d(double(spec_.at("face").at("size_m")[0])/2,
                               double(spec_.at("face").at("size_m")[1])/2);
    const bool partial=spec_.contains("geometry_uncertainty") &&
      spec_.at("geometry_uncertainty").value("kind",std::string())=="BOUNDED_CUBOID_UNION";
    const bool measured_side=spec_.contains("observed_side_envelope");
    if(measured_side) {
      const auto& e=spec_.at("observed_side_envelope");
      const auto front=vec(e.at("front_dimensions_m"));
      if(e.value("source",std::string())!="ALL_ACCEPTED_SIDE_PLANE_SAMPLES_NO_SIZE_PRIOR" ||
         e.at("expansion_m").size()!=2)throw std::runtime_error("SINGLE_CARTON_OBSERVED_SIDE_UNBOUND");
      const double left=e.at("expansion_m")[0],right=e.at("expansion_m")[1];
      if(!std::isfinite(left)||!std::isfinite(right)||left<0||right<0||
         std::abs(front.x()+left+right-2*half_.x())>1e-8 ||
         std::abs(front.y()-2*half_.y())>1e-8 || std::abs(front.z()-2*half_.z())>1e-8 ||
         (front.head<2>()/2-patch_half_).norm()>1e-8 ||
         std::abs(patch_center_.x()-(left-right)/2)>1e-8 || std::abs(patch_center_.y())>1e-8)
        throw std::runtime_error("SINGLE_CARTON_OBSERVED_SIDE_UNBOUND");
    }
    const bool measured=spec_.contains("geometry_uncertainty") &&
      spec_.at("geometry_uncertainty").value("kind",std::string())=="MEASURED_BOUNDARY_INTERVALS";
    if(measured) {
      const auto& m=spec_.at("measured_box");const Eigen::Isometry3d M=target_.inverse()*tf(m.at("pose"));
      const Eigen::Vector3d h=vec(m.at("size_m"))/2;
      if((M.linear()-Eigen::Matrix3d::Identity()).norm()>1e-7 || (h.array()<=0).any() ||
         (M.translation().cwiseAbs()+h-half_).maxCoeff()>1e-7 ||
         (patch_center_-(M*Eigen::Vector3d(0,0,h.z()))).norm()>1e-7 ||
         (patch_half_-h.head<2>()).norm()>1e-9)
        throw std::runtime_error("SINGLE_CARTON_MEASURED_FACE_NOT_BOUND_TO_ENVELOPE");
    }
    if((normal_-target_.linear().col(2)).norm()>1e-7 || (u_-target_.linear().col(0)).norm()>1e-7 ||
       (v_-target_.linear().col(1)).norm()>1e-7 || !patch_half_.allFinite() || (patch_half_.array()<=0).any() ||
       (!measured && std::abs(patch_center_.z()-half_.z())>1e-7) ||
       (patch_center_.head<2>().cwiseAbs()+patch_half_-half_.head<2>()).maxCoeff()>1e-7 ||
       (!partial && !measured && !measured_side && (patch_center_.head<2>().norm()>1e-7 || (patch_half_-half_.head<2>()).norm()>1e-9)))
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
        const size_t ring_begin=rings_.size(),seal_begin=sealing_rings_.size();
        for(const auto& p:c.at("points_m")) {rings_.push_back(vec(p));if(sealing)sealing_rings_.push_back(vec(p));}found=true;
        ring_bounds_.push_back(bound(rings_,ring_begin,rings_.size()));
        if(sealing)sealing_bounds_.push_back(bound(sealing_rings_,seal_begin,sealing_rings_.size()));
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
        const std::string id=support.at("id");bool found=false;
        Eigen::Vector3d support_normal(0.,0.,1.);double top;
        const bool inclined=support.contains("plane");
        // ADJACENT_CONTACT (user decision 2026-11-24): a touching neighbour's
        // side is a vertical contact plane, the same 0.4 mm loaded-payload
        // guard and exact object binding as a support, horizontal normal only.
        const std::string kind=support.value("kind",std::string("SUPPORT"));
        if(kind!="SUPPORT" && (kind!="ADJACENT_CONTACT" || !inclined))
          throw std::runtime_error("SINGLE_CARTON_SUPPORT_KIND_INVALID");
        const bool adjacent=kind=="ADJACENT_CONTACT";
        if(inclined) {
          support_normal=vec(support.at("plane").at("normal_world"));top=support.at("plane").at("offset_m");
          if(std::abs(support_normal.norm()-1.)>1e-8 || !std::isfinite(top) ||
             (adjacent ? std::abs(support_normal.z())>std::sin(10.*M_PI/180.) : support_normal.z()<std::cos(10.*M_PI/180.)))
            throw std::runtime_error("SINGLE_CARTON_SUPPORT_NORMAL_INVALID");
        } else top=support.at("top_world_m");
        for(const auto& object:req.at("world"))if(object.at("id")==id) {
          if(inclined && (object.value("category",std::string())!=(adjacent?"adjacent_contact":"confirmed_stack_support") ||
              object.value("supports_target_id",std::string())!=target ||
              object.value("support_basis",std::string()).empty() ||
              !object.contains("support_plane") || object.at("support_plane")!=support.at("plane")))
            throw std::runtime_error("SINGLE_CARTON_STACK_SUPPORT_IDENTITY_UNBOUND");
          auto T=tf(object.at("pose"));double highest=-INFINITY;
          if(object.contains("convex_mesh"))for(const auto& p:object.at("convex_mesh").at("vertices"))highest=std::max(highest,support_normal.dot(T*vec(p)));
          else {auto size=vec(object.at("size"))/2;highest=support_normal.dot(T.translation())+(T.linear().transpose()*support_normal).cwiseAbs().dot(size);}
          if(std::abs(highest-top)>1e-8)throw std::runtime_error("SINGLE_CARTON_SUPPORT_PLANE_UNBOUND");found=true;
        }
        if(!found)throw std::runtime_error("SINGLE_CARTON_SUPPORT_UNKNOWN");
        support_planes_.emplace_back(support_normal,top);
        allowed.push_back(J::array({target,id}));
      }
    } else if(!spec_.at("supports").empty())throw std::runtime_error("SINGLE_CARTON_UNLOADED_SUPPORT_EXCEPTION");
    // neighbor_lip_contact (user decision 2026-11-24, "允许吸盘碰到邻箱"): the
    // flexible terminal lip of every connected cup may touch the named
    // neighbouring-carton objects; rigid inserts, the tool body and the robot
    // keep every pair rule. Object categories and identity are checked here.
    if(spec_.contains("neighbor_lip_contact")) {
      const auto& nl=spec_.at("neighbor_lip_contact");
      if(nl.value("basis",std::string()).empty())throw std::runtime_error("SINGLE_CARTON_NEIGHBOR_LIP_BASIS_REQUIRED");
      std::set<std::string> kinds={"carton","confirmed_stack_support","adjacent_contact","neighbor_shadow",
                                   "unresolved_carton_volume","observed"};
      // observed_support_basis (user decision 2026-11-25, "允许唇边碰下方箱"):
      // the measured support under the target (category observed_support)
      // joins the named objects only with its own non-empty basis.
      if(nl.contains("observed_support_basis")) {
        if(!nl.at("observed_support_basis").is_string() || nl.at("observed_support_basis").get<std::string>().empty())
          throw std::runtime_error("SINGLE_CARTON_NEIGHBOR_LIP_SUPPORT_BASIS_REQUIRED");
        kinds.insert("observed_support");
      }
      std::vector<std::string> all_lips;
      for(const auto& id:connected)all_lips.push_back(g.at("terminal_band_by_contour").at(id.get<std::string>()));
      for(const auto& oid:nl.at("object_ids")) {
        bool found=false;
        for(const auto& object:req.at("world"))if(object.at("id")==oid) {
          if(oid==target || !kinds.count(object.value("category",std::string())))
            throw std::runtime_error("SINGLE_CARTON_NEIGHBOR_LIP_OBJECT_INVALID");
          found=true;
        }
        if(!found)throw std::runtime_error("SINGLE_CARTON_NEIGHBOR_LIP_OBJECT_UNKNOWN");
        for(const auto& lip:all_lips)allowed.push_back(J::array({oid,lip}));
      }
    }
    if(req.at("allowed_pairs")!=allowed)throw std::runtime_error("SINGLE_CARTON_EXTRA_COLLISION_PERMISSION");
    normal_cad_=vec(spec_.at("geometry").at("normal_cad"));
    face_size_[0]=double(spec_.at("face").at("size_m")[0]);face_size_[1]=double(spec_.at("face").at("size_m")[1]);
    if(loaded_)loaded_seal_=loadedSeal();
  }
  // The state-independent loaded seal conditions, unchanged (see check()).
  J loadedSeal() const {
    Eigen::Isometry3d T=target_.inverse()*flange_cad_;
    if((T.linear()*vec(spec_.at("geometry").at("normal_cad"))).z()>-std::cos(.01))
      return {{"reason","SINGLE_CARTON_CONTACT_NORMAL_MISALIGNED"}};
    for(const auto& p:rings_)if((T*p).z()<patch_center_.z()-.0004)
      return {{"reason","SINGLE_CARTON_SEAL_PLANE_PENETRATION"}};
    for(const auto& p:sealing_rings_) {
      Eigen::Vector3d d=T*p;
      if(std::abs(d.z()-patch_center_.z())>.0004)return {{"reason","SINGLE_CARTON_LOADED_SEAL_GAP"}};
      if(std::abs(d.x()-patch_center_.x())>patch_half_.x()-.003 || std::abs(d.y()-patch_center_.y())>patch_half_.y()-.003)
        return {{"reason","SINGLE_CARTON_SEAL_EDGE_COVERAGE"}};
    }
    return nullptr;
  }
  // Adaptive sent-path audit scope: no contact permission and no support. Then
  // every condition of check() other than the site workspace is either absent
  // (unloaded) or independent of the state (the fixed loaded attachment).
  bool adaptiveEligible() const {return !contact_ && spec_.at("supports").empty();}
  // Site-workspace margin of each link's world box and of the payload corners
  // (>= 0 when inside), with the radius of that point set about its link frame
  // origin. A face of a world-aligned box moves no more than its corners.
  void workspaceMargins(const moveit::core::RobotState& s,
      std::vector<std::tuple<const moveit::core::LinkModel*,double,double>>& out) const {
    out.clear();if(!bounded_workspace_)return;
    for(const auto* link:s.getRobotModel()->getLinkModelsWithCollisionGeometry()) {
      const auto T=s.getGlobalLinkTransform(link);
      const Eigen::Vector3d c=T*link->getCenteredBoundingBoxOffset();
      const Eigen::Vector3d half=T.linear().cwiseAbs()*(link->getShapeExtentsAtOrigin()/2);
      out.emplace_back(link,std::min((c-half-workspace_lo_).minCoeff(),(workspace_hi_-c-half).minCoeff()),
                       link->getCenteredBoundingBoxOffset().norm()+(link->getShapeExtentsAtOrigin()/2).norm());
    }
    if(loaded_) {
      const auto* link=s.getRobotModel()->getLinkModel(nativeFrames().flange_link);
      const auto flange=s.getGlobalLinkTransform(link);double margin=INFINITY,rho=0.;
      for(const auto& p:corners_) {
        const Eigen::Vector3d local=target_*p,world=flange*local;
        margin=std::min(margin,std::min((world-workspace_lo_).minCoeff(),(workspace_hi_-world).minCoeff()));
        rho=std::max(rho,local.norm());
      }
      out.emplace_back(link,margin,rho);
    }
  }
  // exhaustive=true: the per-point reference (every ring point, seal block
  // recomputed); the default path gives the same result (clearance_test).
  nlohmann::json check(const moveit::core::RobotState& s,bool exhaustive=false) const {
    const auto flange=s.getGlobalLinkTransform(nativeFrames().flange_link);
    if(bounded_workspace_) {
      for(const auto* link:s.getRobotModel()->getLinkModelsWithCollisionGeometry()) {
        if(nativeExpired(deadline_))return {{"reason","NATIVE_ABSOLUTE_DEADLINE_EXPIRED"}};
        const auto T=s.getGlobalLinkTransform(link);
        const Eigen::Vector3d c=T*link->getCenteredBoundingBoxOffset();
        const Eigen::Vector3d half=T.linear().cwiseAbs()*(link->getShapeExtentsAtOrigin()/2);
        if((c-half-workspace_lo_).minCoeff()<0 || (workspace_hi_-c-half).minCoeff()<0)
          return {{"reason","ROBOT_OUTSIDE_OBSERVED_OR_MEASURED_SITE_WORKSPACE"},{"link",link->getName()}};
      }
      if(loaded_)for(const auto& p:corners_) {
        Eigen::Vector3d world=flange*target_*p;
        if((world-workspace_lo_).minCoeff()<0 || (workspace_hi_-world).minCoeff()<0)
          return {{"reason","PAYLOAD_UNCERTAINTY_OUTSIDE_SITE_WORKSPACE"}};
      }
    }
    if(loaded_ && !exhaustive) {
      // Fixed flange->payload attachment: the seal conditions do not depend on
      // the state; their unchanged result was computed once (loadedSeal()).
      if(!loaded_seal_.is_null())return loaded_seal_;
      auto box=flange*target_;
      for(std::size_t i=0;i<support_planes_.size();++i)for(const auto& p:corners_)
        if(support_planes_[i].first.dot(box*p)<support_planes_[i].second-.0004)
          return {{"reason","SINGLE_CARTON_PAYLOAD_SUPPORT_PENETRATION"},{"support",spec_.at("supports").at(i).at("id")}};
    } else if(loaded_) {
      auto box=flange*target_;
      // Fixed flange->payload attachment must keep the anticipated seals on
      // the original +Z box face at every loaded state, not merely at grasp.
      Eigen::Isometry3d T=target_.inverse()*flange_cad_;
      if((T.linear()*vec(spec_.at("geometry").at("normal_cad"))).z()>-std::cos(.01))
        return {{"reason","SINGLE_CARTON_CONTACT_NORMAL_MISALIGNED"}};
      for(const auto& p:rings_)if((T*p).z()<patch_center_.z()-.0004)
        return {{"reason","SINGLE_CARTON_SEAL_PLANE_PENETRATION"}};
      for(const auto& p:sealing_rings_) {
        Eigen::Vector3d d=T*p;
        if(std::abs(d.z()-patch_center_.z())>.0004)return {{"reason","SINGLE_CARTON_LOADED_SEAL_GAP"}};
        if(std::abs(d.x()-patch_center_.x())>patch_half_.x()-.003 || std::abs(d.y()-patch_center_.y())>patch_half_.y()-.003)
          return {{"reason","SINGLE_CARTON_SEAL_EDGE_COVERAGE"}};
      }
      for(std::size_t i=0;i<support_planes_.size();++i)for(const auto& p:corners_)
        if(support_planes_[i].first.dot(box*p)<support_planes_[i].second-.0004)
          return {{"reason","SINGLE_CARTON_PAYLOAD_SUPPORT_PENETRATION"},{"support",spec_.at("supports").at(i).at("id")}};
    } else if(contact_ && !exhaustive) {
      auto T=flange*flange_cad_;
      if((T.linear()*normal_cad_).dot(normal_)>-std::cos(.01))
        return {{"reason","SINGLE_CARTON_CONTACT_NORMAL_MISALIGNED"}};
      const Eigen::Matrix3d Rt=T.linear().transpose();
      const Eigen::Vector3d nl=Rt*normal_,ul=Rt*u_,vl=Rt*v_;
      for(const auto& b:ring_bounds_) {
        // Every point's gap is >= the centre's gap - spread: skip only when no point can be below the guard.
        if((T*b.c-center_).dot(normal_)-spread(b,nl)>=-.0004+kContourBoundMargin)continue;
        for(size_t i=b.begin;i<b.end;++i) {
          Eigen::Vector3d d=T*rings_[i]-center_;double gap=d.dot(normal_);
          if(gap<-.0004)return {{"reason","SINGLE_CARTON_SEAL_PLANE_PENETRATION"}};
        }
      }
      for(const auto& b:sealing_bounds_) {
        const Eigen::Vector3d d0=T*b.c-center_;
        if(d0.dot(normal_)-spread(b,nl)>=.0024+kContourBoundMargin)continue;   // gap < 2.4 mm is impossible
        if(std::abs(d0.dot(u_))+spread(b,ul)<=face_size_[0]/2-.003-kContourBoundMargin &&
           std::abs(d0.dot(v_))+spread(b,vl)<=face_size_[1]/2-.003-kContourBoundMargin)continue;   // inside the edge band
        for(size_t i=b.begin;i<b.end;++i) {
          Eigen::Vector3d d=T*sealing_rings_[i]-center_;double gap=d.dot(normal_);
          if(gap<.0024 && (std::abs(d.dot(u_))>face_size_[0]/2-.003 || std::abs(d.dot(v_))>face_size_[1]/2-.003))
            return {{"reason","SINGLE_CARTON_SEAL_EDGE_COVERAGE"}};
        }
      }
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
