#pragma once
#include <Eigen/Geometry>
#include <nlohmann/json.hpp>
#include <set>
#include <stdexcept>
#include <cmath>

namespace m710 {
inline void validateSealContours(const nlohmann::json& geometry) {
  if(geometry.at("geometry_type")!="cad_seal_contours_v1")
    throw std::runtime_error("CONTACT_GEOMETRY_TYPE_MISMATCH");
  const auto& rows=geometry.at("contours");const auto& expected=geometry.at("expected_contour_ids");
  if(!rows.is_array() || rows.size()!=12 || expected.size()!=12)
    throw std::runtime_error("CONTACT_CONTOUR_IDENTITY_OR_COMPLETENESS");
  std::set<std::string> ids;
  for(size_t i=0;i<rows.size();++i) {
    const auto& row=rows.at(i);const std::string id=row.at("id");
    if(id.empty() || !ids.insert(id).second || row.at("id")!=expected.at(i))
      throw std::runtime_error("CONTACT_CONTOUR_IDENTITY_OR_COMPLETENESS");
    const auto nv=row.at("normal").get<std::vector<double>>();
    if(nv.size()!=3) throw std::runtime_error("CONTACT_CONTOUR_NORMAL_INVALID");
    const Eigen::Vector3d n(nv[0],nv[1],nv[2]);
    if(!n.allFinite() || std::abs(n.norm()-1)>1e-8)
      throw std::runtime_error("CONTACT_CONTOUR_NORMAL_INVALID");
    const auto& points=row.at("points_m");
    if(!points.is_array() || points.size()<3) throw std::runtime_error("CONTACT_CONTOUR_GEOMETRY_INVALID");
    std::vector<Eigen::Vector3d> p;Eigen::Vector3d area=Eigen::Vector3d::Zero();
    for(const auto& point:points) {
      auto v=point.get<std::vector<double>>();
      if(v.size()!=3) throw std::runtime_error("CONTACT_CONTOUR_GEOMETRY_INVALID");
      Eigen::Vector3d x(v[0],v[1],v[2]);
      if(!x.allFinite()) throw std::runtime_error("CONTACT_CONTOUR_GEOMETRY_INVALID");
      for(const auto& old:p) if((x-old).norm()<1e-12) throw std::runtime_error("CONTACT_CONTOUR_DUPLICATE_VERTEX");
      p.push_back(x);
    }
    for(size_t j=0;j<p.size();++j) {
      if(std::abs((p[j]-p[0]).dot(n))>1e-7) throw std::runtime_error("CONTACT_CONTOUR_NONPLANAR");
      area+=(p[j]-p[0]).cross(p[(j+1)%p.size()]-p[0]);
    }
    if(area.norm()<1e-12) throw std::runtime_error("CONTACT_CONTOUR_DEGENERATE");
  }
}

// Called with geometry validated and bound at import, never hashes in this loop.
inline nlohmann::json checkSealContours(const nlohmann::json& geometry,
    const Eigen::Isometry3d& local,const Eigen::Vector3d& half,
    double gap,double penetration,double margin,double angle) {
  for(double x:{gap,penetration,margin,angle})
    if(!std::isfinite(x)||x<0)throw std::runtime_error("CONTACT_TOLERANCE_INVALID");
  if(angle>=std::acos(-1.)/2)throw std::runtime_error("CONTACT_TOLERANCE_INVALID");
  const std::string face=geometry.value("target_face",std::string("top"));
  if(face!="top" && face!="front")throw std::runtime_error("CONTACT_TARGET_FACE_INVALID");
  const int axis=face=="top"?2:0;const double sign=face=="top"?1.:-1.;
  const auto active=geometry.contains("active_ids")?geometry.at("active_ids"):geometry.at("expected_contour_ids");
  std::set<std::string> enabled;const auto expected=geometry.at("expected_contour_ids").get<std::set<std::string>>();
  if(!active.is_array() || active.empty())throw std::runtime_error("CONTACT_ACTIVE_SELECTION_INVALID");
  for(const auto& id:active)if(!id.is_string() || !expected.count(id.get<std::string>()) || !enabled.insert(id.get<std::string>()).second)
    throw std::runtime_error("CONTACT_ACTIVE_SELECTION_INVALID");
  nlohmann::json actual=nlohmann::json::array(),ordered=nlohmann::json::array();
  nlohmann::json results=nlohmann::json::array();bool passed=true;
  for(const auto& row:geometry.at("contours")) {
    const auto nv=row.at("normal").get<std::vector<double>>();
    const Eigen::Vector3d normal=local.linear()*Eigen::Vector3d(nv[0],nv[1],nv[2]);
    bool valid=-sign*normal[axis]>=std::cos(angle);double lo=INFINITY,hi=-INFINITY,reserve=INFINITY;
    for(const auto& point:row.at("points_m")) {
      const auto p=point.get<std::vector<double>>();
      const Eigen::Vector3d x=local*Eigen::Vector3d(p[0],p[1],p[2]);
      const double d=sign*x[axis]-half[axis];lo=std::min(lo,d);hi=std::max(hi,d);
      for(int k=0;k<3;++k)if(k!=axis)reserve=std::min(reserve,half[k]-std::abs(x[k]));
    }
    const bool contact=valid && lo>=-penetration-1e-12 && hi<=gap+1e-12;
    valid=contact && reserve>=margin-1e-12;
    const bool on=enabled.count(row.at("id").get<std::string>());
    if(on){passed=passed&&valid;ordered.push_back(row.at("id"));}
    if(valid)actual.push_back(row.at("id"));
    results.push_back({{"id",row.at("id")},{"valid",valid},{"min_gap_m",lo},{"max_gap_m",hi},{"edge_margin_m",reserve},
      {"active",on},{"face_contact",contact}});
  }
  return {{"status",passed?"PASS":"FAIL"},{"contours",results},{"checked_count",results.size()},
    {"target_face",face},{"active_ids",ordered},{"geometrically_eligible_ids",actual},{"actual_sealed_ids",actual},
    {"contact_observation","predicted_geometry_not_physical_contact"}};
}
}
