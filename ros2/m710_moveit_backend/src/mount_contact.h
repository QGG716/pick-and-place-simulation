#pragma once
#include "process_geometry.h"
#include <moveit/robot_state/robot_state.h>
#include <geometric_shapes/shapes.h>
#include <limits>

namespace m710 {
// The identity is fixed at worker init. Recompute the predicate from actual
// loaded MoveIt mesh vertices, not from client AABBs or a blanket ACM label.
inline ProcessJson fixedBaseContactEvidence(const moveit::core::RobotState& state,
    const ProcessJson& request) {
  using J=ProcessJson;constexpr double eps=1e-10;
  J result={{"passed",false},{"allowed_pairs",J::array()},{"active_links_exempted",J::array()}};
  try {
    const auto& identity=request.at("identity");const auto& c=identity.at("fixed_base_contact_contract");
    if(c.at("schema")!="source_bound_fixed_base_plane_v2" || c.at("numerical_tolerance_m")!=eps ||
       c.at("geometry_permission_scope")!="FIXED_BASE_SUPPORTING_SURFACE_ONLY" ||
       c.at("model_binding")!=identity.at("robot_model_binding"))throw std::runtime_error("MOUNT_IDENTITY_MISMATCH");
    const std::string base=c.at("base_link"),support=c.at("support_object");
    if(base!=c.at("model_binding").at("base_link") || support!="mount_plinth")throw std::runtime_error("MOUNT_ROLE_MISMATCH");
    auto* link=state.getRobotModel()->getLinkModel(base);
    if(!link)throw std::runtime_error("MOUNT_BASE_LINK_MISSING");
    for(auto* current=link;current;) {
      const auto* joint=current->getParentJointModel();
      if(joint && joint->getType()!=moveit::core::JointModel::FIXED)throw std::runtime_error("MOUNT_BASE_IS_MOVABLE");
      current=joint?joint->getParentLinkModel():nullptr;
    }
    const J* actual_box=nullptr;
    for(const auto& item:request.at("world"))if(item.at("id")==support) {
      if(actual_box)throw std::runtime_error("MOUNT_DUPLICATE_SUPPORT");actual_box=&item;
    }
    if(!actual_box || actual_box->value("category","")!="mount_structure" || actual_box->contains("solid"))throw std::runtime_error("MOUNT_SUPPORT_MISSING_OR_INVALID");
    const auto box=processBox(*actual_box),expected_box=processBox(c.at("expected_plinth"));
    const auto wb=state.getGlobalLinkTransform(base),bm=processTransform(c.at("model_base_from_cad_support_frame"));
    const auto reference=processTransform(c.at("expected_world_from_installation_reference"));
    const auto declared=processTransform(c.at("declared_installation_reference_from_model_base"));
    const Eigen::Isometry3d expected=reference*declared,measured_frame=wb*bm;
    if((declared.matrix()-processTransform(c.at("model_binding").at("mounting_from_base")).matrix()).cwiseAbs().maxCoeff()>eps ||
        (expected.matrix()-processTransform(c.at("expected_world_from_model_base")).matrix()).cwiseAbs().maxCoeff()>eps)
      throw std::runtime_error("MOUNT_DECLARED_MODEL_BASE_IDENTITY_MISMATCH");
    const auto lb=box.pose.inverse()*wb;
    auto vector=[](const J& v){if(v.size()!=3)throw std::runtime_error("MOUNT_VECTOR_SIZE");Eigen::Vector3d r(v[0].get<double>(),v[1].get<double>(),v[2].get<double>());if(!r.allFinite())throw std::runtime_error("MOUNT_VECTOR_FINITE");return r;};
    const auto point=vector(c.at("plane_point_base_m")),up=vector(c.at("plane_up_base"));
    const auto axis=vector(c.at("first_joint_axis_base"));
    if(std::abs(up.norm()-1)>1e-8 || std::abs(axis.norm()-1)>1e-8)throw std::runtime_error("MOUNT_NORMAL_OR_AXIS_INVALID");
    if((bm.translation()-point).cwiseAbs().maxCoeff()>eps || (bm.linear().col(2)-up).cwiseAbs().maxCoeff()>eps)
      throw std::runtime_error("MOUNT_CAD_SUPPORT_FRAME_INCONSISTENT");
    auto matrix=[](const Eigen::Isometry3d& t){J out=J::array();for(int r=0;r<4;++r){J row=J::array();for(int k=0;k<4;++k)row.push_back(t.matrix()(r,k));out.push_back(row);}return out;};
    J checks={{"source_geometry_verified",c.value("source_geometry_verified",false)},
      {"support_geometry_unchanged",(box.pose.matrix()-expected_box.pose.matrix()).cwiseAbs().maxCoeff()<=eps && (box.half-expected_box.half).cwiseAbs().maxCoeff()<=eps/2},
      {"declared_model_base_pose_unchanged",(wb.matrix()-expected.matrix()).cwiseAbs().maxCoeff()<=eps},
      {"measured_axis_on_installation_reference",(measured_frame.translation()-reference.translation()).cwiseAbs().maxCoeff()<=eps && std::abs((wb.linear()*axis).dot(reference.linear().col(2)))>=1-eps},
      {"measured_normal_matches_installation_reference",(wb.linear()*up).dot(reference.linear().col(2))>=1-eps},
      {"opposed_support_normals",(wb.linear()*up).dot(box.pose.linear().col(2))>=1-eps},
      {"measured_plane_is_support_top",std::abs((lb*point).z()-box.half.z())<=eps}};
    double minimum=std::numeric_limits<double>::infinity(),area=0;size_t patch_count=0,source_matches=0;
    bool nonpenetration=true,covered=true;
    const auto& shapes=link->getShapes();const auto& origins=link->getCollisionOriginTransforms();
    if(shapes.empty() || shapes.size()!=c.at("base_mesh_sources").size())throw std::runtime_error("MOUNT_BASE_MESH_COUNT_CHANGED");
    for(size_t i=0;i<shapes.size();++i) {
      if(shapes[i]->type!=shapes::MESH)throw std::runtime_error("MOUNT_BASE_NOT_MESH");
      const auto* mesh=static_cast<const shapes::Mesh*>(shapes[i].get());
      std::vector<Eigen::Vector3d> vertices;vertices.reserve(mesh->vertex_count);
      for(size_t j=0;j<mesh->vertex_count;++j) {
        const Eigen::Vector3d v=origins[i]*Eigen::Vector3d(mesh->vertices[3*j],mesh->vertices[3*j+1],mesh->vertices[3*j+2]);
        if(!v.allFinite())throw std::runtime_error("MOUNT_MESH_NONFINITE");vertices.push_back(v);
        const double gap=(lb*v).z()-box.half.z();minimum=std::min(minimum,gap);double allowance=0;
        for(const auto& record:c.at("source_vertex_allowances")) {
          const auto source=vector(record.at("vertex_base_m"));
          const Eigen::Array3d tolerance=4*double(std::numeric_limits<float>::epsilon())*source.cwiseAbs().cwiseMax(1e-6).array()+1e-12;
          if(((v-source).cwiseAbs().array()<=tolerance).all()) {
            const double depth=record.at("maximum_source_plane_undershoot_m");
            if(!std::isfinite(depth)||depth<=0 || std::abs(depth+(source-point).dot(up))>eps)throw std::runtime_error("MOUNT_SOURCE_POINT_ALLOWANCE_INVALID");
            allowance=depth;++source_matches;
          }
        }
        if(gap < -allowance-eps)nonpenetration=false;
      }
      for(size_t j=0;j<mesh->triangle_count;++j) {
        const auto& a=vertices.at(mesh->triangles[3*j]);const auto& b=vertices.at(mesh->triangles[3*j+1]);const auto& d=vertices.at(mesh->triangles[3*j+2]);
        const Eigen::Vector3d normal=(b-a).cross(d-a);const double twice=normal.norm();
        if(twice<=0 || std::abs((a-point).dot(up))>eps || std::abs((b-point).dot(up))>eps || std::abs((d-point).dot(up))>eps || normal.dot(up)/twice>=-1+1e-8)continue;
        area+=twice/2;++patch_count;
        for(const auto& v:{a,b,d}) {const auto p=lb*v;if(std::abs(p.x())>box.half.x()+eps || std::abs(p.y())>box.half.y()+eps || std::abs(p.z()-box.half.z())>eps)covered=false;}
      }
    }
    checks["no_unexplained_base_penetration"]=nonpenetration;
    checks["full_contact_region_supported"]=covered&&patch_count>0;
    checks["positive_source_contact_area"]=std::isfinite(area)&&area>0;
    bool passed=true;J failures=J::array();for(auto it=checks.begin();it!=checks.end();++it)if(!it.value().get<bool>()){passed=false;failures.push_back(it.key());}
    result.update({{"passed",passed},{"checks",checks},{"failures",failures},{"minimum_raw_mesh_support_distance_m",minimum},
      {"contact_area_m2",area},{"contact_triangle_count",patch_count},{"matched_source_vertices",source_matches},
      {"installation_verified",c.at("installation_verified")},{"installation_scope",c.at("installation_scope")},
      {"world_from_installation_reference",c.at("expected_world_from_installation_reference")},
      {"expected_world_from_model_base",c.at("expected_world_from_model_base")},
      {"actual_world_from_base",matrix(wb)},{"actual_world_from_cad_support_frame",matrix(measured_frame)},
      {"cad_support_tangent_axes_are_not_installation_clocking",true},
      {"formal_task_readiness_not_implied",true},{"geometry_provider","ACTUAL_MOVEIT_COLLISION_MESH_VERTICES_AND_TRIANGLES"}});
    if(passed)result["allowed_pairs"].push_back({base,support});
  } catch(const std::exception& error) {result["failure"]=error.what();}
  return result;
}
inline bool qualifiedConfiguredPlinthPair(const moveit::core::RobotState& state,const ProcessJson& request,
    const std::string& a,const std::string& b) {
  if(!request.contains("identity"))return false;
  const auto& identity=request.at("identity");
  if(!identity.contains("fixed_base_contact_contract"))return false;
  const auto& c=identity.at("fixed_base_contact_contract");
  if(!((a==c.at("base_link")&&b=="mount_plinth")||(b==c.at("base_link")&&a=="mount_plinth")))return false;
  return fixedBaseContactEvidence(state,request).at("passed").get<bool>();
}
}
