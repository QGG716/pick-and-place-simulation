#pragma once
#include "process_geometry.h"
#include "scene_geometry.h"
#include "joint_direction.h"
#include "native_deadline.h"
#include <moveit/planning_scene/planning_scene.h>
#include <fcl/fcl.h>
#include <geometric_shapes/shapes.h>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace m710 {
using CycleJson=nlohmann::json;
inline constexpr const char* kEcoCycleMode="eco65_coupled_cycle_v1";
inline constexpr const char* kEcoCycleJ1Policy="RAW_J1_FIRST_CW_OUTBOUND_CW_RETURN_CCW_TOTAL_REVERSE_1E6_V2";
inline constexpr double kEcoCycleReverseLimit=1e-6;
inline int ecoCycleDirection(const CycleJson& req) {
  if(req.value("continuous_task_mode",std::string())!=kEcoCycleMode ||
      req.value("j1_direction_policy",std::string())!=kEcoCycleJ1Policy ||
      !req.contains("business_interval") || !req.contains("j1_direction") ||
      req.value("j1_reverse_limit_rad",-1.)!=kEcoCycleReverseLimit)
    throw std::runtime_error("ECO65_BUSINESS_DIRECTION_REQUIRED");
  const std::string interval=req.at("business_interval");
  const int direction=interval=="FIRST_APPROACH"?-1:interval=="OUTBOUND"?-1:interval=="RETURN"?1:2;
  const double used=req.value("j1_reverse_used_rad",-1.);
  if(direction==2 || req.at("j1_direction")!=direction || !std::isfinite(used) ||
      used<0 || used>kEcoCycleReverseLimit)
    throw std::runtime_error("ECO65_BUSINESS_DIRECTION_INVALID");
  return direction;
}
inline std::map<std::string,CycleJson> ecoCycleObjects(const CycleJson& rows) {
  std::map<std::string,CycleJson> result;
  for(const auto& row:rows)
    if(!result.emplace(row.at("id").get<std::string>(),row).second)
      throw std::runtime_error("ECO65_CYCLE_DUPLICATE_OBJECT");
  return result;
}
inline bool ecoCycleSameShape(CycleJson a,CycleJson b) {
  a.erase("pose");b.erase("pose");return a==b;
}
inline bool ecoCycleSamePose(const CycleJson& a,const CycleJson& b,double epsilon=1e-9) {
  return (processTransform(a).matrix()-processTransform(b).matrix()).cwiseAbs().maxCoeff()<=epsilon;
}
inline bool ecoCycleLevelBox(const Eigen::Isometry3d& pose) {
  // A supported box may yaw in the effective receiver rectangle. Its local
  // +Z must still point upward; processTransform already checks proper SE(3).
  return (pose.linear().col(2)-Eigen::Vector3d::UnitZ()).cwiseAbs().maxCoeff()<=1e-5;
}
class EcoCycleBinding {
public:
  CycleJson binding,process_geometry;
  std::map<std::string,CycleJson> cartons,initial_world;
  std::set<std::string> moving,fixed,supports;
  explicit EcoCycleBinding(const CycleJson& source,const CycleJson& geometry):binding(source),process_geometry(geometry) {
    if(binding.at("schema")!="eco65_coupled_cycle_binding_v1" ||
        binding.at("mechanism_spec").at("geometry_revision")!="upper_seam_lower_slider_v3")
      throw std::runtime_error("ECO65_CYCLE_BINDING_INVALID");
    cartons=ecoCycleObjects(binding.at("original_cartons"));
    initial_world=ecoCycleObjects(binding.at("initial_world"));
    const auto& initial=binding.at("mechanism_initial_state");
    moving=initial.at("mechanism").at("moving_object_ids").get<std::set<std::string>>();
    fixed=initial.at("mechanism").at("fixed_structure_ids").get<std::set<std::string>>();
    supports=initial.at("receiving").at("support_ids").get<std::set<std::string>>();
    if(cartons.size()!=12 || !cartons.count(binding.at("initial_target_id").get<std::string>()) ||
        moving.empty() || fixed.empty() || supports.empty())
      throw std::runtime_error("ECO65_CYCLE_INVENTORY_REQUIRED");
    for(const auto& id:moving)if(!initial_world.count(id)||fixed.count(id))
      throw std::runtime_error("ECO65_CYCLE_MOVING_INVENTORY_INVALID");
    for(const auto& id:fixed)if(!initial_world.count(id))
      throw std::runtime_error("ECO65_CYCLE_FIXED_INVENTORY_INVALID");
  }
  void checkRequest(const CycleJson& req) const {
    ecoCycleDirection(req);
    if(!req.contains("process_policy"))return; // IK has no world authority.
    const auto& p=req.at("process_policy");const auto& task=p.at("task_contract");
    const std::string target=p.at("target_id");
    if(!cartons.count(target) || task.at("target_id")!=target ||
        task.value("continuous_task_mode",std::string())!=kEcoCycleMode ||
        task.at("original_support_names")!=cartons.at(target).at("supported_by"))
      throw std::runtime_error("ECO65_CYCLE_TARGET_CONTRACT_INVALID");
    const auto complete=req.at("completed_ids").get<std::set<std::string>>();
    if(req.at("completed_ids").size()!=complete.size() || task.at("completed_ids")!=req.at("completed_ids"))
      throw std::runtime_error("ECO65_CYCLE_COMPLETION_CONTRACT_INVALID");
    std::set<std::string> remaining;
    for(const auto& item:cartons)if(!complete.count(item.first))remaining.insert(item.first);
    for(const auto& id:complete)if(!cartons.count(id))throw std::runtime_error("ECO65_CYCLE_UNKNOWN_COMPLETED_ID");
    if(task.at("stack_carton_ids").get<std::set<std::string>>()!=remaining ||
        task.at("stack_carton_ids").size()!=remaining.size() ||
        p.at("stack_carton_ids")!=task.at("stack_carton_ids"))
      throw std::runtime_error("ECO65_CYCLE_REMAINING_CONTRACT_INVALID");
    const auto& spec=binding.at("mechanism_spec");
    const auto& mechanism=req.at("mechanism_state");const double h=mechanism.at("height_m");
    if(mechanism.at("spec_fingerprint")!=spec.at("fingerprint") || !std::isfinite(h) ||
        h<spec.at("low_height_m").get<double>()-1e-12 || h>spec.at("high_height_m").get<double>()+1e-12)
      throw std::runtime_error("ECO65_CYCLE_MECHANISM_RANGE_OR_IDENTITY");
    const auto& receiver=task.at("receiving");const auto& init=binding.at("mechanism_initial_state").at("receiving");
    if(receiver.at("region_id")!="conveyor_transverse" || receiver.at("support_ids")!=init.at("support_ids") ||
        receiver.at("margin_m")!=init.at("margin_m"))
      throw std::runtime_error("ECO65_CYCLE_RECEIVER_IDENTITY_CHANGED");
    for(int row=0;row<2;++row)for(int k=0;k<3;++k) {
      double expected=init.at("bounds_D_m").at(row).at(k);
      if(k==2)expected+=h-binding.at("mechanism_initial_state").at("height_m").get<double>();
      if(std::abs(receiver.at("bounds_D_m").at(row).at(k).get<double>()-expected)>1e-9)
        throw std::runtime_error("ECO65_CYCLE_RECEIVER_BOUNDS_CHANGED");
    }
    const auto world=ecoCycleObjects(req.at("world"));
    const auto& attachment=req.at("attachment");
    const std::string attached=attachment.is_null()?"":attachment.at("id").get<std::string>();
    if(!attached.empty() && (attached!=target || complete.count(target) ||
        attachment.at("size")!=cartons.at(target).at("size_m")))
      throw std::runtime_error("ECO65_CYCLE_ATTACHMENT_IDENTITY_INVALID");
    for(const auto& item:initial_world) {
      const auto& id=item.first;
      if(!world.count(id) || !ecoCycleSameShape(item.second,world.at(id)) ||
          (!moving.count(id) && !ecoCycleSamePose(item.second.at("pose"),world.at(id).at("pose"),1e-12)))
        throw std::runtime_error("ECO65_CYCLE_STATIC_OR_SOLID_CHANGED:"+id);
    }
    for(const auto& item:cartons) {
      const auto& id=item.first;const bool expected=!complete.count(id)&&attached!=id;
      if(bool(world.count(id))!=expected)throw std::runtime_error("ECO65_CYCLE_CARTON_OWNERSHIP_INVALID:"+id);
      if(expected && world.at(id).at("size")!=item.second.at("size_m"))
        throw std::runtime_error("ECO65_CYCLE_CARTON_SIZE_CHANGED");
      if(expected && id!=target) {
        auto pose=processTransform(world.at(id).at("pose"));
        Eigen::Isometry3d frozen=Eigen::Isometry3d::Identity();
        for(int k=0;k<3;++k)frozen.translation()[k]=item.second.at("pose").at(k);
        if((pose.matrix()-frozen.matrix()).cwiseAbs().maxCoeff()>1e-12)
          throw std::runtime_error("ECO65_CYCLE_OTHER_CARTON_MOVED");
      }
    }
    if(world.size()!=initial_world.size()+remaining.size()-size_t(!attached.empty()))
      throw std::runtime_error("ECO65_CYCLE_UNKNOWN_WORLD_OBJECT");
    // Belts are rigid about their effective top-surface endpoints. The
    // scissor solids come from the same generator; shape/fixed-foot checks
    // above forbid resizing or moving anchored geometry.
    const double length=spec.at("longitudinal_length_m"),rise=h-spec.at("low_height_m").get<double>();
    const double dx=std::sqrt(length*length-rise*rise),co=dx/length,si=rise/length;
    Eigen::Isometry3d transverse=Eigen::Isometry3d::Identity(),longitudinal=Eigen::Isometry3d::Identity();
    transverse.translation()<<spec.at("transverse_center_xy_m").at(0).get<double>(),
      spec.at("transverse_center_xy_m").at(1).get<double>(),h;
    longitudinal.linear()<<co,0,-si,0,1,0,si,0,co;
    longitudinal.translation()<<spec.at("transverse_xmin_m").get<double>()-dx/2,
      spec.at("longitudinal_center_y_m").get<double>(),h-rise/2;
    for(const auto& row:spec.at("belt_templates")) {
      const auto& expected=row.at("owner")=="conveyor_transverse"?transverse:longitudinal;
      if((processTransform(world.at(row.at("id").get<std::string>()).at("pose")).matrix()-expected.matrix()).cwiseAbs().maxCoeff()>1e-9)
        throw std::runtime_error("ECO65_CYCLE_BELT_RIGID_TRANSFORM_INVALID");
    }
  }
};

inline std::shared_ptr<fcl::CollisionGeometryd> ecoCycleFclShape(const shapes::ShapeConstPtr& shape) {
  if(shape->type==shapes::BOX) {
    const auto* b=static_cast<const shapes::Box*>(shape.get());
    return std::make_shared<fcl::Boxd>(b->size[0],b->size[1],b->size[2]);
  }
  if(shape->type==shapes::CYLINDER) {
    const auto* c=static_cast<const shapes::Cylinder*>(shape.get());return std::make_shared<fcl::Cylinderd>(c->radius,c->length);
  }
  if(shape->type==shapes::MESH) {
    const auto* m=static_cast<const shapes::Mesh*>(shape.get());
    auto vertices=std::make_shared<std::vector<fcl::Vector3d>>();auto faces=std::make_shared<std::vector<int>>();
    for(size_t i=0;i<m->vertex_count;++i)vertices->emplace_back(m->vertices[3*i],m->vertices[3*i+1],m->vertices[3*i+2]);
    for(size_t i=0;i<m->triangle_count;++i){faces->push_back(3);for(int k=0;k<3;++k)faces->push_back(m->triangles[3*i+k]);}
    return std::make_shared<fcl::Convexd>(vertices,m->triangle_count,faces,true);
  }
  throw std::runtime_error("ECO65_CYCLE_UNSUPPORTED_COLLISION_SOLID");
}
inline double ecoCycleSolidDistance(const shapes::ShapeConstPtr& a,const Eigen::Isometry3d& ta,
                                    const shapes::ShapeConstPtr& b,const Eigen::Isometry3d& tb) {
  fcl::CollisionObjectd oa(ecoCycleFclShape(a),ta),ob(ecoCycleFclShape(b),tb);
  fcl::DistanceResultd result;const double d=fcl::distance(&oa,&ob,fcl::DistanceRequestd(true,true),result);
  if(!std::isfinite(d))throw std::runtime_error("ECO65_CYCLE_UNKNOWN_DISTANCE");return d;
}
// MoveIt does not query world/world pairs. A released carton therefore gets
// these finite, actual-solid checks during nominal reception.
inline CycleJson ecoCycleWorldTarget(const planning_scene::PlanningScenePtr& scene,const CycleJson& req,
                                    const NativeDeadlinePtr& deadline=nullptr) {
  const std::string target=req.at("process_policy").at("target_id");
  const auto object=scene->getWorld()->getObject(target);
  if(!object || object->shapes_.size()!=1)throw std::runtime_error("ECO65_CYCLE_WORLD_TARGET_REQUIRED");
  const auto& receiver=req.at("process_policy").at("task_contract").at("receiving");
  const auto supports=receiver.at("support_ids").get<std::set<std::string>>();
  const auto p=processBox(req.at("process_policy").at("target"));
  const auto extent=p.pose.linear().cwiseAbs()*p.half;
  bool full=true;const double h=receiver.at("bounds_D_m").at(1).at(2);
  for(int k=0;k<2;++k)full=full &&
    p.pose.translation()[k]-extent[k]>=receiver.at("bounds_D_m").at(0).at(k).get<double>()+receiver.at("margin_m").get<double>()-1e-6 &&
    p.pose.translation()[k]+extent[k]<=receiver.at("bounds_D_m").at(1).at(k).get<double>()-receiver.at("margin_m").get<double>()+1e-6;
  const bool bottom=full && p.pose.translation().z()-extent.z()>=h-.0004 && ecoCycleLevelBox(p.pose);
  for(const auto& id:scene->getWorld()->getObjectIds())if(id!=target) {
    const auto other=scene->getWorld()->getObject(id);
    for(size_t k=0;k<other->shapes_.size();++k) {
      if(nativeExpired(deadline))return {{"reason","NATIVE_ABSOLUTE_DEADLINE_EXPIRED"}};
      const double d=ecoCycleSolidDistance(object->shapes_[0],worldShapePose(*object,0),other->shapes_[k],worldShapePose(*other,k));
      if(nativeExpired(deadline))return {{"reason","NATIVE_ABSOLUTE_DEADLINE_EXPIRED"}};
      const double limit=supports.count(id)&&bottom?-.0004:.0024;
      if(d+1e-9<limit)return {{"reason","ECO65_CYCLE_RELEASED_TARGET_COLLISION"},{"pair",{target,id}},
        {"surface_distance_m",d},{"required_pair_clearance_m",limit}};
    }
  }
  return nullptr;
}
}
