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
inline constexpr const char* kEcoCaptureJ1Policy="RAW_J1_FIRST_CCW_OUTBOUND_CW_RETURN_CCW_TOTAL_REVERSE_1E6_V3";
inline constexpr const char* kEcoFreeJ1Policy="RAW_FINITE_JOINTS_BIDIRECTIONAL_NO_J1_MONOTONICITY_V4";

// A stopped outfeed may transmit the immutable world once. Only the named
// released target's pose is mutable; each pose is still checked by native FCL.
// Legacy full sample requests retain their original contract in the worker.
class EcoHeldOutfeedSamples {
  CycleJson held_,poses_,first_,last_;
  size_t target_index_=0;
public:
  explicit EcoHeldOutfeedSamples(const CycleJson& request):held_(request) {
    const auto& transition=request.at("transition");
    if(transition.at("event_type")!="IDEAL_OUTFEED" ||
       transition.at("sample_contract")!="HELD_WORLD_TARGET_POSES_V2" ||
       !transition.at("sample_requests").empty())
      throw std::runtime_error("ECO65_OUTFEED_COMPACT_CONTRACT");
    poses_=transition.at("outfeed").at("samples");
    if(!poses_.is_array() || poses_.size()<2 || poses_.size()>256)
      throw std::runtime_error("ECO65_OUTFEED_BINDING_OR_SAMPLE_COUNT");
    const auto target=request.at("process_policy").at("target_id");
    size_t found=0;
    for(size_t k=0;k<held_.at("world").size();++k)if(held_["world"][k].at("id")==target){target_index_=k;++found;}
    if(found!=1)throw std::runtime_error("ECO65_OUTFEED_TARGET_REQUIRED");
    first_=held_["world"][target_index_].at("pose");
    found=0;
    for(const auto& row:transition.at("after_request").at("world"))if(row.at("id")==target){last_=row.at("pose");++found;}
    if(found!=1 || poses_.front().at("pose")!=first_ || poses_.back().at("pose")!=last_)
      throw std::runtime_error("ECO65_OUTFEED_SAMPLE_ENDPOINT_WORLD_CHANGED");
    held_.erase("transition");
  }
  size_t size() const {return poses_.size();}
  const CycleJson& sample(size_t index) {
    const auto& pose=poses_.at(index).at("pose");
    const auto actual=processTransform(pose),initial=processTransform(first_);
    if(!actual.matrix().allFinite() ||
       (actual.linear()-initial.linear()).cwiseAbs().maxCoeff()>1e-12)
      throw std::runtime_error("ECO65_OUTFEED_ORIENTATION_CHANGED");
    held_["world"][target_index_]["pose"]=pose;
    held_["process_policy"]["target"]["pose"]=pose;
    return held_;
  }
};

inline bool ecoCycleContactPreservingLift(const CycleJson& prior,const CycleJson& attach) {
  if(prior.value("op",std::string())!="eco65_transition" || !prior.contains("transition"))return false;
  const auto& event=prior.at("transition");
  if(event.value("event_type",std::string())!="COUPLED_LIFT" || !event.contains("after_request"))return false;
  const auto& after=event.at("after_request");
  for(const auto* state:{&prior,&after}) {
    if(state->at("q_start")!=attach.at("q_start") || !state->at("attachment").is_null() ||
       state->at("process_policy").at("selection_binding")!=attach.at("process_policy").at("selection_binding"))return false;
  }
  return true;
}
inline constexpr double kEcoCycleReverseLimit=1e-6;
inline std::string ecoCycleNextInterval(const CycleJson& req,const std::string& event) {
  const std::string goal=req.value("completion_goal",std::string());
  if(!goal.empty() && goal!="ALL_CARTONS_DRAINED")
    throw std::runtime_error("ECO65_CYCLE_COMPLETION_GOAL_INVALID");
  const std::string interval=req.at("business_interval");
  if(event=="ATTACH" && (interval=="FIRST_APPROACH" || goal=="ALL_CARTONS_DRAINED"))return "OUTBOUND";
  return event=="RELEASE"?"RETURN":interval;
}
inline int ecoCycleDirection(const CycleJson& req) {
  if(req.value("continuous_task_mode",std::string())==kEcoCycleMode &&
      req.value("j1_direction_policy",std::string())==kEcoFreeJ1Policy) {
    const auto interval=req.value("business_interval",std::string());
    if((interval!="FIRST_APPROACH"&&interval!="OUTBOUND"&&interval!="RETURN") ||
        req.value("j1_direction",2)!=0 || req.value("j1_reverse_used_rad",-1.)!=0. ||
        req.value("j1_reverse_limit_rad",-1.)!=0.)throw std::runtime_error("ECO65_FREE_JOINT_POLICY_INVALID");
    return 0;
  }
  if(req.value("continuous_task_mode",std::string())!=kEcoCycleMode ||
      (req.value("j1_direction_policy",std::string())!=kEcoCycleJ1Policy &&
       req.value("j1_direction_policy",std::string())!=kEcoCaptureJ1Policy) ||
      !req.contains("business_interval") || !req.contains("j1_direction") ||
      req.value("j1_reverse_limit_rad",-1.)!=kEcoCycleReverseLimit)
    throw std::runtime_error("ECO65_BUSINESS_DIRECTION_REQUIRED");
  const std::string interval=req.at("business_interval");
  const int first=req.at("j1_direction_policy")==kEcoCaptureJ1Policy?1:-1;
  const int direction=interval=="FIRST_APPROACH"?first:interval=="OUTBOUND"?-1:interval=="RETURN"?1:2;
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
    if(cartons.size()!=binding.value("inventory_count",12u) || cartons.empty() ||
        !cartons.count(binding.at("initial_target_id").get<std::string>()) ||
        moving.empty() || fixed.empty() || supports.empty())
      throw std::runtime_error("ECO65_CYCLE_INVENTORY_REQUIRED");
    for(const auto& id:moving)if(!initial_world.count(id)||fixed.count(id))
      throw std::runtime_error("ECO65_CYCLE_MOVING_INVENTORY_INVALID");
    for(const auto& id:fixed)if(!initial_world.count(id))
      throw std::runtime_error("ECO65_CYCLE_FIXED_INVENTORY_INVALID");
  }
  void checkOption(const CycleJson& req) const {
    if(req.at("transverse_process_policy")!=binding.at("transverse_process_policy"))
      throw std::runtime_error("ECO65_TRANSVERSE_POLICY_CHANGED");
    const auto& option=req.at("process_option");
    const std::string target=req.at("process_policy").at("target_id");
    if(option.is_null() || option.at("target_id")!=target || !cartons.count(target) ||
        option.at("face")!=req.at("process_policy").at("target_face"))
      throw std::runtime_error("ECO65_TRANSVERSE_OPTION_REQUIRED");
    const auto& frozen=cartons.at(target);const auto& spec=binding.at("mechanism_spec");
    const double bottom=frozen.at("pose").at(2).get<double>()-frozen.at("size_m").at(2).get<double>()/2;
    const double size=frozen.at("size_m").at(2),floor=spec.at("config").at("floor_z_m");
    const std::string name=option.at("template"),face=option.at("face"),level=option.at("level");
    const std::map<std::string,std::pair<std::string,std::string>> types={
      {"FRONT_DIRECT_HIGH",{"front","HIGH"}}, {"FRONT_DIRECT_LOW",{"front","LOW"}},
      {"FRONT_JUNCTION_LOW",{"front","LOW"}}, {"FRONT_LATERAL_LOW",{"front","LOW"}},
      {"LEFT_INNER_LOW",{"right","LOW"}}, {"RIGHT_INNER_HIGH",{"left","HIGH"}},
      {"TOP_FIXED_MINIMUM_RECEIVER",{"top","MINIMUM"}},
      {"TOP_LIFTED_TRANSVERSE_RECEIVER",{"top","LIFTED"}}};
    const bool prototype=binding.at("transverse_process_policy").at("schema")=="eco65_quarter_transverse_templates_v3";
    if(!types.count(name)||types.at(name)!=std::make_pair(face,level)||
        (face=="top" && !prototype && !(bottom-floor<.100)) ||
        (level=="LIFTED" && !prototype))throw std::runtime_error("ECO65_TRANSVERSE_TEMPLATE_OR_TOP_THRESHOLD");
    const double minimum=spec.at("low_height_m");
    const double drop=binding.at("transverse_process_policy").at("desired_drop_m");
    const double top_height=std::max(minimum,std::min(spec.at("high_height_m").get<double>(),bottom));
    const double height=level=="LIFTED"?top_height:level=="MINIMUM"?minimum:bottom-drop-(level=="LOW"?size:0.);
    const double release=face=="top"?std::max(bottom+drop,height+drop):bottom;
    if(std::abs(option.at("height_m").get<double>()-height)>1e-9 ||
        std::abs(option.at("release_bottom_world_m").get<double>()-release)>1e-9 ||
        std::abs(option.at("expected_drop_m").get<double>()-(release-height))>1e-9 ||
        height<minimum-1e-12 || height>spec.at("high_height_m").get<double>()+1e-12)
      throw std::runtime_error("ECO65_TRANSVERSE_FROZEN_HEIGHT_INVALID");
    const bool attached=!req.at("attachment").is_null();
    const std::string event=req.contains("transition")?req.at("transition").value("event_type",std::string()):"";
    if((attached||event=="ATTACH") &&
        std::abs(req.at("mechanism_state").at("height_m").get<double>()-height)>1e-8)
      throw std::runtime_error("EXTRACTION_BEFORE_REQUIRED_RECEIVE_HEIGHT");
    if(attached && event=="COUPLED_LIFT")throw std::runtime_error("RECEIVER_SYNC_AFTER_ATTACHMENT_DISABLED");
  }
  void checkRequest(const CycleJson& req) const {
    ecoCycleDirection(req);
    const bool process=binding.contains("transverse_process_policy");
    if(process != (req.at("j1_direction_policy")==kEcoFreeJ1Policy))
      throw std::runtime_error("ECO65_CYCLE_STRATEGY_IDENTITY_CHANGED");
    if(!req.contains("process_policy"))return; // IK has no world authority.
    if(process)checkOption(req);
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
    const double length=spec.at("longitudinal_length_m"),rise=h-spec.value("longitudinal_lower_height_m",spec.at("low_height_m").get<double>());
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
// A separating plane of the actual convex solids proves the permitted contact
// tolerance without libccd EPA. EPA aborts on nearly coplanar carton/belt faces
// (the captured second-carton landing has a ~1e-10 rad FK rotation). This is
// only for named receiving surfaces, never a robot/tool collision exemption.
// A negative bound >= -t proves translation <= t separates the entire solids.
// Failure to find such a plane is conservative rejection, not penetration proof.
inline double ecoCycleSupportPlaneGap(const shapes::ShapeConstPtr& a,const Eigen::Isometry3d& ta,
                                     const shapes::ShapeConstPtr& b,const Eigen::Isometry3d& tb) {
  std::vector<Eigen::Vector3d> va,vb,axes;
  auto collect=[&](const shapes::ShapeConstPtr& shape,const Eigen::Isometry3d& t,
                   std::vector<Eigen::Vector3d>& vertices) {
    if(shape->type==shapes::BOX) {
      const auto* box=static_cast<const shapes::Box*>(shape.get());
      for(int x:{-1,1})for(int y:{-1,1})for(int z:{-1,1})
        vertices.push_back(t*Eigen::Vector3d(x*box->size[0]/2,y*box->size[1]/2,z*box->size[2]/2));
      for(int k=0;k<3;++k)axes.push_back(t.linear().col(k));
    } else if(shape->type==shapes::MESH) {
      const auto* mesh=static_cast<const shapes::Mesh*>(shape.get());
      for(size_t i=0;i<mesh->vertex_count;++i)
        vertices.push_back(t*Eigen::Vector3d(mesh->vertices[3*i],mesh->vertices[3*i+1],mesh->vertices[3*i+2]));
      for(size_t i=0;i<mesh->triangle_count;++i) {
        const Eigen::Vector3d normal=(vertices[mesh->triangles[3*i+1]]-vertices[mesh->triangles[3*i]]).cross(
          vertices[mesh->triangles[3*i+2]]-vertices[mesh->triangles[3*i]]);
        if(normal.norm()>1e-12)axes.push_back(normal.normalized());
      }
    } else throw std::runtime_error("ECO65_SUPPORT_PLANE_SOLID_UNSUPPORTED");
  };
  collect(a,ta,va);collect(b,tb,vb);double gap=-INFINITY;
  for(const auto& axis:axes) {
    double amin=INFINITY,amax=-INFINITY,bmin=INFINITY,bmax=-INFINITY;
    for(const auto& v:va){const double x=axis.dot(v);amin=std::min(amin,x);amax=std::max(amax,x);}
    for(const auto& v:vb){const double x=axis.dot(v);bmin=std::min(bmin,x);bmax=std::max(bmax,x);}
    gap=std::max(gap,std::max(amin-bmax,bmin-amax));
  }
  if(!std::isfinite(gap))throw std::runtime_error("ECO65_SUPPORT_PLANE_UNKNOWN");return gap;
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
      const double limit=supports.count(id)&&bottom?-.0004:.0024;
      const bool contact=limit<0;
      const double d=contact?ecoCycleSupportPlaneGap(object->shapes_[0],worldShapePose(*object,0),other->shapes_[k],worldShapePose(*other,k)):
        ecoCycleSolidDistance(object->shapes_[0],worldShapePose(*object,0),other->shapes_[k],worldShapePose(*other,k));
      if(nativeExpired(deadline))return {{"reason","NATIVE_ABSOLUTE_DEADLINE_EXPIRED"}};
      if(d+1e-9<limit)return {{"reason","ECO65_CYCLE_RELEASED_TARGET_COLLISION"},{"pair",{target,id}},
        {"distance_method",contact?"CONSERVATIVE_CONVEX_SUPPORT_PLANE_BOUND":"FCL_SIGNED_DISTANCE"},
        {"surface_distance_m",d},{"required_pair_clearance_m",limit}};
    }
  }
  return nullptr;
}

inline CycleJson ecoCycleOutfeedWorld(const planning_scene::PlanningScenePtr& scene,const CycleJson& req,
    const CycleJson& binding,const CycleJson& tools,const NativeDeadlinePtr& deadline) {
  if(!binding.contains("prototype"))throw std::runtime_error("ECO65_OUTFEED_PROTOTYPE_BINDING_REQUIRED");
  const auto& spec=binding.at("mechanism_spec");const auto& mechanism=req.at("mechanism_state");
  const auto p=processBox(req.at("process_policy").at("target"));
  if((p.pose.linear()-Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff()>1e-6)
    throw std::runtime_error("ECO65_OUTFEED_LEVEL_ORIENTATION_REQUIRED");
  const Eigen::Vector3d lo=p.pose.translation()-p.half,hi=p.pose.translation()+p.half;
  const auto& receiver=req.at("process_policy").at("task_contract").at("receiving");
  const auto& bounds=receiver.at("bounds_D_m");
  // The request carries height/spec identity, not a second mutable mechanism.
  // Endpoints follow the bound rigid-length relation used by Python geometry.
  const double height=mechanism.at("height_m"),ux=spec.at("transverse_xmin_m"),
    lz=spec.at("longitudinal_lower_height_m"),length=spec.at("longitudinal_length_m");
  const double lx=ux-std::sqrt(length*length-(height-lz)*(height-lz));
  double support=-INFINITY;std::set<std::string> contacts;
  if(std::min(hi.x(),bounds.at(1).at(0).get<double>())-std::max(lo.x(),bounds.at(0).at(0).get<double>())>1e-9 &&
      lo.y()>=bounds.at(0).at(1).get<double>()+.01-1e-9 && hi.y()<=bounds.at(1).at(1).get<double>()-.01+1e-9) {
    support=height;contacts=receiver.at("support_ids").get<std::set<std::string>>();
  }
  const double y=spec.at("longitudinal_center_y_m"),half=spec.at("longitudinal_width_m").get<double>()/2;
  if(lo.y()>=y-half+.01-1e-9 && hi.y()<=y+half-.01+1e-9) {
    const double a=std::max(lo.x(),lx),b=std::min(hi.x(),ux);
    if(b-a>1e-9) {
      const double slope=(height-lz)/(ux-lx);support=std::max(support,height+((slope>=0?b:a)-ux)*slope);
      for(const auto& row:spec.at("belt_templates"))if(row.at("owner")=="conveyor_longitudinal")contacts.insert(row.at("id").get<std::string>());
    }
    if(std::min(hi.x(),lx)-std::max(lo.x(),lx-spec.at("config").at("discharge_tray_length_m").get<double>())>1e-9) {
      support=std::max(support,lz);contacts.insert("eco65_discharge_tray");
    }
  }
  if(!std::isfinite(support)||std::abs(lo.z()-support)>1e-6)
    throw std::runtime_error("ECO65_OUTFEED_ACTUAL_SUPPORT_ENVELOPE_INVALID");
  // The carton may touch, or separate by < 2.4 mm from, the three named
  // receiving surfaces throughout the checked conveyance. Leaving a support
  // edge does not instantly require an engineering gap. Penetration remains
  // checked and every frame/hinge/enclosure object keeps the normal margin.
  for(const auto& row:spec.at("belt_templates"))contacts.insert(row.at("id").get<std::string>());
  contacts.insert("eco65_discharge_tray");
  const auto object=scene->getWorld()->getObject(p.id);
  if(!object||object->shapes_.size()!=1)throw std::runtime_error("ECO65_OUTFEED_WORLD_TARGET_REQUIRED");
  for(const auto& id:scene->getWorld()->getObjectIds())if(id!=p.id) {
    const auto other=scene->getWorld()->getObject(id);
    for(size_t k=0;k<other->shapes_.size();++k) {
      deadline->require();
      const double limit=contacts.count(id)?-.0004:.0024;
      const double distance=contacts.count(id)?ecoCycleSupportPlaneGap(object->shapes_[0],worldShapePose(*object,0),other->shapes_[k],worldShapePose(*other,k)):
        ecoCycleSolidDistance(object->shapes_[0],worldShapePose(*object,0),other->shapes_[k],worldShapePose(*other,k));
      if(distance<limit-1e-9)return {{"reason","ECO65_OUTFEED_WORLD_COLLISION"},{"pair",{p.id,id}},
        {"distance_method",contacts.count(id)?"CONSERVATIVE_CONVEX_SUPPORT_PLANE_BOUND":"FCL_SIGNED_DISTANCE"},
        {"surface_distance_m",distance},{"required_pair_clearance_m",limit}};
    }
  }
  // Held robot/world pairs were checked once by cycleScene. Check the only
  // moving body's full envelope against every actual robot/attachment solid;
  // extraction-only tool/carton permissions do not apply to ideal outfeed.
  for(const auto* link:scene->getRobotModel()->getLinkModelsWithCollisionGeometry()) {
    for(size_t k=0;k<link->getShapes().size();++k) {
      deadline->require();
      const double distance=ecoCycleSolidDistance(link->getShapes()[k],scene->getCurrentState().getCollisionBodyTransform(link,k),
          object->shapes_[0],worldShapePose(*object,0));
      if(distance<.0024-1e-9)return {{"reason","ECO65_OUTFEED_ROBOT_OR_TOOL_COLLISION"},{"pair",{p.id,link->getName()}},
        {"surface_distance_m",distance},{"required_pair_clearance_m",.0024}};
    }
  }
  return nullptr;
}
}
