#pragma once
#include "process_geometry.h"
#include "eco65_distance_policy.h"

namespace m710 {
// One known probe / finite datum experiment. No suction, attachment or hardware.
// Only named installation pairs and the designated tip pair leave ordinary
// clearance; every such pair is checked here using actual FCL solids.
class Eco65TouchProcess {
  struct Solid {std::string name;const moveit::core::LinkModel* link=nullptr;
    Eigen::Isometry3d fixed=Eigen::Isometry3d::Identity();std::shared_ptr<fcl::CollisionGeometryd> shape;};
  struct Pair {Solid a,b;double threshold;};
  Json spec_;std::vector<Pair> mounts_;Solid tip_,datum_;
  Eigen::Vector3d p_tip_,p_datum_;std::string board_link_,stage_;
  bool guard_=false,record_=false;
  static Eigen::Vector3d vector(const Json& v){if(v.size()!=3)throw std::runtime_error("TOUCH_VECTOR");return Eigen::Vector3d(v.at(0).get<double>(),v.at(1).get<double>(),v.at(2).get<double>());}
  static Solid solid(const planning_scene::PlanningScenePtr& scene,const std::string& name) {
    Solid out;out.name=name;out.link=scene->getWorld()->hasObject(name)?nullptr:scene->getRobotModel()->getLinkModel(name);shapes::ShapeConstPtr shape;
    if(out.link) {if(out.link->getShapes().size()!=1)throw std::runtime_error("TOUCH_LINK_SHAPE_COUNT");shape=out.link->getShapes()[0];}
    else {auto object=scene->getWorld()->getObject(name);if(!object||object->shapes_.size()!=1)throw std::runtime_error("TOUCH_WORLD_SHAPE_COUNT:"+name);
      shape=object->shapes_[0];out.fixed=object->global_shape_poses_[0];}
    if(shape->type==shapes::MESH) {
      const auto* m=static_cast<const shapes::Mesh*>(shape.get());auto p=std::make_shared<std::vector<fcl::Vector3d>>();auto f=std::make_shared<std::vector<int>>();
      for(size_t i=0;i<m->vertex_count;++i)p->emplace_back(m->vertices[3*i],m->vertices[3*i+1],m->vertices[3*i+2]);
      for(size_t i=0;i<m->triangle_count;++i){f->push_back(3);for(int k=0;k<3;++k)f->push_back(m->triangles[3*i+k]);}
      out.shape=std::make_shared<fcl::Convexd>(p,m->triangle_count,f,true);
    } else if(shape->type==shapes::BOX){auto* b=static_cast<const shapes::Box*>(shape.get());out.shape=std::make_shared<fcl::Boxd>(b->size[0],b->size[1],b->size[2]);}
    else throw std::runtime_error("TOUCH_EXPLICIT_CONVEX_REQUIRED");
    return out;
  }
  static Eigen::Isometry3d at(const Solid& s,const moveit::core::RobotState& state){return s.link?state.getCollisionBodyTransform(s.link,0):s.fixed;}
  static double distance(const Solid& a,const Solid& b,const moveit::core::RobotState& state){
    fcl::CollisionObjectd x(a.shape,at(a,state)),y(b.shape,at(b,state));fcl::DistanceResultd result;
    const double d=fcl::distance(&x,&y,fcl::DistanceRequestd(true,true),result);
    if(!std::isfinite(d))throw std::runtime_error("TOUCH_FCL_DISTANCE_UNKNOWN");return d;
  }
public:
  Json failure=nullptr,evidence_=Json::object();
  Eco65TouchProcess(const planning_scene::PlanningScenePtr& scene,const Json& req):spec_(req.at("process_policy")),stage_(req.at("stage")) {
    if(spec_.at("schema")!="eco65_known_tip_touch_process_v1" || spec_.at("geometry_type")!="known_tip_finite_datum_v1" ||
       !req.at("attachment").is_null() || spec_.at("stage")!=stage_ || !req.at("allowed_pairs").empty() ||
       spec_.at("datum_radius_m")!=.004 || spec_.at("maximum_tip_penetration_m")!=.0001 ||
       spec_.at("collision_policy")!=req.at("clearance_policy").at("source_policy"))throw std::runtime_error("TOUCH_PROCESS_CONTEXT");
    if(stage_!="pregrasp"&&stage_!="transit"&&stage_!="contact"&&stage_!="withdrawal"&&stage_!="residence")throw std::runtime_error("TOUCH_STAGE");
    guard_=stage_=="contact"||stage_=="withdrawal";record_=req.value("record_contact_distances",false);
    p_tip_=vector(spec_.at("p_F_tip_m"));p_datum_=vector(spec_.at("p_P_datum_m"));board_link_=spec_.at("board_frame_link");
    if(!scene->getRobotModel()->getLinkModel(board_link_))throw std::runtime_error("TOUCH_BOARD_FRAME_MISSING");
    tip_=solid(scene,"probe_tip");datum_=solid(scene,spec_.at("target_id"));
    auto& acm=scene->getAllowedCollisionMatrixNonConst();
    for(const auto& row:spec_.at("installation_contacts")) {
      if(row.at("threshold_m")!=-.0004)throw std::runtime_error("TOUCH_INSTALLATION_TOLERANCE");
      auto a=solid(scene,row.at("pair").at(0)),b=solid(scene,row.at("pair").at(1));
      mounts_.push_back({a,b,-.0004});acm.setEntry(a.name,b.name,true);
    }
    acm.setEntry(tip_.name,datum_.name,true);
  }
  bool check(const moveit::core::RobotState& state,bool) {
    failure=nullptr;evidence_={{"distances",Json::array()},{"guarded_neighborhood",guard_}};
    auto accept=[&](const Solid& a,const Solid& b,double d,double limit,bool analytic=false){
      const bool valid=analytic||eco65DistanceAccepted(d,limit);
      if(record_)evidence_["distances"].push_back({{"pair",{a.name,b.name}},{"signed_distance_m",d},{"threshold_m",limit},{"accepted",valid},{"query","FCL_signed_distance"},{"finite_plane_decision",analytic}});
      if(!valid)failure={{"reason","TOUCH_PAIR_CLEARANCE"},{"pair",{a.name,b.name}},{"signed_distance_m",d},{"threshold_m",limit}};
      return valid;
    };
    for(const auto& p:mounts_)if(!accept(p.a,p.b,distance(p.a,p.b,state),p.threshold))return false;
    const auto P=state.getGlobalLinkTransform(board_link_);const auto F=state.getGlobalLinkTransform(nativeFrames().flange_link);
    const Eigen::Vector3d tip=P.inverse()*F*p_tip_;
    const double radial=(tip-p_datum_).head<2>().norm();double gap=1e10;
    const auto* shape=static_cast<const shapes::Mesh*>(tip_.link->getShapes()[0].get());
    const auto T=P.inverse()*at(tip_,state);
    for(size_t i=0;i<shape->vertex_count;++i){const Eigen::Vector3d p=T*Eigen::Vector3d(shape->vertices[3*i],shape->vertices[3*i+1],shape->vertices[3*i+2]);gap=std::min(gap,p_datum_.z()-p.z());}
    const bool finite=guard_&&radial<=.004&&gap>=-.0001;
    evidence_["finite_datum"]={{"radial_error_m",radial},{"separating_plane_gap_m",gap},{"radius_m",.004},{"tip_plane_gap_m",p_datum_.z()-tip.z()},
      {"method","Every original cone vertex versus finite front disk; arrival not required along approach"}};
    return accept(tip_,datum_,distance(tip_,datum_,state),finite?-.0001:.0024,finite);
  }
};
}
