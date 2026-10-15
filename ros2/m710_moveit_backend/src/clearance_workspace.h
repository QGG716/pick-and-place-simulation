#pragma once
#include <moveit/collision_detection_fcl/collision_env_fcl.h>
#include <chrono>
#include <functional>
#include "pair_profile.h"

namespace m710 {
// Reuse MoveIt's unpadded FCL assets. Mutable objects and the world snapshot are
// owned by one candidate; Clearance serializes access (including OMPL threads).
class ClearanceWorkspace : private collision_detection::CollisionEnvFCL {
  using Clock=std::chrono::steady_clock;
  static double elapsed(Clock::time_point t) {return std::chrono::duration<double>(Clock::now()-t).count();}
  using BodyType=collision_detection::BodyType;
  struct Body {
    collision_detection::FCLCollisionObjectPtr object;
    std::string name;
    BodyType type;
    const moveit::core::LinkModel* link=nullptr;
    size_t shape=0;
    std::set<std::string> touch;
    bool fixed_tool=false,fixed_payload=false;
  };
  enum class FixedSource { None, Aabb, Narrow };
  struct Pair {
    size_t a,b;double limit;bool self;
    bool fixed=false,accepted=false,visited=false,qualification_attempted=false;
    FixedSource source=FixedSource::None;
  };
  bool reuse_=false,qualify_fixed_narrow_=true;
  const bool pair_profile_enabled_;
  static bool fixedToFlange(const moveit::core::LinkModel* link) {
    while(link && link->getName()!="flange") {
      const auto* joint=link->getParentJointModel();
      if(!joint || joint->getType()!=moveit::core::JointModel::FIXED)return false;
      link=joint->getParentLinkModel();
    }
    return link!=nullptr;
  }
  moveit::core::RobotState context_;
  collision_detection::FCLObject robot_;
  std::vector<Body> bodies_;
  std::vector<Pair> pairs_;
  size_t moving_=0;
  std::vector<const moveit::core::AttachedBody*> attached_;
public:
  struct Stats {
    size_t queries=0, aabb_tests=0, aabb_skips=0, narrow_calls=0, self_narrow=0, world_narrow=0;
    size_t prepared_objects=0, eligible_pairs=0, exempt_pairs=0;
    size_t fixed_first_checks=0,fixed_property_evaluations=0,fixed_property_reuses=0,dynamic_pair_checks=0;
    size_t fixed_aabb_qualifications=0,fixed_narrow_qualifications=0,fixed_qualification_queries=0;
    size_t fixed_aabb_reuses=0,fixed_narrow_reuses=0;
    std::map<std::string,size_t> fixed_qualification_declines;
    double prepare_s=0, geometry_prepare_s=0, pair_filter_s=0, update_s=0, broad_s=0, narrow_s=0, self_s=0, world_s=0;
  } stats;
  std::map<std::string,Stats> phases;
  struct Failure {std::string a,b;double distance=0,limit=0;bool self=false;};
  ClearanceWorkspace(const collision_detection::CollisionEnvFCL& env,
      const planning_scene::PlanningSceneConstPtr& scene,
      const std::function<double(const std::string&,BodyType,const std::string&,BodyType)>& required,
      const std::set<std::string>& tools={},bool reuse=false,bool qualify_fixed_narrow=true,bool pair_profile_enabled=false)
    : CollisionEnvFCL(env,std::make_shared<collision_detection::World>(*scene->getWorld())),pair_profile_enabled_(pair_profile_enabled),context_(scene->getCurrentState()) {
    reuse_=reuse;qualify_fixed_narrow_=qualify_fixed_narrow;
    const auto t=Clock::now();context_.updateCollisionBodyTransforms();context_.getAttachedBodies(attached_);
    constructFCLObjectRobot(context_,robot_);
    auto add=[&](const collision_detection::FCLCollisionObjectPtr& o) {
      if(!o || !o->collisionGeometry()) throw std::runtime_error("MISSING_FCL_GEOMETRY");
      const auto* d=static_cast<const collision_detection::CollisionGeometryData*>(o->collisionGeometry()->getUserData());
      if(!d) throw std::runtime_error("MISSING_FCL_METADATA");
      Body b;b.object=o;b.name=d->getID();b.type=d->type;b.shape=d->shape_index;
      if(b.type==collision_detection::BodyTypes::ROBOT_LINK) b.link=d->ptr.link;
      if(b.type==collision_detection::BodyTypes::ROBOT_ATTACHED) b.touch=d->ptr.ab->getTouchLinks();
      // Only box tool/payload pairs whose actual model chains are fixed to
      // the flange qualify. Meshes, active links and world bodies always query.
      if(o->collisionGeometry()->getNodeType()==fcl::GEOM_BOX) {
        if(b.link && tools.count(b.name))b.fixed_tool=fixedToFlange(b.link);
        if(b.type==collision_detection::BodyTypes::ROBOT_ATTACHED)
          b.fixed_payload=fixedToFlange(d->ptr.ab->getAttachedLink());
      }
      bodies_.push_back(std::move(b));
    };
    for(const auto& o:robot_.collision_objects_) add(o);
    moving_=bodies_.size();
    size_t expected=0;
    for(const auto* link:context_.getRobotModel()->getLinkModelsWithCollisionGeometry()) expected+=link->getShapes().size();
    for(const auto* body:attached_) expected+=body->getShapes().size();
    if(moving_!=expected) throw std::runtime_error("MISSING_ROBOT_FCL_GEOMETRY");
    for(const auto& item:fcl_objs_) for(const auto& o:item.second.collision_objects_) add(o);
    for(const auto& id:scene->getWorld()->getObjectIds()) expected+=scene->getWorld()->getObject(id)->shapes_.size();
    if(bodies_.size()!=expected) throw std::runtime_error("MISSING_WORLD_FCL_GEOMETRY");
    stats.geometry_prepare_s=elapsed(t);const auto filtering=Clock::now();
    const auto& acm=scene->getAllowedCollisionMatrix();
    for(size_t a=0;a<moving_;++a) for(size_t b=a+1;b<bodies_.size();++b) {
      const auto& x=bodies_[a];const auto& y=bodies_[b];
      collision_detection::AllowedCollision::Type allowed;
      const bool exempt=(x.type==y.type && x.name==y.name) ||
        (acm.getAllowedCollision(x.name,y.name,allowed) && allowed==collision_detection::AllowedCollision::ALWAYS) ||
        (x.type==collision_detection::BodyTypes::ROBOT_LINK && y.type==collision_detection::BodyTypes::ROBOT_ATTACHED && y.touch.count(x.name)) ||
        (y.type==collision_detection::BodyTypes::ROBOT_LINK && x.type==collision_detection::BodyTypes::ROBOT_ATTACHED && x.touch.count(y.name));
      if(exempt) {++stats.exempt_pairs;continue;}
      pairs_.push_back({a,b,required(x.name,x.type,y.name,y.type),b<moving_,
        (x.fixed_tool&&y.fixed_payload)||(y.fixed_tool&&x.fixed_payload),false});
    }
    stats.pair_filter_s=elapsed(filtering);stats.prepared_objects=bodies_.size();stats.eligible_pairs=pairs_.size();stats.prepare_s=elapsed(t);
  }
  bool check(const moveit::core::RobotState& state,Failure& failure,const std::string& phase) {
    auto& stats=phases[phase];
    ++stats.queries;auto updating=Clock::now();
    if(state.getRobotModel()!=context_.getRobotModel()) throw std::runtime_error("WORKSPACE_MODEL_CHANGED");
    std::vector<const moveit::core::AttachedBody*> current;state.getAttachedBodies(current);
    if(current.size()!=attached_.size()) throw std::runtime_error("WORKSPACE_ATTACHMENT_CHANGED");
    for(const auto* expected:attached_) {
      const auto* actual=state.getAttachedBody(expected->getName());
      if(!actual || actual->getAttachedLink()!=expected->getAttachedLink() || actual->getShapes()!=expected->getShapes() ||
         actual->getTouchLinks()!=expected->getTouchLinks()) throw std::runtime_error("WORKSPACE_ATTACHMENT_CHANGED");
      const auto& a=actual->getShapePosesInLinkFrame();const auto& b=expected->getShapePosesInLinkFrame();
      if(a.size()!=b.size()) throw std::runtime_error("WORKSPACE_ATTACHMENT_CHANGED");
      for(size_t i=0;i<a.size();++i) if(a[i].matrix()!=b[i].matrix()) throw std::runtime_error("WORKSPACE_ATTACHMENT_CHANGED");
    }
    bool bounded_transforms=true;
    for(size_t i=0;i<moving_;++i) {
      auto& b=bodies_[i];
      const auto& transform=b.link?state.getCollisionBodyTransform(b.link,b.shape):
        state.getAttachedBody(b.name)->getGlobalCollisionBodyTransforms().at(b.shape);
      if(!transform.matrix().allFinite()) throw std::runtime_error("INVALID_FCL_TRANSFORM");
      if(reuse_ && (b.fixed_tool||b.fixed_payload)) {
        // A guard for numerical reuse, never an acceptance tolerance. Unknown
        // scales/rotations fall back to exactly the original world-space query.
        const auto* box=static_cast<const fcl::Boxd*>(b.object->collisionGeometry().get());
        bounded_transforms=bounded_transforms && transform.translation().cwiseAbs().maxCoeff()<100. &&
          (transform.linear().transpose()*transform.linear()-Eigen::Matrix3d::Identity()).norm()<1e-10 &&
          box->side.allFinite() && (box->side.array()>0).all() && box->side.maxCoeff()<10.;
      }
      b.object->setTransform(collision_detection::transform2fcl(transform));b.object->computeAABB();
    }
    stats.update_s+=elapsed(updating);
    for(auto& p:pairs_) {
      if(reuse_ && p.fixed && bounded_transforms && p.accepted) {
        ++stats.fixed_property_reuses;
        if(p.source==FixedSource::Aabb)++stats.fixed_aabb_reuses;
        else ++stats.fixed_narrow_reuses;
        continue;
      }
      if(p.fixed) {
        if(!p.visited){++stats.fixed_first_checks;p.visited=true;}
        ++stats.fixed_property_evaluations;
      } else ++stats.dynamic_pair_checks;
      const auto& a=bodies_[p.a];const auto& b=bodies_[p.b];PairProfileTimer<> broad(pair_profile_enabled_);++stats.aabb_tests;
      const auto& ab=a.object->getAABB();const auto& bb=b.object->getAABB();
      if(!ab.min_.allFinite() || !ab.max_.allFinite() || !bb.min_.allFinite() || !bb.max_.allFinite())
        throw std::runtime_error("UNKNOWN_FCL_AABB");
      // Per-pair query band is wider than the acceptance threshold. This is
      // conservative broad phase, not padding or a new numerical tolerance.
      const double lower=ab.distance(bb);
      if(!std::isfinite(lower)) throw std::runtime_error("UNKNOWN_AABB_DISTANCE");
      const bool skip=lower>p.limit+.001;
      if(pair_profile_enabled_)stats.broad_s+=broad.elapsed();
      if(skip) {
        ++stats.aabb_skips;
        if(reuse_&&p.fixed&&bounded_transforms) {
          p.accepted=true;p.source=FixedSource::Aabb;++stats.fixed_aabb_qualifications;
        }
        continue;
      }
      PairProfileTimer<> narrow(pair_profile_enabled_);++stats.narrow_calls;
      if(p.self) ++stats.self_narrow;else ++stats.world_narrow;
      fcl::DistanceResultd result;result.min_distance=p.limit+.001;
      const double d=fcl::distance(a.object.get(),b.object.get(),fcl::DistanceRequestd(false),result);
      auto finish_narrow=[&] {
        if(pair_profile_enabled_) {
          const double dt=narrow.elapsed();stats.narrow_s+=dt;(p.self?stats.self_s:stats.world_s)+=dt;
        }
      };
      // Use the same FCL request/default solver and strict contact rejection as
      // MoveIt's reference callback; no signed-distance or geometry surrogate.
      if(!std::isfinite(d) || !std::isfinite(result.min_distance) || d<=0 || d+1e-9<p.limit) {
        finish_narrow();failure={a.name,b.name,d,p.limit,p.self};return false;
      }
      // The acceptance query above is deliberately capped at limit + 1 mm.
      // FCL 0.7 box distance returns that capped minimum; it is NOT a true
      // separation suitable for certification. Try qualification at most once
      // per eligible fixed pair, inside this check (no prewarming).
      if(qualify_fixed_narrow_&&reuse_&&p.fixed&&bounded_transforms&&!p.qualification_attempted) {
        p.qualification_attempted=true;
        const double band=p.limit+.001;
        if(d<band && result.min_distance==d) {
          ++stats.fixed_qualification_declines["WITHIN_REUSE_GUARD"];
        } else if(d!=band || result.min_distance!=band) {
          ++stats.fixed_qualification_declines["INCONSISTENT_CAPPED_RESULT"];
        } else {
          ++stats.fixed_qualification_queries;++stats.narrow_calls;
          if(p.self)++stats.self_narrow;else ++stats.world_narrow;
          try {
            fcl::DistanceResultd full; // default max(), never the acceptance band
            const double distance=fcl::distance(a.object.get(),b.object.get(),fcl::DistanceRequestd(false),full);
            if(!std::isfinite(distance) || !std::isfinite(full.min_distance) ||
                distance==std::numeric_limits<double>::max() || full.o1!=a.object->collisionGeometry().get() ||
                full.o2!=b.object->collisionGeometry().get()) {
              ++stats.fixed_qualification_declines["UNKNOWN_UNCAPPED_RESULT"];
            } else if(distance!=full.min_distance || std::min(band,distance)!=d) {
              ++stats.fixed_qualification_declines["INCONSISTENT_UNCAPPED_RESULT"];
            } else if(distance>band) {
              p.accepted=true;p.source=FixedSource::Narrow;++stats.fixed_narrow_qualifications;
            } else {
              ++stats.fixed_qualification_declines["WITHIN_REUSE_GUARD"];
            }
          } catch(const std::exception&) {
            // Supplementary evidence is optional. Retain the original successful
            // verdict and future original checks, never a success certificate.
            ++stats.fixed_qualification_declines["QUALIFICATION_QUERY_EXCEPTION"];
          }
        }
      }
      finish_narrow();
    }
    return true;
  }
};
}
