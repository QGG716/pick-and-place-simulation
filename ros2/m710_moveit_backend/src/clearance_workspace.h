#pragma once
#include <moveit/collision_detection_fcl/collision_env_fcl.h>
#include <chrono>
#include <functional>
#include <numeric>
#include <algorithm>
#include <cstdlib>
#include <limits>

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
  };
  struct Pair {size_t a,b;double limit;bool self;};
  moveit::core::RobotState context_;
  collision_detection::FCLObject robot_;
  std::vector<Body> bodies_;
  std::vector<Pair> pairs_;
  struct BoundsNode {Eigen::Vector3d lo,hi;int left=-1,right=-1,body=-1;};
  std::vector<BoundsNode> tree_;
  std::vector<int> pair_index_,all_pair_indices_;
  bool hierarchical_=false;
  double maximum_band_=0.;
  double search_payload_reserve_=0.;
  size_t moving_=0;
  std::vector<const moveit::core::AttachedBody*> attached_;
  int buildTree(std::vector<size_t>& ids,size_t first,size_t last) {
    const int index=tree_.size();tree_.emplace_back();
    if(last-first==1)tree_[index].body=int(ids[first]);
    else {
      Eigen::Vector3d lo=Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity()),hi=-lo;
      for(size_t k=first;k<last;++k) {const auto& b=bodies_[ids[k]].object->getAABB();lo=lo.cwiseMin(b.min_);hi=hi.cwiseMax(b.max_);}
      Eigen::Index axis;(hi-lo).maxCoeff(&axis);const size_t middle=(first+last)/2;
      std::nth_element(ids.begin()+first,ids.begin()+middle,ids.begin()+last,[&](size_t a,size_t b) {
        const auto& x=bodies_[a].object->getAABB();const auto& y=bodies_[b].object->getAABB();
        return x.min_[axis]+x.max_[axis]<y.min_[axis]+y.max_[axis];
      });
      const int left=buildTree(ids,first,middle),right=buildTree(ids,middle,last);
      tree_[index].left=left;tree_[index].right=right;
    }
    return index;
  }
  void refitTree(int index) {
    auto& n=tree_[index];
    if(n.body>=0) {const auto& b=bodies_[n.body].object->getAABB();n.lo=b.min_;n.hi=b.max_;return;}
    refitTree(n.left);refitTree(n.right);
    n.lo=tree_[n.left].lo.cwiseMin(tree_[n.right].lo);n.hi=tree_[n.left].hi.cwiseMax(tree_[n.right].hi);
  }
  void nearPairs(size_t a,int index,std::vector<int>& candidates,size_t& visited) const {
    ++visited;const auto& n=tree_[index];const auto& bounds=bodies_[a].object->getAABB();
    const Eigen::Vector3d separation=(bounds.min_-n.hi).cwiseMax(n.lo-bounds.max_).cwiseMax(0.);
    // Every descendant AABB lies inside this refitted node. This cutoff is
    // wider than every eligible pair's narrow-query band, never an allowance.
    if(separation.norm()>maximum_band_)return;
    if(n.body>=0) {
      if(size_t(n.body)>a) {const int pair=pair_index_[a*bodies_.size()+n.body];if(pair>=0)candidates.push_back(pair);}
      return;
    }
    nearPairs(a,n.left,candidates,visited);nearPairs(a,n.right,candidates,visited);
  }
public:
  struct Stats {
    size_t queries=0, aabb_tests=0, aabb_skips=0, narrow_calls=0, self_narrow=0, world_narrow=0;
    size_t prepared_objects=0, eligible_pairs=0, exempt_pairs=0, hierarchy_nodes_visited=0;
    double prepare_s=0, geometry_prepare_s=0, pair_filter_s=0, update_s=0, broad_s=0, narrow_s=0, self_s=0, world_s=0;
  } stats;
  std::map<std::string,Stats> phases;
  struct Failure {std::string a,b;double distance=0,limit=0;bool self=false;};
  ClearanceWorkspace(const collision_detection::CollisionEnvFCL& env,
      const planning_scene::PlanningSceneConstPtr& scene,
      const std::function<double(const std::string&,BodyType,const std::string&,BodyType)>& required,
      bool explicit_convex_meshes=false,double search_payload_reserve=0.)
    : CollisionEnvFCL(env,std::make_shared<collision_detection::World>(*scene->getWorld())),context_(scene->getCurrentState()),
      search_payload_reserve_(search_payload_reserve) {
    const auto t=Clock::now();context_.updateCollisionBodyTransforms();context_.getAttachedBodies(attached_);
    constructFCLObjectRobot(context_,robot_);
    auto add=[&](const collision_detection::FCLCollisionObjectPtr& o) {
      if(!o || !o->collisionGeometry()) throw std::runtime_error("MISSING_FCL_GEOMETRY");
      const auto* d=static_cast<const collision_detection::CollisionGeometryData*>(o->collisionGeometry()->getUserData());
      if(!d) throw std::runtime_error("MISSING_FCL_METADATA");
      Body b;b.object=o;b.name=d->getID();b.type=d->type;b.shape=d->shape_index;
      if(b.type==collision_detection::BodyTypes::ROBOT_LINK) b.link=d->ptr.link;
      if(b.type==collision_detection::BodyTypes::ROBOT_ATTACHED) b.touch=d->ptr.ab->getTouchLinks();
      // ECO65's independent MuJoCo world uses convex solids. A triangle BVH
      // cannot prove containment equivalence merely because its file is the
      // same. Construct real FCL convex solids once for this candidate.
      if(explicit_convex_meshes) {
        const auto shape=b.link?b.link->getShapes().at(b.shape):
          b.type==collision_detection::BodyTypes::WORLD_OBJECT?scene->getWorld()->getObject(b.name)->shapes_.at(b.shape):shapes::ShapeConstPtr{};
        if(shape) {
        if(shape->type==shapes::MESH) {
          const auto* mesh=static_cast<const shapes::Mesh*>(shape.get());
          auto vertices=std::make_shared<std::vector<fcl::Vector3d>>();auto faces=std::make_shared<std::vector<int>>();
          for(size_t i=0;i<mesh->vertex_count;++i)vertices->emplace_back(mesh->vertices[3*i],mesh->vertices[3*i+1],mesh->vertices[3*i+2]);
          for(size_t i=0;i<mesh->triangle_count;++i){faces->push_back(3);for(int k=0;k<3;++k)faces->push_back(mesh->triangles[3*i+k]);}
          auto convex=std::make_shared<fcl::Convexd>(vertices,mesh->triangle_count,faces,true);
          b.object=std::make_shared<fcl::CollisionObjectd>(convex,o->getTransform());
        }
        }
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
      pairs_.push_back({a,b,required(x.name,x.type,y.name,y.type),b<moving_});
    }
    hierarchical_=explicit_convex_meshes && !std::getenv("ECO65_BROAD_PHASE_FULL_SCAN");
    all_pair_indices_.resize(pairs_.size());std::iota(all_pair_indices_.begin(),all_pair_indices_.end(),0);
    if(hierarchical_) {
      pair_index_.assign(moving_*bodies_.size(),-1);
      for(size_t i=0;i<pairs_.size();++i) {const auto& p=pairs_[i];pair_index_[p.a*bodies_.size()+p.b]=int(i);maximum_band_=std::max(maximum_band_,p.limit+search_payload_reserve_+.001);}
      std::vector<size_t> ids(bodies_.size());std::iota(ids.begin(),ids.end(),0);tree_.reserve(2*bodies_.size());buildTree(ids,0,ids.size());
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
    for(size_t i=0;i<moving_;++i) {
      auto& b=bodies_[i];
      const auto& transform=b.link?state.getCollisionBodyTransform(b.link,b.shape):
        state.getAttachedBody(b.name)->getGlobalCollisionBodyTransforms().at(b.shape);
      if(!transform.matrix().allFinite()) throw std::runtime_error("INVALID_FCL_TRANSFORM");
      b.object->setTransform(collision_detection::transform2fcl(transform));b.object->computeAABB();
    }
    // A body AABB is identical for all of its pairs in this state. Validate it
    // once, and time the broad phase as a batch rather than reading the clock
    // twice for each of hundreds of thousands of pairs. The eligible pairs,
    // AABB distance cutoff and actual FCL narrow queries remain unchanged.
    for(const auto& body:bodies_) {
      const auto& bounds=body.object->getAABB();
      if(!bounds.min_.allFinite() || !bounds.max_.allFinite())
        throw std::runtime_error("UNKNOWN_FCL_AABB");
    }
    stats.update_s+=elapsed(updating);
    const auto broad_start=Clock::now();const double narrow_before=stats.narrow_s;
    auto finish_broad=[&]() {stats.broad_s+=elapsed(broad_start)-(stats.narrow_s-narrow_before);};
    std::vector<int> nearby;const auto* selected=&all_pair_indices_;
    if(hierarchical_) {
      refitTree(0);
      for(size_t a=0;a<moving_;++a)nearPairs(a,0,nearby,stats.hierarchy_nodes_visited);
      std::sort(nearby.begin(),nearby.end());selected=&nearby;
      stats.aabb_skips+=pairs_.size()-nearby.size();
    }
    for(int index:*selected) {
      const auto& p=pairs_[index];
      const auto& a=bodies_[p.a];const auto& b=bodies_[p.b];++stats.aabb_tests;
      // The actual dense audit retains the original acceptance threshold.
      // A small, explicit search-only reserve keeps OMPL's coarser segments
      // away from the observed payload/neighbor clearance boundary.
      const bool payload=a.type==collision_detection::BodyTypes::ROBOT_ATTACHED || b.type==collision_detection::BodyTypes::ROBOT_ATTACHED;
      const double limit=p.limit+((phase=="search" && payload)?search_payload_reserve_:0.);
      const auto& ab=a.object->getAABB();const auto& bb=b.object->getAABB();
      // Per-pair query band is wider than the acceptance threshold. This is
      // conservative broad phase, not padding or a new numerical tolerance.
      const double lower=ab.distance(bb);
      if(!std::isfinite(lower)) throw std::runtime_error("UNKNOWN_AABB_DISTANCE");
      const bool skip=lower>limit+.001;
      if(skip) {++stats.aabb_skips;continue;}
      auto narrow=Clock::now();++stats.narrow_calls;
      if(p.self) ++stats.self_narrow;else ++stats.world_narrow;
      fcl::DistanceResultd result;result.min_distance=limit+.001;
      const double d=fcl::distance(a.object.get(),b.object.get(),fcl::DistanceRequestd(false),result);
      const double dt=elapsed(narrow);stats.narrow_s+=dt;(p.self?stats.self_s:stats.world_s)+=dt;
      // Use the same FCL request/default solver and strict contact rejection as
      // MoveIt's reference callback; no signed-distance or geometry surrogate.
      if(!std::isfinite(d) || !std::isfinite(result.min_distance) || d<=0 || d+1e-9<limit) {
        failure={a.name,b.name,d,limit,p.self};finish_broad();return false;
      }
    }
    finish_broad();return true;
  }
};
}
