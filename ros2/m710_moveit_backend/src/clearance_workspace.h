#pragma once
#include <moveit/collision_detection_fcl/collision_env_fcl.h>
#include <chrono>
#include <functional>
#include <numeric>
#include <algorithm>
#include <cstdlib>
#include <limits>
#include <map>
#include <mutex>
#include <tuple>
#include "native_deadline.h"
#include "native_timing.h"

namespace m710 {
// FCL 0.7 computeAABB expands every rotated shape to a radius cube. Transform
// the local enclosing box instead: all original geometry remains inside this
// conservative bound. Used only to reject far pairs, never as collision shape.
inline fcl::AABBd conservativeWorldBounds(const fcl::CollisionObjectd& object) {
  const auto& local=object.collisionGeometry()->aabb_local;
  const Eigen::Vector3d center=object.getTransform()*((local.min_+local.max_)*.5);
  Eigen::Vector3d extent=object.getRotation().cwiseAbs()*((local.max_-local.min_)*.5);
  extent.array()+=1e-12*(1.+center.cwiseAbs().maxCoeff()+extent.maxCoeff());
  return fcl::AABBd(center-extent,center+extent);
}
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
    fcl::AABBd bounds;
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
  bool tight_bounds_=false;
  double maximum_band_=0.;
  double search_payload_reserve_=0.;
  NativeDeadlinePtr deadline_;
  size_t moving_=0;
  std::vector<const moveit::core::AttachedBody*> attached_;
  // -- two-level check (collision_check_mode TWO_LEVEL, physical single carton) --
  // Level 1 proves a pair clear with conservative proxies (body bounding
  // spheres from the solid's own vertices, rigid-group spheres, a static
  // bounds hierarchy, and pairs of constant relative pose resolved once).
  // Every pair it cannot prove clear goes to the unchanged level 2: the same
  // conservative AABB band and the same exact FCL distance, in pair order.
  bool two_level_=false;
  struct GroupSphere {std::vector<size_t> bodies;size_t reference=0;Eigen::Vector3d center_ref=Eigen::Vector3d::Zero();double radius=0.;Eigen::Vector3d world=Eigen::Vector3d::Zero();};
  struct GroupPairs {std::vector<int> pairs;double band=0.;};
  std::vector<Eigen::Vector3d> sphere_local_,sphere_world_;std::vector<double> sphere_radius_;
  std::vector<int> group_;std::vector<GroupSphere> groups_;
  std::vector<int> constant_candidates_;size_t constant_cleared_=0;
  std::map<std::pair<int,int>,GroupPairs> moving_pairs_;
  std::vector<char> payload_pair_;
  std::vector<BoundsNode> static_tree_;
  int buildStatic(std::vector<size_t>& ids,size_t first,size_t last) {
    const int index=static_tree_.size();static_tree_.emplace_back();
    Eigen::Vector3d lo=Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity()),hi=-lo;
    for(size_t k=first;k<last;++k) {const auto& b=bodies_[ids[k]].bounds;lo=lo.cwiseMin(b.min_);hi=hi.cwiseMax(b.max_);}
    static_tree_[index].lo=lo;static_tree_[index].hi=hi;
    if(last-first==1) {static_tree_[index].body=int(ids[first]);return index;}
    Eigen::Index axis;(hi-lo).maxCoeff(&axis);const size_t middle=(first+last)/2;
    std::nth_element(ids.begin()+first,ids.begin()+middle,ids.begin()+last,[&](size_t a,size_t b) {
      const auto& x=bodies_[a].bounds;const auto& y=bodies_[b].bounds;
      return x.min_[axis]+x.max_[axis]<y.min_[axis]+y.max_[axis];
    });
    const int left=buildStatic(ids,first,middle),right=buildStatic(ids,middle,last);
    static_tree_[index].left=left;static_tree_[index].right=right;return index;
  }
  void staticNear(int index,const Eigen::Vector3d& center,double reach,std::vector<size_t>& out,size_t& visited) const {
    ++visited;const auto& n=static_tree_[index];
    // Distance from the group sphere's centre to this node's box; every
    // static body below lies inside the box. reach = sphere radius + band.
    if((n.lo-center).cwiseMax(center-n.hi).cwiseMax(0.).norm()>reach)return;
    if(n.body>=0) {out.push_back(size_t(n.body));return;}
    staticNear(n.left,center,reach,out,visited);staticNear(n.right,center,reach,out,visited);
  }
  // Robot convex solids are immutable for the model's lifetime: build each once
  // per process (keyed by link, shape index and mesh) and share it read-only.
  static std::shared_ptr<fcl::Convexd> buildConvex(const shapes::Mesh* mesh) {
    auto vertices=std::make_shared<std::vector<fcl::Vector3d>>();auto faces=std::make_shared<std::vector<int>>();
    for(size_t i=0;i<mesh->vertex_count;++i)vertices->emplace_back(mesh->vertices[3*i],mesh->vertices[3*i+1],mesh->vertices[3*i+2]);
    for(size_t i=0;i<mesh->triangle_count;++i){faces->push_back(3);for(int k=0;k<3;++k)faces->push_back(mesh->triangles[3*i+k]);}
    return std::make_shared<fcl::Convexd>(vertices,mesh->triangle_count,faces,true);
  }
  static std::shared_ptr<fcl::Convexd> cachedConvex(const moveit::core::LinkModel* link,size_t shape,const shapes::Mesh* mesh) {
    static std::mutex mutex;static std::map<std::tuple<const void*,size_t,const void*,size_t,size_t>,std::shared_ptr<fcl::Convexd>> cache;
    std::lock_guard<std::mutex> lock(mutex);
    auto& slot=cache[std::make_tuple((const void*)link,shape,(const void*)mesh,size_t(mesh->vertex_count),size_t(mesh->triangle_count))];
    if(!slot)slot=buildConvex(mesh);
    return slot;
  }
  void prepareTwoLevel(const planning_scene::PlanningSceneConstPtr& scene) {
    if(!hierarchical_)throw std::runtime_error("TWO_LEVEL_REQUIRES_EXPLICIT_CONVEX_WORKSPACE");
    two_level_=true;const size_t n=bodies_.size();
    sphere_local_.resize(n);sphere_world_.resize(n);sphere_radius_.resize(n);group_.assign(n,0);
    std::map<const moveit::core::LinkModel*,int> ids;
    // Group = the child link of the nearest movable joint up the chain; 0 when
    // every joint up to the root is fixed (static, like the world objects).
    auto group_of=[&](const moveit::core::LinkModel* l)->int {
      while(true) {
        const auto* j=l->getParentJointModel();
        if(!j)return 0;
        if(j->getType()!=moveit::core::JointModel::FIXED) {
          auto it=ids.find(l);if(it==ids.end())it=ids.emplace(l,int(ids.size())+1).first;
          return it->second;
        }
        const auto* p=l->getParentLinkModel();
        if(!p)return 0;
        l=p;
      }
    };
    for(size_t i=0;i<n;++i) {
      const auto sphere=localSphere(*bodies_[i].object);
      sphere_local_[i]=sphere.first;sphere_radius_[i]=sphere.second;
      sphere_world_[i]=bodies_[i].object->getTransform()*sphere.first;
      if(i<moving_) {
        const auto* link=bodies_[i].link?bodies_[i].link:context_.getAttachedBody(bodies_[i].name)->getAttachedLink();
        group_[i]=group_of(link);
      }
    }
    groups_.assign(ids.size()+1,GroupSphere{});
    for(size_t i=0;i<moving_;++i)if(group_[i])groups_[group_[i]].bodies.push_back(i);
    for(size_t g=1;g<groups_.size();++g) {
      auto& G=groups_[g];G.reference=G.bodies.front();
      const auto inverse=bodies_[G.reference].object->getTransform().inverse();
      std::vector<Eigen::Vector3d> rel;for(size_t i:G.bodies)rel.push_back(inverse*sphere_world_[i]);
      Eigen::Vector3d c=Eigen::Vector3d::Zero();for(const auto& r:rel)c+=r;c/=double(rel.size());
      double radius=0.;for(size_t k=0;k<rel.size();++k)radius=std::max(radius,(rel[k]-c).norm()+sphere_radius_[G.bodies[k]]);
      G.center_ref=c;G.radius=radius*(1.+1e-9)+1e-9;
    }
    payload_pair_.resize(pairs_.size());
    for(size_t idx=0;idx<pairs_.size();++idx) {
      if(deadline_ && (idx%4096)==0)deadline_->require();
      const auto& p=pairs_[idx];
      const bool payload=bodies_[p.a].type==collision_detection::BodyTypes::ROBOT_ATTACHED||bodies_[p.b].type==collision_detection::BodyTypes::ROBOT_ATTACHED;
      payload_pair_[idx]=payload;
      // Widest band of any phase (search adds the payload reserve).
      const double band=p.limit+(payload?search_payload_reserve_:0.)+.001;
      const int ga=group_[p.a],gb=group_[p.b];
      if(ga==gb) {
        // Same rigid group (or both static): the true distance never changes,
        // so a conservative lower bound above the band clears it for good.
        const double aabb=bodies_[p.a].bounds.distance(bodies_[p.b].bounds);
        const double sphere=(sphere_world_[p.a]-sphere_world_[p.b]).norm()-sphere_radius_[p.a]-sphere_radius_[p.b];
        if((std::isfinite(aabb)&&aabb>band)||(std::isfinite(sphere)&&sphere>band)) {++constant_cleared_;continue;}
        constant_candidates_.push_back(int(idx));
      } else if(ga&&gb) {
        auto& gp=moving_pairs_[std::minmax(ga,gb)];gp.pairs.push_back(int(idx));gp.band=std::max(gp.band,band);
      }
      // Moving/static pairs are found through the static hierarchy per state.
    }
    std::vector<size_t> statics;for(size_t i=0;i<n;++i)if(!group_[i])statics.push_back(i);
    static_tree_.clear();static_tree_.reserve(2*statics.size()+1);
    if(!statics.empty())buildStatic(statics,0,statics.size());
  }
  int buildTree(std::vector<size_t>& ids,size_t first,size_t last) {
    const int index=tree_.size();tree_.emplace_back();
    if(last-first==1)tree_[index].body=int(ids[first]);
    else {
      Eigen::Vector3d lo=Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity()),hi=-lo;
      for(size_t k=first;k<last;++k) {const auto& b=bodies_[ids[k]].bounds;lo=lo.cwiseMin(b.min_);hi=hi.cwiseMax(b.max_);}
      Eigen::Index axis;(hi-lo).maxCoeff(&axis);const size_t middle=(first+last)/2;
      std::nth_element(ids.begin()+first,ids.begin()+middle,ids.begin()+last,[&](size_t a,size_t b) {
        const auto& x=bodies_[a].bounds;const auto& y=bodies_[b].bounds;
        return x.min_[axis]+x.max_[axis]<y.min_[axis]+y.max_[axis];
      });
      const int left=buildTree(ids,first,middle),right=buildTree(ids,middle,last);
      tree_[index].left=left;tree_[index].right=right;
    }
    return index;
  }
  void refitTree(int index) {
    auto& n=tree_[index];
    if(n.body>=0) {const auto& b=bodies_[n.body].bounds;n.lo=b.min_;n.hi=b.max_;return;}
    refitTree(n.left);refitTree(n.right);
    n.lo=tree_[n.left].lo.cwiseMin(tree_[n.right].lo);n.hi=tree_[n.left].hi.cwiseMax(tree_[n.right].hi);
  }
  void nearPairs(size_t a,int index,std::vector<int>& candidates,size_t& visited) const {
    ++visited;const auto& n=tree_[index];const auto& bounds=bodies_[a].bounds;
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
    size_t proxy_constant_cleared=0, proxy_group_cleared=0, proxy_sphere_cleared=0, proxy_unvisited_cleared=0,
           exact_candidates=0, states_with_exact=0;
    double prepare_s=0, geometry_prepare_s=0, pair_filter_s=0, update_s=0, broad_s=0, narrow_s=0, self_s=0, world_s=0;
  } stats;
  std::map<std::string,Stats> phases;
  struct Failure {std::string a,b;double distance=0,limit=0;bool self=false;std::string reason;};
  ClearanceWorkspace(const collision_detection::CollisionEnvFCL& env,
      const planning_scene::PlanningSceneConstPtr& scene,
      const std::function<double(const std::string&,BodyType,const std::string&,BodyType)>& required,
      bool explicit_convex_meshes=false,double search_payload_reserve=0.,NativeDeadlinePtr deadline={},bool tight_bounds=false,
      bool two_level=false)
    : CollisionEnvFCL(env,std::make_shared<collision_detection::World>(*scene->getWorld())),context_(scene->getCurrentState()),
      search_payload_reserve_(search_payload_reserve),deadline_(std::move(deadline)) {
    tight_bounds_=tight_bounds && !std::getenv("ECO65_DIAGNOSTIC_LOOSE_BOUNDS");
    const auto t=Clock::now();context_.updateCollisionBodyTransforms();context_.getAttachedBodies(attached_);
    constructFCLObjectRobot(context_,robot_);
    auto add=[&](const collision_detection::FCLCollisionObjectPtr& o) {
      if(deadline_)deadline_->require();
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
          // TWO_LEVEL reuses the identical robot solid of this process.
          auto convex=(two_level && b.link)?cachedConvex(b.link,b.shape,mesh):buildConvex(mesh);
          b.object=std::make_shared<fcl::CollisionObjectd>(convex,o->getTransform());
        }
        }
      }
      b.bounds=tight_bounds_?conservativeWorldBounds(*b.object):b.object->getAABB();
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
      if(deadline_ && b==a+1)deadline_->require();
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
    if(two_level)prepareTwoLevel(scene);
    stats.pair_filter_s=elapsed(filtering);stats.prepared_objects=bodies_.size();stats.eligible_pairs=pairs_.size();stats.prepare_s=elapsed(t);
  }
  // A sphere containing the solid: centre of its local AABB, radius from the
  // solid's own vertices (convex) or the local AABB half-diagonal (others),
  // rounded up. Distance(spheres) is a lower bound of distance(solids).
  static std::pair<Eigen::Vector3d,double> localSphere(const fcl::CollisionObjectd& object) {
    const auto* geometry=object.collisionGeometry().get();const auto& box=geometry->aabb_local;
    const Eigen::Vector3d c=(box.min_+box.max_)*.5;double r=0.;
    if(const auto* convex=dynamic_cast<const fcl::Convexd*>(geometry)) {
      for(const auto& v:convex->getVertices())r=std::max(r,(v-c).norm());
    } else r=((box.max_-box.min_)*.5).norm();
    if(!std::isfinite(r)||!c.allFinite())return {c,std::numeric_limits<double>::infinity()};
    return {c,r*(1.+1e-9)+1e-9};
  }
  bool twoLevel() const {return two_level_;}
  bool check(const moveit::core::RobotState& state,Failure& failure,const std::string& phase) {
    auto expired=[&](){if(!nativeExpired(deadline_))return false;
      failure.reason="NATIVE_ABSOLUTE_DEADLINE_EXPIRED";failure.distance=std::numeric_limits<double>::quiet_NaN();return true;};
    if(expired())return false;
    auto& stats=phases[phase];
    // Measurement only (M710_NATIVE_TIMING=1): report this state's deltas of
    // the existing broad-phase and exact FCL counters on every exit path.
    struct TimingDelta {
      bool on;const Stats before;const Stats& after;const std::string& phase;
      ~TimingDelta() {
        if(!on)return;
        NativeTiming::instance().workspace(phase,double(after.hierarchy_nodes_visited-before.hierarchy_nodes_visited),
          double(after.aabb_tests-before.aabb_tests),double(after.aabb_skips-before.aabb_skips),
          double(after.narrow_calls-before.narrow_calls),double(after.self_narrow-before.self_narrow),
          double(after.world_narrow-before.world_narrow),after.update_s-before.update_s,
          after.broad_s-before.broad_s,after.narrow_s-before.narrow_s);
      }
    } timing_delta{NativeTiming::enabled(),stats,stats,phase};
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
      if((i%64)==0 && expired())return false;
      auto& b=bodies_[i];
      const auto& transform=b.link?state.getCollisionBodyTransform(b.link,b.shape):
        state.getAttachedBody(b.name)->getGlobalCollisionBodyTransforms().at(b.shape);
      if(!transform.matrix().allFinite()) throw std::runtime_error("INVALID_FCL_TRANSFORM");
      b.object->setTransform(collision_detection::transform2fcl(transform));b.object->computeAABB();
      b.bounds=tight_bounds_?conservativeWorldBounds(*b.object):b.object->getAABB();
    }
    // A body AABB is identical for all of its pairs in this state. Validate it
    // once, and time the broad phase as a batch rather than reading the clock
    // twice for each of hundreds of thousands of pairs. The eligible pairs,
    // AABB distance cutoff and actual FCL narrow queries remain unchanged.
    for(const auto& body:bodies_) {
      const auto& bounds=body.bounds;
      if(!bounds.min_.allFinite() || !bounds.max_.allFinite())
        throw std::runtime_error("UNKNOWN_FCL_AABB");
    }
    stats.update_s+=elapsed(updating);
    const auto broad_start=Clock::now();const double narrow_before=stats.narrow_s;
    auto finish_broad=[&]() {stats.broad_s+=elapsed(broad_start)-(stats.narrow_s-narrow_before);};
    std::vector<int> nearby;const auto* selected=&all_pair_indices_;
    if(two_level_) {
      // Level 1: proxies. A pair is skipped only when a conservative lower
      // bound of its distance exceeds the same band the AABB test uses.
      const bool search=phase=="search";
      for(size_t i=0;i<moving_;++i)if(group_[i])sphere_world_[i]=bodies_[i].object->getTransform()*sphere_local_[i];
      for(size_t g=1;g<groups_.size();++g)groups_[g].world=bodies_[groups_[g].reference].object->getTransform()*groups_[g].center_ref;
      nearby=constant_candidates_;stats.proxy_constant_cleared+=constant_cleared_;
      auto band_of=[&](int idx){return pairs_[idx].limit+((search&&payload_pair_[idx])?search_payload_reserve_:0.)+.001;};
      size_t visited_pairs=constant_cleared_+constant_candidates_.size();
      for(const auto& entry:moving_pairs_) {
        if(expired()) {finish_broad();return false;}
        const auto& A=groups_[entry.first.first];const auto& B=groups_[entry.first.second];const auto& gp=entry.second;
        visited_pairs+=gp.pairs.size();
        if((A.world-B.world).norm()-A.radius-B.radius>gp.band) {stats.proxy_group_cleared+=gp.pairs.size();continue;}
        for(int idx:gp.pairs) {
          const auto& p=pairs_[idx];
          if((sphere_world_[p.a]-sphere_world_[p.b]).norm()-sphere_radius_[p.a]-sphere_radius_[p.b]>band_of(idx)) {++stats.proxy_sphere_cleared;continue;}
          nearby.push_back(idx);
        }
      }
      std::vector<size_t> statics;const size_t n=bodies_.size();
      for(size_t g=1;g<groups_.size();++g) {
        if(expired()) {finish_broad();return false;}
        const auto& G=groups_[g];statics.clear();
        if(!static_tree_.empty())staticNear(0,G.world,G.radius+maximum_band_,statics,stats.hierarchy_nodes_visited);
        for(size_t m:G.bodies)for(size_t s:statics) {
          const size_t a=std::min(m,s),b=std::max(m,s);const int idx=pair_index_[a*n+b];
          if(idx<0)continue;
          ++visited_pairs;
          if((sphere_world_[m]-sphere_world_[s]).norm()-sphere_radius_[m]-sphere_radius_[s]>band_of(idx)) {++stats.proxy_sphere_cleared;continue;}
          nearby.push_back(idx);
        }
      }
      // Moving/static pairs outside every group's static query are cleared by
      // the group sphere against the static bounds hierarchy.
      stats.proxy_unvisited_cleared+=pairs_.size()-std::min(pairs_.size(),visited_pairs);
      std::sort(nearby.begin(),nearby.end());selected=&nearby;
      stats.exact_candidates+=nearby.size();stats.aabb_skips+=pairs_.size()-nearby.size();
    } else if(hierarchical_) {
      refitTree(0);
      for(size_t a=0;a<moving_;++a) {
        if((a%32)==0 && expired())return false;
        nearPairs(a,0,nearby,stats.hierarchy_nodes_visited);
      }
      std::sort(nearby.begin(),nearby.end());selected=&nearby;
      stats.aabb_skips+=pairs_.size()-nearby.size();
    }
    const size_t narrow_at_start=stats.narrow_calls;
    struct ExactCount {Stats& s;size_t start;bool on;~ExactCount(){if(on&&s.narrow_calls>start)++s.states_with_exact;}} exact_count{stats,narrow_at_start,two_level_};
    for(int index:*selected) {
      if(expired()) {finish_broad();return false;}
      const auto& p=pairs_[index];
      const auto& a=bodies_[p.a];const auto& b=bodies_[p.b];++stats.aabb_tests;
      // The actual dense audit retains the original acceptance threshold.
      // A small, explicit search-only reserve keeps OMPL's coarser segments
      // away from the observed payload/neighbor clearance boundary.
      const bool payload=a.type==collision_detection::BodyTypes::ROBOT_ATTACHED || b.type==collision_detection::BodyTypes::ROBOT_ATTACHED;
      const double limit=p.limit+((phase=="search" && payload)?search_payload_reserve_:0.);
      const auto& ab=a.bounds;const auto& bb=b.bounds;
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
      if(expired()) {finish_broad();return false;}
      // Use the same FCL request/default solver and strict contact rejection as
      // MoveIt's reference callback; no signed-distance or geometry surrogate.
      if(!std::isfinite(d) || !std::isfinite(result.min_distance) || d<=0 || d+1e-9<limit) {
        failure={a.name,b.name,d,limit,p.self};finish_broad();return false;
      }
    }
    finish_broad();return true;
  }
  bool tightBounds() const {return tight_bounds_;}
};
}
