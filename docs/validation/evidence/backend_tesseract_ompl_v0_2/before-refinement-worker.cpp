// Native FK, state/motion validation and one seeded OMPL RRTConnect instance.
// JSON-lines transport; scene structures may be reused, trees/solutions never are.
#include <tesseract/environment/environment.h>
#include <tesseract/environment/commands/add_contact_managers_plugin_info_command.h>
#include <tesseract/common/plugin_info.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/common/collision_margin_data.h>
#include <tesseract/collision/discrete_contact_manager.h>
#include <tesseract/collision/types.h>
#include <tesseract/scene_graph/scene_state.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/MotionValidator.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/PlannerData.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/config.h>
#include <fcl/config.h>
#include <nlohmann/json.hpp>
#include <Eigen/SVD>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <deque>
#include <limits>
#include <map>
#include <cmath>
#include <algorithm>

using json = nlohmann::json;
namespace ob = ompl::base;
namespace og = ompl::geometric;
using Clock = std::chrono::steady_clock;
double elapsed(Clock::time_point t) { return std::chrono::duration<double>(Clock::now()-t).count(); }
Eigen::VectorXd vector(const json& a) {
  Eigen::VectorXd q(a.size());
  for (size_t i=0;i<a.size();++i) q[i]=a.at(i).get<double>();
  return q;
}
json array(const Eigen::VectorXd& q) {
  return std::vector<double>(q.data(),q.data()+q.size());
}
json matrix(const Eigen::Isometry3d& t) {
  json a=json::array();
  for(int i=0;i<4;++i) { json r=json::array(); for(int j=0;j<4;++j) r.push_back(t.matrix()(i,j)); a.push_back(r); }
  return a;
}

struct Profile {
  bool enabled=false;
  std::map<std::string,double> seconds;
  struct Scope {
    Profile& p; const char* name; Clock::time_point start;
    Scope(Profile& p_,const char* n):p(p_),name(n) {if(p.enabled) start=Clock::now();}
    ~Scope() {if(p.enabled) p.seconds[name]+=elapsed(start);}
  };
};

struct Context {
  std::unique_ptr<tesseract::environment::Environment> env;
  std::unique_ptr<tesseract::collision::DiscreteContactManager> manager;
  std::string key, cancel_file, termination;
  json scene, failure, first_failed_sample;
  struct Joint {std::string link; Eigen::Vector3d axis; bool prismatic;};
  std::vector<Joint> joints;
  Eigen::VectorXd lower,upper;
  Eigen::MatrixXd jacobian;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd;
  double joint_margin=0, condition_limit=0, radial_limit=0, edge_resolution=0;
  bool radial_enabled=false;
  std::vector<std::string> moving_names;
  tesseract::common::VectorIsometry3d moving_poses;
  tesseract::collision::ContactResultMap contacts;
  struct Cached {bool valid; json failure;};
  std::unordered_map<std::string,Cached> cache;
  std::deque<std::string> cache_fifo;
  static constexpr size_t cache_capacity=8192;
  uint64_t cache_hits=0, cache_evictions=0;
  std::map<std::string,uint64_t> computed_rejections;
  std::string termination_detail;
  Profile profile;
  uint64_t requests=0, valid_edges=0, invalid_edges=0, incomplete_edges=0;
  std::map<std::string,uint64_t> rejections;
  bool trace=false; json trace_points;
  std::vector<std::string> names;
  uint64_t states=0, edges=0, samples=0, max_states=0, termination_checks=0, collision_queries=0;
  double wall=0, resolution=0.0003125;
  Clock::time_point started;
  bool stopped() {
    Profile::Scope timer(profile,"budget_cancel_poll_s");
    ++termination_checks;
    if(!termination.empty()) return true;
    if(!cancel_file.empty() && std::filesystem::exists(cancel_file)) {termination="CANCELLED";termination_detail="CANCEL_MARKER";}
    else if(states>=max_states) {termination="BUDGET_EXHAUSTED";termination_detail="ACTUAL_STATE_COMPUTATION_LIMIT";}
    else if(requests>=max_states*32) {termination="BUDGET_EXHAUSTED";termination_detail="STATE_REQUEST_GUARD_32X";}
    else if(wall>0 && elapsed(started)>=wall) {termination="BUDGET_EXHAUSTED";termination_detail="EXPLICIT_WALL_LIMIT";}
    return !termination.empty();
  }
  Eigen::VectorXd q(const ob::State* state) const {
    const auto* raw=state->as<ob::RealVectorStateSpace::StateType>();
    return Eigen::Map<const Eigen::VectorXd>(raw->values,names.size());
  }
  void load(const json& input) {
    scene=input; names=scene.at("joint_names").get<std::vector<std::string>>();
    auto next=std::make_unique<tesseract::environment::Environment>();
    auto locator=std::make_shared<tesseract::common::GeneralResourceLocator>();
    if(!next->init(scene.at("urdf").get<std::string>(),scene.at("srdf").get<std::string>(),locator))
      throw std::runtime_error("Tesseract URDF/SRDF initialization failed");
    tesseract::common::ContactManagersPluginInfo plugins;
    plugins.search_paths={TESSERACT_PLUGIN_LIBDIR};
    plugins.search_libraries={"tesseract_collision_fcl_factories"};
    plugins.discrete_plugin_infos.plugins["FCLDiscreteBVHManager"].class_name="FCLDiscreteBVHManagerFactory";
    if(!next->applyCommand(std::make_shared<tesseract::environment::AddContactManagersPluginInfoCommand>(plugins)))
      throw std::runtime_error("FCL plugin registration failed");
    auto collision=next->getDiscreteContactManager("FCLDiscreteBVHManager");
    if(!collision) throw std::runtime_error("FCLDiscreteBVHManager unavailable; no manager fallback");
    for(const auto& name:scene.at("collision_objects")) {
      const auto id=name.get<std::string>();
      if(!collision->hasCollisionObject(id) || !collision->isCollisionObjectEnabled(id) ||
         collision->getCollisionObjectGeometries(id).empty())
        throw std::runtime_error("Missing or disabled native collision geometry: "+id);
    }
    collision->setActiveCollisionObjects(scene.at("active_links").get<std::vector<std::string>>());
    tesseract::common::CollisionMarginData margins(scene.at("default_margin").get<double>());
    for(const auto& pair:scene.at("pair_margins"))
      margins.setCollisionMargin(pair[0].get<std::string>(),pair[1].get<std::string>(),pair[2].get<double>());
    collision->setCollisionMarginData(margins);
    env=std::move(next); manager=std::move(collision); key=scene.at("fingerprint");
    contacts.release();
    // Parse immutable JSON only when the scene changes.
    joints.clear();lower.resize(names.size());upper.resize(names.size());
    const auto& c=scene.at("constraints");
    joint_margin=c.at("joint_margin");condition_limit=c.at("maximum_jacobian_condition");
    radial_enabled=!c.at("radial_limit").is_null();radial_limit=radial_enabled?c.at("radial_limit").get<double>():0.;
    edge_resolution=c.at("edge_resolution_rad");
    if(!std::isfinite(joint_margin) || joint_margin<0 || !std::isfinite(condition_limit) || condition_limit<=0 ||
       (radial_enabled && (!std::isfinite(radial_limit) || radial_limit<=0))) throw std::runtime_error("invalid kinematic constraints");
    for(size_t i=0;i<names.size();++i) {
      lower[i]=scene.at("joint_limits").at(i).at(0);upper[i]=scene.at("joint_limits").at(i).at(1);
      const auto& j=scene.at("joints").at(i);
      auto axis=vector(j.at("axis"));
      if(axis.size()!=3 || !axis.allFinite() || axis.norm()==0 || !std::isfinite(lower[i]) ||
         !std::isfinite(upper[i]) || lower[i]>=upper[i]) throw std::runtime_error("invalid joint definition");
      joints.push_back({j.at("link").get<std::string>(),axis,j.value("type",std::string("revolute"))=="prismatic"});
    }
    jacobian.resize(6,names.size());
    // Update all objects once, then only kinematically affected collision links.
    // Collision active/filter lists are intentionally unchanged, including base.
    manager->setCollisionObjectsTransform(env->getState(names,Eigen::VectorXd::Zero(names.size())).link_transforms);
    const auto affected=env->getActiveLinkNames(names);
    const std::unordered_set<std::string> moving(affected.begin(),affected.end());
    moving_names.clear();
    for(const auto& name:manager->getCollisionObjects()) if(moving.count(name)) moving_names.push_back(name);
    moving_poses.resize(moving_names.size());
  }
  bool valid(const Eigen::VectorXd& qv) {
    ++requests;
    Profile::Scope timer(profile,"state_check_inclusive_s");
    if(stopped()) {++rejections[termination];failure=nullptr;return false;}
    // Exact IEEE double bytes only. Cache lifetime is one immutable request;
    // every scene/attachment/stage/policy/rule change (and every call) clears it.
    const std::string state_key(reinterpret_cast<const char*>(qv.data()),qv.size()*sizeof(double));
    auto found=cache.find(state_key);
    if(found!=cache.end()) {
      ++cache_hits;failure=found->second.failure;
      if(!found->second.valid) ++rejections[failure.at("reason").get<std::string>()];
      return found->second.valid;
    }
    ++states;failure=nullptr;
    const bool accepted=compute(qv);
    if(!accepted) {
      const auto reason=failure.at("reason").get<std::string>();
      ++computed_rejections[reason];++rejections[reason];
    }
    if(cache.size()>=cache_capacity) {cache.erase(cache_fifo.front());cache_fifo.pop_front();++cache_evictions;}
    cache.emplace(state_key,Cached{accepted,failure});cache_fifo.push_back(state_key);
    return accepted;
  }
  bool compute(const Eigen::VectorXd& qv) {
    if(qv.size()!=static_cast<int>(names.size()) || !qv.allFinite()) {
      failure={{"reason","JOINT_VECTOR_INVALID"}}; return false;
    }
    const double actual_margin=(qv-lower).cwiseMin(upper-qv).minCoeff();
    for(int i=0;i<qv.size();++i) {
      if(qv[i]<lower[i]+joint_margin || qv[i]>upper[i]-joint_margin) {
        failure={{"reason","JOINT_MARGIN"},{"joint",names[i]},{"actual_margin_rad",actual_margin},{"required_margin_rad",joint_margin}}; return false;
      }
    }
    auto fk_start=profile.enabled?Clock::now():Clock::time_point{};
    const auto state=env->getState(names,qv);
    if(profile.enabled) profile.seconds["fk_scene_state_s"]+=elapsed(fk_start);
    auto jac_start=profile.enabled?Clock::now():Clock::time_point{};
    const auto& transforms=state.link_transforms;
    const Eigen::Vector3d tcp=transforms.at("backend_tcp").translation();
    for(int i=0;i<qv.size();++i) {
      const auto& j=joints[i];const auto& t=transforms.at(j.link);
      Eigen::Vector3d axis=t.linear()*j.axis;axis.normalize();
      if(j.prismatic) {jacobian.block<3,1>(0,i)=axis;jacobian.block<3,1>(3,i).setZero();}
      else {jacobian.block<3,1>(0,i)=axis.cross(Eigen::Vector3d(tcp-t.translation()));jacobian.block<3,1>(3,i)=axis;}
    }
    svd.compute(jacobian);
    const auto& singular=svd.singularValues();
    const double condition=singular[0]/singular[singular.size()-1];
    if(profile.enabled) profile.seconds["jacobian_svd_s"]+=elapsed(jac_start);
    double radial=0;
    if(radial_enabled) {
      Eigen::Vector3d flange=transforms.at("base_link").inverse()*transforms.at("flange").translation();
      radial=flange.head<2>().norm();
    }
    auto context=[&]() {
      return json{{"tcp_world",matrix(transforms.at("backend_tcp"))},
        {"jacobian_condition",std::isfinite(condition)?json(condition):json(nullptr)},
        {"maximum_jacobian_condition",condition_limit},{"joint_margin_rad",actual_margin},
        {"required_joint_margin_rad",joint_margin},{"radial_m",radial_enabled?json(radial):json(nullptr)},
        {"radial_limit_m",radial_enabled?json(radial_limit):json(nullptr)}};
    };
    if(!std::isfinite(condition) || condition>condition_limit) {
      failure=context();failure["reason"]="SINGULARITY";return false;
    }
    if(radial_enabled && radial>radial_limit) {
      failure=context();failure["reason"]="RADIAL_REACH";return false;
    }
    auto transform_start=profile.enabled?Clock::now():Clock::time_point{};
    for(size_t i=0;i<moving_names.size();++i) moving_poses[i]=transforms.at(moving_names[i]);
    manager->setCollisionObjectsTransform(moving_names,moving_poses);
    if(profile.enabled) profile.seconds["collision_transform_s"]+=elapsed(transform_start);
    contacts.clear();++collision_queries;
    auto contact_start=profile.enabled?Clock::now():Clock::time_point{};
    manager->contactTest(contacts,tesseract::collision::ContactRequest(tesseract::collision::ContactTestType::FIRST));
    if(profile.enabled) profile.seconds["fcl_contact_test_s"]+=elapsed(contact_start);
    if(!contacts.empty()) {
      // ContactResultMap::clear preserves empty pair vectors for reuse.
      // Its first map entry need not contain this query's first contact.
      const auto entry=std::find_if(contacts.begin(),contacts.end(),
                                   [](const auto& item){return !item.second.empty();});
      if(entry==contacts.end()) throw std::runtime_error("inconsistent contact result map");
      const auto& contact=entry->second.front();
      failure=context();failure.update({{"reason","NATIVE_COLLISION"},{"pair",contact.link_names},{"distance_m",contact.distance},
        {"required_margin_m",manager->getCollisionMarginData().getCollisionMargin(contact.link_names[0],contact.link_names[1])}});
      return false;
    }
    return true;
  }
};

struct DenseMotion final : ob::MotionValidator {
  Context& ctx;
  DenseMotion(const ob::SpaceInformationPtr& si,Context& context):ob::MotionValidator(si),ctx(context){}
  bool check(const ob::State* a,const ob::State* b,std::pair<ob::State*,double>* last) const {
    // Parent timer minus nested state time gives exclusive edge scheduling.
    struct EdgeTimer {
      Context& c; Clock::time_point began; double state_before=0;
      explicit EdgeTimer(Context& c_):c(c_) {if(c.profile.enabled) {began=Clock::now();state_before=c.profile.seconds["state_check_inclusive_s"];}}
      ~EdgeTimer() {if(c.profile.enabled) {double total=elapsed(began);c.profile.seconds["edge_check_inclusive_s"]+=total;
        c.profile.seconds["edge_schedule_s"]+=total-(c.profile.seconds["state_check_inclusive_s"]-state_before);}}
    } timer(ctx);
    ++ctx.edges;
    const Eigen::VectorXd start=ctx.q(a),delta=ctx.q(b)-start;
    const double count=std::max(1.,std::max(std::ceil(delta.cwiseAbs().sum()/ctx.resolution),
                                              2*std::ceil(delta.cwiseAbs().maxCoeff()/ctx.edge_resolution)));
    if(!std::isfinite(count) || count>1.e9) throw std::runtime_error("unsupported dense grid extent");
    const size_t n=static_cast<size_t>(count);
    // Breadth-first bisection visits endpoints, midpoint, then every remaining
    // original integer-grid index exactly once. No coarse-layer acceptance.
    std::vector<std::pair<size_t,size_t>> intervals;
    intervals.emplace_back(0,n);
    size_t cursor=0,visited=0;
    auto visit=[&](size_t i) {
      ++ctx.samples;++visited;
      const Eigen::VectorXd sample=start+(double(i)/n)*delta;
      if(ctx.trace) ctx.trace_points.push_back({{"edge",ctx.edges-1},{"index",i},{"subdivisions",n},{"q_rad",array(sample)}});
      if(ctx.valid(sample)) return true;
      if(ctx.termination.empty()) ++ctx.invalid_edges; else ++ctx.incomplete_edges;
      if(ctx.first_failed_sample.is_null()) ctx.first_failed_sample={{"edge",ctx.edges-1},{"index",i},{"subdivisions",n},
        {"fraction",double(i)/n},{"q_rad",array(sample)},{"failure",ctx.failure},
        {"classification",ctx.termination.empty()?"STATE_REJECTION":"C_INCOMPLETE"}};
      // A rejected out-of-order sample says nothing about its preceding prefix.
      // Return the conservative start witness, never a guessed predecessor.
      if(last) {last->second=0.;if(last->first) si_->copyState(last->first,a);}
      ++invalid_;return false;
    };
    if(!visit(0) || !visit(n)) return false;
    while(cursor<intervals.size()) {
      const auto [lo,hi]=intervals[cursor++];
      if(hi-lo<=1) continue;
      const size_t mid=lo+(hi-lo)/2;
      if(!visit(mid)) return false;
      intervals.emplace_back(lo,mid);intervals.emplace_back(mid,hi);
    }
    if(visited!=n+1) throw std::runtime_error("incomplete dense grid coverage");
    ++ctx.valid_edges;++valid_;return true;
  }
  bool checkMotion(const ob::State* a,const ob::State* b) const override { return check(a,b,nullptr); }
  bool checkMotion(const ob::State* a,const ob::State* b,std::pair<ob::State*,double>& last) const override { return check(a,b,&last); }
};
struct SeededSampler final : ob::RealVectorStateSampler {
  SeededSampler(const ob::StateSpace* s,uint32_t seed):ob::RealVectorStateSampler(s) {rng_.setLocalSeed(seed);}
};
struct SeededConnect final : og::RRTConnect {
  SeededConnect(const ob::SpaceInformationPtr& si,uint32_t seed):og::RRTConnect(si) {rng_.setLocalSeed(seed);}
};

json solve(Context& ctx,const json& r) {
  auto total=Clock::now();
  // One supported geometric contract. Reject unsupported inputs before loading
  // geometry, including direct protocol callers that bypass the Python adapter.
  const auto& constraints=r.at("scene").at("constraints");
  auto positive=[&](const char* key,double upper) {
    if(!constraints.contains(key) || !constraints[key].is_number()) return false;
    const double value=constraints[key].get<double>();
    return std::isfinite(value) && value>0 && value<=upper;
  };
  if(!r.value("refinement",json(0)).is_number_integer())
    return {{"status","UNSUPPORTED_CONSTRAINT"},{"error","refinement must be an integer"}};
  const int refinement=r.value("refinement",0);
  const double base_resolution=.00125/4.;
  const double effective_resolution=std::ldexp(base_resolution,-refinement);
  if(!positive("lever_arm_m",4.) || constraints["lever_arm_m"]!=4. ||
     !positive("point_motion_bound_m",.00125) || constraints["point_motion_bound_m"]!=.00125 ||
     !positive("edge_resolution_rad",.055) || refinement<0 || refinement>7 ||
     !r.value("refinement",json(0)).is_number_integer() ||
     r.value("l1_resolution_rad",effective_resolution)!=effective_resolution) {
    return {{"status","UNSUPPORTED_CONSTRAINT"},{"error","supported subdivision contract: 4m / 1.25mm, dyadic refinement 0..7, edge resolution (0,.055]"}};
  }
  const std::string operation=r.value("operation",std::string("plan"));
  if(operation!="plan" && operation!="audit") return {{"status","UNSUPPORTED_CONSTRAINT"},{"error","unknown operation"}};
  json result={{"candidate_found",false},{"exact_solution",false},{"native_validated",false},{"path",json::array()},
    {"versions",{{"tesseract","0.35.0"},{"tesseract_planning",nullptr},
      {"pipeline","direct OMPL; no TaskComposer, TrajOpt or time parameterization"},
      {"ompl",OMPL_VERSION},{"fcl",FCL_VERSION},{"manager","FCLDiscreteBVHManager"},{"implementation","native C++17 worker"}}}};
  bool reused=ctx.env && ctx.key==r.at("scene").at("fingerprint").get<std::string>();
  if(reused && ctx.scene!=r.at("scene")) return {{"status","STALE_SCENE"},{"error","scene content changed under unchanged identity"}};
  result["effective_rule"]={{"name","fixed_4m_1.25mm_l1_grid_v1"},{"lever_arm_m",4.},
    {"point_motion_bound_m",.00125},{"base_l1_resolution_rad",base_resolution},
    {"refinement",refinement},{"l1_resolution_rad",effective_resolution},
    {"edge_resolution_rad",constraints.at("edge_resolution_rad")},
    {"intervals","max(1, ceil(L1/l1_resolution_rad), 2*ceil(Linf/edge_resolution_rad))"}};
  auto init=Clock::now(); if(!reused) ctx.load(r.at("scene"));
  result["timings"]["environment_init_s"]=reused?0.:elapsed(init);
  result["environment_reused"]=reused;
  result["collision_object_count"]=ctx.manager->getCollisionObjects().size();
  result["moving_collision_object_count"]=ctx.moving_names.size();
  result["static_collision_object_count"]=ctx.manager->getCollisionObjects().size()-ctx.moving_names.size();
  ctx.states=ctx.edges=ctx.samples=ctx.termination_checks=ctx.collision_queries=0;ctx.termination.clear();ctx.failure=nullptr;
  ctx.requests=ctx.valid_edges=ctx.invalid_edges=ctx.incomplete_edges=0;ctx.rejections.clear();
  ctx.cache.clear();ctx.cache_fifo.clear();ctx.cache_hits=ctx.cache_evictions=0;
  ctx.computed_rejections.clear();ctx.first_failed_sample=nullptr;ctx.termination_detail.clear();
  ctx.profile.enabled=r.value("profile",false);ctx.profile.seconds.clear();
  ctx.trace=r.value("trace_samples",false);ctx.trace_points=json::array();
  ctx.started=Clock::now();ctx.max_states=r.at("max_state_checks");ctx.cancel_file=r.value("cancel_file","");
  ctx.wall=r.value("wall_time_s",0.);ctx.resolution=effective_resolution;
  if(ctx.resolution<=0 || ctx.resolution>.0003125 || ctx.max_states==0 || ctx.max_states>1000000000 || !std::isfinite(ctx.wall) || ctx.wall<0) throw std::runtime_error("invalid native resource/grid budget");
  auto space=std::make_shared<ob::RealVectorStateSpace>(ctx.names.size());
  ob::RealVectorBounds bounds(ctx.names.size());
  for(size_t i=0;i<ctx.names.size();++i) {
    bounds.setLow(i,ctx.scene["joint_limits"][i][0]);bounds.setHigh(i,ctx.scene["joint_limits"][i][1]);
  }
  space->setBounds(bounds);
  uint32_t seed=r.at("seed");
  space->setStateSamplerAllocator([seed](const ob::StateSpace* s){return std::make_shared<SeededSampler>(s,seed);});
  auto si=std::make_shared<ob::SpaceInformation>(space);
  si->setStateValidityChecker([&ctx](const ob::State* s){return ctx.valid(ctx.q(s));});
  auto motion=std::make_shared<DenseMotion>(si,ctx);si->setMotionValidator(motion);si->setup();
  ob::ScopedState<> start(space),goal(space);
  for(size_t i=0;i<ctx.names.size();++i) {start[i]=r["q_start"][i];goal[i]=r["q_goal"][i];}
  std::string status;
  const bool audit=r.value("operation",std::string("plan"))=="audit";
  if(audit) {
    if(r.contains("states")) {
      result["state_verdicts"]=json::array();
      for(const auto& state:r.at("states")) {
        bool accepted=ctx.valid(vector(state));
        result["state_verdicts"].push_back(ctx.termination.empty()?json(accepted):json(nullptr));
        if(!accepted && !ctx.termination.empty()) result["first_interruption"]={{"q_rad",state},{"status",ctx.termination}};
        else if(!accepted && !result.contains("first_rejection")) result["first_rejection"]={{"q_rad",state},{"failure",ctx.failure}};
        if(!ctx.termination.empty()) break;
      }
      bool clear=!result.contains("first_rejection");
      status=ctx.termination.empty()?(clear?"AUDIT_VALID":"AUDIT_INVALID"):ctx.termination;
    } else {
      const auto& path=r.at("audit_path");
      if(path.empty()) throw std::runtime_error("empty audit path");
      for(const auto& point:path) if(point.size()!=ctx.names.size() || !vector(point).allFinite())
        throw std::runtime_error("invalid audit joint vector");
      bool clear=true;
      if(path.size()==1) clear=ctx.valid(vector(path[0]));
      for(size_t edge=0;edge+1<path.size() && clear;++edge) {
        for(size_t j=0;j<ctx.names.size();++j) {start[j]=path[edge][j];goal[j]=path[edge+1][j];}
        clear=motion->checkMotion(start.get(),goal.get());
      }
      status=ctx.termination.empty()?(clear?"AUDIT_VALID":"AUDIT_INVALID"):ctx.termination;
    }
    result["audit_complete"]=ctx.termination.empty() &&
      (r.contains("states")?result["state_verdicts"].size()==r.at("states").size():status=="AUDIT_VALID");
    if(ctx.trace) result["checked_samples"]=ctx.trace_points;
  }
  auto endpoint=Clock::now();
  if(!audit && !ctx.valid(vector(r.at("q_start")))) status=ctx.termination.empty()?"INVALID_START":ctx.termination;
  else if(!audit && !ctx.valid(vector(r.at("q_goal")))) status=ctx.termination.empty()?"INVALID_GOAL":ctx.termination;
  result["timings"]["endpoint_check_s"]=elapsed(endpoint);
  result["timings"]["direct_check_s"]=0.;result["timings"]["ompl_solve_s"]=0.;
  result["timings"]["path_conversion_s"]=0.;
  if(status.empty()) {
    auto direct=Clock::now();bool clear=motion->checkMotion(start.get(),goal.get());
    result["timings"]["direct_check_s"]=elapsed(direct);result["direct_valid"]=clear;
    if(clear) {
      result["path"]={r["q_start"],r["q_goal"]};status="CANDIDATE";
    } else if(!ctx.termination.empty()) status=ctx.termination;
    else {
      auto problem=std::make_shared<ob::ProblemDefinition>(si);problem->setStartAndGoalStates(start,goal,1e-10);
      auto planner=std::make_shared<SeededConnect>(si,seed);planner->setRange(r.value("range_rad",.18));
      planner->setProblemDefinition(problem);planner->setup();
      auto search=Clock::now();auto solved=planner->solve(ob::PlannerTerminationCondition([&ctx](){return ctx.stopped();}));
      result["timings"]["ompl_solve_s"]=elapsed(search);
      result["ompl_status"]=solved.asString();
      ob::PlannerData data(si);planner->getPlannerData(data);
      result["search_progress"]={{"tree_vertices",data.numVertices()},{"tree_edges",data.numEdges()},
        {"start_vertices",data.numStartVertices()},{"goal_vertices",data.numGoalVertices()},
        {"approximate_goal_distance",problem->hasApproximateSolution()?json(problem->getSolutionDifference()):json(nullptr)},
        {"iterations",nullptr}};
      if(solved==ob::PlannerStatus::EXACT_SOLUTION && problem->hasExactSolution()) {
        auto conversion=Clock::now();
        auto path=std::dynamic_pointer_cast<og::PathGeometric>(problem->getSolutionPath());
        if(!path) throw std::runtime_error("OMPL did not return a geometric path");
        for(auto* state:path->getStates()) result["path"].push_back(array(ctx.q(state)));
        result["timings"]["path_conversion_s"]=elapsed(conversion);
        status="CANDIDATE";
      } else {
        result["approximate_solution_found"]=problem->hasApproximateSolution();
        result["candidate_found"]=problem->hasApproximateSolution();
        status=ctx.termination.empty()?"BUDGET_EXHAUSTED":ctx.termination;
      }
    }
  }
  if(status=="CANDIDATE") {
    result["candidate_found"]=true;result["exact_solution"]=true;result["native_validated"]=true;
    if((vector(result["path"].front())-vector(r["q_start"])).norm()>1e-9 ||
       (vector(result["path"].back())-vector(r["q_goal"])).norm()>1e-9)
      throw std::runtime_error("exact solution endpoint mismatch");
  }
  if(!result.contains("search_progress")) result["search_progress"]={{"tree_vertices",0},{"tree_edges",0},{"iterations",nullptr},{"approximate_goal_distance",nullptr}};
  result["status"]=status;result["failure"]=ctx.failure;
  result["counters"]={{"state_checks",ctx.states},{"edge_checks",ctx.edges},{"subdivision_samples",ctx.samples},
                      {"termination_checks",ctx.termination_checks},{"collision_queries",ctx.collision_queries}};
  result["counters"]["state_requests"]=ctx.requests;
  result["counters"]["valid_edges"]=ctx.valid_edges;
  result["counters"]["invalid_edges"]=ctx.invalid_edges;
  result["counters"]["incomplete_edges"]=ctx.incomplete_edges;
  result["rejections"]=ctx.rejections;
  result["computed_rejections"]=ctx.computed_rejections;
  result["first_failed_sample"]=ctx.first_failed_sample;
  result["termination_detail"]=ctx.termination_detail;
  result["counters"]["actual_state_computations"]=ctx.states;
  result["counters"]["cache_hits"]=ctx.cache_hits;
  result["counters"]["cache_evictions"]=ctx.cache_evictions;
  result["counters"]["ompl_iterations"]=nullptr;
  result["cache"]={{"scope","fresh per immutable request, exact IEEE state bytes"},{"capacity",ctx.cache_capacity},
    {"scene_fingerprint",ctx.key},{"refinement",refinement}};
  result["counter_semantics"]={{"state_checks","actual uncached computations, including bounds/kinematic rejection"},
    {"state_requests","all valid() calls including cache hits and interrupted calls"},
    {"subdivision_samples","grid sample requests, including cached and interrupted samples"},
    {"request_guard",ctx.max_states*32}};
  result["profile"]={{"enabled",ctx.profile.enabled},{"seconds",ctx.profile.seconds}};
  result["timings"]["native_total_s"]=elapsed(total);
  result["scene_fingerprint"]=ctx.key;
  if(r.contains("fk_probes")) {
    result["fk_probes"]=json::array();
    for(const auto& probe:r["fk_probes"]) {
      const auto state=ctx.env->getState(ctx.names,vector(probe));json frames=json::object();
      for(const auto& item:state.link_transforms) frames[item.first]=matrix(item.second);
      result["fk_probes"].push_back(frames);
    }
  }
  return result;
}

int main() {
  // Libraries occasionally log to stdout; reserve the original stream for IPC.
  auto* protocol=std::cout.rdbuf();std::cout.rdbuf(std::cerr.rdbuf());
  std::ostream output(protocol);Context context;
  output << json({{"ready",true},{"protocol",1}}).dump() << std::endl;
  std::string line;
  while(std::getline(std::cin,line)) {
    try {output << solve(context,json::parse(line)).dump() << std::endl;}
    catch(const std::exception& e) {
      context.env.reset();context.manager.reset();context.key.clear();
      output << json({{"status","INTERNAL_ERROR"},{"error",e.what()}}).dump() << std::endl;
    }
  }
}
