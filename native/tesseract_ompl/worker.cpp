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

struct Context {
  std::unique_ptr<tesseract::environment::Environment> env;
  std::unique_ptr<tesseract::collision::DiscreteContactManager> manager;
  std::string key, cancel_file, termination;
  json scene, failure;
  std::vector<std::string> names;
  uint64_t states=0, edges=0, samples=0, max_states=0, termination_checks=0, collision_queries=0;
  double wall=0, resolution=0.0003125;
  Clock::time_point started;
  bool stopped() {
    ++termination_checks;
    if(!termination.empty()) return true;
    if(!cancel_file.empty() && std::filesystem::exists(cancel_file)) termination="CANCELLED";
    else if(states>=max_states || (wall>0 && elapsed(started)>=wall)) termination="BUDGET_EXHAUSTED";
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
  }
  bool valid(const Eigen::VectorXd& qv) {
    if(stopped()) return false;
    ++states;
    const auto& constraints=scene.at("constraints");
    if(qv.size()!=static_cast<int>(names.size()) || !qv.allFinite()) {
      failure={{"reason","JOINT_VECTOR_INVALID"}}; return false;
    }
    const double margin=constraints.at("joint_margin");
    for(int i=0;i<qv.size();++i) {
      if(qv[i]<scene["joint_limits"][i][0].get<double>()+margin || qv[i]>scene["joint_limits"][i][1].get<double>()-margin) {
        failure={{"reason","JOINT_MARGIN"},{"joint",names[i]}}; return false;
      }
    }
    const auto state=env->getState(names,qv);
    const auto& transforms=state.link_transforms;
    const auto tcp=transforms.at("backend_tcp").translation();
    Eigen::MatrixXd jacobian(6,qv.size());
    for(int i=0;i<qv.size();++i) {
      const auto& j=scene["joints"][i];
      const auto& t=transforms.at(j.at("link").get<std::string>());
      Eigen::Vector3d axis=t.linear()*vector(j.at("axis")); axis.normalize();
      if(j.value("type",std::string("revolute"))=="prismatic") {
        jacobian.block<3,1>(0,i)=axis;jacobian.block<3,1>(3,i).setZero();
      } else {
        jacobian.block<3,1>(0,i)=axis.cross(Eigen::Vector3d(tcp-t.translation()));
        jacobian.block<3,1>(3,i)=axis;
      }
    }
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian);
    const auto singular=svd.singularValues();
    double condition=singular[0]/singular[singular.size()-1];
    if(!std::isfinite(condition) || condition>constraints.at("maximum_jacobian_condition").get<double>()) {
      failure={{"reason","SINGULARITY"},{"jacobian_condition",std::isfinite(condition)?json(condition):json(nullptr)}}; return false;
    }
    if(!constraints.at("radial_limit").is_null()) {
      Eigen::Vector3d flange=transforms.at("base_link").inverse()*transforms.at("flange").translation();
      if(flange.head<2>().norm()>constraints.at("radial_limit").get<double>()) {
        failure={{"reason","RADIAL_REACH"},{"radial_m",flange.head<2>().norm()}}; return false;
      }
    }
    manager->setCollisionObjectsTransform(transforms);
    tesseract::collision::ContactResultMap contacts;
    ++collision_queries;
    manager->contactTest(contacts,tesseract::collision::ContactRequest(tesseract::collision::ContactTestType::FIRST));
    if(!contacts.empty()) {
      const auto& contact=contacts.begin()->second.front();
      failure={{"reason","NATIVE_COLLISION"},{"pair",contact.link_names},{"distance_m",contact.distance}};
      return false;
    }
    failure=nullptr; return true;
  }
};

struct DenseMotion final : ob::MotionValidator {
  Context& ctx;
  DenseMotion(const ob::SpaceInformationPtr& si,Context& context):ob::MotionValidator(si),ctx(context){}
  bool check(const ob::State* a,const ob::State* b,std::pair<ob::State*,double>* last) const {
    ++ctx.edges;
    const Eigen::VectorXd start=ctx.q(a),delta=ctx.q(b)-start;
    // Same joint-linear interpolation as authority; L1 grid is at least as
    // fine as its 4m lever-arm / 1.25mm final sampling bound.
    const double rad=ctx.scene["constraints"]["edge_resolution_rad"].get<double>();
    const size_t n=std::max<size_t>(1,std::max(std::ceil(delta.cwiseAbs().sum()/ctx.resolution),
                                              2*std::ceil(delta.cwiseAbs().maxCoeff()/rad)));
    for(size_t i=0;i<=n;++i) {
      ++ctx.samples;
      if(!ctx.valid(start+(double(i)/n)*delta)) {
        if(last) {
          last->second=i==0?0.:double(i-1)/n;
          if(last->first) si_->getStateSpace()->interpolate(a,b,last->second,last->first);
        }
        ++invalid_; return false;
      }
    }
    ++valid_; return true;
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
  json result={{"candidate_found",false},{"exact_solution",false},{"native_validated",false},{"path",json::array()},
    {"versions",{{"tesseract","0.35.0"},{"tesseract_planning",nullptr},
      {"pipeline","direct OMPL; no TaskComposer, TrajOpt or time parameterization"},
      {"ompl",OMPL_VERSION},{"fcl",FCL_VERSION},{"manager","FCLDiscreteBVHManager"},{"implementation","native C++17 worker"}}}};
  bool reused=ctx.env && ctx.key==r.at("scene").at("fingerprint").get<std::string>();
  auto init=Clock::now(); if(!reused) ctx.load(r.at("scene"));
  result["timings"]["environment_init_s"]=reused?0.:elapsed(init);
  result["environment_reused"]=reused;
  result["collision_object_count"]=ctx.manager->getCollisionObjects().size();
  ctx.states=ctx.edges=ctx.samples=ctx.termination_checks=ctx.collision_queries=0;ctx.termination.clear();ctx.failure=nullptr;
  ctx.started=Clock::now();ctx.max_states=r.at("max_state_checks");ctx.cancel_file=r.value("cancel_file","");
  ctx.wall=r.value("wall_time_s",0.);ctx.resolution=r.value("l1_resolution_rad",.0003125);
  if(ctx.resolution<=0 || ctx.resolution>.0003125 || ctx.max_states==0) throw std::runtime_error("invalid native resource/grid budget");
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
  auto endpoint=Clock::now();
  if(!ctx.valid(vector(r.at("q_start")))) status=ctx.termination.empty()?"INVALID_START":ctx.termination;
  else if(!ctx.valid(vector(r.at("q_goal")))) status=ctx.termination.empty()?"INVALID_GOAL":ctx.termination;
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
  result["status"]=status;result["failure"]=ctx.failure;
  result["counters"]={{"state_checks",ctx.states},{"edge_checks",ctx.edges},{"subdivision_samples",ctx.samples},
                      {"termination_checks",ctx.termination_checks},{"collision_queries",ctx.collision_queries}};
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
