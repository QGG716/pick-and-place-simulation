
#include "search_trace.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/util/RandomNumbers.h>
#include <iostream>
#include <chrono>
#include <thread>
int main(int argc,char** argv) {
  namespace ob=ompl::base;namespace og=ompl::geometric;
  const std::string mode=argc>1?argv[1]:"off";
  const bool budget=mode=="budget";
  const bool directed=mode.find("directed-")==0||budget;
  const bool recording=mode!="off"&&mode!="directed-off";
  ompl::RNG::setSeed(71070);ompl::msg::setLogLevel(ompl::msg::LOG_NONE);
  auto space=std::make_shared<ob::RealVectorStateSpace>(2);
  ob::RealVectorBounds bounds(2);bounds.setLow(-1);bounds.setHigh(1);space->setBounds(bounds);
  og::SimpleSetup ss(space);size_t checks=0;nlohmann::json sequence=nlohmann::json::array();
  ss.setStateValidityChecker([&](const ob::State* s) {++checks;auto q=s->as<ob::RealVectorStateSpace::StateType>();
    if(budget)std::this_thread::sleep_for(std::chrono::milliseconds(1));
    const bool valid=budget||!(std::abs(q->values[0])<.16 && q->values[1]<.55);
    std::vector<double> values={q->values[0],q->values[1]};sequence.push_back({values,valid});
    if(m710::extension_recorder)(*m710::extension_recorder)(values,valid,"TEST_CHECKER",{{"reason","TEST_BARRIER"}});
    return valid;});
  ss.getSpaceInformation()->setStateValidityCheckingResolution(.005);
  ob::ScopedState<> start(space),goal(space);start[0]=-.8;start[1]=-.8;goal[0]=.8;goal[1]=-.8;
  ss.setStartAndGoalStates(start,goal,1e-8);
  auto trace=std::make_shared<m710::SearchTrace>();
  if(mode=="cap"||mode=="directed-cap") {trace->event_limit=1;trace->sample_limit=1;}
  m710::active_search_trace=recording?trace:nullptr;
  m710::active_j1_direction=directed?1:0;
  std::shared_ptr<og::RRTConnect> planner;
  if(!recording&&!directed)planner=std::make_shared<og::RRTConnect>(ss.getSpaceInformation());
  else planner=std::make_shared<m710::TraceRRTConnect>(ss.getSpaceInformation(),trace);
  planner->setRange(budget?.001:.2);ss.setPlanner(planner);
  const auto begin=std::chrono::steady_clock::now();const auto solved=ss.solve(budget?.02:10.);
  nlohmann::json out={{"success",bool(solved)},{"exact",ss.haveExactSolutionPath()},
    {"wall_s",std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count()},
    {"checks",checks},{"path",nlohmann::json::array()}};
  if(solved)for(auto s:ss.getSolutionPath().getStates()) {std::vector<double> q;space->copyToReals(q,s);out["path"].push_back(q);}
  out["check_sequence"]=sequence;out["trace"]=trace->planners;std::cout<<out.dump()<<std::endl;
  return solved||budget?0:1;
}
