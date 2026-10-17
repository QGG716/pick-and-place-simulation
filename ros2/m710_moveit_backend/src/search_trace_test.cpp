
#include "search_trace.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/util/RandomNumbers.h>
#include <iostream>
int main(int argc,char** argv) {
  namespace ob=ompl::base;namespace og=ompl::geometric;
  const std::string mode=argc>1?argv[1]:"off";
  ompl::RNG::setSeed(71070);ompl::msg::setLogLevel(ompl::msg::LOG_NONE);
  auto space=std::make_shared<ob::RealVectorStateSpace>(2);
  ob::RealVectorBounds bounds(2);bounds.setLow(-1);bounds.setHigh(1);space->setBounds(bounds);
  og::SimpleSetup ss(space);size_t checks=0;
  ss.setStateValidityChecker([&](const ob::State* s) {++checks;auto q=s->as<ob::RealVectorStateSpace::StateType>();
    return !(std::abs(q->values[0])<.16 && q->values[1]<.55);});
  ss.getSpaceInformation()->setStateValidityCheckingResolution(.005);
  ob::ScopedState<> start(space),goal(space);start[0]=-.8;start[1]=-.8;goal[0]=.8;goal[1]=-.8;
  ss.setStartAndGoalStates(start,goal,1e-8);
  auto trace=std::make_shared<m710::SearchTrace>();
  if(mode=="cap") {trace->event_limit=1;trace->sample_limit=1;}
  std::shared_ptr<og::RRTConnect> planner;
  if(mode=="off")planner=std::make_shared<og::RRTConnect>(ss.getSpaceInformation());
  else planner=std::make_shared<m710::TraceRRTConnect>(ss.getSpaceInformation(),trace);
  planner->setRange(.2);ss.setPlanner(planner);
  const auto solved=ss.solve(10.);
  nlohmann::json out={{"success",bool(solved)},{"checks",checks},{"path",nlohmann::json::array()}};
  if(solved)for(auto s:ss.getSolutionPath().getStates()) {std::vector<double> q;space->copyToReals(q,s);out["path"].push_back(q);}
  out["trace"]=trace->planners;std::cout<<out.dump()<<std::endl;
  return solved?0:1;
}
