#pragma once
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <nlohmann/json.hpp>
#include <vector>
#include <cstdint>
#include <algorithm>

struct EndpointSamplingUsage {
  uint64_t setup=0,global=0,start=0,goal=0,other=0;
  nlohmann::json first_draws=nlohmann::json::array();
};

// One draw per invocation, without validity rejection, historical data or graph
// access. All accepted AND geometrically invalid draws spend the same budget.
struct EndpointMixtureSampler final : ompl::base::RealVectorStateSampler {
  uint64_t& calls;bool& setup_complete;EndpointSamplingUsage& usage;
  const std::vector<double> start,goal;
  double probability,half_width;
  EndpointMixtureSampler(const ompl::base::StateSpace* space,uint32_t seed,uint64_t& count,
      bool& ready,EndpointSamplingUsage& stats,std::vector<double> a,std::vector<double> b,
      double p,double width):RealVectorStateSampler(space),calls(count),setup_complete(ready),
      usage(stats),start(std::move(a)),goal(std::move(b)),probability(p),half_width(width) {rng_.setLocalSeed(seed);}
  void sampleUniform(ompl::base::State* state) override {
    ++calls;
    if(!setup_complete) {++usage.setup;RealVectorStateSampler::sampleUniform(state);return;}
    const char* kind="global";
    if(rng_.uniform01()>=probability) {++usage.global;RealVectorStateSampler::sampleUniform(state);}
    else {
      const bool from_goal=rng_.uniformInt(0,1)==1;
      const auto& center=from_goal?goal:start;
      if(from_goal) {++usage.goal;kind="goal_neighborhood";} else {++usage.start;kind="start_neighborhood";}
      const auto& bounds=space_->as<ompl::base::RealVectorStateSpace>()->getBounds();
      auto* q=state->as<ompl::base::RealVectorStateSpace::StateType>();
      for(size_t i=0;i<center.size();++i) {
        const double radius=half_width*(bounds.high[i]-bounds.low[i]);
        q->values[i]=rng_.uniformReal(std::max(bounds.low[i],center[i]-radius),std::min(bounds.high[i],center[i]+radius));
      }
    }
    if(usage.first_draws.size()<12) {
      const double* q=state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
      usage.first_draws.push_back({{"kind",kind},{"q_rad",std::vector<double>(q,q+start.size())}});
    }
  }
  void sampleUniformNear(ompl::base::State* s,const ompl::base::State* near,double distance) override {
    ++calls;++usage.other;RealVectorStateSampler::sampleUniformNear(s,near,distance);
  }
  void sampleGaussian(ompl::base::State* s,const ompl::base::State* mean,double deviation) override {
    ++calls;++usage.other;RealVectorStateSampler::sampleGaussian(s,mean,deviation);
  }
};
