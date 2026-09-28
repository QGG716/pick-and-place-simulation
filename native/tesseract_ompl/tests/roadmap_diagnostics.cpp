// Real upstream graph mutations on tiny synthetic graphs; no FANUC claim.
#include "../roadmap_diagnostics.h"
#include "../endpoint_sampler.h"
#include <ompl/base/ScopedState.h>
#include <ompl/base/MotionValidator.h>
#include <ompl/base/ProblemDefinition.h>
#include <iostream>
namespace ob=ompl::base;
void require(bool value) {if(!value) throw std::runtime_error("diagnostic regression failed");}
struct Motion : ob::MotionValidator {
  bool accepted;
  Motion(const ob::SpaceInformationPtr& si,bool value):MotionValidator(si),accepted(value) {}
  bool checkMotion(const ob::State*,const ob::State*) const override {return accepted;}
  bool checkMotion(const ob::State*,const ob::State*,std::pair<ob::State*,double>&) const override {return accepted;}
};
struct Fixture : ObservedLazyPRM {
  using ObservedLazyPRM::ObservedLazyPRM;
  Vertex add(double x) {ob::ScopedState<> q(si_->getStateSpace());q[0]=x;q[1]=0.;return addMilestone(si_->cloneState(q.get()));}
  void ends(Vertex a,Vertex b) {startM_={a};goalM_={b};}
  bool validate() {return bool(constructSolution(startM_[0],goalM_[0]));}
  void corruptLabelForTest() {vertexComponentProperty_[goalM_[0]]=vertexComponentProperty_[startM_[0]];}
};
int main() {
  for(int mode=0;mode<3;++mode) {
    auto space=std::make_shared<ob::RealVectorStateSpace>(2);ob::RealVectorBounds bounds(2);bounds.setLow(-1);bounds.setHigh(1);space->setBounds(bounds);
    auto si=std::make_shared<ob::SpaceInformation>(space);
    si->setStateValidityChecker([mode](const ob::State* q) {return mode!=2 || q->as<ob::RealVectorStateSpace::StateType>()->values[0]!=0.;});
    si->setMotionValidator(std::make_shared<Motion>(si,mode!=1));si->setup();
    auto p=std::make_shared<ob::ProblemDefinition>(si);ob::ScopedState<> a(space),b(space);a[0]=-.1;a[1]=0;b[0]=.1;b[1]=0;p->setStartAndGoalStates(a,b);
    Fixture graph(si);graph.setProblemDefinition(p);if(mode==2) graph.setRange(.11);graph.setup();
    auto start=graph.add(-.1),goal=graph.add(.1);graph.ends(start,goal);if(mode==2) graph.add(0.);
    auto before=graph.terminalConnectivity();
    require(before["candidate_graph"]["connected"]==true);
    require(before["verified_subgraph"]["connected"]==false);
    require(before["component_label_audit"]["matches_independent_traversal"]==true);
    require(graph.validate()==(mode==0));
    auto after=graph.terminalConnectivity();
    require(after["candidate_graph"]["connected"]==(mode==0));
    require(after["verified_subgraph"]["connected"]==(mode==0));
    require(after["component_label_audit"]["matches_independent_traversal"]==true);
    if(mode!=0) {
      require(graph.snapshot(0,0,"test")["start"][0]["degree"]==0);
      graph.corruptLabelForTest();
      require(graph.terminalConnectivity()["component_label_audit"]["matches_independent_traversal"]==false);
    }
  }
  std::cout << "3 real upstream graph fixtures passed: UNKNOWN vs VALID, edge deletion, vertex deletion; deliberate label corruption detected\n";
  {
    auto space=std::make_shared<ob::RealVectorStateSpace>(2);ob::RealVectorBounds bounds(2);bounds.setLow(-1);bounds.setHigh(1);space->setBounds(bounds);
    std::vector<double> start{-.9,-.2},goal{.9,.2};uint64_t count=0;bool ready=false;EndpointSamplingUsage usage;
    EndpointMixtureSampler sampler(space.get(),71081,count,ready,usage,start,goal,.5,.05);
    ob::ScopedState<> q(space);
    for(int i=0;i<100;++i) sampler.sampleUniform(q.get());
    require(count==100 && usage.setup==100 && usage.global==0 && usage.start==0 && usage.goal==0);
    ready=true;
    for(int i=0;i<1000;++i) {
      auto old_start=usage.start,old_goal=usage.goal;
      sampler.sampleUniform(q.get());require(space->satisfiesBounds(q.get()));
      if(usage.start>old_start || usage.goal>old_goal) {
        const auto& center=usage.start>old_start?start:goal;
        for(int j=0;j<2;++j) require(std::abs(q[j]-center[j])<=.1+1e-12);
      }
    }
    require(count==1100 && usage.setup+usage.global+usage.start+usage.goal==count);
    require(usage.global>0 && usage.start>0 && usage.goal>0 && usage.other==0);
    require(start==std::vector<double>({-.9,-.2}) && goal==std::vector<double>({.9,.2}));
    std::cout << "Endpoint sampler fixture passed: setup uniform, local/global accounting, local bounds, immutable endpoints\n";
  }
}
