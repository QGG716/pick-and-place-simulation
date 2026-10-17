#pragma once
#include <nlohmann/json.hpp>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/PlannerData.h>
#include <mutex>
#include <atomic>
#include <array>
namespace m710 {
using TraceJson=nlohmann::json;
// Request-local output; no global path/answer cache. JSON is materialized after solve.
struct SearchTrace {
  std::mutex mutex;
  std::vector<TraceJson> planners;
  TraceJson input;
  std::atomic<size_t> next_instance{0};
  size_t event_limit=100000, sample_limit=250000;
};
extern std::shared_ptr<SearchTrace> active_search_trace;
extern thread_local TraceJson validity_witness;
extern thread_local bool witness_requested;
using SampleRecorder=std::function<void(const std::vector<double>&,bool,const char*,const TraceJson&)>;
extern thread_local SampleRecorder* extension_recorder;
void traceFailure(const char* checker,const TraceJson& original);
class TraceRRTConnect : public ompl::geometric::RRTConnect {
 public:
  explicit TraceRRTConnect(const ompl::base::SpaceInformationPtr& si,
      std::shared_ptr<SearchTrace> trace): RRTConnect(si),trace_(std::move(trace)) {}
  ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition& ptc) override;
 private:
  GrowState growTree(TreeData&,TreeGrowingInfo&,Motion*);
  ompl::base::PlannerStatus solveRecorded(const ompl::base::PlannerTerminationCondition& ptc);
  struct Event {size_t sequence;bool start,accepted;std::vector<double> from,to;size_t sample_begin,sample_end;std::string outcome;};
  struct Sample {std::vector<double> q;bool valid;TraceJson witness;};
  std::shared_ptr<SearchTrace> trace_;
  std::vector<Event> events_;
  std::vector<Sample> samples_;
  size_t event_count_=0,sample_count_=0,event_sample_begin_=0;
  bool in_extension_=false;
  SampleRecorder recorder_;
  std::vector<double> values(const ompl::base::State* s) const;
  void extension(const ompl::base::State* from,const ompl::base::State* to,bool start,bool accepted,const char* outcome);
  void snapshot(const ompl::base::PlannerStatus& status);
};
}
