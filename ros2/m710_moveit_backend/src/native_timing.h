#pragma once
// Development latency measurement for the ECO65 real-site baseline.
//
// Disabled unless the launching process sets M710_NATIVE_TIMING=1. While
// disabled nothing is recorded and no reply byte changes. While enabled the
// only observable difference is an added top-level "native_timing" reply
// field; no decision, threshold, sampling density or check is affected.
// Values are kept in memory for one request and serialized with its reply.
#include <nlohmann/json.hpp>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace m710 {

// Log-spaced histogram, 20 bins per decade from 1e-7 s to 1e3 s, plus an
// underflow and an overflow bin. Quantiles are reported as bin upper edges.
struct NativeHistogram {
  static constexpr double kMin=1e-7;
  static constexpr int kPerDecade=20, kDecades=10, kBins=kPerDecade*kDecades+2;
  std::vector<unsigned long long> bins=std::vector<unsigned long long>(kBins,0);
  unsigned long long count=0;
  double sum=0., minimum=INFINITY, maximum=0.;
  static double upper(int bin) {
    if(bin<=0) return kMin;
    if(bin>=kBins-1) return INFINITY;
    return kMin*std::pow(10.,double(bin)/kPerDecade);
  }
  void add(double seconds) {
    if(!std::isfinite(seconds) || seconds<0.) seconds=0.;
    int bin=seconds<=kMin?0:int(std::ceil(std::log10(seconds/kMin)*kPerDecade));
    if(bin<0) bin=0;
    if(bin>kBins-1) bin=kBins-1;
    ++bins[bin];++count;sum+=seconds;
    if(seconds<minimum) minimum=seconds;
    if(seconds>maximum) maximum=seconds;
  }
  nlohmann::json json() const {
    nlohmann::json sparse=nlohmann::json::array();
    for(int i=0;i<kBins;++i) if(bins[i]) sparse.push_back({upper(i),bins[i]});
    auto quantile=[&](double q)->nlohmann::json {
      if(!count) return nullptr;
      const double target=q*double(count);unsigned long long seen=0;
      for(int i=0;i<kBins;++i) {seen+=bins[i];if(double(seen)>=target) return std::isfinite(upper(i))?nlohmann::json(upper(i)):nlohmann::json(maximum);}
      return maximum;
    };
    return {{"count",count},{"sum_s",sum},{"min_s",count?nlohmann::json(minimum):nlohmann::json(nullptr)},
            {"max_s",maximum},{"p50_upper_s",quantile(.5)},{"p90_upper_s",quantile(.9)},
            {"p99_upper_s",quantile(.99)},{"bins_upper_s_count",sparse},
            {"bin_scheme","LOG10_20_PER_DECADE_FROM_1E-7_S"}};
  }
};

class NativeTiming {
  using Clock=std::chrono::steady_clock;
  std::mutex mutex_;
  struct Segment {unsigned long long count=0;double seconds=0.;};
  std::map<std::string,Segment> segments_;
  std::map<std::string,NativeHistogram> states_;
  std::map<std::string,std::map<std::string,double>> workspace_;
  std::map<std::string,double> values_;
  std::string stage_;
 public:
  static bool enabled() {
    static const bool on=[]{const char* v=std::getenv("M710_NATIVE_TIMING");return v && std::strcmp(v,"1")==0;}();
    return on;
  }
  static NativeTiming& instance() {static NativeTiming timing;return timing;}
  void reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    segments_.clear();states_.clear();workspace_.clear();values_.clear();stage_.clear();
  }
  void stage(const std::string& value) {std::lock_guard<std::mutex> lock(mutex_);stage_=value;}
  void add(const std::string& name,double seconds) {
    std::lock_guard<std::mutex> lock(mutex_);auto& s=segments_[name];++s.count;s.seconds+=seconds;
  }
  void value(const std::string& name,double v) {std::lock_guard<std::mutex> lock(mutex_);values_[name]=v;}
  // One complete exact clearance check of one robot state.
  void state(const std::string& phase,bool valid,double seconds) {
    std::lock_guard<std::mutex> lock(mutex_);
    states_[phase+(valid?"/valid":"/rejected")].add(seconds);
  }
  // Per-state deltas of the broad phase and the exact FCL distance queries.
  void workspace(const std::string& phase,double tree_nodes,double aabb_tests,double aabb_skips,
                 double narrow_calls,double self_narrow,double world_narrow,
                 double update_s,double broad_s,double narrow_s) {
    std::lock_guard<std::mutex> lock(mutex_);auto& w=workspace_[phase];
    w["states"]+=1;w["broad_tree_nodes_visited"]+=tree_nodes;w["broad_aabb_tests"]+=aabb_tests;
    w["broad_aabb_skips"]+=aabb_skips;w["fcl_distance_calls"]+=narrow_calls;
    w["fcl_self_distance_calls"]+=self_narrow;w["fcl_world_distance_calls"]+=world_narrow;
    w["transform_aabb_update_s"]+=update_s;w["broad_phase_s"]+=broad_s;w["fcl_distance_s"]+=narrow_s;
  }
  nlohmann::json json() {
    std::lock_guard<std::mutex> lock(mutex_);
    nlohmann::json segments=nlohmann::json::object(),states=nlohmann::json::object(),workspace=nlohmann::json::object();
    for(const auto& s:segments_) segments[s.first]={{"count",s.second.count},{"seconds",s.second.seconds}};
    for(const auto& s:states_) states[s.first]=s.second.json();
    for(const auto& w:workspace_) workspace[w.first]=w.second;
    return {{"schema","m710_native_timing_v1"},{"clock","std::chrono::steady_clock"},{"request_stage",stage_},
            {"segments",segments},{"exact_state_check_s",states},{"workspace_by_phase",workspace},
            {"values",values_},{"scope","DEVELOPMENT_LATENCY_MEASUREMENT_NO_DECISION_EFFECT"}};
  }
};

// Adds the elapsed steady_clock time of one scope to a named segment.
class NativeSpan {
  using Clock=std::chrono::steady_clock;
  const char* name_;Clock::time_point start_;bool on_;
 public:
  explicit NativeSpan(const char* name):name_(name),on_(NativeTiming::enabled()) {if(on_) start_=Clock::now();}
  ~NativeSpan() {if(on_) NativeTiming::instance().add(name_,std::chrono::duration<double>(Clock::now()-start_).count());}
  NativeSpan(const NativeSpan&)=delete;NativeSpan& operator=(const NativeSpan&)=delete;
};

}  // namespace m710
