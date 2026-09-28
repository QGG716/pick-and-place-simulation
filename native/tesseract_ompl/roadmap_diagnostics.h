#pragma once
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/prm/ConnectionStrategy.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <nlohmann/json.hpp>
#include <map>
#include <set>
#include <vector>
#include <algorithm>
#include <cmath>

// Read-only observation of the actual OMPL graph. No construction, A*,
// validity or component implementation is replaced here.
struct ObservedLazyPRM : ompl::geometric::LazyPRM {
  using Json=nlohmann::json;
  explicit ObservedLazyPRM(const ompl::base::SpaceInformationPtr& si):LazyPRM(si,false) {}
  unsigned int neighbors() const {
    const auto* strategy=connectionStrategy_.target<ompl::geometric::KBoundedStrategy<Vertex>>();
    if(!strategy || starStrategy_) throw std::runtime_error("unexpected LazyPRM connection strategy");
    return strategy->getNumNeighbors();
  }
  Json parameters() const {
    const double threshold=opt_->getCostThreshold().value();
    if(!std::isinf(threshold) || threshold<0 || !opt_->isSatisfied(ompl::base::Cost(1.)))
      throw std::runtime_error("LazyPRM must stop at first validated finite-cost solution");
    return {{"type","ompl::geometric::LazyPRM"},{"star",false},
      {"connection_strategy","default KBoundedStrategy"},{"max_nearest_neighbors",neighbors()},
      {"max_connection_distance",getRange()},{"distance","RealVectorStateSpace unweighted Euclidean L2"},
      {"objective","default PathLengthOptimizationObjective"},{"cost_threshold","positive infinity"},
      {"nondefault_planner_parameters",Json::object()}};
  }
  bool endpoint(Vertex v) const {
    return std::find(startM_.begin(),startM_.end(),v)!=startM_.end() ||
           std::find(goalM_.begin(),goalM_.end(),v)!=goalM_.end();
  }
  bool known(Vertex v) const {return endpoint(v) || (vertexValidityProperty_[v]&VALIDITY_TRUE);}
  bool endpointsAdded() const {return !startM_.empty() && !goalM_.empty();}
  Json q(Vertex v) const {
    const double* p=stateProperty_[v]->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    return std::vector<double>(p,p+si_->getStateDimension());
  }
  Json progress() const {
    uint64_t vn=0,en=0;
    for(auto v:boost::make_iterator_range(boost::vertices(g_))) if(known(v)) ++vn;
    for(auto e:boost::make_iterator_range(boost::edges(g_))) if(edgeValidityProperty_[e]&VALIDITY_TRUE) ++en;
    return {{"roadmap_vertices",milestoneCount()},{"roadmap_edges",edgeCount()},
      {"known_valid_nodes",vn},{"unknown_nodes",milestoneCount()-vn},
      {"known_valid_edges",en},{"unknown_edges",edgeCount()-en},
      {"edge_count_semantics","underlying undirected graph, each edge once; not PlannerData arcs"},
      {"node_validity_semantics","upstream VALIDITY_TRUE or explicitly validated start/goal"},
      {"iterations",iterations_},{"candidate_path_validations",nullptr},{"candidate_researches",nullptr},
      {"approximate_goal_distance",nullptr},{"roadmap_reused",false}};
  }
  Json endpointSummary(Vertex v) const {
    uint64_t ve=0,vn=0,total=0;
    Json adjacent=Json::array();
    for(auto e:boost::make_iterator_range(boost::out_edges(v,g_))) {
      Vertex other=boost::source(e,g_)==v?boost::target(e,g_):boost::source(e,g_);
      bool ev=edgeValidityProperty_[e]&VALIDITY_TRUE;
      ++total;if(ev) ++ve;if(known(other)) ++vn;
      if(adjacent.size()<16) adjacent.push_back({{"q_rad",q(other)},{"edge_valid",ev},
        {"node_valid",known(other)},{"distance_rad",si_->distance(stateProperty_[v],stateProperty_[other])}});
    }
    auto label=vertexComponentProperty_[v];auto it=componentSize_.find(label);
    return {{"q_rad",q(v)},{"degree",total},{"valid_edges",ve},{"unknown_edges",total-ve},
      {"valid_neighbors",vn},{"unknown_neighbors",total-vn},{"neighbors_first_16",adjacent},
      {"ompl_component_label",label},{"ompl_component_size",it==componentSize_.end()?0:it->second},
      {"explicit_endpoint_validation",true},{"raw_vertex_validity_flag",vertexValidityProperty_[v]}};
  }
  Json snapshot(uint64_t samples,uint64_t computations,const std::string& event) const {
    Json out=progress();out["event"]=event;out["cumulative_samples"]=samples;
    out["actual_state_computations"]=computations;out["start"]=Json::array();out["goal"]=Json::array();
    for(auto v:startM_) out["start"].push_back(endpointSummary(v));
    for(auto v:goalM_) out["goal"].push_back(endpointSummary(v));
    out["candidate_connected_by_ompl_labels"]=endpointsAdded()?Json(vertexComponentProperty_[startM_[0]]==vertexComponentProperty_[goalM_[0]]):Json(nullptr);
    return out;
  }
  Json terminalConnectivity() const {
    // Separate local indexing, never the mutable OMPL vertex index property.
    std::vector<Vertex> vertices;std::map<Vertex,size_t> index;
    for(auto v:boost::make_iterator_range(boost::vertices(g_))) {index[v]=vertices.size();vertices.push_back(v);}
    auto components=[&](bool verified) {
      std::vector<int> ids(vertices.size(),-1);std::vector<size_t> sizes;
      for(size_t i=0;i<vertices.size();++i) {
        if(ids[i]>=0 || (verified && !known(vertices[i]))) continue;
        int id=sizes.size();std::vector<size_t> queue{i};ids[i]=id;
        for(size_t j=0;j<queue.size();++j) {
          Vertex v=vertices[queue[j]];
          for(auto e:boost::make_iterator_range(boost::out_edges(v,g_))) {
            Vertex w=boost::source(e,g_)==v?boost::target(e,g_):boost::source(e,g_);
            if(verified && (!(edgeValidityProperty_[e]&VALIDITY_TRUE) || !known(w))) continue;
            size_t k=index.at(w);if(ids[k]<0) {ids[k]=id;queue.push_back(k);}
          }
        }
        sizes.push_back(queue.size());
      }
      return std::make_pair(ids,sizes);
    };
    auto candidate=components(false),verified=components(true);
    auto summary=[&](const auto& c) {
      Json out={{"component_count",c.second.size()},{"start",Json::array()},{"goal",Json::array()},
        {"connected",nullptr},{"largest_component_size",c.second.empty()?0:*std::max_element(c.second.begin(),c.second.end())}};
      auto describe=[&](Vertex v) {int id=c.first.at(index.at(v));return Json{{"component",id},{"size",id<0?0:c.second.at(id)}};};
      for(auto v:startM_) out["start"].push_back(describe(v));
      for(auto v:goalM_) out["goal"].push_back(describe(v));
      if(endpointsAdded()) {int a=c.first[index.at(startM_[0])],b=c.first[index.at(goalM_[0])];out["connected"]=a>=0 && a==b;}
      return out;
    };
    std::map<unsigned long,std::set<int>> labelComponents;
    std::map<int,std::set<unsigned long>> componentLabels;
    std::map<unsigned long,size_t> actualSizes;
    for(size_t i=0;i<vertices.size();++i) {
      auto label=vertexComponentProperty_[vertices[i]];
      labelComponents[label].insert(candidate.first[i]);componentLabels[candidate.first[i]].insert(label);++actualSizes[label];
    }
    size_t merged=0,split=0,wrongSize=0;
    for(const auto& x:labelComponents) if(x.second.size()!=1) ++merged;
    for(const auto& x:componentLabels) if(x.second.size()!=1) ++split;
    for(const auto& x:actualSizes) {auto it=componentSize_.find(x.first);if(it==componentSize_.end() || it->second!=x.second) ++wrongSize;}
    return {{"candidate_graph",summary(candidate)},{"verified_subgraph",summary(verified)},
      {"component_label_audit",{{"matches_independent_traversal",merged==0 && split==0 && wrongSize==0},
        {"labels_spanning_disconnected_components",merged},{"components_with_multiple_labels",split},{"wrong_live_label_sizes",wrongSize}}},
      {"verified_node_rule","VALIDITY_TRUE or endpoint explicitly accepted by the same state validator"},
      {"candidate_unknown_is_not_safe",true}};
  }
};
