#pragma once
#include "clearance.h"
#include "process_policy.h"
#include <queue>
#include <random>
#include <limits>

namespace m710 {
// Bounded joint PRM. Static edges are suggestions; every used edge is checked
// against the exact current full scene. This is the primary edge validator,
// not the disabled dense output audit. No endpoint-only acceptance.
class StaticPriorGraph {
  using Q=std::vector<double>;
  using Clock=std::chrono::steady_clock;
  static double elapsed(Clock::time_point t) {return std::chrono::duration<double>(Clock::now()-t).count();}
  static double distance(const Q& a,const Q& b) {double d=0;for(size_t k=0;k<a.size();++k)d+=std::abs(a[k]-b[k]);return d;}
  static double halton(size_t i,size_t b) {double f=1,v=0;while(i){f/=b;v+=f*(i%b);i/=b;}return v;}
public:
  static void validate(const Json& graph) {
    if(graph.at("status")!="SUCCESS" || graph.at("prior_id").get<std::string>().empty() ||
      graph.at("context").at("schema")!="m710_static_prior_context_v1") throw std::runtime_error("PRIOR_GRAPH_IDENTITY_INVALID");
    const auto& nodes=graph.at("nodes");const auto& edges=graph.at("edges");const auto& limits=graph.at("context").at("joint_limits_rad");
    if(!nodes.is_array()||nodes.size()>512||!edges.is_array()||edges.size()>12288||limits.size()!=6)
      throw std::runtime_error("PRIOR_GRAPH_SIZE_INVALID");
    const double step=graph.at("interpolation_l1_step_rad");
    if(!std::isfinite(step)||step<=0||step>.01)throw std::runtime_error("PRIOR_INTERPOLATION_INVALID");
    for(const auto& node:nodes) {
      if(!node.is_array()||node.size()!=6)throw std::runtime_error("PRIOR_NODE_INVALID");
      for(size_t i=0;i<6;++i) {
        const double value=node.at(i),lo=limits.at(i).at(0),hi=limits.at(i).at(1);
        if(!std::isfinite(value)||!std::isfinite(lo)||!std::isfinite(hi)||lo>=hi||value<lo||value>hi)
          throw std::runtime_error("PRIOR_NODE_BOUNDS_INVALID");
      }
    }
    if(graph.contains("accepted_portals")) {
      const auto& portals=graph.at("accepted_portals");std::set<std::string> ids;
      if(!portals.is_array()||portals.size()>128)throw std::runtime_error("PRIOR_PORTAL_LIMIT_INVALID");
      for(const auto& portal:portals) {
        const std::string id=portal.at("portal_id");
        if(id.empty()||!ids.insert(id).second||!portal.at("node_index").is_number_integer()||
          portal.at("node_index").get<int64_t>()<0||portal.at("node_index").get<size_t>()>=nodes.size())
          throw std::runtime_error("PRIOR_PORTAL_MANIFEST_INVALID");
        processTransform(portal.at("pose"));
      }
    }
    for(const auto& edge:edges) {
      if(!edge.is_array()||edge.size()!=2||!edge.at(0).is_number_integer()||!edge.at(1).is_number_integer()||
        edge.at(0).get<int64_t>()<0||edge.at(1).get<int64_t>()<0||edge.at(0)==edge.at(1)||
        edge.at(0).get<size_t>()>=nodes.size()||edge.at(1).get<size_t>()>=nodes.size())
        throw std::runtime_error("PRIOR_EDGE_INVALID");
    }
  }
  static Json applicability(const Json& stored,const Json& current) {
    for(const auto* key:{"schema","identity","flange_from_task_tcp","tool_links","compliant_tool_links","collision_policy","static_world","allowed_pairs","joint_limits_rad","interpolation","receiver_reserve_m","placement_policy_sha256"})
      if(!stored.contains(key)||!current.contains(key)||stored.at(key)!=current.at(key))return {{"valid",false},{"reason",std::string("PRIOR_CONTEXT_MISMATCH:")+key}};
    for(auto it=stored.begin();it!=stored.end();++it)
      if(it.key()!="attachment" && (!current.contains(it.key())||current.at(it.key())!=it.value()))
        return {{"valid",false},{"reason","PRIOR_CONTEXT_MISMATCH:"+it.key()}};
    for(auto it=current.begin();it!=current.end();++it)
      if(!stored.contains(it.key()))return {{"valid",false},{"reason","PRIOR_CONTEXT_MISMATCH:"+it.key()}};
    const auto& a=stored.at("attachment");const auto& b=current.at("attachment");
    if(a.at("mode")!=b.at("mode"))return {{"valid",false},{"reason","PRIOR_PAYLOAD_MODE_MISMATCH"}};
    if(a.at("mode")=="loaded") {
      if(a.at("size")!=b.at("size")||a.at("touch_links")!=b.at("touch_links"))return {{"valid",false},{"reason","PRIOR_PAYLOAD_GEOMETRY_MISMATCH"}};
      Eigen::Matrix3d ra,rb;double translation=0;
      for(int r=0;r<3;++r){translation+=std::pow(a.at("pose").at(r).at(3).get<double>()-b.at("pose").at(r).at(3).get<double>(),2);for(int c=0;c<3;++c){ra(r,c)=a.at("pose").at(r).at(c);rb(r,c)=b.at("pose").at(r).at(c);}}
      if(std::sqrt(translation)>.025 || Eigen::AngleAxisd(ra.transpose()*rb).angle()>.05)return {{"valid",false},{"reason","PRIOR_ATTACHMENT_OUT_OF_SCOPE"}};
    }
    return {{"valid",true},{"current_geometry_check_required",true},{"static_clearance_inherited",false}};
  }
  static bool edge(moveit::core::RobotState& state,const Q& a,const Q& b,
      Clearance& clearance,ProcessPolicy* process,double step,Clock::time_point begin,double budget,size_t& checks,
      Json* samples=nullptr,Json* failure=nullptr) {
    const size_t n=std::max(size_t(1),size_t(std::ceil(distance(a,b)/step)));
    auto reject=[&](const Json& actual,const char* origin,size_t sample) {
      if(failure) {
        *failure={{"fraction",double(sample)/n},{"failure_origin",origin}};
        if(elapsed(begin)>=budget)(*failure)["reason"]="TIMEOUT";
        else if(actual.is_object())for(const auto* key:{"reason","pair","stage","signed_distance_m"})
          if(actual.contains(key))(*failure)[key]=actual.at(key);
      }
      return false;
    };
    for(size_t i=0;i<=n;++i) {
      if(elapsed(begin)>=budget)return reject({{"reason","TIMEOUT"}},"DEADLINE",i);
      Q q=a;for(size_t j=0;j<q.size();++j)q[j]+=(b[j]-a[j])*double(i)/n;
      if(i==n)q=b;state.setJointGroupPositions("manipulator",q);state.update();++checks;
      if(!state.satisfiesBounds())return reject({{"reason","JOINT_BOUNDS"}},"JOINT_BOUNDS",i);
      if(!clearance.check(state))return reject(clearance.last_failure,"CLEARANCE",i);
      if(process&&!process->check(state))return reject(process->last_failure,"PROCESS",i);
      if(samples) samples->push_back(q);
    }return true;
  }
  static Json build(const Json& req,const planning_scene::PlanningScenePtr& scene,Clearance& clearance) {
    const auto begin=Clock::now();const auto& spec=req.at("prior_build");const auto& context=req.at("prior_context");
    const size_t limit=spec.value("node_limit",160),attempts=spec.value("attempt_limit",1600),neighbors=spec.value("neighbors",10);
    if(limit<2||limit>512||attempts>10000||neighbors==0||neighbors>24)throw std::runtime_error("PRIOR_BUILD_LIMIT_INVALID");
    const double budget=req.at("allowed_planning_time_s"),step=std::min(.01,req.at("clearance_policy").at("edge_resolution_rad").get<double>());
    if(!std::isfinite(budget)||budget<=0||!std::isfinite(step)||step<=0)throw std::runtime_error("PRIOR_BUILD_BUDGET_OR_STEP_INVALID");
    auto state=scene->getCurrentState();std::vector<Q> nodes;Json edges=Json::array();size_t checks=0,tried=0,edge_attempts=0;
    const auto& limits=context.at("joint_limits_rad");const size_t primes[]={2,3,5,7,11,13};
    const auto portals=spec.value("portal_candidates",Json::array());std::set<std::string> portal_ids;
    if(!portals.is_array()||portals.size()>128||portals.size()>limit)throw std::runtime_error("PRIOR_PORTAL_LIMIT_INVALID");
    for(const auto& portal:portals) {
      const std::string id=portal.at("portal_id");const auto& q=portal.at("q");
      if(id.empty()||!portal_ids.insert(id).second||!q.is_array()||q.size()!=6)
        throw std::runtime_error("PRIOR_PORTAL_INPUT_INVALID");
      processTransform(portal.at("pose"));
      for(size_t j=0;j<6;++j) {
        const double v=q.at(j),lo=limits.at(j).at(0),hi=limits.at(j).at(1);
        if(!std::isfinite(v)||v<lo||v>hi)throw std::runtime_error("PRIOR_PORTAL_BOUNDS_INVALID");
      }
      state.setJointGroupPositions("manipulator",q.get<Q>());state.update();
      if(!state.satisfiesBounds())throw std::runtime_error("PRIOR_PORTAL_MODEL_BOUNDS_INVALID");
    }
    auto existing=[&](const Q& q) {
      for(size_t i=0;i<nodes.size();++i) {
        bool same=true;for(size_t j=0;j<q.size();++j)if(std::abs(q[j]-nodes[i][j])>1e-9){same=false;break;}
        if(same)return i;
      }
      return nodes.size();
    };
    Json accepted_portals=Json::array();size_t portal_attempts=0,portal_duplicates=0;
    clearance.phase="prior_build";
    for(const auto& portal:portals) {
      if(elapsed(begin)>=budget)break;++portal_attempts;
      const Q q=portal.at("q").get<Q>();state.setJointGroupPositions("manipulator",q);state.update();++checks;
      if(!clearance.check(state))continue;
      const size_t index=existing(q);const bool duplicate=index<nodes.size();
      if(!duplicate)nodes.push_back(q);else ++portal_duplicates;
      accepted_portals.push_back({{"portal_id",portal.at("portal_id")},{"node_index",index},
        {"pose",portal.at("pose")},{"deduplicated",duplicate}});
    }
    for(size_t k=0;k<attempts && nodes.size()<limit && elapsed(begin)<budget;++k){++tried;Q q(6);
      for(size_t j=0;j<6;++j){double lo=limits.at(j).at(0),hi=limits.at(j).at(1);q[j]=lo+(hi-lo)*halton(k+1+spec.value("seed",71071)%997,primes[j]);}
      state.setJointGroupPositions("manipulator",q);state.update();++checks;
      if(state.satisfiesBounds()&&clearance.check(state)&&existing(q)==nodes.size()) nodes.push_back(q);
    }
    std::set<std::pair<size_t,size_t>> pairs;
    for(size_t i=0;i<nodes.size();++i){std::vector<std::pair<double,size_t>> near;
      for(size_t j=0;j<nodes.size();++j)if(i!=j)near.push_back({distance(nodes[i],nodes[j]),j});std::sort(near.begin(),near.end());
      for(size_t k=0;k<std::min(neighbors,near.size());++k)pairs.insert(std::minmax(i,near[k].second));}
    for(auto pair:pairs){if(elapsed(begin)>=budget)break;++edge_attempts;
      if(edge(state,nodes[pair.first],nodes[pair.second],clearance,nullptr,step,begin,budget,checks))edges.push_back({pair.first,pair.second});}
    return {{"status",elapsed(begin)>=budget?"PRIOR_BUILD_BUDGET_EXHAUSTED":"SUCCESS"},{"nodes",nodes},{"edges",edges},{"context",context},
      {"accepted_portals",accepted_portals},{"portal_candidates",portals.size()},{"portal_attempts",portal_attempts},
      {"portal_duplicates",portal_duplicates},{"build_s",elapsed(begin)},{"sample_attempts",tried},{"edge_attempts",edge_attempts},{"state_checks",checks},
      {"interpolation_l1_step_rad",step},{"sampling",portals.empty()?"bounded_Halton_full_declared_joint_limits":"configured_workspace_portals_then_bounded_Halton"},
      {"continuous_swept_proof",false},{"static_geometry_only",true},{"clearance",clearance.evidence()}};
  }
  static Json query(const Json& graph,const Json& req,const planning_scene::PlanningScenePtr& scene,Clearance& clearance,ProcessPolicy* process) {
    const auto begin=Clock::now();validate(graph);auto app=applicability(graph.at("context"),req.at("prior_context"));
    if(!app.at("valid").get<bool>())return {{"status","PRIOR_CONTEXT_MISMATCH"},{"termination","CONTEXT_MISMATCH"},{"failure",app}};
    auto nodes=graph.at("nodes").get<std::vector<Q>>();const size_t count=nodes.size(),start=count,goal=count+1;
    nodes.push_back(req.at("q_start").get<Q>());nodes.push_back(req.at("q_goal").get<Q>());
    std::vector<std::vector<size_t>> adjacency(nodes.size());
    auto add=[&](size_t a,size_t b){if(a==b||a>=nodes.size()||b>=nodes.size())throw std::runtime_error("PRIOR_EDGE_INVALID");adjacency[a].push_back(b);adjacency[b].push_back(a);};
    for(const auto& e:graph.at("edges")){size_t a=e.at(0),b=e.at(1);if(a>=count||b>=count)throw std::runtime_error("PRIOR_EDGE_INVALID");add(a,b);}
    // Query-dependent neighbors and additional configured portals are selected
    // online. Every new connector still uses the same current edge checker.
    std::set<size_t> portal_nodes;size_t portal_connections_added=0;
    if(graph.contains("accepted_portals"))for(const auto& p:graph.at("accepted_portals"))portal_nodes.insert(p.at("node_index").get<size_t>());
    for(size_t endpoint:{start,goal}) {
      std::vector<std::pair<double,size_t>> near;std::set<size_t> connected;
      for(size_t i=0;i<count;++i)near.push_back({distance(nodes[endpoint],nodes[i]),i});
      std::sort(near.begin(),near.end());
      for(size_t k=0;k<std::min(size_t(20),near.size());++k){add(endpoint,near[k].second);connected.insert(near[k].second);}
      size_t extra=0;
      for(const auto& candidate:near)if(portal_nodes.count(candidate.second)&&!connected.count(candidate.second)) {
        add(endpoint,candidate.second);++portal_connections_added;if(++extra==8)break;
      }
    }
    add(start,goal);std::map<std::pair<size_t,size_t>,bool> validity;
    const double budget=req.at("allowed_planning_time_s"),step=graph.at("interpolation_l1_step_rad");
    if(!std::isfinite(budget)||budget<=0)return {{"status","PRIOR_QUERY_TIMEOUT"},{"termination","TIMEOUT"},{"query_s",elapsed(begin)}};
    if(step<=0||step>std::min(.01,req.at("clearance_policy").at("edge_resolution_rad").get<double>()))throw std::runtime_error("PRIOR_INTERPOLATION_CHANGED");
    auto state=scene->getCurrentState();size_t checks=0,edge_checks=0,rejected=0;Json route=Json::array(),path=Json::array();clearance.phase="prior_online";
    constexpr size_t edge_limit=128;
    Json rejected_by_kind={{"start_connector",0},{"goal_connector",0},{"graph_edge",0},{"direct_connector",0}};
    Json failed_edge_witnesses=Json::array();size_t timed_out_edges=0;
    auto edge_kind=[&](size_t a,size_t b) {
      if((a==start&&b==goal)||(a==goal&&b==start))return std::string("direct_connector");
      if(a==start||b==start)return std::string("start_connector");
      if(a==goal||b==goal)return std::string("goal_connector");
      return std::string("graph_edge");
    };
    bool graph_disconnected=false;
    while(elapsed(begin)<budget && edge_checks<edge_limit){
      std::vector<double> costs(nodes.size(),std::numeric_limits<double>::infinity());std::vector<size_t> parent(nodes.size(),nodes.size());costs[start]=0;
      using Item=std::pair<double,size_t>;std::priority_queue<Item,std::vector<Item>,std::greater<Item>> queue;queue.push({0,start});
      while(!queue.empty()){auto [cost,u]=queue.top();queue.pop();if(cost!=costs[u])continue;if(u==goal)break;
        for(size_t v:adjacency[u]){auto key=std::minmax(u,v);if(validity.count(key)&&!validity[key])continue;double next=cost+distance(nodes[u],nodes[v]);if(next<costs[v]){costs[v]=next;parent[v]=u;queue.push({next,v});}}}
      if(parent[goal]==nodes.size()){graph_disconnected=true;break;}std::vector<size_t> candidate;for(size_t v=goal;v!=start;v=parent[v])candidate.push_back(v);candidate.push_back(start);std::reverse(candidate.begin(),candidate.end());
      bool valid=true;
      for(size_t i=1;i<candidate.size();++i) {
        const size_t from=candidate[i-1],to=candidate[i];auto key=std::minmax(from,to);
        if(!validity.count(key)) {
          if(edge_checks>=edge_limit){valid=false;break;}++edge_checks;Json failure;
          validity[key]=edge(state,nodes[from],nodes[to],clearance,process,step,begin,budget,checks,nullptr,&failure);
          if(!validity[key]) {
            const std::string kind=edge_kind(from,to);
            if(failure.value("reason",std::string())=="TIMEOUT")++timed_out_edges;
            else {++rejected;rejected_by_kind[kind]=rejected_by_kind.at(kind).get<size_t>()+1;}
            if(failed_edge_witnesses.size()<4) {
              failure["kind"]=kind;failure["from"]=from;failure["to"]=to;failed_edge_witnesses.push_back(failure);
            }
          }
        }
        if(!validity[key]){valid=false;break;}
      }
      if(valid){route=candidate;
        // Geometric samples reproduce the checked interpolation. Do not run a
        // second independent output audit or import timed trajectories.
        for(size_t i=1;i<candidate.size();++i){const auto& a=nodes[candidate[i-1]];const auto& b=nodes[candidate[i]];size_t n=std::max(size_t(1),size_t(std::ceil(distance(a,b)/step)));
          for(size_t k=(i==1?0:1);k<=n;++k){Q q=a;for(size_t j=0;j<q.size();++j)q[j]+=(b[j]-a[j])*double(k)/n;if(k==n)q=b;path.push_back(q);}}
        break;}
    }
    size_t reused=0,connectors=0,nodes_reused=0;
    for(const auto& node:route)if(node.get<size_t>()<count)++nodes_reused;
    for(size_t i=1;i<route.size();++i) {
      if(route.at(i-1).get<size_t>()<count && route.at(i).get<size_t>()<count)++reused;else ++connectors;
    }
    const std::string termination=!path.empty()?"COMPLETE":elapsed(begin)>=budget?"TIMEOUT":
      graph_disconnected?"GRAPH_DISCONNECTED":"EDGE_CHECK_LIMIT";
    const std::string status=termination=="COMPLETE"?"SUCCESS":termination=="TIMEOUT"?"PRIOR_QUERY_TIMEOUT":
      termination=="EDGE_CHECK_LIMIT"?"PRIOR_EDGE_CHECK_LIMIT":"PRIOR_NOT_CONNECTED";
    return {{"status",status},{"termination",termination},{"path",path},
      {"query_s",elapsed(begin)},{"native_solver_s",0.},{"prior_usage",{{"prior_id",graph.at("prior_id")},{"context_checked",true},{"current_geometry_checked",!path.empty()},
        {"route_nodes",route},{"portal_connections_added",portal_connections_added},{"prior_nodes_reused",nodes_reused},{"prior_edges_reused",reused},{"connector_edges",connectors},{"prior_hit",nodes_reused>0},{"connector_kind","CHECKED_JOINT_INTERPOLATION"},{"edge_checks",edge_checks},{"edge_check_limit",edge_limit},{"rejected_edges",rejected},{"rejected_by_kind",rejected_by_kind},{"timed_out_edges",timed_out_edges},
        {"failed_edge_witnesses",failed_edge_witnesses},{"state_checks",checks},
        {"interpolation_l1_step_rad",step},{"continuous_swept_proof",false},{"static_clearance_inherited",false},{"local_repairs",0},{"slow_path",false}}}};
  }
};
}
