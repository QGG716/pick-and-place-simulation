#pragma once
// Realtime step 7 (planning_module NATIVE): composite single-carton requests.
//
// sc_endpoint_screen and sc_plan_candidate run, inside one worker, the exact
// sequence of native sub-requests that single_carton/planner.py issues for
// screen_candidate / endpoint_precheck / endpoint_ik and plan_candidate
// (plan_candidate_segments, plan_segment, verify_candidate, sent_check),
// with the same branching. Python builds every request template with its own
// functions (context, release_check, detach_goals, ...); here only q_start,
// IK seeds, retry scales, pipeline fallback and chunking are applied. Each
// sub-request is dispatched through the worker's normal entry and recorded
// (request without world, result). Python then replays its original code
// against these records: every request it issues must equal a recorded one
// (NATIVE_FLOW_DIVERGED otherwise), so decisions and evidence come from the
// unchanged Python code. Each sub-call is also appended to this worker's
// attempt journal before and after it runs, so a timeout or crash leaves it.
#include <nlohmann/json.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <set>
#include <string>
#include <unistd.h>
#include <vector>
#include "native_deadline.h"
#include "native_prepare.h"

namespace m710 {
class CompositeRunner {
  using J=nlohmann::json;
  using Clock=std::chrono::steady_clock;
 public:
  struct Stop {std::string reason;};
  std::function<J(const J&)> dispatch;
  std::function<void()> restore;   // re-register the composite's own deadline after each sub-call
  NativeDeadlinePtr own;
  J request,calls=J::array(),prepares=J::array();
  std::ofstream journal;std::string journal_path;size_t sequence=0;
  CompositeRunner(std::function<J(const J&)> d,std::function<void()> r,NativeDeadlinePtr o,const J& req)
      :dispatch(std::move(d)),restore(std::move(r)),own(std::move(o)),request(req) {
    const char* dir=std::getenv("M710_WORKER_JOURNAL_DIR");
    journal_path=std::string(dir&&*dir?dir:"/tmp")+"/m710_worker_journal_"+std::to_string(::getpid())+".jsonl";
    journal.open(journal_path,std::ios::app);
  }
  static double nowS() {return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();}
  void log(const J& entry) {
    if(!journal)return;
    J line=entry;line["monotonic_s"]=nowS();line["composite_request_id"]=request.value("request_id",std::string());
    line["composite_op"]=request.value("op",std::string());journal<<line.dump()<<"\n";journal.flush();
  }
  // Budget fields as SinglePlanner.native_call / deadline_native_call set them.
  J budgeted(J sub,double timeout) {
    const auto& d=request.value("deadline",J());
    if(d.is_null()) {sub["request_budget_s"]=timeout-10.;return sub;}
    const double left=d.at("deadline_monotonic_s").get<double>()-d.at("reply_reserve_s").get<double>()-nowS();
    if(left<=0)throw Stop{"PLANNING_BUDGET_EXHAUSTED_BEFORE_DISPATCH"};
    if(sub.contains("allowed_planning_time_s")&&left<sub.at("allowed_planning_time_s").get<double>())sub["allowed_planning_time_s"]=std::max(left,1e-3);
    sub["request_budget_s"]=std::min(timeout-10.,left);
    sub["absolute_deadline_monotonic_s"]=d.at("deadline_monotonic_s").get<double>()-d.at("reply_reserve_s").get<double>();
    sub["deadline_clock_reference_monotonic_s"]=d.at("reference_monotonic_s");
    return sub;
  }
  J call(J sub,const std::string& name,double timeout) {
    if(own&&own->expired())throw Stop{own->cancelled()?"CANCELLED":"PLANNING_BUDGET_EXHAUSTED"};
    sub=budgeted(std::move(sub),timeout);
    const size_t index=sequence++;const auto t=Clock::now();
    log({{"event","START"},{"index",index},{"name",name},{"op",sub.value("op",std::string())},{"stage",sub.value("stage",J())},
         {"pipeline_id",sub.value("pipeline_id",J())},{"planner_id",sub.value("planner_id",J())}});
    J result;
    try {result=dispatch(sub);} catch(const std::exception& e) {result={{"status","ERROR"},{"detail",e.what()}};}
    restore();
    const double elapsed=std::chrono::duration<double>(Clock::now()-t).count();
    log({{"event","END"},{"index",index},{"name",name},{"status",result.value("status",J())},
         {"native_valid",result.value("native_valid",J())},{"elapsed_s",elapsed}});
    // The replay key compares the world through scene_fingerprint and a probe
    // path through its length and end states (native_flow.replay_key does the same).
    J echoed=sub;echoed.erase("world");
    if(echoed.contains("probe_path")) {const J path=echoed.at("probe_path");
      echoed["probe_path"]={{"length",path.size()},{"first",path.front()},{"last",path.back()}};}
    calls.push_back({{"index",index},{"name",name},{"request",echoed},{"result",result},{"elapsed_s",elapsed}});
    if(result.value("status",std::string())=="ERROR")throw Stop{"SUB_REQUEST_ERROR"};
    if(own&&own->cancelled())throw Stop{"CANCELLED"};
    return result;
  }
  static std::string status(const J& r) {return r.value("status",std::string());}
  static bool startsWith(const std::string& s,const char* p) {return s.rfind(p,0)==0;}
  // ---- prepare (SinglePlanner.plan_segment's SEND_SLOWDOWN_STEPS loop) ----
  J prepare(const J& points,double scale,double reach,std::string& error) {
    const J& p=request.at("prepare");
    const auto& slow=p.at("slowdown_steps");
    for(size_t k=0;k<slow.size();++k) {
      const double ts=scale*slow[k].get<double>();
      J entry={{"points",points},{"time_scale",ts},{"period_s",p.at("period_s")},{"reach_bound_m",reach}};
      try {
        PrepareInput in;
        for(const auto& x:points) {in.t.push_back(x.at("t").get<double>());
          for(const auto& v:x.at("q"))in.q.push_back(v.get<double>());for(const auto& v:x.at("v"))in.v.push_back(v.get<double>());
          for(const auto& v:x.at("a"))in.a.push_back(v.get<double>());}
        const auto& L=p.at("limits");
        PrepareLimits limits{L.at("lower").get<std::vector<double>>(),L.at("upper").get<std::vector<double>>(),
                             L.at("velocity").get<std::vector<double>>(),L.at("acceleration").get<std::vector<double>>()};
        const auto out=nativePrepare(in,limits,ts,p.at("period_s").get<double>(),reach);
        auto rows=[](const std::vector<double>& f){J o=J::array();for(size_t i=0;i+5<f.size();i+=6)o.push_back(std::vector<double>(f.begin()+i,f.begin()+i+6));return o;};
        entry["status"]="PREPARED";
        entry["prepared"]={{"scaled_duration_s",out.scaled_duration},{"source_duration_s",out.source_duration},{"send_duration_s",out.send_duration},
          {"terminal_hold_s",out.terminal_hold},{"subdivisions_per_send",out.subdivisions},{"analytic_speed_bounds_rad_s",out.speed_bounds},
          {"t",out.t},{"q",rows(out.q)},{"v",rows(out.v)},{"a",rows(out.a)},{"collision_path",rows(out.collision)}};
        prepares.push_back(entry);
        return prepares.back().at("prepared");
      } catch(const std::runtime_error& e) {
        entry["status"]="PREPARE_REJECTED";entry["error"]=e.what();prepares.push_back(entry);
        const std::string what=e.what();
        if(!(startsWith(what,"INTERPOLATION_LIMIT")||startsWith(what,"DISCRETE_COMMAND_LIMIT"))||k+1==slow.size()) {error=what;return nullptr;}
      }
    }
    error="PREPARE_NOT_ATTEMPTED";return nullptr;
  }
  // ---- SinglePlanner.plan_segment ----
  struct Segment {std::string stage,record;J context,prepared,result;bool rejected=false;std::string prepare_error;bool checked=false;};
  Segment planSegment(J req,const J& context,const std::string& stage,const std::string& record,bool linear,double scale,double payload_reach) {
    Segment s;s.stage=stage;s.record=record;s.context=context;
    J result=call(req,record+"-pilz",120.);
    if(status(result)!="SUCCESS"&&linear&&!startsWith(status(result),"INVALID_START")) {
      const std::pair<double,const char*> retries[3]={{.5,"0.5"},{.25,"0.25"},{.1,"0.1"}};
      for(const auto& [f,name]:retries) {
        req["velocity_scale"]=f;req["acceleration_scale"]=f;
        result=call(req,record+"-pilz-s"+name,120.);
        if(status(result)=="SUCCESS")break;
      }
    }
    if(status(result)!="SUCCESS"&&!linear&&!startsWith(status(result),"INVALID_START")) {
      req["pipeline_id"]="ompl";req["planner_id"]="RRTConnectkConfigDefault";
      result=call(req,record+"-ompl",120.);
    }
    s.result=result;
    const std::string output=result.value("native_output_status",std::string());
    const bool accepted=output=="PASS"||(req.value("defer_output_check_to_sent_path",false)&&output=="DEFERRED_TO_CALLER_SENT_PATH_CHECK");
    if(status(result)!="SUCCESS"||!accepted) {s.rejected=true;return s;}
    s.prepared=prepare(result.at("points"),scale,request.at("prepare").at("reach_bound_m").get<double>()+payload_reach,s.prepare_error);
    if(s.prepared.is_null())throw Stop{"PREPARE_FAILED:"+s.prepare_error};
    return s;
  }
  static std::vector<double> lastQ(const Segment& s) {return s.prepared.at("q").back().get<std::vector<double>>();}
  static double maxAbsDiff(const std::vector<double>& a,const std::vector<double>& b) {
    double m=0;for(size_t i=0;i<a.size()&&i<b.size();++i)m=std::max(m,std::abs(a[i]-b[i]));return m;
  }
  // ---- SinglePlanner.sent_check (all chunks, as the parallel checkers would) ----
  bool sentCheck(Segment& s) {
    const auto path=s.prepared.at("collision_path");const size_t n=path.size();
    const size_t workers=request.at("check_workers").get<size_t>();
    const size_t count=std::max<size_t>(1,std::min<size_t>(workers,(n-1)/500));
    std::vector<size_t> edges(count+1);
    const double step=double(n-1)/double(count);
    for(size_t k=0;k<=count;++k)edges[k]=k==count?n-1:size_t(std::nearbyint(double(k)*step));
    const bool adaptive=request.value("sent_path_audit",std::string("SAMPLED"))=="ADAPTIVE"&&
      !s.context.at("single_carton_contact").at("allow_contact").get<bool>()&&s.context.at("single_carton_contact").at("supports").empty();
    bool valid=true;
    for(size_t k=0;k<count;++k) {
      J check=s.context;check["op"]="validate";
      check["probe_path"]=J(std::vector<J>(path.begin()+long(edges[k]),path.begin()+long(edges[k+1])+1));
      if(request.value("path_endpoint_reuse",false))check["path_endpoint_reuse"]=true;
      if(adaptive)check["sent_path_audit"]="ADAPTIVE";
      const J r=call(check,s.record+"-sent"+(count==1?std::string():"-p"+std::to_string(k)),600.);
      valid&=r.value("native_valid",false)&&r.value("legacy_intersection_valid",false)&&r.value("path_failure",J()).is_null();
    }
    return valid;
  }
  // ---- sc_plan_candidate CONTACT: approach (unless screened) and contact ----
  J contactPhase() {
    const J& c=request.at("contact_phase");std::vector<double> qpre;
    if(c.contains("approach")) {
      auto a=planSegment(c.at("approach").at("request"),c.at("approach").at("context"),"approach","approach",false,
                         request.at("scale").get<double>(),0.);
      if(a.rejected)return {{"outcome","SEGMENT_REJECTED"},{"stage","approach"}};
      qpre=lastQ(a);
    } else qpre=c.at("known_approach_q").get<std::vector<double>>();
    J req=c.at("contact").at("request");J context=c.at("contact").at("context");req["q_start"]=qpre;context["q_start"]=qpre;
    auto s=planSegment(req,context,"contact","contact",true,request.at("scale").get<double>(),0.);
    if(s.rejected)return {{"outcome","SEGMENT_REJECTED"},{"stage","contact"}};
    return {{"outcome","PLANNED"},{"contact_end_q",lastQ(s)}};
  }
  // ---- sc_plan_candidate DETACH_AND_VERIFY: detach profiles, approach, return, verify, replan ----
  J detachPhase() {
    const J& d=request.at("detach_phase");const double scale=request.at("scale").get<double>();
    const auto q=d.at("q").get<std::vector<double>>();const double payload_reach=d.at("payload_reach_m").get<double>();
    std::set<std::string> excluded;J rejected=J::array();
    // approach / contact as already planned in the contact phase: re-planned
    // here only when the approach was not planned there (screened known q).
    Segment approach,contact;bool have_approach=false;
    if(d.contains("approach_planned")) {approach.stage="approach";approach.record="approach";approach.context=d.at("approach_planned").at("context");
      approach.prepared=d.at("approach_planned").at("prepared");have_approach=true;}
    contact.stage="contact";contact.record="contact";contact.context=d.at("contact_planned").at("context");contact.prepared=d.at("contact_planned").at("prepared");
    while(true) {
      Segment extraction,returning;std::string profile;bool planned=false;
      for(const auto& p:d.at("profiles")) {
        const std::string name=p.at("name");if(excluded.count(name))continue;
        extraction=planSegment(p.at("request"),p.at("context"),"detach",p.at("record"),true,scale,payload_reach);
        if(extraction.rejected) {
          const J failure=extraction.result.value("failure",J());
          if(startsWith(status(extraction.result),"INVALID_START")&&payloadWorldBlocked(failure,d))
            return {{"outcome","TARGET_BLOCKED"},{"profile",name}};
          excluded.insert(name);rejected.push_back(name);continue;
        }
        const auto qe=lastQ(extraction);
        J rreq=d.at("return").at("request");J rctx=d.at("return").at("context");rreq["q_start"]=qe;rctx["q_start"]=qe;
        if(!have_approach) {
          approach=planSegment(d.at("approach").at("request"),d.at("approach").at("context"),"approach","approach",false,scale,0.);
          if(approach.rejected)return {{"outcome","SEGMENT_REJECTED"},{"stage","approach"}};
          have_approach=true;
          if(maxAbsDiff(lastQ(approach),d.at("qpre").get<std::vector<double>>())>1e-7)return {{"outcome","APPROACH_END_NOT_SCREENED_CONFIGURATION"}};
        }
        returning=planSegment(rreq,rctx,"return","return",false,scale,payload_reach);
        if(returning.rejected) {excluded.insert(name);rejected.push_back(name);continue;}
        if(maxAbsDiff(lastQ(returning),q)>1e-7)return {{"outcome","RETURN_NOT_LOCKED_Q_START"}};
        profile=name;planned=true;break;
      }
      if(!planned)return {{"outcome","ALL_PROFILES_REJECTED"},{"rejected",rejected}};
      // verify_candidate: unchecked segments by collision-path length (stable).
      std::vector<Segment*> order={&approach,&contact,&extraction,&returning};
      std::stable_sort(order.begin(),order.end(),[](const Segment* a,const Segment* b){return a->prepared.at("collision_path").size()<b->prepared.at("collision_path").size();});
      std::string failed;
      for(auto* s:order) {
        if(s->checked)continue;
        if(!sentCheck(*s)) {failed=s->stage;break;}
        s->checked=true;
      }
      if(failed.empty())return {{"outcome","VERIFIED"},{"profile",profile},{"rejected",rejected}};
      if(failed=="approach"||failed=="contact")return {{"outcome","SENT_PATH_REJECTED"},{"stage",failed}};
      excluded.insert(profile);rejected.push_back(profile);
    }
  }
  // SinglePlanner.payload_world_blocked
  static bool payloadWorldBlocked(const J& failure,const J& d) {
    if(!failure.is_object()||!failure.contains("pair")||!failure.at("pair").is_array()||failure.at("pair").size()!=2)return false;
    const std::string box=d.at("box_id");const auto ids=d.at("world_ids").get<std::set<std::string>>();
    const std::string a=failure.at("pair")[0],b=failure.at("pair")[1];
    if(a!=box&&b!=box)return false;
    const std::string other=a==box?b:a;
    if(other!=box&&!ids.count(other))return false;
    const J distance=failure.value("surface_distance_m",J());const double required=failure.value("required_pair_clearance_m",0.);
    if(!distance.is_number())return false;
    return distance.get<double>()<0||required-distance.get<double>()>1e-4;
  }
  // ---- sc_endpoint_screen: SinglePlanner.screen_candidate for every candidate, in order ----
  static bool allclose(const std::vector<double>& a,const std::vector<double>& b) {
    for(size_t i=0;i<a.size();++i)if(!(std::abs(a[i]-b[i])<=1e-8+1e-5*std::abs(b[i])))return false;return true;
  }
  std::vector<std::vector<double>> endpointIk(const J& templ,const std::vector<std::vector<double>>& seeds,const std::vector<double>& reference,const std::string& name) {
    std::vector<std::vector<double>> solutions;
    for(const auto& seed:seeds) {
      J r=templ;r["q_start"]=seed;
      const J reply=call(r,name,120.);
      if(status(reply)=="SUCCESS") {
        const auto qv=reply.at("q").get<std::vector<double>>();
        bool distinct=true;for(const auto& x:solutions)if(maxAbsDiff(qv,x)<1e-3) {distinct=false;break;}
        if(distinct)solutions.push_back(qv);
      }
    }
    auto norm=[&](const std::vector<double>& x){double s=0;for(size_t i=0;i<x.size();++i)s+=(x[i]-reference[i])*(x[i]-reference[i]);return std::sqrt(s);};
    std::stable_sort(solutions.begin(),solutions.end(),[&](const auto& a,const auto& b){return norm(a)<norm(b);});
    return solutions;
  }
  J screenPhase() {
    const auto q=request.at("q").get<std::vector<double>>();
    const auto lower=request.at("limits_lower").get<std::vector<double>>(),upper=request.at("limits_upper").get<std::vector<double>>();
    const auto spread=request.at("spread_seeds").get<std::vector<std::vector<double>>>();
    std::vector<std::vector<double>> solutions;J outcomes=J::array();
    for(const auto& c:request.at("candidates")) {
      if(c.contains("skip")) {outcomes.push_back({{"index",c.at("index")},{"outcome","SKIPPED_BY_PYTHON"}});continue;}
      const J loaded=call(c.at("loaded_start"),"loaded-start",120.);
      if(!loaded.value("native_valid",false)||!loaded.value("legacy_intersection_valid",false)) {
        outcomes.push_back({{"index",c.at("index")},{"outcome","LOADED_START_REJECTED"}});continue;
      }
      std::vector<std::vector<double>> seeds={q};
      for(int k=int(solutions.size())-1;k>=std::max(0,int(solutions.size())-2);--k)
        for(double turn:{0.,M_PI/2,-M_PI/2,M_PI}) {
          auto seed=solutions[size_t(k)];seed[5]+=turn;bool inside=true;
          for(size_t i=0;i<seed.size();++i)inside&=seed[i]>=lower[i]&&seed[i]<=upper[i];
          bool fresh=true;for(const auto& x:seeds)if(allclose(seed,x)) {fresh=false;break;}
          if(inside&&fresh)seeds.push_back(seed);
        }
      seeds.insert(seeds.end(),spread.begin(),spread.end());
      const auto branches=endpointIk(c.at("approach_ik"),seeds,q,"approach-endpoint-ik");
      if(branches.empty()) {outcomes.push_back({{"index",c.at("index")},{"outcome","NOT_CONCLUSIVE"}});continue;}
      const auto tool=c.at("tool_links").get<std::set<std::string>>();const std::string box=c.at("box_id");
      const auto ids=c.at("world_ids").get<std::set<std::string>>();
      int notes=0;std::string verdict;
      auto check=[&](const char* stage,const std::vector<double>& qe,bool attached)->bool {
        J r=c.at(std::string(stage)+"_check");r["q_start"]=qe;
        const J result=call(r,std::string(stage)+"-endpoint"+(notes?"-b"+std::to_string(notes):""),120.);
        const J failure=result.value("failure",J());const bool valid=result.value("native_valid",false)&&result.value("legacy_intersection_valid",false);
        std::set<std::string> movable=tool;if(attached)movable.insert(box);
        bool blocked=false;
        if(!valid&&failure.is_object()&&failure.contains("pair")&&failure.at("pair").is_array()&&failure.at("pair").size()==2) {
          const std::string a=failure.at("pair")[0],b=failure.at("pair")[1];
          auto other=[&](const std::string& x){return ids.count(x)&&!movable.count(x);};
          blocked=(movable.count(a)&&other(b))||(movable.count(b)&&other(a));
        }
        if(blocked&&attached&&payloadWorldBlocked(failure,J{{"box_id",box},{"world_ids",ids}}))verdict="TARGET_BLOCKED";
        else if(blocked)verdict="ENDPOINT_TOOL_OR_PAYLOAD_BLOCKED";
        return valid;
      };
      size_t tried=0;
      for(const auto& qa:branches) {
        if(tried++>=3)break;
        if(!check("approach",qa,false)) {if(!verdict.empty())break;++notes;continue;}
        const auto contacts=endpointIk(c.at("contact_ik"),{qa},qa,"contact-endpoint-ik"+(notes?"-b"+std::to_string(notes):""));
        if(contacts.empty()) {++notes;continue;}
        if(!check("contact",contacts[0],false)) {if(!verdict.empty())break;++notes;continue;}
        if(!check("detach",contacts[0],true)) {if(!verdict.empty())break;++notes;continue;}
        solutions.push_back(qa);verdict="CLEAR";break;
      }
      outcomes.push_back({{"index",c.at("index")},{"outcome",verdict.empty()?"NOT_CONCLUSIVE":verdict}});
      if(verdict=="TARGET_BLOCKED")break;
    }
    return {{"outcomes",outcomes}};
  }
  J run() {
    J out;const auto t=Clock::now();
    log({{"event","COMPOSITE_START"}});
    try {
      const std::string op=request.at("op");
      if(op=="sc_endpoint_screen")out=screenPhase();
      else if(request.at("phase")=="CONTACT")out=contactPhase();
      else if(request.at("phase")=="DETACH_AND_VERIFY")out=detachPhase();
      else throw std::runtime_error("COMPOSITE_PHASE_INVALID");
      out["stopped"]=nullptr;
    } catch(const Stop& s) {out["stopped"]=s.reason;}
    out["status"]="COMPOSITE_DONE";out["calls"]=calls;out["prepares"]=prepares;out["journal_path"]=journal_path;
    out["sub_requests"]=calls.size();out["composite_s"]=std::chrono::duration<double>(Clock::now()-t).count();
    log({{"event","COMPOSITE_END"},{"stopped",out["stopped"]},{"sub_requests",calls.size()}});
    return out;
  }
};
}
