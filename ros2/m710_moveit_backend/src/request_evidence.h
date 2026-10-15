#pragma once
#include <nlohmann/json.hpp>
#include <exception>
namespace m710 {
// Reads only request-local aggregates. Never invokes predicates or planners.
// Existing successful snapshots are reused; failed replies get the same schema.
template<class ClearanceRead, class ProcessRead>
void completeRequestEvidence(nlohmann::json& out, bool has_clearance, bool has_process,
                             ClearanceRead clearance_read, ProcessRead process_read) {
  using J=nlohmann::json;
  if(out.contains("request_check_evidence")) return;
  J meta={{"scope","CURRENT_REQUEST_ONLY"},{"clearance","NOT_CREATED"},{"process_policy","NOT_CREATED"}};
  auto collect=[&](const char* key,bool created,auto read) {
    if(!created && !out.contains(key)) {out[key]=nullptr;return;}
    try {
      const bool collected=!out.contains(key);
      if(collected) out[key]=read();
      if(!out.at(key).is_object()) {meta[key]="UNAVAILABLE";return;}
      auto& value=out[key];
      if(std::string(key)=="clearance") {
        if(collected) value.erase("policy"); // Don't repeat immutable tool/pair geometry.
        size_t queries=0;
        for(const auto& p:value.at("counts")) queries+=p.at("queries").template get<size_t>();
        meta[key]=queries ? "AVAILABLE_EXECUTED" : "CREATED_NOT_CHECKED";
        meta["clearance_queries"]=queries;
        J seconds=nullptr;
        if(queries) {double sum=0.;for(const auto& p:value.at("counts"))sum+=p.at("seconds").template get<double>();seconds=sum;}
        meta["clearance_check_s"]=seconds;
      } else {
        meta[key]=value.value("state_checks",size_t(0)) ? "AVAILABLE_EXECUTED" : "CREATED_NOT_CHECKED";
      }
    } catch(const std::exception& e) {meta[key]="UNAVAILABLE";meta[std::string(key)+"_error"]=e.what();}
  };
  collect("clearance",has_clearance,clearance_read);
  // Existing diagnostic entries use this compatible name.
  if(!out.contains("process_policy") && out.contains("process_policy_evidence"))
    out["process_policy"]=out.at("process_policy_evidence");
  collect("process_policy",has_process,process_read);
  if(!meta.contains("clearance_check_s")) meta["clearance_check_s"]=nullptr;
  if(!meta.contains("clearance_queries")) meta["clearance_queries"]=nullptr;
  meta["request_status"]=out.value("status",std::string("UNKNOWN"));
  meta["completion"]=out.value("status",std::string())=="SUCCESS"?"COMPLETED_REQUEST":"PARTIAL_OR_FAILED_REQUEST";
  out["request_check_evidence"]=std::move(meta);
}
}
