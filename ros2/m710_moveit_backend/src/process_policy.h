#pragma once
#include "process_geometry.h"
#include "clearance.h"

namespace m710 {
// Candidate-local process permissions. ACM exempts only the named pairs whose
// geometry is checked here; the ordered verifier restores free-space rules.
class ProcessPolicy {
  Json spec_,source_,initial_;
  std::string stage_,target_id_;
  std::set<std::string> compliant_,stack_,supports_;
  std::map<std::string,ProcessBox> world_;
  ProcessBox target_;
  bool attached_=false,relaxed_=false;
  double tolerance_=0,gap_=0;
  size_t state_checks_=0,ordered_checks_=0;
  Json terminal_proximity_=nullptr;
  static double surfaceGap(const ProcessBox& a,const ProcessBox& b) {
    auto sa=std::make_shared<fcl::Boxd>(2*a.half.x(),2*a.half.y(),2*a.half.z());
    auto sb=std::make_shared<fcl::Boxd>(2*b.half.x(),2*b.half.y(),2*b.half.z());
    fcl::CollisionObjectd oa(sa,a.pose),ob(sb,b.pose);fcl::DistanceResultd result;
    const double d=fcl::distance(&oa,&ob,fcl::DistanceRequestd(false),result);
    if(!std::isfinite(d)) throw std::runtime_error("PROCESS_DISTANCE_UNKNOWN");return d;
  }
  ProcessBox targetAt(const moveit::core::RobotState& state) const {
    auto b=target_;
    if(attached_) {
      const auto* body=state.getAttachedBody(target_id_);
      if(!body || body->getShapes().size()!=1 || body->getShapes()[0]->type!=shapes::BOX)
        throw std::runtime_error("PROCESS_ATTACHMENT_CHANGED");
      const auto* shape=static_cast<const shapes::Box*>(body->getShapes()[0].get());
      for(int i=0;i<3;++i) if(std::abs(shape->size[i]-2*b.half[i])>1e-12) throw std::runtime_error("PROCESS_TARGET_GEOMETRY_CHANGED");
      b.pose=body->getGlobalCollisionBodyTransforms().at(0);
    }
    return b;
  }
  Json fail(const std::string& reason,const std::string& other,double d) const {
    return {{"reason",reason},{"pair",{target_id_,other}},{"signed_distance_m",d},{"stage",stage_}};
  }
public:
  Json last_failure=nullptr,last_seal=nullptr;
  ProcessPolicy(const planning_scene::PlanningScenePtr& scene,const Json& request)
    :spec_(request.at("process_policy")),source_(request.at("clearance_policy").at("source_policy")),
     initial_(spec_.at("initial_proximity")),stage_(request.at("stage")),target_id_(spec_.at("target_id")),
     compliant_(request.at("clearance_policy").at("compliant_tool_links").get<std::set<std::string>>()),
     stack_(spec_.at("stack_carton_ids").get<std::set<std::string>>()),supports_(spec_.at("support_names").get<std::set<std::string>>()),
     target_(processBox(spec_.at("target"))) {
    if(spec_.at("schema")!="m710_native_process_v1" || spec_.at("stage")!=stage_ || spec_.at("collision_policy")!=source_ ||
       target_.id!=target_id_ || request.at("clearance_policy").at("target_id")!=target_id_)
      throw std::runtime_error("PROCESS_CONTEXT_MISMATCH");
    const std::set<std::string> stages={"pregrasp","contact","contact_endpoint","next_contact","support-release","extraction","transit","place","placement","withdrawal","residence"};
    if(!stages.count(stage_)) throw std::runtime_error("UNSUPPORTED_PROCESS_STAGE");
    if((stage_=="transit" || stage_=="residence") && !supports_.empty())
      throw std::runtime_error("NATIVE_FREE_SPACE_SUPPORT_PERMISSION_FORBIDDEN");
    if(spec_.at("cups").size()!=72 || spec_.at("commanded_active_mask").size()!=72 ||
       spec_.at("geometrically_eligible_mask").size()!=72 || spec_.at("actual_contact_mask").size()!=72)
      throw std::runtime_error("PROCESS_REQUIRES_72_CUP_GEOMETRIES_AND_MASKS");
    for(const auto key:{"commanded_active_mask","geometrically_eligible_mask","actual_contact_mask"})
      for(const auto& bit:spec_.at(key)) if(!bit.is_boolean()) throw std::runtime_error("PROCESS_MASK_NOT_BOOLEAN");
    gap_=source_.at("required_pair_clearance_m");tolerance_=spec_.at("maximum_penetration_m");
    if(!std::isfinite(tolerance_) || tolerance_<0 || tolerance_>.0002+1e-12) throw std::runtime_error("PROCESS_CONTACT_TOLERANCE_INVALID");
    for(const auto& box:request.at("world")) {auto b=processBox(box);world_.emplace(b.id,b);}
    if(world_.size()!=request.at("world").size()) throw std::runtime_error("DUPLICATE_PROCESS_OBJECT");
    const auto& state=scene->getCurrentState();attached_=state.hasAttachedBody(target_id_);
    for(const auto& name:stack_) if(!world_.count(name) && !(name==target_id_ && attached_)) throw std::runtime_error("PROCESS_STACK_OBJECT_MISSING");
    for(const auto& name:supports_) if(!world_.count(name) || name==target_id_) throw std::runtime_error("PROCESS_SUPPORT_OBJECT_MISSING");
    if(attached_==scene->getWorld()->hasObject(target_id_)) throw std::runtime_error("PROCESS_TARGET_IDENTITY_INVALID");
    if(!attached_ && !world_.count(target_id_)) throw std::runtime_error("PROCESS_TARGET_MISSING");
    if(!attached_ && (world_.at(target_id_).half-target_.half).cwiseAbs().maxCoeff()>1e-12)
      throw std::runtime_error("PROCESS_TARGET_GEOMETRY_CHANGED");
    const auto actual=targetAt(state);
    if((actual.pose.matrix()-target_.pose.matrix()).cwiseAbs().maxCoeff()>1e-7)
      throw std::runtime_error("PROCESS_TARGET_POSE_MISMATCH");
    const auto relaxation_stages=source_.at("stack_contact_stages").get<std::set<std::string>>();
    relaxed_=attached_ && relaxation_stages.count(stage_) && !initial_.is_null();
    if(relaxed_ && source_.at("stack_contact_mode")!="planner_relaxed_physics_checked" && !initial_.contains("pairs"))
      throw std::runtime_error("PROCESS_PROXIMITY_HISTORY_REQUIRED");
    if(!initial_.is_null() && !attached_) throw std::runtime_error("PROCESS_PROXIMITY_REQUIRES_ATTACHMENT");
    for(const auto& pair:request.at("allowed_pairs")) {
      if(pair.size()!=2) throw std::runtime_error("PROCESS_ALLOWED_PAIR_INVALID");
      const std::string a=pair.at(0),b=pair.at(1);
      const bool assembly=(a=="base_link" && b=="chassis") || (b=="base_link" && a=="chassis");
      const bool neighbor=source_.at("compliant_cup_neighbor_contact_mode")=="ignore" &&
        ((compliant_.count(a)&&stack_.count(b)&&b!=target_id_) || (compliant_.count(b)&&stack_.count(a)&&a!=target_id_));
      const bool contact_stage=attached_ || stage_=="contact" || stage_=="contact_endpoint" || stage_=="next_contact" || stage_=="withdrawal";
      const bool target_cup=contact_stage && ((compliant_.count(a)&&b==target_id_) || (compliant_.count(b)&&a==target_id_));
      if(!assembly&&!neighbor&&!target_cup) throw std::runtime_error("PROCESS_UNSCOPED_ACM_PERMISSION");
    }
    auto& acm=scene->getAllowedCollisionMatrixNonConst();
    // Explicit stage-local named neighbor permissions retain the persistent
    // flexible-lip exception, never rigid inserts or unknown world objects.
    if(source_.at("compliant_cup_neighbor_contact_mode")=="ignore")
      for(const auto& cup:compliant_) for(const auto& neighbor:stack_) if(neighbor!=target_id_) acm.setEntry(cup,neighbor,true);
    const bool contact=attached_ || stage_=="contact" || stage_=="contact_endpoint" || stage_=="next_contact" || stage_=="withdrawal";
    if(contact) for(const auto& cup:compliant_) acm.setEntry(cup,target_id_,true);
    if(attached_) for(const auto& support:supports_) acm.setEntry(target_id_,support,true);
    if(relaxed_) {
      if(source_.at("stack_contact_mode")=="planner_relaxed_physics_checked") {
        if(initial_.at("mode")!=source_.at("stack_contact_mode") || initial_.at("target")!=target_id_)
          throw std::runtime_error("PROCESS_STACK_TRACKER_MISMATCH");
        auto declared=initial_.at("stack_carton_names").get<std::set<std::string>>();auto expected=stack_;expected.erase(target_id_);
        if(declared!=expected) throw std::runtime_error("PROCESS_STACK_TRACKER_COVERAGE_MISMATCH");
        for(const auto& neighbor:declared) acm.setEntry(target_id_,neighbor,true);
      } else {
        if(initial_.at("payload")!=target_id_) throw std::runtime_error("PROCESS_PROXIMITY_TARGET_MISMATCH");
        for(const auto& pair:initial_.at("pairs")) {
          const std::string other=pair.at("obstacle");if(!stack_.count(other)||other==target_id_) throw std::runtime_error("PROCESS_PROXIMITY_OBJECT_INVALID");
          acm.setEntry(target_id_,other,true);
        }
      }
    }
  }
  bool check(const moveit::core::RobotState& state,bool endpoint=false) {
    ++state_checks_;last_failure=nullptr;const auto target=targetAt(state);
    const bool contact=attached_ || stage_=="contact" || stage_=="contact_endpoint" || stage_=="next_contact" || stage_=="withdrawal";
    if(contact) for(const auto& name:compliant_) {
      const auto* link=state.getRobotModel()->getLinkModel(name);
      if(!link || link->getShapes().empty()) throw std::runtime_error("PROCESS_CUP_GEOMETRY_MISSING");
      for(size_t i=0;i<link->getShapes().size();++i) {
        if(link->getShapes()[i]->type!=shapes::BOX) throw std::runtime_error("PROCESS_CUP_GEOMETRY_UNSUPPORTED");
        const auto* shape=static_cast<const shapes::Box*>(link->getShapes()[i].get());
        ProcessBox cup;cup.id=name;cup.pose=state.getCollisionBodyTransform(link,i);cup.half=Eigen::Vector3d(shape->size[0],shape->size[1],shape->size[2])/2;
        const double d=processSignedGap(cup,target);
        if(d < -source_.at("maximum_compliant_cup_additional_compression_m").get<double>()) {
          last_failure={{"reason","FLEXIBLE_CUP_TARGET_COMPRESSION"},{"pair",{name,target_id_}},{"signed_distance_m",d}};return false;
        }
      }
    }
    if(attached_) {
      for(const auto& name:supports_) if(!(relaxed_&&stack_.count(name)) && !processSupportSeparated(target,world_.at(name),tolerance_)) {
        last_failure=fail("SUPPORT_SIDE_OR_DEEP_PENETRATION",name,processSignedGap(target,world_.at(name)));return false;
      }
      if(relaxed_) for(const auto& name:stack_) if(name!=target_id_) {
        if(source_.at("stack_contact_mode")=="planner_relaxed_physics_checked") {
          const double d=processSignedGap(target,world_.at(name));
          if(d < -source_.at("maximum_planned_stack_penetration_m").get<double>()) {last_failure=fail("GROSS_PLANNED_STACK_PENETRATION",name,d);return false;}
        }
      }
    }
    if(attached_ || (endpoint && (stage_=="contact" || stage_=="contact_endpoint" || stage_=="next_contact"))) {
      last_seal=processSealRings(spec_,target,state.getGlobalLinkTransform("flange"));
      if(last_seal.value("status",std::string())!="PASS") {last_failure=last_seal;return false;}
    }
    return true;
  }
  Json checkPath(moveit::core::RobotState state,const Json& path,double resolution) {
    if(path.empty() || !std::isfinite(resolution) || resolution<=0) throw std::runtime_error("INVALID_PROCESS_CHECK_PATH");
    bool released=!initial_.is_null() && initial_.value("fully_released",false);
    std::map<std::string,Json> pairs;if(relaxed_ && initial_.contains("pairs"))
      for(const auto& item:initial_.at("pairs")) pairs[item.at("obstacle").get<std::string>()]=item;
    for(size_t edge=0;edge<std::max(size_t(1),path.size()-1);++edge) {
      const auto a=path.at(edge).get<std::vector<double>>(),b=path.at(std::min(edge+1,path.size()-1)).get<std::vector<double>>();double maximum=0,sum=0;
      for(size_t i=0;i<a.size();++i) {maximum=std::max(maximum,std::abs(b[i]-a[i]));sum+=std::abs(b[i]-a[i]);}
      const size_t samples=2*std::max({size_t(1),size_t(std::ceil(maximum/resolution)),size_t(std::ceil(4*sum/.0025))});
      for(size_t k=(edge?1:0);k<=samples;++k) {
        ++ordered_checks_;auto q=a;for(size_t i=0;i<q.size();++i) q[i]+=(b[i]-a[i])*double(k)/samples;
        state.setJointGroupPositions("manipulator",q);state.update();
        auto bad=[&](Json failure) {failure["edge"]=edge;failure["fraction"]=double(k)/samples;failure["q_rad"]=q;return failure;};
        if(!check(state,edge+2>=path.size() && k==samples)) return bad(last_failure);
        if(!relaxed_) continue;const auto target=targetAt(state);
        if(source_.at("stack_contact_mode")=="planner_relaxed_physics_checked") {
          bool clear=true;
          for(const auto& name:stack_) if(name!=target_id_) {
            const double d=surfaceGap(target,world_.at(name));
            if(released && (d<=0 || d+1e-9<gap_)) return bad(fail("PAYLOAD_COLLISION_FREE_SPACE_RULES_RESTORED",name,d));
            clear=clear && d>=source_.at("free_space_clearance_m").get<double>();
          }
          released=released||clear;
        } else for(auto& item:pairs) {
          auto& pair=item.second;const double d=processSignedGap(target,world_.at(item.first));
          const double tolerance=initial_.at("monotonic_tolerance_m"),penetration=initial_.at("penetration_tolerance_m");
          if(d < -penetration) return bad(fail("PAYLOAD_PROXIMITY_PENETRATION",item.first,d));
          if(pair.at("released").get<bool>()) {
            if(surfaceGap(target,world_.at(item.first))+1e-9<gap_) return bad(fail("PAYLOAD_PROXIMITY_REENTRY",item.first,d));
          } else if(d<pair.at("initial_signed_distance_m").get<double>()-tolerance || d<pair.at("last_signed_distance_m").get<double>()-tolerance)
            return bad(fail("PAYLOAD_PROXIMITY_WORSENED",item.first,d));
          pair["last_signed_distance_m"]=d;
          if(surfaceGap(target,world_.at(item.first))>=gap_) pair["released"]=true;
        }
      }
    }
    if(relaxed_) {
      terminal_proximity_=initial_;terminal_proximity_["fully_released"]=released;
      if(initial_.contains("pairs")) {
        terminal_proximity_["pairs"]=Json::array();bool all=true;
        for(const auto& item:pairs) {terminal_proximity_["pairs"].push_back(item.second);all=all&&item.second.at("released").get<bool>();}
        terminal_proximity_["fully_released"]=all;
      }
    }
    return nullptr;
  }
  Json terminalProximity() const {return terminal_proximity_;}
  Json evidence() const {return {{"schema","m710_native_process_evidence_v1"},{"stage",stage_},{"target_id",target_id_},
    {"attached",attached_},{"state_checks",state_checks_},{"ordered_checks",ordered_checks_},{"relaxed_stack",relaxed_},
    {"support_names",supports_},{"last_seal",last_seal},{"last_failure",last_failure},{"terminal_proximity",terminal_proximity_},{"continuous_swept_proof",false}};}
};
}
