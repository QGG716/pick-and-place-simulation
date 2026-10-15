#pragma once
#include "process_geometry.h"
#include "native_robot.h"
#include "clearance.h"
#include "eco65_distance_policy.h"

namespace m710 {
// Separate ECO65 policy: no circular masks, neighbor permission, or FANUC wrist
// exemption. Only the terminal bands, two named mounts and target supports are
// removed from ordinary clearance and checked here at every candidate state.
class Eco65Process {
  Json spec_;
  std::string stage_,target_id_;
  ProcessBox target_;
  bool attached_;
  bool record_distances_=false;
  std::set<std::string> bands_,supports_;
  std::map<std::string,ProcessBox> world_;
  struct Solid {const moveit::core::LinkModel* link;size_t index;std::shared_ptr<fcl::CollisionGeometryd> shape;};
  struct Pair {Solid a;Solid b;};
  std::vector<Solid> bands;
  std::vector<Pair> mount_pairs;
  Solid base_solid;
  static Solid solid(const moveit::core::RobotState& state,const std::string& name) {
    const auto* link=state.getRobotModel()->getLinkModel(name);
    if(!link || link->getShapes().size()!=1) throw std::runtime_error("ECO65_SINGLE_CONVEX_SOLID_REQUIRED:"+name);
    auto shape=link->getShapes()[0];std::shared_ptr<fcl::CollisionGeometryd> result;
    if(shape->type==shapes::MESH) {
      const auto* m=static_cast<const shapes::Mesh*>(shape.get());
      auto points=std::make_shared<std::vector<fcl::Vector3d>>();
      auto faces=std::make_shared<std::vector<int>>();
      for(size_t i=0;i<m->vertex_count;++i)points->emplace_back(m->vertices[3*i],m->vertices[3*i+1],m->vertices[3*i+2]);
      for(size_t i=0;i<m->triangle_count;++i) {faces->push_back(3);for(int j=0;j<3;++j)faces->push_back(m->triangles[3*i+j]);}
      result=std::make_shared<fcl::Convexd>(points,m->triangle_count,faces,true);
    } else if(shape->type==shapes::BOX) {
      auto* b=static_cast<const shapes::Box*>(shape.get());result=std::make_shared<fcl::Boxd>(b->size[0],b->size[1],b->size[2]);
    } else if(shape->type==shapes::CYLINDER) {
      auto* c=static_cast<const shapes::Cylinder*>(shape.get());result=std::make_shared<fcl::Cylinderd>(c->radius,c->length);
    } else throw std::runtime_error("ECO65_CONTACT_SOLID_UNSUPPORTED");
    return {link,0,result};
  }
  static double distance(const Solid& a,const moveit::core::RobotState& state,const ProcessBox& b) {
    fcl::CollisionObjectd oa(a.shape,state.getCollisionBodyTransform(a.link,a.index));
    fcl::CollisionObjectd ob(std::make_shared<fcl::Boxd>(2*b.half.x(),2*b.half.y(),2*b.half.z()),b.pose);
    fcl::DistanceRequestd request(true,true);fcl::DistanceResultd result;
    const double d=fcl::distance(&oa,&ob,request,result);
    if(!std::isfinite(d))throw std::runtime_error("ECO65_SIGNED_DISTANCE_UNKNOWN");return d;
  }
  static double distance(const Pair& pair,const moveit::core::RobotState& state) {
    fcl::CollisionObjectd a(pair.a.shape,state.getCollisionBodyTransform(pair.a.link,pair.a.index));
    fcl::CollisionObjectd b(pair.b.shape,state.getCollisionBodyTransform(pair.b.link,pair.b.index));
    fcl::DistanceRequestd request(true,true);fcl::DistanceResultd result;
    const double d=fcl::distance(&a,&b,request,result);
    if(!std::isfinite(d))throw std::runtime_error("ECO65_SIGNED_DISTANCE_UNKNOWN");return d;
  }
  static double distance(const ProcessBox& a,const ProcessBox& b) {
    fcl::CollisionObjectd oa(std::make_shared<fcl::Boxd>(2*a.half.x(),2*a.half.y(),2*a.half.z()),a.pose);
    fcl::CollisionObjectd ob(std::make_shared<fcl::Boxd>(2*b.half.x(),2*b.half.y(),2*b.half.z()),b.pose);
    fcl::DistanceRequestd request(true,true);fcl::DistanceResultd result;
    const double d=fcl::distance(&oa,&ob,request,result);
    if(!std::isfinite(d))throw std::runtime_error("ECO65_SIGNED_DISTANCE_UNKNOWN");return d;
  }
public:
  Json failure=nullptr,seal=nullptr,distance_samples=Json::array();
  Eco65Process(const planning_scene::PlanningScenePtr& scene,const Json& req)
    :spec_(req.at("process_policy")),stage_(req.at("stage")),target_id_(spec_.at("target_id")),
     target_(processBox(spec_.at("target"))),attached_(!req.at("attachment").is_null()),
     bands_(req.at("clearance_policy").at("compliant_tool_links").get<std::set<std::string>>()),
     supports_(spec_.at("support_names").get<std::set<std::string>>()) {
    record_distances_=req.at("op")=="check_state_offline" && req.value("record_contact_distances",false);
    validateSealContours(spec_);processTransform(spec_.at("flange_from_contact_frame"));
    if(spec_.at("schema")!="eco65_native_process_v1" || spec_.at("stage")!=stage_ ||
       target_id_!=req.at("clearance_policy").at("target_id") || target_.id!=target_id_ ||
       spec_.at("collision_policy")!=req.at("clearance_policy").at("source_policy") ||
       spec_.at("maximum_penetration_m")!=.0004 || spec_.at("suction_edge_margin_m")!=.003 ||
       spec_.at("target_face")!="top" || bands_.size()!=12)
      throw std::runtime_error("ECO65_PROCESS_CONTEXT_MISMATCH");
    if(stage_!="pregrasp" && stage_!="contact" && stage_!="extraction" && stage_!="transit" &&
       stage_!="place" && stage_!="withdrawal" && stage_!="residence") throw std::runtime_error("ECO65_PROCESS_STAGE_INVALID");
    if((stage_=="transit" || stage_=="residence")&&!supports_.empty())throw std::runtime_error("ECO65_FREE_SUPPORT_FORBIDDEN");
    if(target_id_!="carton_r00_l01_c00")throw std::runtime_error("ECO65_FIXED_TARGET_REQUIRED");
    const std::set<std::string> allowed_supports=stage_=="extraction"?
      std::set<std::string>{"carton_r00_l00_c00"}:stage_=="place"?
      std::set<std::string>{"conveyor_longitudinal_belt"}:std::set<std::string>{};
    for(const auto& id:supports_)if(!allowed_supports.count(id))throw std::runtime_error("ECO65_SUPPORT_PERMISSION_INVALID");
    for(const auto& row:req.at("world")) {auto b=processBox(row);if(!world_.emplace(b.id,b).second)throw std::runtime_error("DUPLICATE_PROCESS_OBJECT");}
    for(const auto& id:supports_)if(!world_.count(id)||id==target_id_)throw std::runtime_error("ECO65_SUPPORT_MISSING");
    if(req.at("allowed_pairs")!=Json::array({Json::array({"base_link","robot_mount_plate"})}))
      throw std::runtime_error("ECO65_UNSCOPED_ACM_PERMISSION");
    const auto& state=scene->getCurrentState();
    if(attached_!=state.hasAttachedBody(target_id_) || attached_==scene->getWorld()->hasObject(target_id_))
      throw std::runtime_error("ECO65_TARGET_OWNERSHIP_INVALID");
    if(attached_) {
      const auto* a=state.getAttachedBody(target_id_);
      if(a->getTouchLinks()!=bands_)throw std::runtime_error("ECO65_TOUCH_LINKS_INVALID");
      if(a->getShapes().size()!=1 || a->getShapes()[0]->type!=shapes::BOX)
        throw std::runtime_error("ECO65_ATTACHMENT_SHAPE_INVALID");
      const auto* shape=static_cast<const shapes::Box*>(a->getShapes()[0].get());
      for(int i=0;i<3;++i)if(std::abs(shape->size[i]-2*target_.half[i])>1e-12)
        throw std::runtime_error("ECO65_TARGET_GEOMETRY_CHANGED");
      if((a->getGlobalCollisionBodyTransforms().at(0).matrix()-target_.pose.matrix()).cwiseAbs().maxCoeff()>1e-7)
        throw std::runtime_error("ECO65_TARGET_POSE_CHANGED");
    } else {
      if((world_.at(target_id_).half-target_.half).cwiseAbs().maxCoeff()>1e-12 ||
         (world_.at(target_id_).pose.matrix()-target_.pose.matrix()).cwiseAbs().maxCoeff()>1e-7)
        throw std::runtime_error("ECO65_TARGET_GEOMETRY_OR_POSE_CHANGED");
    }
    const std::set<std::string> cartons={"carton_r00_l00_c00","carton_r00_l00_c01","carton_r00_l01_c00","carton_r00_l01_c01"};
    if(spec_.at("stack_carton_ids").get<std::set<std::string>>()!=cartons)
      throw std::runtime_error("ECO65_FOUR_CARTONS_REQUIRED");
    for(const auto& id:cartons)if(id!=target_id_ && !world_.count(id))throw std::runtime_error("ECO65_OTHER_CARTON_MISSING");
    auto& acm=scene->getAllowedCollisionMatrixNonConst();
    for(const auto& id:bands_) {bands.push_back(solid(state,id));acm.setEntry(id,target_id_,true);}
    base_solid=solid(state,nativeFrames().base_link);
    const auto mounts=spec_.at("mounting_contacts");
    if(!mounts.is_array() || mounts.empty())throw std::runtime_error("ECO65_MOUNT_IDENTITY_REQUIRED");
    std::set<std::string> mounted;
    const auto owned=req.at("clearance_policy").at("tool_links").get<std::set<std::string>>();
    for(const auto& mount:mounts) {
      const std::string robot=mount.at("robot_link"),tool=mount.at("tool_collision");
      if(robot!=nativeFrames().flange_link || !owned.count(tool) || bands_.count(tool) ||
         !mounted.insert(tool).second || mount.at("maximum_penetration_m")!=.0004)
        throw std::runtime_error("ECO65_MOUNT_IDENTITY_INVALID");
      mount_pairs.push_back({solid(state,robot),solid(state,tool)});
      acm.setEntry(robot,tool,true);
    }
    if(attached_)for(const auto& id:supports_)acm.setEntry(target_id_,id,true);
  }
  bool check(const moveit::core::RobotState& state,bool endpoint) {
    failure=nullptr;distance_samples=Json::array();auto target=target_;
    if(attached_) {
      const auto* a=state.getAttachedBody(target_id_);
      if(!a || a->getShapes().size()!=1 || a->getShapes()[0]->type!=shapes::BOX)throw std::runtime_error("ECO65_ATTACHMENT_CHANGED");
      target.pose=a->getGlobalCollisionBodyTransforms().at(0);
    }
    const auto flange=state.getGlobalLinkTransform(nativeFrames().flange_link);
    auto geometry=spec_;
    const bool entering=!attached_&&(stage_=="pregrasp"||stage_=="contact"||stage_=="withdrawal");
    geometry["max_attachment_gap_m"]=entering?.0024:.0004;
    // Preserve the source's symmetric near-contact envelope; endpoint binding
    // below still requires the actual .4 mm seal/contact tolerance.
    geometry["maximum_penetration_m"]=entering?.0024:.0004;
    seal=processSealRings(geometry,target,flange);
    const bool fit=seal.at("status")=="PASS";
    auto accept=[&](const std::string& a,const std::string& b,double d,double threshold) {
      if(record_distances_)distance_samples.push_back({{"pair",{a,b}},{"signed_distance_m",d},
        {"threshold_m",threshold},{"accepted",eco65DistanceAccepted(d,threshold)},{"query","FCL_signed_distance"}});
      if(!eco65DistanceAccepted(d,threshold)) {failure={{"reason","ECO65_CONTACT_OR_CLEARANCE"},{"pair",{a,b}},
        {"signed_distance_m",d},{"threshold_m",threshold},{"stage",stage_}};return false;}return true;
    };
    if(!accept(nativeFrames().base_link,"robot_mount_plate",distance(base_solid,state,world_.at("robot_mount_plate")),-.0004))return false;
    for(const auto& pair:mount_pairs)if(!accept(pair.a.link->getName(),pair.b.link->getName(),distance(pair,state),-.0004))return false;
    for(const auto& band:bands)if(!accept(band.link->getName(),target_id_,distance(band,state,target),fit?-.0004:.0024))return false;
    if(attached_)for(const auto& id:supports_) {
      const auto& support=world_.at(id);
      const Eigen::Vector3d extent=target.pose.linear().cwiseAbs()*target.half;
      bool overlap=true;
      for(int k=0;k<2;++k)overlap=overlap &&
        std::min(target.pose.translation()[k]+extent[k],support.pose.translation()[k]+support.half[k])>
        std::max(target.pose.translation()[k]-extent[k],support.pose.translation()[k]-support.half[k]);
      const bool top=overlap && (support.pose.linear()-Eigen::Matrix3d::Identity()).norm()<1e-8 &&
        processSupportSeparated(target,support,.0004);
      if(!accept(target_id_,id,distance(target,support),top?-.0004:.0024))return false;
    }
    if(attached_ || (endpoint&&stage_=="contact")) {
      geometry["max_attachment_gap_m"]=.0004;geometry["maximum_penetration_m"]=.0004;
      seal=processSealRings(geometry,target,flange);
      if(seal.at("status")!="PASS") {failure=seal;return false;}
    }
    return true;
  }
};
}
