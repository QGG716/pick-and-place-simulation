#pragma once
#include "process_geometry.h"
#include "native_robot.h"
#include "clearance.h"
#include "eco65_distance_policy.h"

namespace m710 {
// Separate ECO65 policy: no circular masks, neighbor permission, or FANUC wrist
// exemption. Only the terminal bands, bound mount interfaces and target supports are
// removed from ordinary clearance and checked here at every candidate state.
class Eco65Process {
  Json spec_;
  std::string stage_,target_id_;
  ProcessBox target_;
  bool attached_;
  bool record_distances_=false;
  bool fixed_mounts_accepted_=false;
  std::set<std::string> bands_,supports_;
  std::map<std::string,ProcessBox> world_;
  struct WorldSolid {std::shared_ptr<fcl::CollisionGeometryd> shape;Eigen::Isometry3d pose;};
  std::map<std::string,WorldSolid> world_solids_;
  struct Solid {const moveit::core::LinkModel* link;size_t index;std::shared_ptr<fcl::CollisionGeometryd> shape;};
  struct Pair {Solid a;Solid b;double fixed_distance=0.;bool fixed_chain=false;};
  std::vector<Solid> bands;
  std::vector<Pair> mount_pairs;
  Solid base_solid;
  static Solid solid(const moveit::core::RobotState& state,const std::string& name,size_t index=0,bool compound=false) {
    const auto* link=state.getRobotModel()->getLinkModel(name);
    if(!link || index>=link->getShapes().size() || (!compound && link->getShapes().size()!=1))
      throw std::runtime_error("ECO65_SINGLE_CONVEX_SOLID_REQUIRED:"+name);
    auto shape=link->getShapes()[index];std::shared_ptr<fcl::CollisionGeometryd> result;
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
    return {link,index,result};
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
  static double distance(const ProcessBox& a,const WorldSolid& b) {
    fcl::CollisionObjectd oa(std::make_shared<fcl::Boxd>(2*a.half.x(),2*a.half.y(),2*a.half.z()),a.pose);
    fcl::CollisionObjectd ob(b.shape,b.pose);
    fcl::DistanceRequestd request(true,true);fcl::DistanceResultd result;
    const double d=fcl::distance(&oa,&ob,request,result);
    if(!std::isfinite(d))throw std::runtime_error("ECO65_SIGNED_DISTANCE_UNKNOWN");return d;
  }
  void importWorld(const Json& row) {
    const std::string id=row.at("id");
    if(world_.count(id))throw std::runtime_error("DUPLICATE_PROCESS_OBJECT");
    if(!row.contains("convex_mesh")) {
      auto box=processBox(row);world_.emplace(id,box);
      if(supports_.count(id))world_solids_.emplace(id,WorldSolid{
        std::make_shared<fcl::Boxd>(2*box.half.x(),2*box.half.y(),2*box.half.z()),box.pose});
      return;
    }
    // The finite conveyor pieces remain real FCL convex solids. Their world
    // envelope below is only a conservative top-contact condition; it never
    // replaces the actual target-to-conveyor signed-distance query.
    const auto pose=processTransform(row.at("pose"));const auto& mesh=row.at("convex_mesh");
    auto vertices=std::make_shared<std::vector<fcl::Vector3d>>();auto faces=std::make_shared<std::vector<int>>();
    Eigen::Vector3d low=Eigen::Vector3d::Constant(INFINITY),high=Eigen::Vector3d::Constant(-INFINITY);
    for(const auto& item:mesh.at("vertices")) {
      const auto p=item.get<std::vector<double>>();
      if(p.size()!=3)throw std::runtime_error("ECO65_WORLD_CONVEX_VERTEX_INVALID");
      Eigen::Vector3d v(p[0],p[1],p[2]);if(!v.allFinite())throw std::runtime_error("ECO65_WORLD_CONVEX_VERTEX_INVALID");
      vertices->push_back(v);const Eigen::Vector3d transformed=pose*v;
      low=low.cwiseMin(transformed);high=high.cwiseMax(transformed);
    }
    for(const auto& item:mesh.at("triangles")) {
      const auto triangle=item.get<std::vector<int>>();
      if(triangle.size()!=3)throw std::runtime_error("ECO65_WORLD_CONVEX_FACE_INVALID");
      faces->push_back(3);
      for(int index:triangle) {
        if(index<0 || size_t(index)>=vertices->size())throw std::runtime_error("ECO65_WORLD_CONVEX_INDEX_INVALID");
        faces->push_back(index);
      }
    }
    if(vertices->size()<4 || faces->size()<16 || (high-low).minCoeff()<=0)
      throw std::runtime_error("ECO65_WORLD_CONVEX_EMPTY");
    ProcessBox envelope;envelope.id=id;envelope.pose.translation()=(low+high)/2;envelope.half=(high-low)/2;
    world_.emplace(id,envelope);
    // Clearance already owns every world solid. This policy needs an extra
    // FCL shape only for the named support pairs it checks independently.
    if(supports_.count(id))world_solids_.emplace(id,WorldSolid{
      std::make_shared<fcl::Convexd>(vertices,int(mesh.at("triangles").size()),faces,true),pose});
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
       (spec_.at("target_face")!="top" && spec_.at("target_face")!="front") || bands_.size()!=12)
      throw std::runtime_error("ECO65_PROCESS_CONTEXT_MISMATCH");
    if(stage_!="pregrasp" && stage_!="contact" && stage_!="extraction" && stage_!="transit" &&
       stage_!="place" && stage_!="withdrawal" && stage_!="residence") throw std::runtime_error("ECO65_PROCESS_STAGE_INVALID");
    if((stage_=="transit" || stage_=="residence")&&!supports_.empty())throw std::runtime_error("ECO65_FREE_SUPPORT_FORBIDDEN");
    std::set<std::string> cartons,original_supports,receiver_supports;
    const bool contracted=spec_.contains("task_contract");
    if(contracted) {
      // Worker initialization binds every task_contract field to the scene's
      // contact geometry. A motion request cannot widen these permissions.
      const auto& task=spec_.at("task_contract");
      const bool independent=task.value("cup_selection_mode",std::string())=="SIMULATED_INDEPENDENT_CUPS";
      if(task.at("schema")!="eco65_single_carton_task_v1" || task.at("target_id")!=target_id_ ||
         (!independent && (task.at("target_face")!="top" || spec_.at("target_face")!="top")))
        throw std::runtime_error("ECO65_TASK_CONTRACT_MISMATCH");
      if(independent) {
        if(spec_.value("cup_selection_mode",std::string())!="SIMULATED_INDEPENDENT_CUPS" ||
           task.at("allowed_faces")!=Json::array({"top","front"}) || !spec_.contains("active_ids") ||
           !spec_.contains("selection_binding"))throw std::runtime_error("ECO65_CUP_SELECTION_REQUIRED");
        const auto& binding=spec_.at("selection_binding");
        if(binding.at("active_ids")!=spec_.at("active_ids") || binding.at("target_id")!=target_id_ ||
           binding.at("target_face")!=spec_.at("target_face") || binding.at("task_id")!=req.at("task_id") ||
           binding.at("identity")!=req.at("identity") || binding.at("candidate_id").get<std::string>().empty())
          throw std::runtime_error("ECO65_CUP_SELECTION_BINDING_MISMATCH");
      }
      cartons=task.at("stack_carton_ids").get<std::set<std::string>>();
      original_supports=task.at("original_support_names").get<std::set<std::string>>();
      receiver_supports=task.at("receiving").at("support_ids").get<std::set<std::string>>();
      const bool coverage=task.contains("coverage_context");
      if(coverage) {
        const auto& context=task.at("coverage_context");
        auto declared=context.at("remaining_carton_ids").get<std::set<std::string>>();
        auto removed=context.at("removed_carton_ids").get<std::set<std::string>>();
        if(context.at("scope")!="CONDITIONAL_COVERAGE_DIAGNOSTIC_NOT_COMPLETE_TASK" || declared!=cartons ||
           declared.size()+removed.size()!=12)throw std::runtime_error("ECO65_COVERAGE_CONTEXT_INVALID");
        for(const auto& id:removed)if(cartons.count(id))throw std::runtime_error("ECO65_COVERAGE_CARTON_DUPLICATE");
      }
      if((!coverage && cartons.size()!=12) || task.at("stack_carton_ids").size()!=cartons.size() || !cartons.count(target_id_) ||
         original_supports.empty() || original_supports.count(target_id_) || receiver_supports.empty() ||
         task.at("original_support_names").size()!=original_supports.size() ||
         task.at("receiving").at("support_ids").size()!=receiver_supports.size())
        throw std::runtime_error("ECO65_TASK_IDENTITIES_INVALID");
      for(const auto& id:receiver_supports)if(cartons.count(id))throw std::runtime_error("ECO65_RECEIVER_IS_CARTON");
    } else {
      if(target_id_!="carton_r00_l01_c00")throw std::runtime_error("ECO65_FIXED_TARGET_REQUIRED");
      cartons={"carton_r00_l00_c00","carton_r00_l00_c01","carton_r00_l01_c00","carton_r00_l01_c01"};
      original_supports={"carton_r00_l00_c00"};receiver_supports={"conveyor_longitudinal_belt"};
    }
    const std::set<std::string> allowed_supports=stage_=="extraction"?original_supports:
      stage_=="place"?receiver_supports:std::set<std::string>{};
    if(contracted && supports_!=allowed_supports)throw std::runtime_error("ECO65_SUPPORT_CONTRACT_CHANGED");
    for(const auto& id:supports_)if(!allowed_supports.count(id))throw std::runtime_error("ECO65_SUPPORT_PERMISSION_INVALID");
    for(const auto& row:req.at("world")) importWorld(row);
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
    if(spec_.at("stack_carton_ids").get<std::set<std::string>>()!=cartons)
      throw std::runtime_error(contracted?"ECO65_TWELVE_CARTONS_REQUIRED":"ECO65_FOUR_CARTONS_REQUIRED");
    if(spec_.at("stack_carton_ids").size()!=cartons.size())throw std::runtime_error("ECO65_CARTON_ID_DUPLICATE");
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
    if(spec_.contains("fixed_attachment_interfaces")) {
      const auto& interfaces=spec_.at("fixed_attachment_interfaces");
      auto expected=[](const std::string& name) {
        for(const std::string base:{"eco65_mast_mast_part_001","eco65_mast_mounting_seat"}) {
          if(name==base)return true;
          const auto prefix=base+"__convex_";
          if(name.size()==prefix.size()+3 && name.compare(0,prefix.size(),prefix)==0 &&
             name.find_first_not_of("0123456789",prefix.size())==std::string::npos)return true;
        }
        return false;
      };
      std::set<std::string> seen;
      std::map<std::string,std::vector<Solid>> parent_solids;
      if(!contracted || !interfaces.is_array())
        throw std::runtime_error("ECO65_FIXED_ATTACHMENT_INTERFACES_INVALID");
      for(const auto& mount:interfaces) {
        const std::string robot=mount.at("parent_link"),attachment=mount.at("attachment_id");
        if(robot!="link_1" || !expected(attachment) || !seen.insert(attachment).second ||
           !mount.contains("source") || mount.at("source").empty() ||
           mount.at("maximum_penetration_m")!=.0004 || owned.count(attachment) || bands_.count(attachment))
          throw std::runtime_error("ECO65_FIXED_ATTACHMENT_INTERFACE_UNBOUND");
        const auto* parent=state.getRobotModel()->getLinkModel(robot);
        if(!parent || parent->getShapes().empty())throw std::runtime_error("ECO65_FIXED_PARENT_GEOMETRY_MISSING");
        const auto child=solid(state,attachment);
        auto& parents=parent_solids[robot];
        if(parents.empty())for(size_t i=0;i<parent->getShapes().size();++i)
          parents.push_back(solid(state,robot,i,true));
        for(const auto& piece:parents)mount_pairs.push_back({piece,child});
        // Restore the finite fixed-contact predicate below instead of an
        // unconditional SRDF exemption or a wider request-level ACM entry.
        acm.setEntry(robot,attachment,true);
      }
    }
    if(attached_)for(const auto& id:supports_)acm.setEntry(target_id_,id,true);
    // These interfaces consist solely of fixed joints below one moving link.
    // Query real FCL once at request import, then reuse that rigid invariant
    // through this request's path. No articulated pair gets this optimization.
    for(auto& pair:mount_pairs) {
      auto* child=pair.b.link;
      while(child && child!=pair.a.link) {
        const auto* joint=child->getParentJointModel();
        if(!joint || joint->getType()!=moveit::core::JointModel::FIXED)break;
        child=child->getParentLinkModel();
      }
      pair.fixed_chain=child==pair.a.link;
      if(!pair.fixed_chain)throw std::runtime_error("ECO65_MOUNT_NOT_A_FIXED_JOINT_CHAIN");
      pair.fixed_distance=distance(pair,state);
    }
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
    if(!fixed_mounts_accepted_ || record_distances_) for(const auto& pair:mount_pairs) {
      const bool valid=accept(pair.a.link->getName(),pair.b.link->getName(),pair.fixed_distance,-.0004);
      if(record_distances_) {
        distance_samples.back()["robot_shape_index"]=pair.a.index;
        distance_samples.back()["attachment_shape_index"]=pair.b.index;
        distance_samples.back()["query_scope"]="FCL_AT_REQUEST_IMPORT_FIXED_JOINT_CHAIN_INVARIANT";
      }
      if(!valid) {failure["robot_shape_index"]=pair.a.index;failure["attachment_shape_index"]=pair.b.index;return false;}
    }
    fixed_mounts_accepted_=true;
    for(const auto& band:bands) {
      bool contact=fit;
      if(spec_.value("cup_selection_mode",std::string())=="SIMULATED_INDEPENDENT_CUPS") {
        contact=false;
        for(const auto& row:seal.at("contours")) {
          if(spec_.at("terminal_band_by_contour").at(row.at("id").get<std::string>())==band.link->getName())
            contact=row.at("face_contact").get<bool>();
        }
      }
      // Every band, enabled or disabled, still gets a real signed-distance
      // query. Face-aligned terminal contact permits only the original .4 mm;
      // it grants no neighbor, rigid-tool or other-face permission.
      if(!accept(band.link->getName(),target_id_,distance(band,state,target),contact?-.0004:.0024))return false;
    }
    if(attached_)for(const auto& id:supports_) {
      const auto& support=world_.at(id);
      const Eigen::Vector3d extent=target.pose.linear().cwiseAbs()*target.half;
      bool overlap=true;
      for(int k=0;k<2;++k)overlap=overlap &&
        std::min(target.pose.translation()[k]+extent[k],support.pose.translation()[k]+support.half[k])>
        std::max(target.pose.translation()[k]-extent[k],support.pose.translation()[k]-support.half[k]);
      const bool top=overlap && (support.pose.linear()-Eigen::Matrix3d::Identity()).norm()<1e-8 &&
        processSupportSeparated(target,support,.0004);
      if(!accept(target_id_,id,distance(target,world_solids_.at(id)),top?-.0004:.0024))return false;
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
