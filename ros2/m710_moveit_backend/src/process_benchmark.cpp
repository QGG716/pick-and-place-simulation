// Fixed archived-input checker benchmark. No solver, export or execution API.
#include "process_policy.h"
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <fstream>
#include <iostream>
using J=nlohmann::json;
using Clock=std::chrono::steady_clock;
double seconds(Clock::time_point t){return std::chrono::duration<double>(Clock::now()-t).count();}
J read(const char* path){std::ifstream f(path);if(!f)throw std::runtime_error("INPUT_MISSING");J j;f>>j;return j;}
moveit_msgs::msg::CollisionObject object(const J& b,const std::string& frame="world"){
  const auto box=m710::processBox(b);moveit_msgs::msg::CollisionObject o;
  o.id=box.id;o.header.frame_id=frame;o.operation=o.ADD;
  shape_msgs::msg::SolidPrimitive s;s.type=s.BOX;s.dimensions={2*box.half.x(),2*box.half.y(),2*box.half.z()};
  geometry_msgs::msg::Pose p;Eigen::Quaterniond q(box.pose.linear());
  p.orientation.x=q.x();p.orientation.y=q.y();p.orientation.z=q.z();p.orientation.w=q.w();
  p.position.x=box.pose.translation().x();p.position.y=box.pose.translation().y();p.position.z=box.pose.translation().z();
  o.primitives.push_back(s);o.primitive_poses.push_back(p);return o;
}
int main(int argc,char** argv){
  if(argc<4)throw std::runtime_error("usage: benchmark model.json fixed-input.json reference|optimized [profile]");
  rclcpp::init(argc,argv);const auto init=read(argv[1]),input=read(argv[2]);
  const bool optimized=std::string(argv[3])=="optimized";
  if(!optimized&&std::string(argv[3])!="reference")throw std::runtime_error("INVALID_MODE");
  auto load=Clock::now();auto u=urdf::parseURDF(init.at("parameters").at("robot_description"));
  auto srdf=std::make_shared<srdf::Model>();
  if(!u||!srdf->initString(*u,init.at("parameters").at("robot_description_semantic")))throw std::runtime_error("MODEL_INVALID");
  auto model=std::make_shared<moveit::core::RobotModel>(u,srdf);
  for(auto it=init.at("expected_collision_shapes").begin();it!=init.at("expected_collision_shapes").end();++it){
    const auto* l=model->getLinkModel(it.key());
    if(!l||l->getShapes().size()!=it.value().get<size_t>())throw std::runtime_error("MODEL_GEOMETRY_MISSING");
  }
  J output={{"model_load_s",seconds(load)},{"mode",argv[3]},{"status","CHECKER_BENCHMARK_NOT_EXECUTABLE"},
    {"qualification_status","NOT_EVALUATED"},{"continuous_swept_proof",false},{"cases",J::array()}};
  for(const auto& c:input.at("cases")){
    const auto begin=Clock::now();auto req=c.at("request");
    req["native_check_reuse"]=optimized;req["native_process_profile"]=(argc>4);
    if(!req.at("process_policy").at("initial_proximity").is_null())throw std::runtime_error("ORDERED_HISTORY_NOT_SUPPORTED_BY_FRAGMENT_BENCHMARK");
    if(req.at("process_policy").at("cups")!=init.at("process_geometry").at("cups")||
       req.at("process_policy").at("flange_from_physical_contact")!=init.at("process_geometry").at("flange_from_physical_contact")||
       req.at("clearance_policy").at("source_policy")!=init.at("collision_policy")||
       req.at("clearance_policy").at("tool_links")!=init.at("tool_links")||
       req.at("clearance_policy").at("compliant_tool_links")!=init.at("compliant_tool_links"))
      throw std::runtime_error("BENCHMARK_MODEL_POLICY_BINDING_MISMATCH");
    auto scene=std::make_shared<planning_scene::PlanningScene>(model);
    auto& initial=scene->getCurrentStateNonConst();initial.setToDefaultValues();
    initial.setJointGroupPositions("manipulator",req.at("q_start").get<std::vector<double>>());initial.update();
    for(const auto& b:req.at("world"))if(!scene->processCollisionObjectMsg(object(b)))throw std::runtime_error("WORLD_IMPORT");
    for(const auto& p:req.at("allowed_pairs"))scene->getAllowedCollisionMatrixNonConst().setEntry(p.at(0).get<std::string>(),p.at(1).get<std::string>(),true);
    if(!req.at("attachment").is_null()){
      const auto& b=req.at("attachment");moveit_msgs::msg::AttachedCollisionObject a;a.link_name="flange";a.object=object(b,"flange");
      a.touch_links=b.at("touch_links").get<std::vector<std::string>>();
      if(!scene->processAttachedCollisionObjectMsg(a))throw std::runtime_error("ATTACH_IMPORT");
    }
    m710::ProcessPolicy process(scene,req);
    m710::Clearance clearance(scene,req.at("clearance_policy"),"optimized",optimized);clearance.phase="fixed_fragment";
    auto state=scene->getCurrentState();const double setup=seconds(begin);const auto traversal=Clock::now();
    const auto& path=c.at("path");const double resolution=req.at("clearance_policy").at("edge_resolution_rad");
    J verdicts=J::array();size_t visits=0;double process_s=0,clearance_s=0;
    for(size_t edge=0;edge+1<path.size();++edge){
      auto a=path.at(edge).get<std::vector<double>>(),b=path.at(edge+1).get<std::vector<double>>();
      if(a.size()!=model->getJointModelGroup("manipulator")->getVariableCount()||a.size()!=b.size())throw std::runtime_error("INVALID_PATH_JOINT_COUNT");
      double maximum=0,sum=0;for(size_t i=0;i<a.size();++i){if(!std::isfinite(a[i])||!std::isfinite(b[i]))throw std::runtime_error("INVALID_PATH_VALUES");maximum=std::max(maximum,std::abs(b[i]-a[i]));sum+=std::abs(b[i]-a[i]);}
      const size_t samples=2*std::max({size_t(1),size_t(std::ceil(maximum/resolution)),size_t(std::ceil(4.*sum/.0025))});
      for(size_t k=0;k<=samples;++k){
        const double fraction=double(k)/samples;auto q=a;for(size_t i=0;i<q.size();++i)q[i]+=fraction*(b[i]-a[i]);
        state.setJointGroupPositions("manipulator",q);state.update();++visits;
        auto t=Clock::now();bool cv=clearance.check(state);clearance_s+=seconds(t);
        t=Clock::now();bool pv=process.check(state,false);process_s+=seconds(t);
        verdicts.push_back({{"clearance",cv},{"process",pv},{"failure",cv?process.last_failure:clearance.last_failure}});
      }
    }
    const double check=seconds(traversal);
    output["cases"].push_back({{"name",c.at("name")},{"setup_s",setup},{"check_s",check},{"total_s",setup+check},
      {"process_s",process_s},{"clearance_s",clearance_s},{"visited_states",visits},{"verdicts",verdicts},
      {"process",process.evidence()},{"clearance",clearance.evidence()}});
  }
  std::cout<<output.dump()<<std::endl;rclcpp::shutdown();
}
