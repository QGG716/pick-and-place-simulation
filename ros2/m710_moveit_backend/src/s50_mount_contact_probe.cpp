// Real model diagnostic, no IK, planning, execution or collision suppression.
#include "mount_contact.h"
#include <moveit/planning_scene/planning_scene.h>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <iostream>
using J=nlohmann::json;
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  try {
    if(argc!=2)throw std::runtime_error("Expected input JSON path");std::ifstream f(argv[1]);J input;f>>input;
    J output={{"schema","s50_native_mount_contact_diagnostic_v1"},{"task_planning_performed",false},{"cases",J::array()}};
    for(const auto& row:input.at("cases")) {
      auto u=urdf::parseURDF(row.at("urdf").get<std::string>());auto srdf=std::make_shared<srdf::Model>();
      if(!u || !srdf->initString(*u,row.at("srdf").get<std::string>()))throw std::runtime_error("MODEL_PARSE_FAILED");
      auto model=std::make_shared<moveit::core::RobotModel>(u,srdf);auto scene=std::make_shared<planning_scene::PlanningScene>(model);
      auto& state=scene->getCurrentStateNonConst();state.setToDefaultValues();
      state.setJointGroupPositions("manipulator",row.at("q_rad").get<std::vector<double>>());state.update();
      const auto& request=row.at("request");const auto box=m710::processBox(request.at("world").at(0));
      moveit_msgs::msg::CollisionObject object;object.id=box.id;object.header.frame_id=model->getModelFrame();object.operation=object.ADD;
      shape_msgs::msg::SolidPrimitive shape;shape.type=shape.BOX;shape.dimensions={2*box.half.x(),2*box.half.y(),2*box.half.z()};object.primitives.push_back(shape);
      geometry_msgs::msg::Pose pose;pose.position.x=box.pose.translation().x();pose.position.y=box.pose.translation().y();pose.position.z=box.pose.translation().z();
      const Eigen::Quaterniond rotation(box.pose.linear());pose.orientation.w=rotation.w();pose.orientation.x=rotation.x();pose.orientation.y=rotation.y();pose.orientation.z=rotation.z();object.primitive_poses.push_back(pose);
      if(!scene->processCollisionObjectMsg(object))throw std::runtime_error("WORLD_BOX_FAILED");
      const auto evidence=m710::fixedBaseContactEvidence(state,request);
      J pairs=J::array();const std::string base=request.at("identity").at("fixed_base_contact_contract").at("base_link");
      for(const auto& name:model->getLinkModelNamesWithCollisionGeometry()) {
        // Isolate a pair only for measurement; this ACM is never installed in
        // the planning scene or returned as acceptance policy.
        collision_detection::AllowedCollisionMatrix acm(model->getLinkModelNames(),true);acm.setEntry(box.id,true);acm.setEntry(name,box.id,false);
        collision_detection::CollisionRequest query;query.contacts=true;query.max_contacts=20;query.max_contacts_per_pair=8;
        collision_detection::CollisionResult measured;scene->checkCollision(query,measured,state,acm);
        pairs.push_back({{"pair",{name,box.id}},{"raw_intersects",measured.collision},{"raw_contact_count",measured.contact_count},
          {"permission_applies",m710::qualifiedConfiguredPlinthPair(state,request,name,box.id)}});
      }
      output["cases"].push_back({{"case_id",row.at("case_id")},{"q_rad",row.at("q_rad")},{"mount_contact",evidence},
        {"synthetic_fixture",row.value("synthetic_fixture",J(nullptr))},{"raw_link_support_pairs",pairs}});
    }
    std::cout<<output.dump(2)<<std::endl;rclcpp::shutdown();return 0;
  } catch(const std::exception& error) {std::cerr<<error.what()<<std::endl;rclcpp::shutdown();return 2;}
}
