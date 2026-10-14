// JSONL probe for supplied contour/scalar-policy parity; not a ROS worker.
#include "process_geometry.h"
#include "native_robot.h"
#include "eco65_distance_policy.h"
#include <iostream>
int main() {
  std::string line;
  while(std::getline(std::cin,line)) {
    try {
      auto j=nlohmann::json::parse(line);
      if(j.value("op",std::string())=="distance_policy") {
        std::cout<<nlohmann::json({{"accepted",m710::eco65DistanceAccepted(j.at("distance"),j.at("threshold"))}}).dump()<<std::endl;
        continue;
      }
      auto p=j.at("policy");
      m710::validateSealContours(p);
      auto result=m710::processSealRings(p,m710::processBox(j.at("target")),m710::processTransform(j.at("flange")));
      std::cout<<result.dump()<<std::endl;
    } catch(const std::exception& e) {std::cout<<nlohmann::json({{"status","REJECTED"},{"error",e.what()}}).dump()<<std::endl;}
  }
}
