// JSONL probe for synthetic Python/C++ parity. Not a real tool asset test.
#include "process_geometry.h"
#include "native_robot.h"
#include <iostream>
int main() {
  std::string line;
  while(std::getline(std::cin,line)) {
    try {
      auto j=nlohmann::json::parse(line);auto p=j.at("policy");
      m710::validateSealContours(p);
      auto result=m710::processSealRings(p,m710::processBox(j.at("target")),m710::processTransform(j.at("flange")));
      std::cout<<result.dump()<<std::endl;
    } catch(const std::exception& e) {std::cout<<nlohmann::json({{"status","REJECTED"},{"error",e.what()}}).dump()<<std::endl;}
  }
}
