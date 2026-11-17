// Actual CompositeRunner and nativePrepare, scripted native geometry replies.
// No ROS runtime, devices or executable permit. Python verifies the call replay.
#include "sc_composite.h"
#include <iostream>
using J=nlohmann::json;
int main() {
  try {
    J input;std::cin>>input;size_t index=0;
    m710::CompositeRunner runner([&](const J&){
      if(index>=input.at("replies").size())throw std::runtime_error("SCRIPTED_REPLIES_EXHAUSTED");
      return input.at("replies").at(index++);
    },[]{},nullptr,input.at("request"));
    J result=runner.run();result["script_consumed"]=index;
    std::cout<<result.dump()<<std::endl;
  } catch(const std::exception& e) {std::cerr<<e.what()<<std::endl;return 1;}
}
