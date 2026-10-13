#pragma once
#include <nlohmann/json.hpp>
#include <string>
#include <stdexcept>

namespace m710 {
// Active bound RobotModel during synchronous JSONL dispatch. Worker restores
// its immutable identity before each call; frames never come from motion input.
struct NativeFrames {
  std::string planning_group="manipulator", base_link="base_link", flange_link="flange";
};
inline NativeFrames& nativeFrames() { static NativeFrames frames; return frames; }
inline void bindNativeFrames(const nlohmann::json& identity) {
  auto& f=nativeFrames();
  f.planning_group=identity.value("planning_group",std::string("manipulator"));
  f.base_link=identity.value("base_link",std::string("base_link"));
  f.flange_link=identity.value("flange_link",std::string("flange"));
  if(f.planning_group.empty() || f.base_link.empty() || f.flange_link.empty())
    throw std::runtime_error("NATIVE_ROBOT_FRAME_CONFIG_INVALID");
}
}
