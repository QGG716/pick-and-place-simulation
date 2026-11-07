#pragma once
#include "process_geometry.h"
namespace m710 {
inline bool gapSearchLayout(const ProcessJson& layout) {
  return layout.is_object() && layout.value("schema","")=="fixed40_gap_search_20261107_v1";
}
inline void validateGapLayout(const ProcessJson& layout) {
  const double g=layout.at("gap_m");
  if(!gapSearchLayout(layout) || !std::isfinite(g) || g<.005 ||
      layout.at("base_xyz_a_m")!=ProcessJson::array({.6,-.2,1.}) ||
      layout.at("mount_rpy_rad")!=ProcessJson::array({0.,0.,0.}) ||
      layout.at("minimum_surface_height_m")!=.55 || layout.at("maximum_surface_height_m")!=1.5 ||
      layout.at("mount_design")!="SOLID_PLINTH")throw std::runtime_error("GAP_FIXED_LAYOUT_CONTRACT");
  const auto base=layout.at("base_xyz_world_m").get<std::vector<double>>();
  const auto assembly=layout.at("assembly_world_xyz_m").get<std::vector<double>>();
  if(base.size()!=3 || assembly.size()!=3 || std::abs(base[0]+1.15+g)>1e-10 || base[1]!=.15 || base[2]!=1. ||
      std::abs(assembly[0]+1.75+g)>1e-10 || assembly[1]!=.35 || assembly[2]!=0.)
    throw std::runtime_error("GAP_FIXED_WORLD_TRANSFORM");
}
inline void validateGapReceive(const ProcessJson& policy,const ProcessJson& initial,const ProcessJson& layout,double expected) {
  validateGapLayout(layout);
  const auto target=processBox(initial);const double bottom=target.pose.translation().z()-target.pose.linear().row(2).cwiseAbs().dot(target.half);
  const auto face=policy.at("face").get<std::string>();const auto mode=policy.at("mode").get<std::string>();
  const bool upper=bottom>=1.5-1e-9;
  const double release=upper?bottom:(face=="z+"?std::max(bottom+.05,expected+.05):std::max(bottom,expected+.05));
  if(policy.at("schema")!="fixed40_gap_search_20261107_v1" || policy.at("target_id")!=target.id ||
      policy.at("minimum_m")!=.55 || policy.at("maximum_m")!=1.5 || policy.at("d_m")!=.05 ||
      expected<.55-1e-9 || expected>1.5+1e-9 || !std::isfinite(expected) ||
      std::abs(policy.at("original_bottom_m").get<double>()-bottom)>1e-9 ||
      std::abs(policy.at("height_m").get<double>()-expected)>1e-9 ||
      std::abs(policy.at("release_bottom_m").get<double>()-release)>1e-9 || release-expected<.05-1e-9 ||
      (upper && (face!="x-" || mode!="FRONT_DIRECT_HIGH_DROP")) ||
      (!upper && mode!=(face=="z+"?"TOP_LIFT_TRANSFER":"EXTRACT_RAISE_TRANSFER")))
    throw std::runtime_error("GAP_RECEIVE_MODE_HEIGHT_OR_ORIGINAL_LAYER");
}
}
