#pragma once
#include <moveit/collision_detection/world.h>
#include <cstddef>

namespace m710 {
// MoveIt stores shape_poses_ relative to the object's own pose. Ownership
// transitions and scene identity must compare the actual world geometry.
inline const Eigen::Isometry3d& worldShapePose(const collision_detection::World::Object& object,
                                             std::size_t index) {
  return object.global_shape_poses_.at(index);
}
}
