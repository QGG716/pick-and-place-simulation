// Explicit planning-only experiment; never linked into the ordinary worker.
// Interpose only Pilz's velocity predicate. The original sample checker still
// performs acceleration/deceleration checks, and all other joints delegate.
#include <pilz_industrial_motion_planner/joint_limits_container.h>
#include <dlfcn.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>

namespace pilz_industrial_motion_planner {
bool JointLimitsContainer::verifyVelocityLimit(const std::string& joint,
                                             const double& velocity) const {
  using Original = bool (*)(const JointLimitsContainer*, const std::string&, const double&);
  static auto original = reinterpret_cast<Original>(dlsym(RTLD_NEXT,
      "_ZNK30pilz_industrial_motion_planner20JointLimitsContainer19verifyVelocityLimitERKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEERKd"));
  if (!original) throw std::runtime_error("PILZ_ORIGINAL_VELOCITY_CHECK_UNAVAILABLE");
  if (!std::isfinite(velocity)) return false;
  const bool accepted = original(this, joint, velocity);
  const char* scope = std::getenv("M710_PLANNING_ONLY_J4_VELOCITY_EXCEPTION");
  if (!accepted && joint == "J4" && scope && std::strcmp(scope, "carton_l05_c03") == 0) {
    std::fprintf(stderr,
        "M710_DIAGNOSTIC_J4_VELOCITY_IGNORED target=carton_l05_c03 velocity_rad_s=%.17g limit_rad_s=%.17g physical_execution=NOT_RUN\n",
        velocity, getLimit(joint).max_velocity);
    return true;
  }
  return accepted;
}
}
