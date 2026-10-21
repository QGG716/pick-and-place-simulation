// User-requested planning-only rate-limit diagnostic for one named carton.
// No solver, joint position, collision, or process predicate is replaced.
#include <pilz_industrial_motion_planner/trajectory_functions.h>
#include <dlfcn.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>

namespace pilz_industrial_motion_planner {
bool verifySampleJointLimits(const std::map<std::string,double>& previous,
    const std::map<std::string,double>& previous_velocity,
    const std::map<std::string,double>& current, double previous_dt, double dt,
    const JointLimitsContainer& limits) {
  using Original=bool (*)(const std::map<std::string,double>&,
      const std::map<std::string,double>&,const std::map<std::string,double>&,
      double,double,const JointLimitsContainer&);
  static Original original=[] {
    Dl_info info{};
    if (!dladdr(reinterpret_cast<void*>(&verifySampleJointLimits),&info) || !info.dli_sname)
      throw std::runtime_error("PILZ_SAMPLE_CHECK_SYMBOL_UNAVAILABLE");
    auto fn=reinterpret_cast<Original>(dlsym(RTLD_NEXT,info.dli_sname));
    if (!fn) throw std::runtime_error("PILZ_ORIGINAL_SAMPLE_CHECK_UNAVAILABLE");
    return fn;
  }();
  const char* scope=std::getenv("M710_PLANNING_ONLY_ALL_JOINT_RATES_EXCEPTION");
  if (!scope || std::strcmp(scope,"carton_l05_c03")!=0)
    return original(previous,previous_velocity,current,previous_dt,dt,limits);
  if (!std::isfinite(dt) || !std::isfinite(previous_dt) || dt<=1e-5 || previous_dt<0)
    return false;
  JointLimitsContainer diagnostic_limits;
  for (const auto& item:limits) {
    auto bound=item.second;
    bound.has_velocity_limits=false;
    bound.has_acceleration_limits=false;
    bound.has_deceleration_limits=false;
    if (!diagnostic_limits.addLimit(item.first,bound))
      throw std::runtime_error("PILZ_DIAGNOSTIC_LIMIT_COPY_FAILED");
  }
  for (const auto& item:current) {
    const double last=previous.at(item.first),last_v=previous_velocity.at(item.first);
    const double v=(item.second-last)/dt,a=2*(v-last_v)/(previous_dt+dt);
    if (!std::isfinite(item.second) || !std::isfinite(last) || !std::isfinite(last_v)
        || !std::isfinite(v) || !std::isfinite(a)) return false;
    const auto bound=limits.getLimit(item.first);
    const bool accelerating=std::abs(last_v)<=std::abs(v);
    const bool violation=(bound.has_velocity_limits && std::abs(v)>bound.max_velocity) ||
      (accelerating && bound.has_acceleration_limits && std::abs(a)>std::abs(bound.max_acceleration)) ||
      (!accelerating && bound.has_deceleration_limits && std::abs(a)>std::abs(bound.max_deceleration));
    if (violation) std::fprintf(stderr,
      "M710_DIAGNOSTIC_JOINT_RATE_IGNORED target=carton_l05_c03 joint=%s velocity_rad_s=%.17g acceleration_rad_s2=%.17g delta_q_rad=%.17g dt_s=%.17g velocity_limit_rad_s=%.17g acceleration_limit_rad_s2=%.17g deceleration_limit_rad_s2=%.17g physical_execution=NOT_RUN\n",
      item.first.c_str(),v,a,item.second-last,dt,bound.max_velocity,bound.max_acceleration,bound.max_deceleration);
  }
  return original(previous,previous_velocity,current,previous_dt,dt,diagnostic_limits);
}
}
