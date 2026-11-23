// C ABI of native_prepare.h for the Python planning flow (user decision C,
// 2026-11-23: Python calls the same C++ prepare as the worker). No ROS,
// MoveIt or global state; every call owns its result handle. Loaded with
// ctypes, which releases the interpreter lock for the duration of a call.
#include "native_prepare.h"
#include <cstring>
#include <exception>
#include <string>

namespace {
struct Result {m710::PreparedTrajectory trajectory;std::string error;};
}

extern "C" {
// limits: lower[6], upper[6], velocity[6], acceleration[6]. Never throws.
void* m710_prepare_run(const double* t,const double* q,const double* v,const double* a,size_t n,const double* limits,
                       double time_scale,double period,double reach_bound,int geometric) {
  auto* r=new Result();
  try {
    m710::PrepareInput in;in.t.assign(t,t+n);in.q.assign(q,q+6*n);in.v.assign(v,v+6*n);in.a.assign(a,a+6*n);
    m710::PrepareLimits lim;lim.lower.assign(limits,limits+6);lim.upper.assign(limits+6,limits+12);
    lim.velocity.assign(limits+12,limits+18);lim.acceleration.assign(limits+18,limits+24);
    r->trajectory=m710::nativePrepare(in,lim,time_scale,period,reach_bound,geometric!=0);
  } catch(const std::exception& e) {r->error=e.what();if(r->error.empty())r->error="NATIVE_PREPARE_FAILED";}
  catch(...) {r->error="NATIVE_PREPARE_FAILED";}
  return r;
}
// Empty string when the call succeeded.
const char* m710_prepare_error(void* handle) {return static_cast<Result*>(handle)->error.c_str();}
// which: 0 send samples, 1 probe states, 2 knot intervals (GEOMETRIC), 3 speed bounds.
size_t m710_prepare_size(void* handle,int which) {
  const auto& p=static_cast<Result*>(handle)->trajectory;
  switch(which) {case 0:return p.t.size();case 1:return p.collision.size()/6;case 2:return p.per_interval.size();case 3:return p.speed_bounds.size();default:return 0;}
}
// scalars: source_duration, scaled_duration, send_duration, terminal_hold, subdivisions.
void m710_prepare_copy(void* handle,double* t,double* q,double* v,double* a,double* collision,double* speed_bounds,
                       long* per_interval,double* scalars) {
  const auto& p=static_cast<Result*>(handle)->trajectory;
  std::memcpy(t,p.t.data(),p.t.size()*sizeof(double));std::memcpy(q,p.q.data(),p.q.size()*sizeof(double));
  std::memcpy(v,p.v.data(),p.v.size()*sizeof(double));std::memcpy(a,p.a.data(),p.a.size()*sizeof(double));
  std::memcpy(collision,p.collision.data(),p.collision.size()*sizeof(double));
  std::memcpy(speed_bounds,p.speed_bounds.data(),p.speed_bounds.size()*sizeof(double));
  if(!p.per_interval.empty())std::memcpy(per_interval,p.per_interval.data(),p.per_interval.size()*sizeof(long));
  scalars[0]=p.source_duration;scalars[1]=p.scaled_duration;scalars[2]=p.send_duration;scalars[3]=p.terminal_hold;scalars[4]=double(p.subdivisions);
}
void m710_prepare_free(void* handle) {delete static_cast<Result*>(handle);}
// Interface version: callers refuse any other.
int m710_prepare_abi_version() {return 1;}
}
