#pragma once
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>

namespace m710 {
// Shared by a request's immutable collision contexts and the active solve.
// Atomic ticks also cover OMPL's goal-sampling validity callback thread.
class NativeDeadline {
public:
  using Clock=std::chrono::steady_clock;
  explicit NativeDeadline(Clock::time_point end):end_ns_(ticks(end)) {}
  void set(Clock::time_point end) {end_ns_.store(ticks(end),std::memory_order_relaxed);}
  bool expired() const {return ticks(Clock::now())>=end_ns_.load(std::memory_order_relaxed);}
  void require() const {
    if(expired())throw std::runtime_error("NATIVE_ABSOLUTE_DEADLINE_EXPIRED");
  }
private:
  static int64_t ticks(Clock::time_point t) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count();
  }
  std::atomic<int64_t> end_ns_;
};
using NativeDeadlinePtr=std::shared_ptr<NativeDeadline>;
inline bool nativeExpired(const NativeDeadlinePtr& deadline) {return deadline&&deadline->expired();}
}
