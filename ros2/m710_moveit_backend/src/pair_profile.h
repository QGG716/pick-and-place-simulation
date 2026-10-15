#pragma once
#include <chrono>

namespace m710 {
// Only performance observation. Never use this optional clock for deadlines.
// Clock is injectable in its small unit test; production has no call counter.
template<class Clock=std::chrono::steady_clock>
class PairProfileTimer {
  bool enabled_;
  typename Clock::time_point start_{};
public:
  explicit PairProfileTimer(bool enabled):enabled_(enabled) {
    if(enabled_)start_=Clock::now();
  }
  double elapsed() const {
    if(!enabled_)return 0.;
    return std::chrono::duration<double>(Clock::now()-start_).count();
  }
};
}
