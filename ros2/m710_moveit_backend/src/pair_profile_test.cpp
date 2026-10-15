#include "pair_profile.h"
#include <iostream>
#include <stdexcept>
struct CountingClock {
  using time_point=std::chrono::steady_clock::time_point;
  inline static size_t calls=0;
  static time_point now() {
    ++calls;return time_point(std::chrono::seconds(calls));
  }
};
int main() {
  m710::PairProfileTimer<CountingClock> off(false);
  if(off.elapsed()!=0.||CountingClock::calls!=0)throw std::runtime_error("DISABLED_CLOCK_READ");
  m710::PairProfileTimer<CountingClock> on(true);
  if(CountingClock::calls!=1||on.elapsed()!=1.||CountingClock::calls!=2)
    throw std::runtime_error("ENABLED_CLOCK_READS");
  std::cout<<"{\"status\":\"PASS\",\"disabled_clock_reads\":0,\"enabled_clock_reads\":2}\n";
}
