













#ifndef NAV2_UTIL__EXECUTION_TIMER_HPP_
#define NAV2_UTIL__EXECUTION_TIMER_HPP_

#include <chrono>

namespace nav2_util
{


class ExecutionTimer
{
public:
  using Clock = std::chrono::high_resolution_clock;
  using nanoseconds = std::chrono::nanoseconds;


  void start() {start_ = Clock::now();}


  void end() {end_ = Clock::now();}


  nanoseconds elapsed_time() {return end_ - start_;}


  double elapsed_time_in_seconds()
  {
    return std::chrono::duration<double>(end_ - start_).count();
  }

protected:
  Clock::time_point start_;
  Clock::time_point end_;
};

}

#endif
