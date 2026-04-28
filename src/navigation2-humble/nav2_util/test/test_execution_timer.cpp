













#include <chrono>
#include <thread>

#include "nav2_util/execution_timer.hpp"
#include "gtest/gtest.h"

using nav2_util::ExecutionTimer;
using std::this_thread::sleep_for;
using namespace std::chrono_literals;

TEST(ExecutionTimer, BasicDelay)
{
  ExecutionTimer t;
  t.start();
  sleep_for(10ns);
  t.end();
  ASSERT_GE(t.elapsed_time(), 10ns);
  ASSERT_GE(t.elapsed_time_in_seconds(), 1e-8);
}
