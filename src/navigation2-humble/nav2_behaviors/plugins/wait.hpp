













#ifndef NAV2_BEHAVIORS__PLUGINS__WAIT_HPP_
#define NAV2_BEHAVIORS__PLUGINS__WAIT_HPP_

#include <chrono>
#include <string>
#include <memory>

#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_msgs/action/wait.hpp"

namespace nav2_behaviors
{
using WaitAction = nav2_msgs::action::Wait;



class Wait : public TimedBehavior<WaitAction>
{
public:
  

  Wait();
  ~Wait();

  

  Status onRun(const std::shared_ptr<const WaitAction::Goal> command) override;

  

  Status onCycleUpdate() override;

protected:
  std::chrono::time_point<std::chrono::steady_clock> wait_end_;
  WaitAction::Feedback::SharedPtr feedback_;
};

}

#endif
