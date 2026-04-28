













#include <chrono>
#include <memory>

#include "wait.hpp"

namespace nav2_behaviors
{

Wait::Wait()
: TimedBehavior<WaitAction>(),
  feedback_(std::make_shared<WaitAction::Feedback>())
{
}

Wait::~Wait() = default;

Status Wait::onRun(const std::shared_ptr<const WaitAction::Goal> command)
{
  wait_end_ = std::chrono::steady_clock::now() +
    rclcpp::Duration(command->time).to_chrono<std::chrono::nanoseconds>();
  return Status::SUCCEEDED;
}

Status Wait::onCycleUpdate()
{
  auto current_point = std::chrono::steady_clock::now();
  auto time_left =
    std::chrono::duration_cast<std::chrono::nanoseconds>(wait_end_ - current_point).count();

  feedback_->time_left = rclcpp::Duration(rclcpp::Duration::from_nanoseconds(time_left));
  action_server_->publish_feedback(feedback_);

  if (time_left > 0) {
    return Status::RUNNING;
  } else {
    return Status::SUCCEEDED;
  }
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_behaviors::Wait, nav2_core::Behavior)
