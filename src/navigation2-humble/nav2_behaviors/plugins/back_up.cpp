













#include "back_up.hpp"

namespace nav2_behaviors
{

Status BackUp::onRun(const std::shared_ptr<const BackUpAction::Goal> command)
{
  if (command->target.y != 0.0 || command->target.z != 0.0) {
    RCLCPP_INFO(
      logger_,
      "Backing up in Y and Z not supported, will only move in X.");
    return Status::FAILED;
  }


  command_x_ = -std::fabs(command->target.x);
  command_speed_ = -std::fabs(command->speed);
  command_time_allowance_ = command->time_allowance;

  end_time_ = steady_clock_.now() + command_time_allowance_;

  if (!nav2_util::getCurrentPose(
      initial_pose_, *tf_, global_frame_, robot_base_frame_,
      transform_tolerance_))
  {
    RCLCPP_ERROR(logger_, "Initial robot pose is not available.");
    return Status::FAILED;
  }

  return Status::SUCCEEDED;
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_behaviors::BackUp, nav2_core::Behavior)
