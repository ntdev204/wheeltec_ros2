













#ifndef NAV2_BEHAVIORS__PLUGINS__ASSISTED_TELEOP_HPP_
#define NAV2_BEHAVIORS__PLUGINS__ASSISTED_TELEOP_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/empty.hpp"
#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_msgs/action/assisted_teleop.hpp"

namespace nav2_behaviors
{
using AssistedTeleopAction = nav2_msgs::action::AssistedTeleop;



class AssistedTeleop : public TimedBehavior<AssistedTeleopAction>
{
public:
  AssistedTeleop();

  

  Status onRun(const std::shared_ptr<const AssistedTeleopAction::Goal> command) override;

  

  void onActionCompletion() override;

  

  Status onCycleUpdate() override;

protected:
  

  void onConfigure() override;

  

  geometry_msgs::msg::Pose2D projectPose(
    const geometry_msgs::msg::Pose2D & pose,
    const geometry_msgs::msg::Twist & twist,
    double projection_time);

  

  void teleopVelocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  

  void preemptTeleopCallback(const std_msgs::msg::Empty::SharedPtr msg);

  AssistedTeleopAction::Feedback::SharedPtr feedback_;


  double projection_time_;
  double simulation_time_step_;

  geometry_msgs::msg::Twist teleop_twist_;
  bool preempt_teleop_{false};


  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr preempt_teleop_sub_;

  rclcpp::Duration command_time_allowance_{0, 0};
  rclcpp::Time end_time_;
};
}

#endif
