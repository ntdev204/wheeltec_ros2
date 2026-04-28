













#ifndef NAV2_BEHAVIORS__PLUGINS__SPIN_HPP_
#define NAV2_BEHAVIORS__PLUGINS__SPIN_HPP_

#include <chrono>
#include <string>
#include <memory>

#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_msgs/action/spin.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

namespace nav2_behaviors
{
using SpinAction = nav2_msgs::action::Spin;



class Spin : public TimedBehavior<SpinAction>
{
public:
  

  Spin();
  ~Spin();

  

  Status onRun(const std::shared_ptr<const SpinAction::Goal> command) override;

  

  void onConfigure() override;

  

  Status onCycleUpdate() override;

protected:
  

  bool isCollisionFree(
    const double & distance,
    geometry_msgs::msg::Twist * cmd_vel,
    geometry_msgs::msg::Pose2D & pose2d);

  SpinAction::Feedback::SharedPtr feedback_;

  double min_rotational_vel_;
  double max_rotational_vel_;
  double rotational_acc_lim_;
  double cmd_yaw_;
  double prev_yaw_;
  double relative_yaw_;
  double simulate_ahead_time_;
  rclcpp::Duration command_time_allowance_{0, 0};
  rclcpp::Time end_time_;
};

}

#endif
