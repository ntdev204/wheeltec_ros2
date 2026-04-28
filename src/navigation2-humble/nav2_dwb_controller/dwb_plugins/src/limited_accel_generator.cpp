


#include "dwb_plugins/limited_accel_generator.hpp"
#include <vector>
#include <memory>
#include <string>
#include "nav_2d_utils/parameters.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "dwb_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"

namespace dwb_plugins
{

void LimitedAccelGenerator::initialize(
  const nav2_util::LifecycleNode::SharedPtr & nh,
  const std::string & plugin_name)
{
  plugin_name_ = plugin_name;
  StandardTrajectoryGenerator::initialize(nh, plugin_name_);

  try {
    nav2_util::declare_parameter_if_not_declared(
      nh, plugin_name + ".sim_period", rclcpp::PARAMETER_DOUBLE);
    if (!nh->get_parameter(plugin_name + ".sim_period", acceleration_time_)) {




      throw std::runtime_error("Failed to get 'sim_period' value");
    }
  } catch (std::exception &) {
    RCLCPP_WARN(
      rclcpp::get_logger("LimitedAccelGenerator"),
      "'sim_period' parameter is not set for %s", plugin_name.c_str());
    double controller_frequency = nav_2d_utils::searchAndGetParam(
      nh, "controller_frequency", 20.0);
    if (controller_frequency > 0) {
      acceleration_time_ = 1.0 / controller_frequency;
    } else {
      RCLCPP_WARN(
        rclcpp::get_logger("LimitedAccelGenerator"),
        "A controller_frequency less than or equal to 0 has been set. "
        "Ignoring the parameter, assuming a rate of 20Hz");
      acceleration_time_ = 0.05;
    }
  }
}

void LimitedAccelGenerator::startNewIteration(const nav_2d_msgs::msg::Twist2D & current_velocity)
{

  velocity_iterator_->startNewIteration(current_velocity, acceleration_time_);
}

nav_2d_msgs::msg::Twist2D LimitedAccelGenerator::computeNewVelocity(
  const nav_2d_msgs::msg::Twist2D & cmd_vel,
  const nav_2d_msgs::msg::Twist2D & ,
  const double )
{
  return cmd_vel;
}

}

PLUGINLIB_EXPORT_CLASS(dwb_plugins::LimitedAccelGenerator, dwb_core::TrajectoryGenerator)
