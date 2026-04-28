














#ifndef NAV2_UTIL__ODOMETRY_UTILS_HPP_
#define NAV2_UTIL__ODOMETRY_UTILS_HPP_

#include <cmath>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <deque>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav2_util
{



class OdomSmoother
{
public:
  

  explicit OdomSmoother(
    const rclcpp::Node::WeakPtr & parent,
    double filter_duration = 0.3,
    const std::string & odom_topic = "odom");

  

  explicit OdomSmoother(
    const nav2_util::LifecycleNode::WeakPtr & parent,
    double filter_duration = 0.3,
    const std::string & odom_topic = "odom");

  

  inline geometry_msgs::msg::Twist getTwist() {return vel_smooth_.twist;}

  

  inline geometry_msgs::msg::TwistStamped getTwistStamped() {return vel_smooth_;}

protected:
  

  void odomCallback(nav_msgs::msg::Odometry::SharedPtr msg);

  

  void updateState();

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  nav_msgs::msg::Odometry odom_cumulate_;
  geometry_msgs::msg::TwistStamped vel_smooth_;
  std::mutex odom_mutex_;

  rclcpp::Duration odom_history_duration_;
  std::deque<nav_msgs::msg::Odometry> odom_history_;
};

}

#endif
