


#ifndef NAV_2D_UTILS__ODOM_SUBSCRIBER_HPP_
#define NAV_2D_UTILS__ODOM_SUBSCRIBER_HPP_

#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include "nav_2d_msgs/msg/twist2_d_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav_2d_utils
{



class OdomSubscriber
{
public:
  

  explicit OdomSubscriber(
    nav2_util::LifecycleNode::SharedPtr nh,
    std::string default_topic = "odom")
  {
    nav2_util::declare_parameter_if_not_declared(
      nh, "odom_topic", rclcpp::ParameterValue(default_topic));

    std::string odom_topic;
    nh->get_parameter_or("odom_topic", odom_topic, default_topic);
    odom_sub_ =
      nh->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic,
      rclcpp::SystemDefaultsQoS(),
      std::bind(&OdomSubscriber::odomCallback, this, std::placeholders::_1));
  }

  inline nav_2d_msgs::msg::Twist2D getTwist() {return odom_vel_.velocity;}
  inline nav_2d_msgs::msg::Twist2DStamped getTwistStamped() {return odom_vel_;}

protected:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {

    std::lock_guard<std::mutex> lock(odom_mutex_);
    odom_vel_.header = msg->header;
    odom_vel_.velocity.x = msg->twist.twist.linear.x;
    odom_vel_.velocity.y = msg->twist.twist.linear.y;
    odom_vel_.velocity.theta = msg->twist.twist.angular.z;
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  nav_2d_msgs::msg::Twist2DStamped odom_vel_;
  std::mutex odom_mutex_;
};

}

#endif
