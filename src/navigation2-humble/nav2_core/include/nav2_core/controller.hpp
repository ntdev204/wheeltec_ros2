


#ifndef NAV2_CORE__CONTROLLER_HPP_
#define NAV2_CORE__CONTROLLER_HPP_

#include <memory>
#include <string>

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/transform_listener.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_core/goal_checker.hpp"


namespace nav2_core
{



class Controller
{
public:
  using Ptr = std::shared_ptr<nav2_core::Controller>;


  

  virtual ~Controller() {}

  

  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
    std::string name, std::shared_ptr<tf2_ros::Buffer>,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) = 0;

  

  virtual void cleanup() = 0;

  

  virtual void activate() = 0;

  

  virtual void deactivate() = 0;

  

  virtual void setPlan(const nav_msgs::msg::Path & path) = 0;

  

  virtual geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) = 0;

  

  virtual void setSpeedLimit(const double & speed_limit, const bool & percentage) = 0;
};

}

#endif
