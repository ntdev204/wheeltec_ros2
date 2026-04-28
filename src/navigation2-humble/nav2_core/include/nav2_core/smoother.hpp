













#ifndef NAV2_CORE__SMOOTHER_HPP_
#define NAV2_CORE__SMOOTHER_HPP_

#include <memory>
#include <string>

#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_costmap_2d/footprint_subscriber.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "nav_msgs/msg/path.hpp"


namespace nav2_core
{



class Smoother
{
public:
  using Ptr = std::shared_ptr<nav2_core::Smoother>;

  

  virtual ~Smoother() {}

  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
    std::string name, std::shared_ptr<tf2_ros::Buffer>,
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber>,
    std::shared_ptr<nav2_costmap_2d::FootprintSubscriber>) = 0;

  

  virtual void cleanup() = 0;

  

  virtual void activate() = 0;

  

  virtual void deactivate() = 0;

  

  virtual bool smooth(
    nav_msgs::msg::Path & path,
    const rclcpp::Duration & max_time) = 0;
};

}

#endif
