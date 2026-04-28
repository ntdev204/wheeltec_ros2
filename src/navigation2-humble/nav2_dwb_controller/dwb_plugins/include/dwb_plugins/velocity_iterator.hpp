


#ifndef DWB_PLUGINS__VELOCITY_ITERATOR_HPP_
#define DWB_PLUGINS__VELOCITY_ITERATOR_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_2d_msgs/msg/twist2_d.hpp"
#include "dwb_plugins/kinematic_parameters.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace dwb_plugins
{
class VelocityIterator
{
public:
  virtual ~VelocityIterator() {}
  virtual void initialize(
    const nav2_util::LifecycleNode::SharedPtr & nh,
    KinematicsHandler::Ptr kinematics,
    const std::string & plugin_name) = 0;
  virtual void startNewIteration(const nav_2d_msgs::msg::Twist2D & current_velocity, double dt) = 0;
  virtual bool hasMoreTwists() = 0;
  virtual nav_2d_msgs::msg::Twist2D nextTwist() = 0;
};
}

#endif
