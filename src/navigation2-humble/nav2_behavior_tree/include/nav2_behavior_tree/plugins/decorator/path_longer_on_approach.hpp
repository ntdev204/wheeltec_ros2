













#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__PATH_LONGER_ON_APPROACH_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__PATH_LONGER_ON_APPROACH_HPP_

#include <string>
#include <memory>
#include <limits>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "behaviortree_cpp_v3/decorator_node.h"
#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{



class PathLongerOnApproach : public BT::DecoratorNode
{
public:
  

  PathLongerOnApproach(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("path", "Planned Path"),
      BT::InputPort<double>(
        "prox_len", 3.0,
        "Proximity length (m) for the path to be longer on approach"),
      BT::InputPort<double>(
        "length_factor", 2.0,
        "Length multiplication factor to check if the path is significantly longer"),
    };
  }

  

  BT::NodeStatus tick() override;

private:
  

  bool isPathUpdated(
    nav_msgs::msg::Path & new_path,
    nav_msgs::msg::Path & old_path);

  

  bool isRobotInGoalProximity(
    nav_msgs::msg::Path & old_path,
    double & prox_leng);

  

  bool isNewPathLonger(
    nav_msgs::msg::Path & new_path,
    nav_msgs::msg::Path & old_path,
    double & length_factor);

private:
  nav_msgs::msg::Path new_path_;
  nav_msgs::msg::Path old_path_;
  double prox_len_ = std::numeric_limits<double>::max();
  double length_factor_ = std::numeric_limits<double>::max();
  rclcpp::Node::SharedPtr node_;
  bool first_time_ = true;
};

}

#endif
