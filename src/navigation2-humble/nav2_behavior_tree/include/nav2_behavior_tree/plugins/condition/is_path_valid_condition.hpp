













#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_PATH_VALID_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_PATH_VALID_CONDITION_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/srv/is_path_valid.hpp"

namespace nav2_behavior_tree
{



class IsPathValidCondition : public BT::ConditionNode
{
public:
  

  IsPathValidCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsPathValidCondition() = delete;

  

  BT::NodeStatus tick() override;

  

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("path", "Path to Check"),
      BT::InputPort<std::chrono::milliseconds>("server_timeout")
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<nav2_msgs::srv::IsPathValid>::SharedPtr client_;


  std::chrono::milliseconds server_timeout_;
};

}

#endif
