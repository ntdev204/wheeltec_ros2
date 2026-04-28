













#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__GOAL_UPDATED_CONTROLLER_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__GOAL_UPDATED_CONTROLLER_HPP_

#include <chrono>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/decorator_node.h"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_behavior_tree
{



class GoalUpdatedController : public BT::DecoratorNode
{
public:
  

  GoalUpdatedController(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  

  static BT::PortsList providedPorts()
  {
    return {};
  }

private:
  

  BT::NodeStatus tick() override;

  bool goal_was_updated_;
  geometry_msgs::msg::PoseStamped goal_;
  std::vector<geometry_msgs::msg::PoseStamped> goals_;
};

}

#endif
