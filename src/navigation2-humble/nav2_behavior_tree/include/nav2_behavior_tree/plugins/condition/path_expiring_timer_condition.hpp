













#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__PATH_EXPIRING_TIMER_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__PATH_EXPIRING_TIMER_CONDITION_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "nav_msgs/msg/path.hpp"

namespace nav2_behavior_tree
{



class PathExpiringTimerCondition : public BT::ConditionNode
{
public:
  

  PathExpiringTimerCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  PathExpiringTimerCondition() = delete;

  

  BT::NodeStatus tick() override;

  

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("seconds", 1.0, "Seconds"),
      BT::InputPort<nav_msgs::msg::Path>("path")
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Time start_;
  nav_msgs::msg::Path prev_path_;
  double period_;
  bool first_time_;
};

}

#endif
