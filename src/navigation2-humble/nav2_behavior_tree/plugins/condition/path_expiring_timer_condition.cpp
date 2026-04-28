













#include <string>
#include <memory>

#include "behaviortree_cpp_v3/condition_node.h"

#include "nav2_behavior_tree/plugins/condition/path_expiring_timer_condition.hpp"

namespace nav2_behavior_tree
{

PathExpiringTimerCondition::PathExpiringTimerCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  period_(1.0),
  first_time_(true)
{
  getInput("seconds", period_);
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::NodeStatus PathExpiringTimerCondition::tick()
{
  if (first_time_) {
    getInput("path", prev_path_);
    first_time_ = false;
    start_ = node_->now();
    return BT::NodeStatus::FAILURE;
  }


  nav_msgs::msg::Path path;
  getInput("path", path);


  if (prev_path_ != path) {
    prev_path_ = path;
    start_ = node_->now();
  }


  auto elapsed = node_->now() - start_;


  auto seconds = elapsed.seconds();

  if (seconds < period_) {
    return BT::NodeStatus::FAILURE;
  }

  start_ = node_->now();
  return BT::NodeStatus::SUCCESS;
}

}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::PathExpiringTimerCondition>("PathExpiringTimer");
}
