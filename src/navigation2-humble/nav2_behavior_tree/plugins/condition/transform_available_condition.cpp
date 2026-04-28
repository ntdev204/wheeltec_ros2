













#include <string>
#include <chrono>
#include <memory>

#include "nav2_behavior_tree/plugins/condition/transform_available_condition.hpp"

using namespace std::chrono_literals;

namespace nav2_behavior_tree
{

TransformAvailableCondition::TransformAvailableCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  was_found_(false)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");

  getInput("child", child_frame_);
  getInput("parent", parent_frame_);

  if (child_frame_.empty() || parent_frame_.empty()) {
    RCLCPP_FATAL(
      node_->get_logger(), "Child frame (%s) or parent frame (%s) were empty.",
      child_frame_.c_str(), parent_frame_.c_str());
    exit(-1);
  }

  RCLCPP_DEBUG(node_->get_logger(), "Initialized an TransformAvailableCondition BT node");
}

TransformAvailableCondition::~TransformAvailableCondition()
{
  RCLCPP_DEBUG(node_->get_logger(), "Shutting down TransformAvailableCondition BT node");
}

BT::NodeStatus TransformAvailableCondition::tick()
{
  if (was_found_) {
    return BT::NodeStatus::SUCCESS;
  }

  std::string tf_error;
  bool found = tf_->canTransform(
    child_frame_, parent_frame_, tf2::TimePointZero, &tf_error);

  if (found) {
    was_found_ = true;
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_INFO(
    node_->get_logger(), "Transform from %s to %s was not found, tf error: %s",
    child_frame_.c_str(), parent_frame_.c_str(), tf_error.c_str());

  return BT::NodeStatus::FAILURE;
}

}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::TransformAvailableCondition>("TransformAvailable");
}
