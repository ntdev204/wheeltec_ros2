













#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__TRANSFORM_AVAILABLE_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__TRANSFORM_AVAILABLE_CONDITION_HPP_

#include <string>
#include <atomic>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "tf2_ros/buffer.h"

namespace nav2_behavior_tree
{



class TransformAvailableCondition : public BT::ConditionNode
{
public:
  

  TransformAvailableCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  TransformAvailableCondition() = delete;

  

  ~TransformAvailableCondition();

  

  BT::NodeStatus tick() override;

  

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("child", std::string(), "Child frame for transform"),
      BT::InputPort<std::string>("parent", std::string(), "parent frame for transform")
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;

  std::atomic<bool> was_found_;

  std::string child_frame_;
  std::string parent_frame_;
};

}

#endif
