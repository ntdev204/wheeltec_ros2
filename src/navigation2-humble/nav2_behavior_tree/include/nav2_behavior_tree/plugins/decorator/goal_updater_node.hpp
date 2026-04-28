














#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__GOAL_UPDATER_NODE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__GOAL_UPDATER_NODE_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "behaviortree_cpp_v3/decorator_node.h"

#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{



class GoalUpdater : public BT::DecoratorNode
{
public:
  

  GoalUpdater(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("input_goal", "Original Goal"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>(
        "output_goal",
        "Received Goal by subscription"),
    };
  }

private:
  

  BT::NodeStatus tick() override;

  

  void callback_updated_goal(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

  geometry_msgs::msg::PoseStamped last_goal_received_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
};

}

#endif
