













#include <memory>
#include <string>

#include "nav2_behavior_tree/plugins/action/navigate_through_poses_action.hpp"

namespace nav2_behavior_tree
{

NavigateThroughPosesAction::NavigateThroughPosesAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::NavigateThroughPoses>(xml_tag_name, action_name, conf)
{
}

void NavigateThroughPosesAction::on_tick()
{
  if (!getInput("goals", goal_.poses)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "NavigateThroughPosesAction: goal not provided");
    return;
  }
  getInput("behavior_tree", goal_.behavior_tree);
}

}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::NavigateThroughPosesAction>(
        name, "navigate_through_poses", config);
    };

  factory.registerBuilder<nav2_behavior_tree::NavigateThroughPosesAction>(
    "NavigateThroughPoses", builder);
}
