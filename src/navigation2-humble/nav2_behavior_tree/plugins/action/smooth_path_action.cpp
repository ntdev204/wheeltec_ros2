














#include <memory>
#include <string>

#include "nav2_behavior_tree/plugins/action/smooth_path_action.hpp"

namespace nav2_behavior_tree
{

SmoothPathAction::SmoothPathAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::SmoothPath>(xml_tag_name, action_name, conf)
{
}

void SmoothPathAction::on_tick()
{
  getInput("unsmoothed_path", goal_.path);
  getInput("smoother_id", goal_.smoother_id);
  double max_smoothing_duration;
  getInput("max_smoothing_duration", max_smoothing_duration);
  goal_.max_smoothing_duration = rclcpp::Duration::from_seconds(max_smoothing_duration);
  getInput("check_for_collisions", goal_.check_for_collisions);
}

BT::NodeStatus SmoothPathAction::on_success()
{
  setOutput("smoothed_path", result_.result->path);
  setOutput("smoothing_duration", rclcpp::Duration(result_.result->smoothing_duration).seconds());
  setOutput("was_completed", result_.result->was_completed);
  return BT::NodeStatus::SUCCESS;
}

}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::SmoothPathAction>(
        name, "smooth_path", config);
    };

  factory.registerBuilder<nav2_behavior_tree::SmoothPathAction>(
    "SmoothPath", builder);
}
