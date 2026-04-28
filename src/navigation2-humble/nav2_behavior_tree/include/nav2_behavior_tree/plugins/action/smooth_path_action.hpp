














#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__SMOOTH_PATH_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__SMOOTH_PATH_ACTION_HPP_

#include <string>

#include "nav2_msgs/action/smooth_path.hpp"
#include "nav_msgs/msg/path.h"
#include "nav2_behavior_tree/bt_action_node.hpp"

namespace nav2_behavior_tree
{



class SmoothPathAction : public nav2_behavior_tree::BtActionNode<nav2_msgs::action::SmoothPath>
{
public:
  

  SmoothPathAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  

  void on_tick() override;

  

  BT::NodeStatus on_success() override;

  

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::OutputPort<nav_msgs::msg::Path>(
          "smoothed_path",
          "Path smoothed by SmootherServer node"),
        BT::OutputPort<double>("smoothing_duration", "Time taken to smooth path"),
        BT::OutputPort<bool>(
          "was_completed", "True if smoothing was not interrupted by time limit"),
        BT::InputPort<nav_msgs::msg::Path>("unsmoothed_path", "Path to be smoothed"),
        BT::InputPort<double>("max_smoothing_duration", 3.0, "Maximum smoothing duration"),
        BT::InputPort<bool>(
          "check_for_collisions", false,
          "If true collision check will be performed after smoothing"),
        BT::InputPort<std::string>("smoother_id", ""),
      });
  }
};

}

#endif
