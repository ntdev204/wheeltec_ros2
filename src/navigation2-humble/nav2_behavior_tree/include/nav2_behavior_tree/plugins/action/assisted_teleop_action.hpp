














#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__ASSISTED_TELEOP_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__ASSISTED_TELEOP_ACTION_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/assisted_teleop.hpp"

namespace nav2_behavior_tree
{



class AssistedTeleopAction : public BtActionNode<nav2_msgs::action::AssistedTeleop>
{
public:
  

  AssistedTeleopAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  

  void on_tick() override;

  BT::NodeStatus on_aborted() override;

  

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<double>("time_allowance", 10.0, "Allowed time for running assisted teleop"),
        BT::InputPort<bool>("is_recovery", false, "If true the recovery count will be incremented")
      });
  }

private:
  bool is_recovery_;
};

}

#endif
