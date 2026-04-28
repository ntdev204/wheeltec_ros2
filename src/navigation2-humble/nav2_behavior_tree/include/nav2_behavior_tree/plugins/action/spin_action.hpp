













#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__SPIN_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__SPIN_ACTION_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/spin.hpp"

namespace nav2_behavior_tree
{



class SpinAction : public BtActionNode<nav2_msgs::action::Spin>
{
public:
  

  SpinAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  

  void on_tick() override;

  

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<double>("spin_dist", 1.57, "Spin distance"),
        BT::InputPort<double>("time_allowance", 10.0, "Allowed time for spinning"),
        BT::InputPort<bool>("is_recovery", true, "True if recovery")
      });
  }

private:
  bool is_recovery_;
};

}

#endif
