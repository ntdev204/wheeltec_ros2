













#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__DRIVE_ON_HEADING_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__DRIVE_ON_HEADING_ACTION_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/drive_on_heading.hpp"

namespace nav2_behavior_tree
{



class DriveOnHeadingAction : public BtActionNode<nav2_msgs::action::DriveOnHeading>
{
public:
  

  DriveOnHeadingAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<double>("dist_to_travel", 0.15, "Distance to travel"),
        BT::InputPort<double>("speed", 0.025, "Speed at which to travel"),
        BT::InputPort<double>("time_allowance", 10.0, "Allowed time for driving on heading")
      });
  }
};

}

#endif
