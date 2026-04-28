













#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__DRIVE_ON_HEADING_CANCEL_NODE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__DRIVE_ON_HEADING_CANCEL_NODE_HPP_

#include <memory>
#include <string>

#include "nav2_msgs/action/drive_on_heading.hpp"

#include "nav2_behavior_tree/bt_cancel_action_node.hpp"

namespace nav2_behavior_tree
{



class DriveOnHeadingCancel : public BtCancelActionNode<nav2_msgs::action::DriveOnHeading>
{
public:
  

  DriveOnHeadingCancel(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
      });
  }
};

}

#endif
