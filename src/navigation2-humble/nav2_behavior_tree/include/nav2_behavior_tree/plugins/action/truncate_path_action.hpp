














#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__TRUNCATE_PATH_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__TRUNCATE_PATH_ACTION_HPP_

#include <memory>
#include <string>

#include "nav_msgs/msg/path.hpp"

#include "behaviortree_cpp_v3/action_node.h"

namespace nav2_behavior_tree
{



class TruncatePath : public BT::ActionNodeBase
{
public:
  

  TruncatePath(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("input_path", "Original Path"),
      BT::OutputPort<nav_msgs::msg::Path>("output_path", "Path truncated to a certain distance"),
      BT::InputPort<double>("distance", 1.0, "distance"),
    };
  }

private:
  

  void halt() override {}

  

  BT::NodeStatus tick() override;

  double distance_;
};

}

#endif
