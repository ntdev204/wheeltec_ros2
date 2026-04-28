













#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__GLOBALLY_UPDATED_GOAL_CONDITION_HPP_
#define  NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__GLOBALLY_UPDATED_GOAL_CONDITION_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"


namespace nav2_behavior_tree
{


class GloballyUpdatedGoalCondition : public BT::ConditionNode
{
public:
  

  GloballyUpdatedGoalCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  GloballyUpdatedGoalCondition() = delete;

  

  BT::NodeStatus tick() override;


  

  static BT::PortsList providedPorts()
  {
    return {};
  }

private:
  bool first_time;
  rclcpp::Node::SharedPtr node_;
  geometry_msgs::msg::PoseStamped goal_;
  std::vector<geometry_msgs::msg::PoseStamped> goals_;
};

}


#endif
