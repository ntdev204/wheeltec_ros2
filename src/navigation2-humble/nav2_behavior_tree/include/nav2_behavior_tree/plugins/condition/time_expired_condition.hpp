














#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__TIME_EXPIRED_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__TIME_EXPIRED_CONDITION_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

namespace nav2_behavior_tree
{



class TimeExpiredCondition : public BT::ConditionNode
{
public:
  

  TimeExpiredCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  TimeExpiredCondition() = delete;

  

  BT::NodeStatus tick() override;

  

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("seconds", 1.0, "Seconds")
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Time start_;
  double period_;
};

}

#endif
