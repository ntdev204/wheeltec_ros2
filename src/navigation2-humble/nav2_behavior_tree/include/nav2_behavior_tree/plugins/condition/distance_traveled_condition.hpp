














#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__DISTANCE_TRAVELED_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__DISTANCE_TRAVELED_CONDITION_HPP_

#include <string>
#include <memory>

#include "behaviortree_cpp_v3/condition_node.h"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_behavior_tree
{



class DistanceTraveledCondition : public BT::ConditionNode
{
public:
  

  DistanceTraveledCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  DistanceTraveledCondition() = delete;

  

  BT::NodeStatus tick() override;

  

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("distance", 1.0, "Distance"),
      BT::InputPort<std::string>("global_frame", std::string("map"), "Global frame"),
      BT::InputPort<std::string>("robot_base_frame", std::string("base_link"), "Robot base frame")
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;

  geometry_msgs::msg::PoseStamped start_pose_;

  double distance_;
  double transform_tolerance_;
  std::string global_frame_;
  std::string robot_base_frame_;
};

}

#endif
