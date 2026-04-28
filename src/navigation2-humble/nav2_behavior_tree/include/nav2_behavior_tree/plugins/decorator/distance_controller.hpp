














#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__DISTANCE_CONTROLLER_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__DISTANCE_CONTROLLER_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"

#include "behaviortree_cpp_v3/decorator_node.h"

namespace nav2_behavior_tree
{



class DistanceController : public BT::DecoratorNode
{
public:
  

  DistanceController(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("distance", 1.0, "Distance"),
      BT::InputPort<std::string>("global_frame", std::string("map"), "Global frame"),
      BT::InputPort<std::string>("robot_base_frame", std::string("base_link"), "Robot base frame")
    };
  }

private:
  

  BT::NodeStatus tick() override;

  rclcpp::Node::SharedPtr node_;

  std::shared_ptr<tf2_ros::Buffer> tf_;
  double transform_tolerance_;

  geometry_msgs::msg::PoseStamped start_pose_;
  double distance_;

  std::string global_frame_;
  std::string robot_base_frame_;

  bool first_time_;
};

}

#endif
