














#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__TRUNCATE_PATH_LOCAL_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__TRUNCATE_PATH_LOCAL_ACTION_HPP_

#include <memory>
#include <string>
#include <limits>

#include "nav_msgs/msg/path.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "tf2_ros/buffer.h"

namespace nav2_behavior_tree
{



class TruncatePathLocal : public BT::ActionNodeBase
{
public:
  

  TruncatePathLocal(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("input_path", "Original Path"),
      BT::OutputPort<nav_msgs::msg::Path>(
        "output_path", "Path truncated to a certain distance around robot"),
      BT::InputPort<double>(
        "distance_forward", 8.0,
        "Distance in forward direction"),
      BT::InputPort<double>(
        "distance_backward", 4.0,
        "Distance in backward direction"),
      BT::InputPort<std::string>(
        "robot_frame", "base_link",
        "Robot base frame id"),
      BT::InputPort<double>(
        "transform_tolerance", 0.2,
        "Transform lookup tolerance"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>(
        "pose", "Manually specified pose to be used"
        "if overriding current robot pose"),
      BT::InputPort<double>(
        "angular_distance_weight", 0.0,
        "Weight of angular distance relative to positional distance when finding which path "
        "pose is closest to robot. Not applicable on paths without orientations assigned"),
      BT::InputPort<double>(
        "max_robot_pose_search_dist", std::numeric_limits<double>::infinity(),
        "Maximum forward integrated distance along the path (starting from the last detected pose) "
        "to bound the search for the closest pose to the robot. When set to infinity (default), "
        "whole path is searched every time"),
    };
  }

private:
  

  void halt() override {}

  

  BT::NodeStatus tick() override;

  

  bool getRobotPose(std::string path_frame_id, geometry_msgs::msg::PoseStamped & pose);

  

  static double poseDistance(
    const geometry_msgs::msg::PoseStamped & pose1,
    const geometry_msgs::msg::PoseStamped & pose2,
    const double angular_distance_weight);

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  nav_msgs::msg::Path path_;
  nav_msgs::msg::Path::_poses_type::iterator closest_pose_detection_begin_;
};

}

#endif
