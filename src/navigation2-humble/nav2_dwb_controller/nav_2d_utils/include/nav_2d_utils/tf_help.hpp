


#ifndef NAV_2D_UTILS__TF_HELP_HPP_
#define NAV_2D_UTILS__TF_HELP_HPP_

#include <string>
#include <memory>
#include "tf2_ros/buffer.h"
#include "nav_2d_utils/conversions.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_2d_msgs/msg/pose2_d_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav_2d_utils
{


bool transformPose(
  const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::string frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose,
  rclcpp::Duration & transform_tolerance
);



bool transformPose(
  const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::string frame,
  const nav_2d_msgs::msg::Pose2DStamped & in_pose,
  nav_2d_msgs::msg::Pose2DStamped & out_pose,
  rclcpp::Duration & transform_tolerance
);

}

#endif
