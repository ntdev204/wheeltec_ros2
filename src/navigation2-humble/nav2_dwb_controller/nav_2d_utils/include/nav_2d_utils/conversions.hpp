


#ifndef NAV_2D_UTILS__CONVERSIONS_HPP_
#define NAV_2D_UTILS__CONVERSIONS_HPP_

#include <vector>
#include <string>
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_2d_msgs/msg/twist2_d.hpp"
#include "nav_2d_msgs/msg/path2_d.hpp"
#include "nav_2d_msgs/msg/pose2_d_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/convert.h"

namespace nav_2d_utils
{
geometry_msgs::msg::Twist twist2Dto3D(const nav_2d_msgs::msg::Twist2D & cmd_vel_2d);
nav_2d_msgs::msg::Twist2D twist3Dto2D(const geometry_msgs::msg::Twist & cmd_vel);

nav_2d_msgs::msg::Pose2DStamped poseStampedToPose2D(const geometry_msgs::msg::PoseStamped & pose);
geometry_msgs::msg::Pose2D poseToPose2D(const geometry_msgs::msg::Pose & pose);
geometry_msgs::msg::Pose pose2DToPose(const geometry_msgs::msg::Pose2D & pose2d);
geometry_msgs::msg::PoseStamped pose2DToPoseStamped(
  const nav_2d_msgs::msg::Pose2DStamped & pose2d);
geometry_msgs::msg::PoseStamped pose2DToPoseStamped(
  const geometry_msgs::msg::Pose2D & pose2d,
  const std::string & frame, const rclcpp::Time & stamp);
nav_msgs::msg::Path posesToPath(const std::vector<geometry_msgs::msg::PoseStamped> & poses);
nav_2d_msgs::msg::Path2D pathToPath2D(const nav_msgs::msg::Path & path);
nav_msgs::msg::Path poses2DToPath(
  const std::vector<geometry_msgs::msg::Pose2D> & poses,
  const std::string & frame, const rclcpp::Time & stamp);
nav_msgs::msg::Path pathToPath(const nav_2d_msgs::msg::Path2D & path2d);

}

#endif
