


#ifndef DWB_CORE__TRAJECTORY_UTILS_HPP_
#define DWB_CORE__TRAJECTORY_UTILS_HPP_

#include "rclcpp/rclcpp.hpp"
#include "dwb_msgs/msg/trajectory2_d.hpp"

namespace dwb_core
{



const geometry_msgs::msg::Pose2D & getClosestPose(
  const dwb_msgs::msg::Trajectory2D & trajectory,
  const double time_offset);



geometry_msgs::msg::Pose2D projectPose(
  const dwb_msgs::msg::Trajectory2D & trajectory,
  const double time_offset);

}

#endif
