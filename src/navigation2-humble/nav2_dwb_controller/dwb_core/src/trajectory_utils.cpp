


#include <dwb_core/trajectory_utils.hpp>

#include <cmath>

#include "rclcpp/duration.hpp"

#include "dwb_core/exceptions.hpp"

namespace dwb_core
{
const geometry_msgs::msg::Pose2D & getClosestPose(
  const dwb_msgs::msg::Trajectory2D & trajectory,
  const double time_offset)
{
  rclcpp::Duration goal_time = rclcpp::Duration::from_seconds(time_offset);
  const unsigned int num_poses = trajectory.poses.size();
  if (num_poses == 0) {
    throw nav2_core::PlannerException("Cannot call getClosestPose on empty trajectory.");
  }
  unsigned int closest_index = num_poses;
  double closest_diff = 0.0;
  for (unsigned int i = 0; i < num_poses; ++i) {
    double diff = std::fabs((rclcpp::Duration(trajectory.time_offsets[i]) - goal_time).seconds());
    if (closest_index == num_poses || diff < closest_diff) {
      closest_index = i;
      closest_diff = diff;
    }
    if (goal_time < rclcpp::Duration(trajectory.time_offsets[i])) {
      break;
    }
  }
  return trajectory.poses[closest_index];
}

geometry_msgs::msg::Pose2D projectPose(
  const dwb_msgs::msg::Trajectory2D & trajectory,
  const double time_offset)
{
  rclcpp::Duration goal_time = rclcpp::Duration::from_seconds(time_offset);
  const unsigned int num_poses = trajectory.poses.size();
  if (num_poses == 0) {
    throw nav2_core::PlannerException("Cannot call projectPose on empty trajectory.");
  }
  if (goal_time <= (trajectory.time_offsets[0])) {
    return trajectory.poses[0];
  } else if (goal_time >= rclcpp::Duration(trajectory.time_offsets[num_poses - 1])) {
    return trajectory.poses[num_poses - 1];
  }

  for (unsigned int i = 0; i < num_poses - 1; ++i) {
    if (goal_time >= rclcpp::Duration(trajectory.time_offsets[i]) &&
      goal_time < rclcpp::Duration(trajectory.time_offsets[i + 1]))
    {
      double time_diff =
        (rclcpp::Duration(trajectory.time_offsets[i + 1]) -
        rclcpp::Duration(trajectory.time_offsets[i])).seconds();
      double ratio = (goal_time - rclcpp::Duration(trajectory.time_offsets[i])).seconds() /
        time_diff;
      double inv_ratio = 1.0 - ratio;
      const geometry_msgs::msg::Pose2D & pose_a = trajectory.poses[i];
      const geometry_msgs::msg::Pose2D & pose_b = trajectory.poses[i + 1];
      geometry_msgs::msg::Pose2D projected;
      projected.x = pose_a.x * inv_ratio + pose_b.x * ratio;
      projected.y = pose_a.y * inv_ratio + pose_b.y * ratio;
      projected.theta = pose_a.theta * inv_ratio + pose_b.theta * ratio;
      return projected;
    }
  }


  return trajectory.poses[num_poses - 1];
}


}
