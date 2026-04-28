


#include "dwb_critics/goal_align.hpp"
#include <vector>
#include <string>
#include "dwb_critics/alignment_util.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav_2d_utils/parameters.hpp"

namespace dwb_critics
{

void GoalAlignCritic::onInit()
{
  GoalDistCritic::onInit();
  stop_on_failure_ = false;

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  forward_point_distance_ = nav_2d_utils::searchAndGetParam(
    node,
    dwb_plugin_name_ + "." + name_ + ".forward_point_distance", 0.325);
}

bool GoalAlignCritic::prepare(
  const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D & vel,
  const geometry_msgs::msg::Pose2D & goal,
  const nav_2d_msgs::msg::Path2D & global_plan)
{





  double angle_to_goal = atan2(goal.y - pose.y, goal.x - pose.x);

  nav_2d_msgs::msg::Path2D target_poses = global_plan;
  target_poses.poses.back().x += forward_point_distance_ * cos(angle_to_goal);
  target_poses.poses.back().y += forward_point_distance_ * sin(angle_to_goal);

  return GoalDistCritic::prepare(pose, vel, goal, target_poses);
}

double GoalAlignCritic::scorePose(const geometry_msgs::msg::Pose2D & pose)
{
  return GoalDistCritic::scorePose(getForwardPose(pose, forward_point_distance_));
}

}

PLUGINLIB_EXPORT_CLASS(dwb_critics::GoalAlignCritic, dwb_core::TrajectoryCritic)
