


#include "dwb_critics/path_align.hpp"
#include <vector>
#include <string>
#include "dwb_critics/alignment_util.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav_2d_utils/parameters.hpp"

namespace dwb_critics
{

void PathAlignCritic::onInit()
{
  PathDistCritic::onInit();
  stop_on_failure_ = false;

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  forward_point_distance_ = nav_2d_utils::searchAndGetParam(
    node,
    dwb_plugin_name_ + "." + name_ + ".forward_point_distance", 0.325);
}

bool PathAlignCritic::prepare(
  const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D & vel,
  const geometry_msgs::msg::Pose2D & goal,
  const nav_2d_msgs::msg::Path2D & global_plan)
{
  double dx = pose.x - goal.x;
  double dy = pose.y - goal.y;
  double sq_dist = dx * dx + dy * dy;
  if (sq_dist > forward_point_distance_ * forward_point_distance_) {
    zero_scale_ = false;
  } else {

    zero_scale_ = true;
    return true;
  }

  return PathDistCritic::prepare(pose, vel, goal, global_plan);
}

double PathAlignCritic::getScale() const
{
  if (zero_scale_) {
    return 0.0;
  } else {
    return costmap_->getResolution() * 0.5 * scale_;
  }
}

double PathAlignCritic::scorePose(const geometry_msgs::msg::Pose2D & pose)
{
  return PathDistCritic::scorePose(getForwardPose(pose, forward_point_distance_));
}

}

PLUGINLIB_EXPORT_CLASS(dwb_critics::PathAlignCritic, dwb_core::TrajectoryCritic)
