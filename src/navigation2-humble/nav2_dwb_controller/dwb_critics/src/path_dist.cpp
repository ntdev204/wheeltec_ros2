


#include "dwb_critics/path_dist.hpp"
#include <vector>
#include "pluginlib/class_list_macros.hpp"
#include "nav_2d_utils/path_ops.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

namespace dwb_critics
{
bool PathDistCritic::prepare(
  const geometry_msgs::msg::Pose2D &, const nav_2d_msgs::msg::Twist2D &,
  const geometry_msgs::msg::Pose2D &,
  const nav_2d_msgs::msg::Path2D & global_plan)
{
  reset();
  bool started_path = false;

  nav_2d_msgs::msg::Path2D adjusted_global_plan =
    nav_2d_utils::adjustPlanResolution(global_plan, costmap_->getResolution());

  if (adjusted_global_plan.poses.size() != global_plan.poses.size()) {
    RCLCPP_DEBUG(
      rclcpp::get_logger(
        "PathDistCritic"), "Adjusted global plan resolution, added %zu points",
      adjusted_global_plan.poses.size() - global_plan.poses.size());
  }

  unsigned int i;

  for (i = 0; i < adjusted_global_plan.poses.size(); ++i) {
    double g_x = adjusted_global_plan.poses[i].x;
    double g_y = adjusted_global_plan.poses[i].y;
    unsigned int map_x, map_y;
    if (costmap_->worldToMap(
        g_x, g_y, map_x,
        map_y) && costmap_->getCost(map_x, map_y) != nav2_costmap_2d::NO_INFORMATION)
    {
      int index = costmap_->getIndex(map_x, map_y);
      cell_values_[index] = 0.0;
      queue_->enqueueCell(map_x, map_y);
      started_path = true;
    } else if (started_path) {
      break;
    }
  }
  if (!started_path) {
    RCLCPP_ERROR(
      rclcpp::get_logger("PathDistCritic"),
      "None of the %d first of %zu (%zu) points of the global plan were in "
      "the local costmap and free",
      i, adjusted_global_plan.poses.size(), global_plan.poses.size());
    return false;
  }

  propogateManhattanDistances();

  return true;
}

}

PLUGINLIB_EXPORT_CLASS(dwb_critics::PathDistCritic, dwb_core::TrajectoryCritic)
