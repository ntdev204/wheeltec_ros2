


#include "dwb_critics/goal_dist.hpp"
#include <vector>
#include "pluginlib/class_list_macros.hpp"
#include "nav_2d_utils/path_ops.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

namespace dwb_critics
{
bool GoalDistCritic::prepare(
  const geometry_msgs::msg::Pose2D &, const nav_2d_msgs::msg::Twist2D &,
  const geometry_msgs::msg::Pose2D &,
  const nav_2d_msgs::msg::Path2D & global_plan)
{
  reset();

  unsigned int local_goal_x, local_goal_y;
  if (!getLastPoseOnCostmap(global_plan, local_goal_x, local_goal_y)) {
    return false;
  }


  int index = costmap_->getIndex(local_goal_x, local_goal_y);
  cell_values_[index] = 0.0;
  queue_->enqueueCell(local_goal_x, local_goal_y);

  propogateManhattanDistances();

  return true;
}

bool GoalDistCritic::getLastPoseOnCostmap(
  const nav_2d_msgs::msg::Path2D & global_plan,
  unsigned int & x, unsigned int & y)
{
  nav_2d_msgs::msg::Path2D adjusted_global_plan = nav_2d_utils::adjustPlanResolution(
    global_plan,
    costmap_->getResolution());
  bool started_path = false;


  for (unsigned int i = 0; i < adjusted_global_plan.poses.size(); ++i) {
    double g_x = adjusted_global_plan.poses[i].x;
    double g_y = adjusted_global_plan.poses[i].y;
    unsigned int map_x, map_y;
    if (costmap_->worldToMap(
        g_x, g_y, map_x,
        map_y) && costmap_->getCost(map_x, map_y) != nav2_costmap_2d::NO_INFORMATION)
    {

      x = map_x;
      y = map_y;
      started_path = true;
    } else if (started_path) {

      return true;
    }

  }

  if (started_path) {
    return true;
  } else {
    RCLCPP_ERROR(
      rclcpp::get_logger(
        "GoalDistCritic"), "None of the points of the global plan were in the local costmap.");
    return false;
  }
}

}

PLUGINLIB_EXPORT_CLASS(dwb_critics::GoalDistCritic, dwb_core::TrajectoryCritic)
