

#include <vector>
#include <string>
#include <utility>

#include "dwb_critics/base_obstacle.hpp"
#include "dwb_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"

PLUGINLIB_EXPORT_CLASS(dwb_critics::BaseObstacleCritic, dwb_core::TrajectoryCritic)

namespace dwb_critics
{

void BaseObstacleCritic::onInit()
{
  costmap_ = costmap_ros_->getCostmap();

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  nav2_util::declare_parameter_if_not_declared(
    node,
    dwb_plugin_name_ + "." + name_ + ".sum_scores", rclcpp::ParameterValue(false));
  node->get_parameter(dwb_plugin_name_ + "." + name_ + ".sum_scores", sum_scores_);
}

double BaseObstacleCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj)
{
  double score = 0.0;
  for (unsigned int i = 0; i < traj.poses.size(); ++i) {
    double pose_score = scorePose(traj.poses[i]);


    score = static_cast<double>(sum_scores_) * score + pose_score;
  }
  return score;
}

double BaseObstacleCritic::scorePose(const geometry_msgs::msg::Pose2D & pose)
{
  unsigned int cell_x, cell_y;
  if (!costmap_->worldToMap(pose.x, pose.y, cell_x, cell_y)) {
    throw dwb_core::
          IllegalTrajectoryException(name_, "Trajectory Goes Off Grid.");
  }
  unsigned char cost = costmap_->getCost(cell_x, cell_y);
  if (!isValidCost(cost)) {
    throw dwb_core::
          IllegalTrajectoryException(name_, "Trajectory Hits Obstacle.");
  }
  return cost;
}

bool BaseObstacleCritic::isValidCost(const unsigned char cost)
{
  return cost != nav2_costmap_2d::LETHAL_OBSTACLE &&
         cost != nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE &&
         cost != nav2_costmap_2d::NO_INFORMATION;
}

void BaseObstacleCritic::addCriticVisualization(
  std::vector<std::pair<std::string, std::vector<float>>> & cost_channels)
{
  std::pair<std::string, std::vector<float>> grid_scores;
  grid_scores.first = name_;

  unsigned int size_x = costmap_->getSizeInCellsX();
  unsigned int size_y = costmap_->getSizeInCellsY();
  grid_scores.second.resize(size_x * size_y);
  unsigned int i = 0;
  for (unsigned int cy = 0; cy < size_y; cy++) {
    for (unsigned int cx = 0; cx < size_x; cx++) {
      grid_scores.second[i] = costmap_->getCost(cx, cy);
      i++;
    }
  }
  cost_channels.push_back(grid_scores);
}

}
