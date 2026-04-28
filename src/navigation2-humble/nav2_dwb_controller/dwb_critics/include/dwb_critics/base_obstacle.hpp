


#ifndef DWB_CRITICS__BASE_OBSTACLE_HPP_
#define DWB_CRITICS__BASE_OBSTACLE_HPP_

#include <string>
#include <vector>
#include <utility>

#include "dwb_core/trajectory_critic.hpp"

namespace dwb_critics
{


class BaseObstacleCritic : public dwb_core::TrajectoryCritic
{
public:
  void onInit() override;
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj) override;
  void addCriticVisualization(
    std::vector<std::pair<std::string, std::vector<float>>> & cost_channels) override;

  

  virtual double scorePose(const geometry_msgs::msg::Pose2D & pose);

  

  virtual bool isValidCost(const unsigned char cost);

protected:
  nav2_costmap_2d::Costmap2D * costmap_;
  bool sum_scores_;
};
}

#endif
