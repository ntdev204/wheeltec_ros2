

#ifndef DWB_CRITICS__ROTATE_TO_GOAL_HPP_
#define DWB_CRITICS__ROTATE_TO_GOAL_HPP_

#include <string>
#include <vector>
#include "dwb_core/trajectory_critic.hpp"

namespace dwb_critics
{



class RotateToGoalCritic : public dwb_core::TrajectoryCritic
{
public:
  void onInit() override;
  void reset() override;
  bool prepare(
    const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D & vel,
    const geometry_msgs::msg::Pose2D & goal, const nav_2d_msgs::msg::Path2D & global_plan) override;
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj) override;
  

  virtual double scoreRotation(const dwb_msgs::msg::Trajectory2D & traj);

private:
  bool in_window_;
  bool rotating_;
  double goal_yaw_;
  double xy_goal_tolerance_;
  double xy_goal_tolerance_sq_;
  double current_xy_speed_sq_, stopped_xy_velocity_sq_;
  double slowing_factor_;
  double lookahead_time_;
};

}
#endif
