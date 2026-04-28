


#ifndef DWB_CRITICS__OSCILLATION_HPP_
#define DWB_CRITICS__OSCILLATION_HPP_

#include <vector>
#include <string>
#include <chrono>
#include "dwb_core/trajectory_critic.hpp"

using namespace std::chrono_literals;

namespace dwb_critics
{



class OscillationCritic : public dwb_core::TrajectoryCritic
{
public:
  OscillationCritic()
  : oscillation_reset_time_(0s) {}
  void onInit() override;
  bool prepare(
    const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D & vel,
    const geometry_msgs::msg::Pose2D & goal, const nav_2d_msgs::msg::Path2D & global_plan) override;
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj) override;
  void reset() override;
  void debrief(const nav_2d_msgs::msg::Twist2D & cmd_vel) override;

private:
  

  class CommandTrend
  {
public:
    CommandTrend();
    void reset();

    

    bool update(double velocity);

    

    bool isOscillating(double velocity);

    

    bool hasSignFlipped();

private:


    enum class Sign { ZERO, POSITIVE, NEGATIVE };

    Sign sign_;
    bool positive_only_, negative_only_;
  };

  

  bool setOscillationFlags(const nav_2d_msgs::msg::Twist2D & cmd_vel);

  

  bool resetAvailable();

  CommandTrend x_trend_, y_trend_, theta_trend_;
  double oscillation_reset_dist_, oscillation_reset_angle_, x_only_threshold_;
  rclcpp::Duration oscillation_reset_time_;


  double oscillation_reset_dist_sq_;


  geometry_msgs::msg::Pose2D pose_, prev_stationary_pose_;

  rclcpp::Time prev_reset_time_;
  rclcpp::Clock::SharedPtr clock_;
};

}
#endif
