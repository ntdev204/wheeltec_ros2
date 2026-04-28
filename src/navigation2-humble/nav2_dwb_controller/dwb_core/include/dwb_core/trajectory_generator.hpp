


#ifndef DWB_CORE__TRAJECTORY_GENERATOR_HPP_
#define DWB_CORE__TRAJECTORY_GENERATOR_HPP_

#include <vector>
#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav_2d_msgs/msg/twist2_d.hpp"
#include "dwb_msgs/msg/trajectory2_d.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace dwb_core
{



class TrajectoryGenerator
{
public:
  typedef std::shared_ptr<dwb_core::TrajectoryGenerator> Ptr;

  virtual ~TrajectoryGenerator() {}

  

  virtual void initialize(
    const nav2_util::LifecycleNode::SharedPtr & nh,
    const std::string & plugin_name) = 0;
  virtual void reset() {}
  

  virtual void startNewIteration(const nav_2d_msgs::msg::Twist2D & current_velocity) = 0;

  

  virtual bool hasMoreTwists() = 0;

  

  virtual nav_2d_msgs::msg::Twist2D nextTwist() = 0;

  

  virtual std::vector<nav_2d_msgs::msg::Twist2D> getTwists(
    const nav_2d_msgs::msg::Twist2D & current_velocity)
  {
    std::vector<nav_2d_msgs::msg::Twist2D> twists;
    startNewIteration(current_velocity);
    while (hasMoreTwists()) {
      twists.push_back(nextTwist());
    }
    return twists;
  }

  

  virtual dwb_msgs::msg::Trajectory2D generateTrajectory(
    const geometry_msgs::msg::Pose2D & start_pose,
    const nav_2d_msgs::msg::Twist2D & start_vel,
    const nav_2d_msgs::msg::Twist2D & cmd_vel) = 0;

  

  virtual void setSpeedLimit(const double & speed_limit, const bool & percentage) = 0;
};

}

#endif
