


#ifndef DWB_PLUGINS__STANDARD_TRAJ_GENERATOR_HPP_
#define DWB_PLUGINS__STANDARD_TRAJ_GENERATOR_HPP_

#include <vector>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "dwb_core/trajectory_generator.hpp"
#include "dwb_plugins/velocity_iterator.hpp"
#include "dwb_plugins/kinematic_parameters.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace dwb_plugins
{



class StandardTrajectoryGenerator : public dwb_core::TrajectoryGenerator
{
public:

  void initialize(
    const nav2_util::LifecycleNode::SharedPtr & nh,
    const std::string & plugin_name) override;
  void startNewIteration(const nav_2d_msgs::msg::Twist2D & current_velocity) override;
  bool hasMoreTwists() override;
  nav_2d_msgs::msg::Twist2D nextTwist() override;

  dwb_msgs::msg::Trajectory2D generateTrajectory(
    const geometry_msgs::msg::Pose2D & start_pose,
    const nav_2d_msgs::msg::Twist2D & start_vel,
    const nav_2d_msgs::msg::Twist2D & cmd_vel) override;

  

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override
  {
    if (kinematics_handler_) {
      kinematics_handler_->setSpeedLimit(speed_limit, percentage);
    }
  }

protected:
  

  virtual void initializeIterator(const nav2_util::LifecycleNode::SharedPtr & nh);

  

  virtual nav_2d_msgs::msg::Twist2D computeNewVelocity(
    const nav_2d_msgs::msg::Twist2D & cmd_vel, const nav_2d_msgs::msg::Twist2D & start_vel,
    const double dt);

  

  virtual geometry_msgs::msg::Pose2D computeNewPosition(
    const geometry_msgs::msg::Pose2D start_pose, const nav_2d_msgs::msg::Twist2D & vel,
    const double dt);


  

  virtual std::vector<double> getTimeSteps(const nav_2d_msgs::msg::Twist2D & cmd_vel);

  KinematicsHandler::Ptr kinematics_handler_;
  std::shared_ptr<VelocityIterator> velocity_iterator_;

  double sim_time_;


  bool discretize_by_time_;


  double time_granularity_;


  double linear_granularity_;


  double angular_granularity_;


  std::string plugin_name_;

  

  bool include_last_point_;
};


}

#endif
