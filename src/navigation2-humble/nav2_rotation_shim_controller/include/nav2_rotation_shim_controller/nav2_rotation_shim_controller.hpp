













#ifndef NAV2_ROTATION_SHIM_CONTROLLER__NAV2_ROTATION_SHIM_CONTROLLER_HPP_
#define NAV2_ROTATION_SHIM_CONTROLLER__NAV2_ROTATION_SHIM_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "angles/angles.h"

namespace nav2_rotation_shim_controller
{



class RotationShimController : public nav2_core::Controller
{
public:
  

  RotationShimController();

  

  ~RotationShimController() override = default;

  

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  

  void cleanup() override;

  

  void activate() override;

  

  void deactivate() override;

  

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * ) override;

  

  void setPlan(const nav_msgs::msg::Path & path) override;

  

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  

  geometry_msgs::msg::PoseStamped getSampledPathPt();

  

  geometry_msgs::msg::Pose transformPoseToBaseFrame(const geometry_msgs::msg::PoseStamped & pt);

  

  geometry_msgs::msg::TwistStamped computeRotateToHeadingCommand(
    const double & angular_distance,
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity);

  

  void isCollisionFree(
    const geometry_msgs::msg::TwistStamped & cmd_vel,
    const double & angular_distance_to_heading,
    const geometry_msgs::msg::PoseStamped & pose);

  

  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  rclcpp::Logger logger_ {rclcpp::get_logger("RotationShimController")};
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>
  collision_checker_;

  pluginlib::ClassLoader<nav2_core::Controller> lp_loader_;
  nav2_core::Controller::Ptr primary_controller_;
  bool path_updated_;
  nav_msgs::msg::Path current_path_;
  double forward_sampling_distance_, angular_dist_threshold_;
  double rotate_to_heading_angular_vel_, max_angular_accel_;
  double control_duration_, simulate_ahead_time_;


  std::mutex mutex_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
};

}

#endif
