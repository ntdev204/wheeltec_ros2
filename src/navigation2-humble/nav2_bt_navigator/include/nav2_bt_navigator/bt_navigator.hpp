













#ifndef NAV2_BT_NAVIGATOR__BT_NAVIGATOR_HPP_
#define NAV2_BT_NAVIGATOR__BT_NAVIGATOR_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "nav2_bt_navigator/navigators/navigate_to_pose.hpp"
#include "nav2_bt_navigator/navigators/navigate_through_poses.hpp"

namespace nav2_bt_navigator
{



class BtNavigator : public nav2_util::LifecycleNode
{
public:
  

  explicit BtNavigator(rclcpp::NodeOptions options = rclcpp::NodeOptions());
  

  ~BtNavigator();

protected:
  

  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  

  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  

  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  

  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  

  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;


  std::unique_ptr<nav2_bt_navigator::Navigator<nav2_msgs::action::NavigateToPose>> pose_navigator_;
  std::unique_ptr<nav2_bt_navigator::Navigator<nav2_msgs::action::NavigateThroughPoses>>
  poses_navigator_;
  nav2_bt_navigator::NavigatorMuxer plugin_muxer_;


  std::shared_ptr<nav2_util::OdomSmoother> odom_smoother_;


  std::string robot_frame_;
  std::string global_frame_;
  double transform_tolerance_;
  std::string odom_topic_;


  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}

#endif
