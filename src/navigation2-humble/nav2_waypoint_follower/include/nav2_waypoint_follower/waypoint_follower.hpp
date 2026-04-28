













#ifndef NAV2_WAYPOINT_FOLLOWER__WAYPOINT_FOLLOWER_HPP_
#define NAV2_WAYPOINT_FOLLOWER__WAYPOINT_FOLLOWER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include "nav2_util/lifecycle_node.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav2_util/node_utils.hpp"
#include "nav2_core/waypoint_task_executor.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace nav2_waypoint_follower
{

enum class ActionStatus
{
  UNKNOWN = 0,
  PROCESSING = 1,
  FAILED = 2,
  SUCCEEDED = 3
};



class WaypointFollower : public nav2_util::LifecycleNode
{
public:
  using ActionT = nav2_msgs::action::FollowWaypoints;
  using ClientT = nav2_msgs::action::NavigateToPose;
  using ActionServer = nav2_util::SimpleActionServer<ActionT>;
  using ActionClient = rclcpp_action::Client<ClientT>;

  

  explicit WaypointFollower(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  

  ~WaypointFollower();

protected:
  

  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  

  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  

  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  

  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  

  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  

  void followWaypoints();

  

  void resultCallback(const rclcpp_action::ClientGoalHandle<ClientT>::WrappedResult & result);

  

  void goalResponseCallback(const rclcpp_action::ClientGoalHandle<ClientT>::SharedPtr & goal);

  

  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);


  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;


  std::unique_ptr<ActionServer> action_server_;
  ActionClient::SharedPtr nav_to_pose_client_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  std::shared_future<rclcpp_action::ClientGoalHandle<ClientT>::SharedPtr> future_goal_handle_;
  bool stop_on_failure_;
  ActionStatus current_goal_status_;
  int loop_rate_;
  std::vector<int> failed_ids_;


  pluginlib::ClassLoader<nav2_core::WaypointTaskExecutor>
  waypoint_task_executor_loader_;
  pluginlib::UniquePtr<nav2_core::WaypointTaskExecutor>
  waypoint_task_executor_;
  std::string waypoint_task_executor_id_;
  std::string waypoint_task_executor_type_;
};

}

#endif
