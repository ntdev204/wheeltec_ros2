













#ifndef NAV2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_CLIENT_HPP_
#define NAV2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_CLIENT_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/empty.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "nav2_util/service_client.hpp"

namespace nav2_lifecycle_manager
{


enum class SystemStatus {ACTIVE, INACTIVE, TIMEOUT};


class LifecycleManagerClient
{
public:
  

  explicit LifecycleManagerClient(
    const std::string & name,
    std::shared_ptr<rclcpp::Node> parent_node);


  

  bool startup(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));
  

  bool shutdown(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));
  

  bool pause(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));
  

  bool resume(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));
  

  bool reset(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));
  

  SystemStatus is_active(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));


  

  void set_initial_pose(double x, double y, double theta);
  

  bool navigate_to_pose(double x, double y, double theta);

protected:
  using ManageLifecycleNodes = nav2_msgs::srv::ManageLifecycleNodes;

  

  bool callService(
    uint8_t command,
    const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));


  rclcpp::Node::SharedPtr node_;

  std::shared_ptr<nav2_util::ServiceClient<ManageLifecycleNodes>> manager_client_;
  std::shared_ptr<nav2_util::ServiceClient<std_srvs::srv::Trigger>> is_active_client_;
  std::string manage_service_name_;
  std::string active_service_name_;
};

}

#endif
