













#ifndef NAV2_UTIL__LIFECYCLE_SERVICE_CLIENT_HPP_
#define NAV2_UTIL__LIFECYCLE_SERVICE_CLIENT_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "nav2_util/service_client.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav2_util
{


class LifecycleServiceClient
{
public:
  explicit LifecycleServiceClient(const std::string & lifecycle_node_name);
  LifecycleServiceClient(
    const std::string & lifecycle_node_name,
    rclcpp::Node::SharedPtr parent_node);


  

  bool change_state(
    const uint8_t transition,
    const std::chrono::seconds timeout);


  bool change_state(std::uint8_t transition);


  

  uint8_t get_state(const std::chrono::seconds timeout = std::chrono::seconds(2));

protected:
  rclcpp::Node::SharedPtr node_;
  ServiceClient<lifecycle_msgs::srv::ChangeState> change_state_;
  ServiceClient<lifecycle_msgs::srv::GetState> get_state_;
};

}

#endif
