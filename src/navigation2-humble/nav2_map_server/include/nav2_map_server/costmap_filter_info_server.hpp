













#ifndef NAV2_MAP_SERVER__COSTMAP_FILTER_INFO_SERVER_HPP_
#define NAV2_MAP_SERVER__COSTMAP_FILTER_INFO_SERVER_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_msgs/msg/costmap_filter_info.hpp"

namespace nav2_map_server
{

class CostmapFilterInfoServer : public nav2_util::LifecycleNode
{
public:
  

  CostmapFilterInfoServer();
  

  ~CostmapFilterInfoServer();

protected:
  

  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  

  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  

  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  

  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  

  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  rclcpp_lifecycle::LifecyclePublisher<nav2_msgs::msg::CostmapFilterInfo>::SharedPtr publisher_;

  nav2_msgs::msg::CostmapFilterInfo msg_;
};

}

#endif
