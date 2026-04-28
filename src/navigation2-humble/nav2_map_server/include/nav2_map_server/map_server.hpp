













#ifndef NAV2_MAP_SERVER__MAP_SERVER_HPP_
#define NAV2_MAP_SERVER__MAP_SERVER_HPP_

#include <string>
#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "nav2_msgs/srv/load_map.hpp"

namespace nav2_map_server
{



class MapServer : public nav2_util::LifecycleNode
{
public:
  

  explicit MapServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  

  ~MapServer();

protected:
  

  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  

  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  

  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  

  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  

  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  

  bool loadMapResponseFromYaml(
    const std::string & yaml_file,
    std::shared_ptr<nav2_msgs::srv::LoadMap::Response> response);

  

  void updateMsgHeader();

  

  void getMapCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav_msgs::srv::GetMap::Request> request,
    std::shared_ptr<nav_msgs::srv::GetMap::Response> response);

  

  void loadMapCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav2_msgs::srv::LoadMap::Request> request,
    std::shared_ptr<nav2_msgs::srv::LoadMap::Response> response);


  const std::string service_name_{"map"};


  const std::string load_map_service_name_{"load_map"};


  rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr occ_service_;


  rclcpp::Service<nav2_msgs::srv::LoadMap>::SharedPtr load_map_service_;


  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_pub_;


  std::string frame_id_;


  nav_msgs::msg::OccupancyGrid msg_;


  bool map_available_;
};

}

#endif
