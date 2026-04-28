














#ifndef NAV2_MAP_SERVER__MAP_SAVER_HPP_
#define NAV2_MAP_SERVER__MAP_SAVER_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_msgs/srv/save_map.hpp"

#include "map_io.hpp"

namespace nav2_map_server
{



class MapSaver : public nav2_util::LifecycleNode
{
public:
  

  explicit MapSaver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  

  ~MapSaver();

  

  bool saveMapTopicToFile(
    const std::string & map_topic,
    const SaveParameters & save_parameters);

  

  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  

  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  

  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  

  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  

  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

protected:
  

  void saveMapCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav2_msgs::srv::SaveMap::Request> request,
    std::shared_ptr<nav2_msgs::srv::SaveMap::Response> response);


  std::shared_ptr<rclcpp::Duration> save_map_timeout_;

  double free_thresh_default_;
  double occupied_thresh_default_;

  bool map_subscribe_transient_local_;


  const std::string save_map_service_name_{"save_map"};

  rclcpp::Service<nav2_msgs::srv::SaveMap>::SharedPtr save_map_service_;
};

}

#endif
