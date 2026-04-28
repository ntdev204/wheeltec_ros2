





#include "nav2_map_server/map_server.hpp"

#include <string>
#include <memory>
#include <fstream>
#include <stdexcept>
#include <utility>

#include "yaml-cpp/yaml.h"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_map_server/map_io.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace nav2_map_server
{

MapServer::MapServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("map_server", "", options), map_available_(false)
{
  RCLCPP_INFO(get_logger(), "Creating");


  declare_parameter("yaml_filename", rclcpp::PARAMETER_STRING);
  declare_parameter("topic_name", "map");
  declare_parameter("frame_id", "map");
}

MapServer::~MapServer()
{
}

nav2_util::CallbackReturn
MapServer::on_configure(const rclcpp_lifecycle::State & )
{
  RCLCPP_INFO(get_logger(), "Configuring");


  std::string yaml_filename = get_parameter("yaml_filename").as_string();
  std::string topic_name = get_parameter("topic_name").as_string();
  frame_id_ = get_parameter("frame_id").as_string();


  if (!yaml_filename.empty()) {


    std::shared_ptr<nav2_msgs::srv::LoadMap::Response> rsp =
      std::make_shared<nav2_msgs::srv::LoadMap::Response>();

    if (!loadMapResponseFromYaml(yaml_filename, rsp)) {
      throw std::runtime_error("Failed to load map yaml file: " + yaml_filename);
    }
  } else {
    RCLCPP_INFO(
      get_logger(),
      "yaml-filename parameter is empty, set map through '%s'-service",
      load_map_service_name_.c_str());
  }


  const std::string service_prefix = get_name() + std::string("/");


  occ_service_ = create_service<nav_msgs::srv::GetMap>(
    service_prefix + std::string(service_name_),
    std::bind(&MapServer::getMapCallback, this, _1, _2, _3));


  occ_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    topic_name,
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());


  load_map_service_ = create_service<nav2_msgs::srv::LoadMap>(
    service_prefix + std::string(load_map_service_name_),
    std::bind(&MapServer::loadMapCallback, this, _1, _2, _3));

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MapServer::on_activate(const rclcpp_lifecycle::State & )
{
  RCLCPP_INFO(get_logger(), "Activating");


  occ_pub_->on_activate();
  if (map_available_) {
    auto occ_grid = std::make_unique<nav_msgs::msg::OccupancyGrid>(msg_);
    occ_pub_->publish(std::move(occ_grid));
  }


  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MapServer::on_deactivate(const rclcpp_lifecycle::State & )
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  occ_pub_->on_deactivate();


  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MapServer::on_cleanup(const rclcpp_lifecycle::State & )
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  occ_pub_.reset();
  occ_service_.reset();
  load_map_service_.reset();
  map_available_ = false;
  msg_ = nav_msgs::msg::OccupancyGrid();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MapServer::on_shutdown(const rclcpp_lifecycle::State & )
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

void MapServer::getMapCallback(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<nav_msgs::srv::GetMap::Request>,
  std::shared_ptr<nav_msgs::srv::GetMap::Response> response)
{

  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_WARN(
      get_logger(),
      "Received GetMap request but not in ACTIVE state, ignoring!");
    return;
  }
  RCLCPP_INFO(get_logger(), "Handling GetMap request");
  response->map = msg_;
}

void MapServer::loadMapCallback(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<nav2_msgs::srv::LoadMap::Request> request,
  std::shared_ptr<nav2_msgs::srv::LoadMap::Response> response)
{

  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_WARN(
      get_logger(),
      "Received LoadMap request but not in ACTIVE state, ignoring!");
    response->result = response->RESULT_UNDEFINED_FAILURE;
    return;
  }
  RCLCPP_INFO(get_logger(), "Handling LoadMap request");

  if (loadMapResponseFromYaml(request->map_url, response)) {
    auto occ_grid = std::make_unique<nav_msgs::msg::OccupancyGrid>(msg_);
    occ_pub_->publish(std::move(occ_grid));
  }
}

bool MapServer::loadMapResponseFromYaml(
  const std::string & yaml_file,
  std::shared_ptr<nav2_msgs::srv::LoadMap::Response> response)
{
  switch (loadMapFromYaml(yaml_file, msg_)) {
    case MAP_DOES_NOT_EXIST:
      response->result = nav2_msgs::srv::LoadMap::Response::RESULT_MAP_DOES_NOT_EXIST;
      return false;
    case INVALID_MAP_METADATA:
      response->result = nav2_msgs::srv::LoadMap::Response::RESULT_INVALID_MAP_METADATA;
      return false;
    case INVALID_MAP_DATA:
      response->result = nav2_msgs::srv::LoadMap::Response::RESULT_INVALID_MAP_DATA;
      return false;
    case LOAD_MAP_SUCCESS:

      updateMsgHeader();

      map_available_ = true;
      response->map = msg_;
      response->result = nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS;
  }

  return true;
}

void MapServer::updateMsgHeader()
{
  msg_.info.map_load_time = now();
  msg_.header.frame_id = frame_id_;
  msg_.header.stamp = now();
}

}

#include "rclcpp_components/register_node_macro.hpp"




RCLCPP_COMPONENTS_REGISTER_NODE(nav2_map_server::MapServer)
