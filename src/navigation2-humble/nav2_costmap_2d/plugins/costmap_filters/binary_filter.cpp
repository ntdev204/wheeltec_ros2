


#include "nav2_costmap_2d/costmap_filters/binary_filter.hpp"

#include <cmath>
#include <utility>
#include <memory>
#include <string>

#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_util/occ_grid_values.hpp"

namespace nav2_costmap_2d
{

BinaryFilter::BinaryFilter()
: filter_info_sub_(nullptr), mask_sub_(nullptr),
  binary_state_pub_(nullptr), filter_mask_(nullptr), mask_frame_(""), global_frame_(""),
  default_state_(false), binary_state_(default_state_)
{
}

void BinaryFilter::initializeFilter(
  const std::string & filter_info_topic)
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  rclcpp_lifecycle::LifecycleNode::SharedPtr node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }


  std::string binary_state_topic;
  declareParameter("default_state", rclcpp::ParameterValue(false));
  node->get_parameter(name_ + "." + "default_state", default_state_);
  declareParameter("binary_state_topic", rclcpp::ParameterValue("binary_state"));
  node->get_parameter(name_ + "." + "binary_state_topic", binary_state_topic);
  declareParameter("flip_threshold", rclcpp::ParameterValue(50.0));
  node->get_parameter(name_ + "." + "flip_threshold", flip_threshold_);

  filter_info_topic_ = filter_info_topic;

  RCLCPP_INFO(
    logger_,
    "BinaryFilter: Subscribing to \"%s\" topic for filter info...",
    filter_info_topic_.c_str());
  filter_info_sub_ = node->create_subscription<nav2_msgs::msg::CostmapFilterInfo>(
    filter_info_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&BinaryFilter::filterInfoCallback, this, std::placeholders::_1));


  global_frame_ = layered_costmap_->getGlobalFrameID();


  binary_state_pub_ = node->create_publisher<std_msgs::msg::Bool>(
    binary_state_topic, rclcpp::QoS(10));
  binary_state_pub_->on_activate();


  base_ = BASE_DEFAULT;
  multiplier_ = MULTIPLIER_DEFAULT;


  changeState(default_state_);
}

void BinaryFilter::filterInfoCallback(
  const nav2_msgs::msg::CostmapFilterInfo::SharedPtr msg)
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  rclcpp_lifecycle::LifecycleNode::SharedPtr node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  if (!mask_sub_) {
    RCLCPP_INFO(
      logger_,
      "BinaryFilter: Received filter info from %s topic.", filter_info_topic_.c_str());
  } else {
    RCLCPP_WARN(
      logger_,
      "BinaryFilter: New costmap filter info arrived from %s topic. Updating old filter info.",
      filter_info_topic_.c_str());

    mask_sub_.reset();
  }

  if (msg->type != BINARY_FILTER) {
    RCLCPP_ERROR(logger_, "BinaryFilter: Mode %i is not supported", msg->type);
    return;
  }


  base_ = msg->base;
  multiplier_ = msg->multiplier;

  mask_topic_ = msg->filter_mask_topic;


  RCLCPP_INFO(
    logger_,
    "BinaryFilter: Subscribing to \"%s\" topic for filter mask...",
    mask_topic_.c_str());
  mask_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    mask_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&BinaryFilter::maskCallback, this, std::placeholders::_1));
}

void BinaryFilter::maskCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  if (!filter_mask_) {
    RCLCPP_INFO(
      logger_,
      "BinaryFilter: Received filter mask from %s topic.", mask_topic_.c_str());
  } else {
    RCLCPP_WARN(
      logger_,
      "BinaryFilter: New filter mask arrived from %s topic. Updating old filter mask.",
      mask_topic_.c_str());
    filter_mask_.reset();
  }

  filter_mask_ = msg;
  mask_frame_ = msg->header.frame_id;
}

void BinaryFilter::process(
  nav2_costmap_2d::Costmap2D & ,
  int , int , int , int ,
  const geometry_msgs::msg::Pose2D & pose)
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  if (!filter_mask_) {

    RCLCPP_WARN_THROTTLE(
      logger_, *(clock_), 2000,
      "BinaryFilter: Filter mask was not received");
    return;
  }

  geometry_msgs::msg::Pose2D mask_pose;


  if (!transformPose(global_frame_, pose, mask_frame_, mask_pose)) {
    return;
  }


  unsigned int mask_robot_i, mask_robot_j;
  if (!worldToMask(filter_mask_, mask_pose.x, mask_pose.y, mask_robot_i, mask_robot_j)) {

    RCLCPP_WARN(
      logger_,
      "BinaryFilter: Robot is outside of filter mask. Resetting binary state to default.");
    changeState(default_state_);
    return;
  }


  int8_t mask_data = getMaskData(filter_mask_, mask_robot_i, mask_robot_j);
  if (mask_data == nav2_util::OCC_GRID_UNKNOWN) {


    RCLCPP_WARN_THROTTLE(
      logger_, *(clock_), 2000,
      "BinaryFilter: Filter mask [%i, %i] data is unknown. Do nothing.",
      mask_robot_i, mask_robot_j);
    return;
  }

  if (base_ + mask_data * multiplier_ > flip_threshold_) {
    if (binary_state_ == default_state_) {
      changeState(!default_state_);
    }
  } else {
    if (binary_state_ != default_state_) {
      changeState(default_state_);
    }
  }
}

void BinaryFilter::resetFilter()
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  RCLCPP_INFO(logger_, "BinaryFilter: Resetting the filter to default state");
  changeState(default_state_);

  filter_info_sub_.reset();
  mask_sub_.reset();
  if (binary_state_pub_) {
    binary_state_pub_->on_deactivate();
    binary_state_pub_.reset();
  }
}

bool BinaryFilter::isActive()
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  if (filter_mask_) {
    return true;
  }
  return false;
}

void BinaryFilter::changeState(const bool state)
{
  binary_state_ = state;
  if (state) {
    RCLCPP_INFO(logger_, "BinaryFilter: Switched on");
  } else {
    RCLCPP_INFO(logger_, "BinaryFilter: Switched off");
  }


  std::unique_ptr<std_msgs::msg::Bool> msg =
    std::make_unique<std_msgs::msg::Bool>();
  msg->data = state;
  binary_state_pub_->publish(std::move(msg));
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::BinaryFilter, nav2_costmap_2d::Layer)
