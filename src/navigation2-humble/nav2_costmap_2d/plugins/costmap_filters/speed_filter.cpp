


#include "nav2_costmap_2d/costmap_filters/speed_filter.hpp"

#include <cmath>
#include <utility>
#include <memory>
#include <string>

#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

namespace nav2_costmap_2d
{

SpeedFilter::SpeedFilter()
: filter_info_sub_(nullptr), mask_sub_(nullptr),
  speed_limit_pub_(nullptr), filter_mask_(nullptr), mask_frame_(""), global_frame_(""),
  speed_limit_(NO_SPEED_LIMIT), speed_limit_prev_(NO_SPEED_LIMIT)
{
}

void SpeedFilter::initializeFilter(
  const std::string & filter_info_topic)
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  rclcpp_lifecycle::LifecycleNode::SharedPtr node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }


  std::string speed_limit_topic;
  declareParameter("speed_limit_topic", rclcpp::ParameterValue("speed_limit"));
  node->get_parameter(name_ + "." + "speed_limit_topic", speed_limit_topic);

  filter_info_topic_ = filter_info_topic;

  RCLCPP_INFO(
    logger_,
    "SpeedFilter: Subscribing to \"%s\" topic for filter info...",
    filter_info_topic_.c_str());
  filter_info_sub_ = node->create_subscription<nav2_msgs::msg::CostmapFilterInfo>(
    filter_info_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&SpeedFilter::filterInfoCallback, this, std::placeholders::_1));


  global_frame_ = layered_costmap_->getGlobalFrameID();


  speed_limit_pub_ = node->create_publisher<nav2_msgs::msg::SpeedLimit>(
    speed_limit_topic, rclcpp::QoS(10));
  speed_limit_pub_->on_activate();


  base_ = BASE_DEFAULT;
  multiplier_ = MULTIPLIER_DEFAULT;
  percentage_ = false;
}

void SpeedFilter::filterInfoCallback(
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
      "SpeedFilter: Received filter info from %s topic.", filter_info_topic_.c_str());
  } else {
    RCLCPP_WARN(
      logger_,
      "SpeedFilter: New costmap filter info arrived from %s topic. Updating old filter info.",
      filter_info_topic_.c_str());

    mask_sub_.reset();
  }


  base_ = msg->base;
  multiplier_ = msg->multiplier;
  if (msg->type == SPEED_FILTER_PERCENT) {

    percentage_ = true;
    RCLCPP_INFO(
      logger_,
      "SpeedFilter: Using expressed in a percent from maximum speed"
      "speed_limit = %f + filter_mask_data * %f",
      base_, multiplier_);
  } else if (msg->type == SPEED_FILTER_ABSOLUTE) {

    percentage_ = false;
    RCLCPP_INFO(
      logger_,
      "SpeedFilter: Using absolute speed_limit = %f + filter_mask_data * %f",
      base_, multiplier_);
  } else {
    RCLCPP_ERROR(logger_, "SpeedFilter: Mode is not supported");
    return;
  }

  mask_topic_ = msg->filter_mask_topic;


  RCLCPP_INFO(
    logger_,
    "SpeedFilter: Subscribing to \"%s\" topic for filter mask...",
    mask_topic_.c_str());
  mask_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    mask_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&SpeedFilter::maskCallback, this, std::placeholders::_1));
}

void SpeedFilter::maskCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  if (!filter_mask_) {
    RCLCPP_INFO(
      logger_,
      "SpeedFilter: Received filter mask from %s topic.", mask_topic_.c_str());
  } else {
    RCLCPP_WARN(
      logger_,
      "SpeedFilter: New filter mask arrived from %s topic. Updating old filter mask.",
      mask_topic_.c_str());
    filter_mask_.reset();
  }

  filter_mask_ = msg;
  mask_frame_ = msg->header.frame_id;
}

void SpeedFilter::process(
  nav2_costmap_2d::Costmap2D & ,
  int , int , int , int ,
  const geometry_msgs::msg::Pose2D & pose)
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  if (!filter_mask_) {

    RCLCPP_WARN_THROTTLE(
      logger_, *(clock_), 2000,
      "SpeedFilter: Filter mask was not received");
    return;
  }

  geometry_msgs::msg::Pose2D mask_pose;


  if (!transformPose(global_frame_, pose, mask_frame_, mask_pose)) {
    return;
  }


  unsigned int mask_robot_i, mask_robot_j;
  if (!worldToMask(filter_mask_, mask_pose.x, mask_pose.y, mask_robot_i, mask_robot_j)) {
    return;
  }



  int8_t speed_mask_data = getMaskData(filter_mask_, mask_robot_i, mask_robot_j);
  if (speed_mask_data == SPEED_MASK_NO_LIMIT) {


    speed_limit_ = NO_SPEED_LIMIT;
  } else if (speed_mask_data == SPEED_MASK_UNKNOWN) {


    RCLCPP_ERROR(
      logger_,
      "SpeedFilter: Found unknown cell in filter_mask[%i, %i], "
      "which is invalid for this kind of filter",
      mask_robot_i, mask_robot_j);
    return;
  } else {

    speed_limit_ = speed_mask_data * multiplier_ + base_;
    if (percentage_) {
      if (speed_limit_ < 0.0 || speed_limit_ > 100.0) {
        RCLCPP_WARN(
          logger_,
          "SpeedFilter: Speed limit in filter_mask[%i, %i] is %f%%, "
          "out of bounds of [0, 100]. Setting it to no-limit value.",
          mask_robot_i, mask_robot_j, speed_limit_);
        speed_limit_ = NO_SPEED_LIMIT;
      }
    } else {
      if (speed_limit_ < 0.0) {
        RCLCPP_WARN(
          logger_,
          "SpeedFilter: Speed limit in filter_mask[%i, %i] is less than 0 m/s, "
          "which can not be true. Setting it to no-limit value.",
          mask_robot_i, mask_robot_j);
        speed_limit_ = NO_SPEED_LIMIT;
      }
    }
  }

  if (speed_limit_ != speed_limit_prev_) {
    if (speed_limit_ != NO_SPEED_LIMIT) {
      RCLCPP_DEBUG(logger_, "SpeedFilter: Speed limit is set to %f", speed_limit_);
    } else {
      RCLCPP_DEBUG(logger_, "SpeedFilter: Speed limit is set to its default value");
    }


    std::unique_ptr<nav2_msgs::msg::SpeedLimit> msg =
      std::make_unique<nav2_msgs::msg::SpeedLimit>();
    msg->header.frame_id = global_frame_;
    msg->header.stamp = clock_->now();
    msg->percentage = percentage_;
    msg->speed_limit = speed_limit_;
    speed_limit_pub_->publish(std::move(msg));

    speed_limit_prev_ = speed_limit_;
  }
}

void SpeedFilter::resetFilter()
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  filter_info_sub_.reset();
  mask_sub_.reset();
  if (speed_limit_pub_) {
    speed_limit_pub_->on_deactivate();
    speed_limit_pub_.reset();
  }
}

bool SpeedFilter::isActive()
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  if (filter_mask_) {
    return true;
  }
  return false;
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::SpeedFilter, nav2_costmap_2d::Layer)
