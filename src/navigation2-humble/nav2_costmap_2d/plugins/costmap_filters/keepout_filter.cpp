


#include <string>
#include <memory>
#include <algorithm>
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "nav2_costmap_2d/costmap_filters/keepout_filter.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

namespace nav2_costmap_2d
{

KeepoutFilter::KeepoutFilter()
: filter_info_sub_(nullptr), mask_sub_(nullptr), mask_costmap_(nullptr),
  mask_frame_(""), global_frame_("")
{
}

void KeepoutFilter::initializeFilter(
  const std::string & filter_info_topic)
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  rclcpp_lifecycle::LifecycleNode::SharedPtr node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  filter_info_topic_ = filter_info_topic;

  RCLCPP_INFO(
    logger_,
    "KeepoutFilter: Subscribing to \"%s\" topic for filter info...",
    filter_info_topic_.c_str());
  filter_info_sub_ = node->create_subscription<nav2_msgs::msg::CostmapFilterInfo>(
    filter_info_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&KeepoutFilter::filterInfoCallback, this, std::placeholders::_1));

  global_frame_ = layered_costmap_->getGlobalFrameID();
}

void KeepoutFilter::filterInfoCallback(
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
      "KeepoutFilter: Received filter info from %s topic.", filter_info_topic_.c_str());
  } else {
    RCLCPP_WARN(
      logger_,
      "KeepoutFilter: New costmap filter info arrived from %s topic. Updating old filter info.",
      filter_info_topic_.c_str());

    mask_sub_.reset();
  }


  if (msg->base != BASE_DEFAULT || msg->multiplier != MULTIPLIER_DEFAULT) {
    RCLCPP_ERROR(
      logger_,
      "KeepoutFilter: For proper use of keepout filter base and multiplier"
      " in CostmapFilterInfo message should be set to their default values (%f and %f)",
      BASE_DEFAULT, MULTIPLIER_DEFAULT);
  }

  mask_topic_ = msg->filter_mask_topic;


  RCLCPP_INFO(
    logger_,
    "KeepoutFilter: Subscribing to \"%s\" topic for filter mask...",
    mask_topic_.c_str());
  mask_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    mask_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&KeepoutFilter::maskCallback, this, std::placeholders::_1));
}

void KeepoutFilter::maskCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  rclcpp_lifecycle::LifecycleNode::SharedPtr node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  if (!mask_costmap_) {
    RCLCPP_INFO(
      logger_,
      "KeepoutFilter: Received filter mask from %s topic.", mask_topic_.c_str());
  } else {
    RCLCPP_WARN(
      logger_,
      "KeepoutFilter: New filter mask arrived from %s topic. Updating old filter mask.",
      mask_topic_.c_str());
    mask_costmap_.reset();
  }


  mask_costmap_ = std::make_unique<Costmap2D>(*msg);
  mask_frame_ = msg->header.frame_id;
}

void KeepoutFilter::process(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j,
  const geometry_msgs::msg::Pose2D & )
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  if (!mask_costmap_) {

    RCLCPP_WARN_THROTTLE(
      logger_, *(clock_), 2000,
      "KeepoutFilter: Filter mask was not received");
    return;
  }

  tf2::Transform tf2_transform;
  tf2_transform.setIdentity();
  int mg_min_x, mg_min_y;
  int mg_max_x, mg_max_y;

  if (mask_frame_ != global_frame_) {


    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_->lookupTransform(
        mask_frame_, global_frame_, tf2::TimePointZero,
        transform_tolerance_);
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(
        logger_,
        "KeepoutFilter: Failed to get costmap frame (%s) "
        "transformation to mask frame (%s) with error: %s",
        global_frame_.c_str(), mask_frame_.c_str(), ex.what());
      return;
    }
    tf2::fromMsg(transform.transform, tf2_transform);

    mg_min_x = min_i;
    mg_min_y = min_j;
    mg_max_x = max_i;
    mg_max_y = max_j;
  } else {




















    double wx, wy;



    const double half_cell_size = 0.5 * mask_costmap_->getResolution();
    wx = mask_costmap_->getOriginX() + half_cell_size;
    wy = mask_costmap_->getOriginY() + half_cell_size;
    master_grid.worldToMapNoBounds(wx, wy, mg_min_x, mg_min_y);

    if (mg_min_x >= max_i || mg_min_y >= max_j) {

      return;
    }
    mg_min_x = std::max(min_i, mg_min_x);
    mg_min_y = std::max(min_j, mg_min_y);



    wx = mask_costmap_->getOriginX() +
      mask_costmap_->getSizeInCellsX() * mask_costmap_->getResolution() + half_cell_size;
    wy = mask_costmap_->getOriginY() +
      mask_costmap_->getSizeInCellsY() * mask_costmap_->getResolution() + half_cell_size;
    master_grid.worldToMapNoBounds(wx, wy, mg_max_x, mg_max_y);

    if (mg_max_x <= min_i || mg_max_y <= min_j) {

      return;
    }
    mg_max_x = std::min(max_i, mg_max_x);
    mg_max_y = std::min(max_j, mg_max_y);
  }


  unsigned const int mg_min_x_u = static_cast<unsigned int>(mg_min_x);
  unsigned const int mg_min_y_u = static_cast<unsigned int>(mg_min_y);
  unsigned const int mg_max_x_u = static_cast<unsigned int>(mg_max_x);
  unsigned const int mg_max_y_u = static_cast<unsigned int>(mg_max_y);

  unsigned int i, j;
  unsigned int index;
  double gl_wx, gl_wy;
  double msk_wx, msk_wy;
  unsigned int mx, my;
  unsigned char data, old_data;



  unsigned char * master_array = master_grid.getCharMap();
  for (i = mg_min_x_u; i < mg_max_x_u; i++) {
    for (j = mg_min_y_u; j < mg_max_y_u; j++) {
      index = master_grid.getIndex(i, j);
      old_data = master_array[index];


      master_grid.mapToWorld(i, j, gl_wx, gl_wy);
      if (mask_frame_ != global_frame_) {

        tf2::Vector3 point(gl_wx, gl_wy, 0);
        point = tf2_transform * point;
        msk_wx = point.x();
        msk_wy = point.y();
      } else {

        msk_wx = gl_wx;
        msk_wy = gl_wy;
      }

      if (mask_costmap_->worldToMap(msk_wx, msk_wy, mx, my)) {
        data = mask_costmap_->getCost(mx, my);

        if (data == NO_INFORMATION) {
          continue;
        }
        if (data > old_data || old_data == NO_INFORMATION) {
          master_array[index] = data;
        }
      }
    }
  }
}

void KeepoutFilter::resetFilter()
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  filter_info_sub_.reset();
  mask_sub_.reset();
}

bool KeepoutFilter::isActive()
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  if (mask_costmap_) {
    return true;
  }
  return false;
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::KeepoutFilter, nav2_costmap_2d::Layer)
