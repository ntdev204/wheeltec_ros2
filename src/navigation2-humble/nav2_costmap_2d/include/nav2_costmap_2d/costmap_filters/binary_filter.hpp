


#ifndef NAV2_COSTMAP_2D__COSTMAP_FILTERS__BINARY_FILTER_HPP_
#define NAV2_COSTMAP_2D__COSTMAP_FILTERS__BINARY_FILTER_HPP_

#include <memory>
#include <string>

#include "nav2_costmap_2d/costmap_filters/costmap_filter.hpp"

#include "std_msgs/msg/bool.hpp"
#include "nav2_msgs/msg/costmap_filter_info.hpp"

namespace nav2_costmap_2d
{


class BinaryFilter : public CostmapFilter
{
public:
  

  BinaryFilter();

  

  void initializeFilter(
    const std::string & filter_info_topic);

  

  void process(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j,
    const geometry_msgs::msg::Pose2D & pose);

  

  void resetFilter();

  

  bool isActive();

private:
  

  void filterInfoCallback(const nav2_msgs::msg::CostmapFilterInfo::SharedPtr msg);
  

  void maskCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  

  void changeState(const bool state);


  rclcpp::Subscription<nav2_msgs::msg::CostmapFilterInfo>::SharedPtr filter_info_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mask_sub_;

  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr binary_state_pub_;

  nav_msgs::msg::OccupancyGrid::SharedPtr filter_mask_;

  std::string mask_frame_;
  std::string global_frame_;

  double base_, multiplier_;


  double flip_threshold_;

  bool default_state_;
  bool binary_state_;
};

}

#endif
