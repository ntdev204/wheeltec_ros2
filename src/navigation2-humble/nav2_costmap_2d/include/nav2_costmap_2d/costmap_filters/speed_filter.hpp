


#ifndef NAV2_COSTMAP_2D__COSTMAP_FILTERS__SPEED_FILTER_HPP_
#define NAV2_COSTMAP_2D__COSTMAP_FILTERS__SPEED_FILTER_HPP_

#include <memory>
#include <string>

#include "nav2_costmap_2d/costmap_filters/costmap_filter.hpp"

#include "nav2_msgs/msg/costmap_filter_info.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"

namespace nav2_costmap_2d
{


class SpeedFilter : public CostmapFilter
{
public:
  

  SpeedFilter();

  

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

  rclcpp::Subscription<nav2_msgs::msg::CostmapFilterInfo>::SharedPtr filter_info_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mask_sub_;

  rclcpp_lifecycle::LifecyclePublisher<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_limit_pub_;

  nav_msgs::msg::OccupancyGrid::SharedPtr filter_mask_;

  std::string mask_frame_;
  std::string global_frame_;

  double base_, multiplier_;
  bool percentage_;
  double speed_limit_, speed_limit_prev_;
};

}

#endif
