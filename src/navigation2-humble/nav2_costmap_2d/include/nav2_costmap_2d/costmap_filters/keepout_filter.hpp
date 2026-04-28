


#ifndef NAV2_COSTMAP_2D__COSTMAP_FILTERS__KEEPOUT_FILTER_HPP_
#define NAV2_COSTMAP_2D__COSTMAP_FILTERS__KEEPOUT_FILTER_HPP_

#include <string>
#include <memory>

#include "nav2_costmap_2d/costmap_filters/costmap_filter.hpp"

#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/msg/costmap_filter_info.hpp"

namespace nav2_costmap_2d
{



class KeepoutFilter : public CostmapFilter
{
public:
  

  KeepoutFilter();

  

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

  std::unique_ptr<Costmap2D> mask_costmap_;

  std::string mask_frame_;
  std::string global_frame_;
};

}

#endif
