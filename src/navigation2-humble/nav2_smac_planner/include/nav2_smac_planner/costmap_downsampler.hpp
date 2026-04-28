













#ifndef NAV2_SMAC_PLANNER__COSTMAP_DOWNSAMPLER_HPP_
#define NAV2_SMAC_PLANNER__COSTMAP_DOWNSAMPLER_HPP_

#include <algorithm>
#include <string>
#include <memory>

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_smac_planner/constants.hpp"

namespace nav2_smac_planner
{



class CostmapDownsampler
{
public:
  

  CostmapDownsampler();

  

  ~CostmapDownsampler();

  

  void on_configure(
    const nav2_util::LifecycleNode::WeakPtr & node,
    const std::string & global_frame,
    const std::string & topic_name,
    nav2_costmap_2d::Costmap2D * const costmap,
    const unsigned int & downsampling_factor,
    const bool & use_min_cost_neighbor = false);

  

  void on_activate();

  

  void on_deactivate();

  

  void on_cleanup();

  

  nav2_costmap_2d::Costmap2D * downsample(const unsigned int & downsampling_factor);

  

  void resizeCostmap();

protected:
  

  void updateCostmapSize();

  

  void setCostOfCell(
    const unsigned int & new_mx,
    const unsigned int & new_my);

  unsigned int _size_x;
  unsigned int _size_y;
  unsigned int _downsampled_size_x;
  unsigned int _downsampled_size_y;
  unsigned int _downsampling_factor;
  bool _use_min_cost_neighbor;
  float _downsampled_resolution;
  nav2_costmap_2d::Costmap2D * _costmap;
  std::unique_ptr<nav2_costmap_2d::Costmap2D> _downsampled_costmap;
  std::unique_ptr<nav2_costmap_2d::Costmap2DPublisher> _downsampled_costmap_pub;
};

}

#endif
