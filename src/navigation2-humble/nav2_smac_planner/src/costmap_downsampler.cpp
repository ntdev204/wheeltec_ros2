














#include "nav2_smac_planner/costmap_downsampler.hpp"

#include <string>
#include <memory>
#include <algorithm>

namespace nav2_smac_planner
{

CostmapDownsampler::CostmapDownsampler()
: _costmap(nullptr),
  _downsampled_costmap(nullptr),
  _downsampled_costmap_pub(nullptr)
{
}

CostmapDownsampler::~CostmapDownsampler()
{
}

void CostmapDownsampler::on_configure(
  const nav2_util::LifecycleNode::WeakPtr & node,
  const std::string & global_frame,
  const std::string & topic_name,
  nav2_costmap_2d::Costmap2D * const costmap,
  const unsigned int & downsampling_factor,
  const bool & use_min_cost_neighbor)
{
  _costmap = costmap;
  _downsampling_factor = downsampling_factor;
  _use_min_cost_neighbor = use_min_cost_neighbor;
  updateCostmapSize();

  _downsampled_costmap = std::make_unique<nav2_costmap_2d::Costmap2D>(
    _downsampled_size_x, _downsampled_size_y, _downsampled_resolution,
    _costmap->getOriginX(), _costmap->getOriginY(), UNKNOWN);

  if (!node.expired()) {
    _downsampled_costmap_pub = std::make_unique<nav2_costmap_2d::Costmap2DPublisher>(
      node, _downsampled_costmap.get(), global_frame, topic_name, false);
  }
}

void CostmapDownsampler::on_activate()
{
  if (_downsampled_costmap_pub) {
    _downsampled_costmap_pub->on_activate();
  }
}

void CostmapDownsampler::on_deactivate()
{
  if (_downsampled_costmap_pub) {
    _downsampled_costmap_pub->on_deactivate();
  }
}

void CostmapDownsampler::on_cleanup()
{
  _costmap = nullptr;
  _downsampled_costmap.reset();
  _downsampled_costmap_pub.reset();
}

nav2_costmap_2d::Costmap2D * CostmapDownsampler::downsample(
  const unsigned int & downsampling_factor)
{
  _downsampling_factor = downsampling_factor;
  updateCostmapSize();


  if (_downsampled_costmap->getSizeInCellsX() != _downsampled_size_x ||
    _downsampled_costmap->getSizeInCellsY() != _downsampled_size_y ||
    _downsampled_costmap->getResolution() != _downsampled_resolution)
  {
    resizeCostmap();
  }


  for (unsigned int i = 0; i < _downsampled_size_x; ++i) {
    for (unsigned int j = 0; j < _downsampled_size_y; ++j) {
      setCostOfCell(i, j);
    }
  }

  if (_downsampled_costmap_pub) {
    _downsampled_costmap_pub->publishCostmap();
  }
  return _downsampled_costmap.get();
}

void CostmapDownsampler::updateCostmapSize()
{
  _size_x = _costmap->getSizeInCellsX();
  _size_y = _costmap->getSizeInCellsY();
  _downsampled_size_x = ceil(static_cast<float>(_size_x) / _downsampling_factor);
  _downsampled_size_y = ceil(static_cast<float>(_size_y) / _downsampling_factor);
  _downsampled_resolution = _downsampling_factor * _costmap->getResolution();
}

void CostmapDownsampler::resizeCostmap()
{
  _downsampled_costmap->resizeMap(
    _downsampled_size_x,
    _downsampled_size_y,
    _downsampled_resolution,
    _costmap->getOriginX(),
    _costmap->getOriginY());
}

void CostmapDownsampler::setCostOfCell(
  const unsigned int & new_mx,
  const unsigned int & new_my)
{
  unsigned int mx, my;
  unsigned char cost = _use_min_cost_neighbor ? 255 : 0;
  unsigned int x_offset = new_mx * _downsampling_factor;
  unsigned int y_offset = new_my * _downsampling_factor;

  for (unsigned int i = 0; i < _downsampling_factor; ++i) {
    mx = x_offset + i;
    if (mx >= _size_x) {
      continue;
    }
    for (unsigned int j = 0; j < _downsampling_factor; ++j) {
      my = y_offset + j;
      if (my >= _size_y) {
        continue;
      }
      cost = _use_min_cost_neighbor ?
        std::min(cost, _costmap->getCost(mx, my)) :
        std::max(cost, _costmap->getCost(mx, my));
    }
  }

  _downsampled_costmap->setCost(new_mx, new_my, cost);
}

}
