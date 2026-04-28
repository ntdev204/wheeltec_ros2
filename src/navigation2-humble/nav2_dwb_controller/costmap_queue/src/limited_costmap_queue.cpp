


#include "costmap_queue/limited_costmap_queue.hpp"

namespace costmap_queue
{

LimitedCostmapQueue::LimitedCostmapQueue(
  nav2_costmap_2d::Costmap2D & costmap,
  const int distance_limit)
: CostmapQueue(costmap)
{
  max_distance_ = distance_limit;
  reset();
}

bool LimitedCostmapQueue::validCellToQueue(const CellData & cell)
{
  return cell.distance_ <= max_distance_;
}

}
