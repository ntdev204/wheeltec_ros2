


#ifndef COSTMAP_QUEUE__LIMITED_COSTMAP_QUEUE_HPP_
#define COSTMAP_QUEUE__LIMITED_COSTMAP_QUEUE_HPP_

#include "costmap_queue/costmap_queue.hpp"

namespace costmap_queue
{



class LimitedCostmapQueue : public CostmapQueue
{
public:
  

  LimitedCostmapQueue(nav2_costmap_2d::Costmap2D & costmap, const int cell_distance_limit);
  bool validCellToQueue(const CellData & cell) override;
};
}

#endif
