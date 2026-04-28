


#ifndef COSTMAP_QUEUE__COSTMAP_QUEUE_HPP_
#define COSTMAP_QUEUE__COSTMAP_QUEUE_HPP_

#include <cmath>
#include <vector>
#include <limits>
#include <memory>
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "costmap_queue/map_based_queue.hpp"

namespace costmap_queue
{


class CellData
{
public:
  

  CellData(
    const double d, const unsigned int i, const unsigned int x, const unsigned int y,
    const unsigned int sx, const unsigned int sy)
  : distance_(d), index_(i), x_(x), y_(y), src_x_(sx), src_y_(sy)
  {
  }

  

  CellData()
  : distance_(std::numeric_limits<double>::max()), index_(0), x_(0), y_(0), src_x_(0), src_y_(0)
  {
  }

  static unsigned absolute_difference(const unsigned x, const unsigned y)
  {
    return (x > y) ? (x - y) : (y - x);
  }

  double distance_;
  unsigned int index_;
  unsigned int x_, y_;
  unsigned int src_x_, src_y_;
};



class CostmapQueue : public MapBasedQueue<CellData>
{
public:
  

  explicit CostmapQueue(nav2_costmap_2d::Costmap2D & costmap, bool manhattan = false);

  

  void reset() override;

  

  void enqueueCell(unsigned int x, unsigned int y);

  

  CellData getNextCell();

  

  virtual bool validCellToQueue(const CellData & ) {return true;}
  

  typedef std::shared_ptr<CostmapQueue> Ptr;

protected:
  

  void enqueueCell(
    unsigned int index, unsigned int cur_x, unsigned int cur_y, unsigned int src_x,
    unsigned int src_y);

  

  void computeCache();

  nav2_costmap_2d::Costmap2D & costmap_;
  std::vector<bool> seen_;
  int max_distance_;
  bool manhattan_;

protected:
  

  inline double distanceLookup(
    const unsigned int cur_x, const unsigned int cur_y,
    const unsigned int src_x, const unsigned int src_y)
  {
    unsigned int dx = CellData::absolute_difference(cur_x, src_x);
    unsigned int dy = CellData::absolute_difference(cur_y, src_y);
    return cached_distances_[dx][dy];
  }
  std::vector<std::vector<double>> cached_distances_;
  int cached_max_distance_;
};
}

#endif
