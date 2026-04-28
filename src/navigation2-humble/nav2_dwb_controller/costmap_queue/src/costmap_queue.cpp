

#include "costmap_queue/costmap_queue.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

using std::hypot;

namespace costmap_queue
{

CostmapQueue::CostmapQueue(nav2_costmap_2d::Costmap2D & costmap, bool manhattan)
: MapBasedQueue(), costmap_(costmap), max_distance_(-1), manhattan_(manhattan),
  cached_max_distance_(-1)
{
  reset();
}

void CostmapQueue::reset()
{
  unsigned int size_x = costmap_.getSizeInCellsX(), size_y = costmap_.getSizeInCellsY();
  if (seen_.size() != size_x * size_y) {
    seen_.resize(size_x * size_y);
  }
  std::fill(seen_.begin(), seen_.end(), false);
  computeCache();
  MapBasedQueue::reset();
}

void CostmapQueue::enqueueCell(unsigned int x, unsigned int y)
{
  unsigned int index = costmap_.getIndex(x, y);
  enqueueCell(index, x, y, x, y);
}

void CostmapQueue::enqueueCell(
  unsigned int index, unsigned int cur_x, unsigned int cur_y,
  unsigned int src_x, unsigned int src_y)
{
  if (seen_[index]) {return;}



  double distance = distanceLookup(cur_x, cur_y, src_x, src_y);
  CellData data(distance, index, cur_x, cur_y, src_x, src_y);
  if (validCellToQueue(data)) {
    seen_[index] = true;
    enqueue(distance, data);
  }
}

CellData CostmapQueue::getNextCell()
{

  CellData current_cell = front();
  pop();

  unsigned int index = current_cell.index_;
  unsigned int mx = current_cell.x_;
  unsigned int my = current_cell.y_;
  unsigned int sx = current_cell.src_x_;
  unsigned int sy = current_cell.src_y_;


  unsigned int size_x = costmap_.getSizeInCellsX();
  if (mx > 0) {
    enqueueCell(index - 1, mx - 1, my, sx, sy);
  }
  if (my > 0) {
    enqueueCell(index - size_x, mx, my - 1, sx, sy);
  }
  if (mx < size_x - 1) {
    enqueueCell(index + 1, mx + 1, my, sx, sy);
  }
  if (my < costmap_.getSizeInCellsY() - 1) {
    enqueueCell(index + size_x, mx, my + 1, sx, sy);
  }

  return current_cell;
}

void CostmapQueue::computeCache()
{
  if (max_distance_ == -1) {
    max_distance_ = std::max(costmap_.getSizeInCellsX(), costmap_.getSizeInCellsY());
  }
  if (max_distance_ == cached_max_distance_) {return;}
  cached_distances_.clear();

  cached_distances_.resize(max_distance_ + 2);

  for (unsigned int i = 0; i < cached_distances_.size(); ++i) {
    cached_distances_[i].resize(max_distance_ + 2);
    for (unsigned int j = 0; j < cached_distances_[i].size(); ++j) {
      if (manhattan_) {
        cached_distances_[i][j] = i + j;
      } else {
        cached_distances_[i][j] = hypot(i, j);
      }
    }
  }
  cached_max_distance_ = max_distance_;
}

}
