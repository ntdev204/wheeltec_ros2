













#ifndef NAV2_SMAC_PLANNER__NODE_2D_HPP_
#define NAV2_SMAC_PLANNER__NODE_2D_HPP_

#include <math.h>
#include <vector>
#include <iostream>
#include <memory>
#include <queue>
#include <limits>
#include <utility>
#include <functional>

#include "nav2_smac_planner/types.hpp"
#include "nav2_smac_planner/constants.hpp"
#include "nav2_smac_planner/collision_checker.hpp"
#include "nav2_smac_planner/node_hybrid.hpp"

namespace nav2_smac_planner
{



class Node2D
{
public:
  typedef Node2D * NodePtr;
  typedef std::unique_ptr<std::vector<Node2D>> Graph;
  typedef std::vector<NodePtr> NodeVector;

  

  struct Coordinates
  {
    Coordinates() {}
    Coordinates(const float & x_in, const float & y_in)
    : x(x_in), y(y_in)
    {}

    float x, y;
  };
  typedef std::vector<Coordinates> CoordinateVector;

  

  explicit Node2D(const unsigned int index);

  

  ~Node2D();

  

  bool operator==(const Node2D & rhs)
  {
    return this->_index == rhs._index;
  }

  

  void reset();
  

  inline float & getAccumulatedCost()
  {
    return _accumulated_cost;
  }

  

  inline void setAccumulatedCost(const float & cost_in)
  {
    _accumulated_cost = cost_in;
  }

  

  inline float & getCost()
  {
    return _cell_cost;
  }

  

  inline void setCost(const float & cost)
  {
    _cell_cost = cost;
  }

  

  inline bool & wasVisited()
  {
    return _was_visited;
  }

  

  inline void visited()
  {
    _was_visited = true;
    _is_queued = false;
  }

  

  inline bool & isQueued()
  {
    return _is_queued;
  }

  

  inline void queued()
  {
    _is_queued = true;
  }

  

  inline unsigned int & getIndex()
  {
    return _index;
  }

  

  bool isNodeValid(const bool & traverse_unknown, GridCollisionChecker * collision_checker);

  

  float getTraversalCost(const NodePtr & child);

  

  static inline unsigned int getIndex(
    const unsigned int & x, const unsigned int & y, const unsigned int & width)
  {
    return x + y * width;
  }

  

  static inline Coordinates getCoords(
    const unsigned int & index, const unsigned int & width, const unsigned int & angles)
  {
    if (angles != 1) {
      throw std::runtime_error("Node type Node2D does not have a valid angle quantization.");
    }

    return Coordinates(index % width, index / width);
  }

  

  static inline Coordinates getCoords(const unsigned int & index)
  {
    const unsigned int & size_x = _neighbors_grid_offsets[3];
    return Coordinates(index % size_x, index / size_x);
  }

  

  static float getHeuristicCost(
    const Coordinates & node_coords,
    const Coordinates & goal_coordinates,
    const nav2_costmap_2d::Costmap2D * costmap);

  

  static void initMotionModel(
    const MotionModel & motion_model,
    unsigned int & size_x,
    unsigned int & size_y,
    unsigned int & num_angle_quantization,
    SearchInfo & search_info);

  

  void getNeighbors(
    std::function<bool(const unsigned int &, nav2_smac_planner::Node2D * &)> & validity_checker,
    GridCollisionChecker * collision_checker,
    const bool & traverse_unknown,
    NodeVector & neighbors);

  

  bool backtracePath(CoordinateVector & path);

  Node2D * parent;
  static float cost_travel_multiplier;
  static std::vector<int> _neighbors_grid_offsets;

private:
  float _cell_cost;
  float _accumulated_cost;
  unsigned int _index;
  bool _was_visited;
  bool _is_queued;
};

}

#endif
