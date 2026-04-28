














#include "nav2_smac_planner/node_2d.hpp"

#include <vector>
#include <limits>

namespace nav2_smac_planner
{


std::vector<int> Node2D::_neighbors_grid_offsets;
float Node2D::cost_travel_multiplier = 2.0;

Node2D::Node2D(const unsigned int index)
: parent(nullptr),
  _cell_cost(std::numeric_limits<float>::quiet_NaN()),
  _accumulated_cost(std::numeric_limits<float>::max()),
  _index(index),
  _was_visited(false),
  _is_queued(false)
{
}

Node2D::~Node2D()
{
  parent = nullptr;
}

void Node2D::reset()
{
  parent = nullptr;
  _cell_cost = std::numeric_limits<float>::quiet_NaN();
  _accumulated_cost = std::numeric_limits<float>::max();
  _was_visited = false;
  _is_queued = false;
}

bool Node2D::isNodeValid(
  const bool & traverse_unknown,
  GridCollisionChecker * collision_checker)
{
  if (collision_checker->inCollision(this->getIndex(), traverse_unknown)) {
    return false;
  }

  _cell_cost = collision_checker->getCost();
  return true;
}

float Node2D::getTraversalCost(const NodePtr & child)
{
  float normalized_cost = child->getCost() / 252.0;
  const Coordinates A = getCoords(child->getIndex());
  const Coordinates B = getCoords(this->getIndex());
  const float & dx = A.x - B.x;
  const float & dy = A.y - B.y;
  static float sqrt_2 = sqrt(2);


  if ((dx * dx + dy * dy) > 1.05) {
    return sqrt_2 * (1.0 + cost_travel_multiplier * normalized_cost);
  }


  return 1.0 + cost_travel_multiplier * normalized_cost;
}

float Node2D::getHeuristicCost(
  const Coordinates & node_coords,
  const Coordinates & goal_coordinates,
  const nav2_costmap_2d::Costmap2D * )
{


  auto dx = goal_coordinates.x - node_coords.x;
  auto dy = goal_coordinates.y - node_coords.y;
  return std::sqrt(dx * dx + dy * dy);
}

void Node2D::initMotionModel(
  const MotionModel & motion_model,
  unsigned int & x_size_uint,
  unsigned int & ,
  unsigned int & ,
  SearchInfo & search_info)
{
  if (motion_model != MotionModel::TWOD) {
    throw std::runtime_error("Invalid motion model for 2D node.");
  }

  int x_size = static_cast<int>(x_size_uint);
  cost_travel_multiplier = search_info.cost_penalty;
  _neighbors_grid_offsets = {-1, +1, -x_size, +x_size, -x_size - 1,
    -x_size + 1, +x_size - 1, +x_size + 1};
}

void Node2D::getNeighbors(
  std::function<bool(const unsigned int &, nav2_smac_planner::Node2D * &)> & NeighborGetter,
  GridCollisionChecker * collision_checker,
  const bool & traverse_unknown,
  NodeVector & neighbors)
{











  int index;
  NodePtr neighbor;
  int node_i = this->getIndex();
  const Coordinates parent = getCoords(this->getIndex());
  Coordinates child;

  for (unsigned int i = 0; i != _neighbors_grid_offsets.size(); ++i) {
    index = node_i + _neighbors_grid_offsets[i];


    child = getCoords(index);
    if (fabs(parent.x - child.x) > 1 || fabs(parent.y - child.y) > 1) {
      continue;
    }

    if (NeighborGetter(index, neighbor)) {
      if (neighbor->isNodeValid(traverse_unknown, collision_checker) && !neighbor->wasVisited()) {
        neighbors.push_back(neighbor);
      }
    }
  }
}

bool Node2D::backtracePath(CoordinateVector & path)
{
  if (!this->parent) {
    return false;
  }

  NodePtr current_node = this;

  while (current_node->parent) {
    path.push_back(
      Node2D::getCoords(current_node->getIndex()));
    current_node = current_node->parent;
  }


  path.push_back(Node2D::getCoords(current_node->getIndex()));

  return true;
}

}
