














#include <math.h>
#include <chrono>
#include <vector>
#include <memory>
#include <algorithm>
#include <queue>
#include <limits>
#include <utility>

#include "ompl/base/ScopedState.h"
#include "ompl/base/spaces/DubinsStateSpace.h"
#include "ompl/base/spaces/ReedsSheppStateSpace.h"

#include "nav2_smac_planner/node_hybrid.hpp"

using namespace std::chrono;

namespace nav2_smac_planner
{


LookupTable NodeHybrid::obstacle_heuristic_lookup_table;
double NodeHybrid::travel_distance_cost = sqrt(2);
HybridMotionTable NodeHybrid::motion_table;
float NodeHybrid::size_lookup = 25;
LookupTable NodeHybrid::dist_heuristic_lookup_table;
nav2_costmap_2d::Costmap2D * NodeHybrid::sampled_costmap = nullptr;
CostmapDownsampler NodeHybrid::downsampler;
ObstacleHeuristicQueue NodeHybrid::obstacle_heuristic_queue;










void HybridMotionTable::initDubin(
  unsigned int & size_x_in,
  unsigned int & ,
  unsigned int & num_angle_quantization_in,
  SearchInfo & search_info)
{
  size_x = size_x_in;
  change_penalty = search_info.change_penalty;
  non_straight_penalty = search_info.non_straight_penalty;
  cost_penalty = search_info.cost_penalty;
  reverse_penalty = search_info.reverse_penalty;
  travel_distance_reward = 1.0f - search_info.retrospective_penalty;


  if (num_angle_quantization_in == num_angle_quantization &&
    min_turning_radius == search_info.minimum_turning_radius &&
    motion_model == MotionModel::DUBIN)
  {
    return;
  }

  num_angle_quantization = num_angle_quantization_in;
  num_angle_quantization_float = static_cast<float>(num_angle_quantization);
  min_turning_radius = search_info.minimum_turning_radius;
  motion_model = MotionModel::DUBIN;











  float angle = 2.0 * asin(sqrt(2.0) / (2 * min_turning_radius));


  bin_size =
    2.0f * static_cast<float>(M_PI) / static_cast<float>(num_angle_quantization);
  float increments;
  if (angle < bin_size) {
    increments = 1.0f;
  } else {


    increments = ceil(angle / bin_size);
  }
  angle = increments * bin_size;




  float delta_x = min_turning_radius * sin(angle);


  float delta_y = min_turning_radius - (min_turning_radius * cos(angle));

  projections.clear();
  projections.reserve(3);
  projections.emplace_back(hypotf(delta_x, delta_y), 0.0, 0.0);
  projections.emplace_back(delta_x, delta_y, increments);
  projections.emplace_back(delta_x, -delta_y, -increments);


  state_space = std::make_unique<ompl::base::DubinsStateSpace>(min_turning_radius);


  delta_xs.resize(projections.size());
  delta_ys.resize(projections.size());
  trig_values.resize(num_angle_quantization);

  for (unsigned int i = 0; i != projections.size(); i++) {
    delta_xs[i].resize(num_angle_quantization);
    delta_ys[i].resize(num_angle_quantization);

    for (unsigned int j = 0; j != num_angle_quantization; j++) {
      double cos_theta = cos(bin_size * j);
      double sin_theta = sin(bin_size * j);
      if (i == 0) {

        trig_values[j] = {cos_theta, sin_theta};
      }
      delta_xs[i][j] = projections[i]._x * cos_theta - projections[i]._y * sin_theta;
      delta_ys[i][j] = projections[i]._x * sin_theta + projections[i]._y * cos_theta;
    }
  }
}




void HybridMotionTable::initReedsShepp(
  unsigned int & size_x_in,
  unsigned int & ,
  unsigned int & num_angle_quantization_in,
  SearchInfo & search_info)
{
  size_x = size_x_in;
  change_penalty = search_info.change_penalty;
  non_straight_penalty = search_info.non_straight_penalty;
  cost_penalty = search_info.cost_penalty;
  reverse_penalty = search_info.reverse_penalty;
  travel_distance_reward = 1.0f - search_info.retrospective_penalty;


  if (num_angle_quantization_in == num_angle_quantization &&
    min_turning_radius == search_info.minimum_turning_radius &&
    motion_model == MotionModel::REEDS_SHEPP)
  {
    return;
  }

  num_angle_quantization = num_angle_quantization_in;
  num_angle_quantization_float = static_cast<float>(num_angle_quantization);
  min_turning_radius = search_info.minimum_turning_radius;
  motion_model = MotionModel::REEDS_SHEPP;

  float angle = 2.0 * asin(sqrt(2.0) / (2 * min_turning_radius));
  bin_size =
    2.0f * static_cast<float>(M_PI) / static_cast<float>(num_angle_quantization);
  float increments;
  if (angle < bin_size) {
    increments = 1.0f;
  } else {
    increments = ceil(angle / bin_size);
  }
  angle = increments * bin_size;

  float delta_x = min_turning_radius * sin(angle);
  float delta_y = min_turning_radius - (min_turning_radius * cos(angle));

  projections.clear();
  projections.reserve(6);
  projections.emplace_back(hypotf(delta_x, delta_y), 0.0, 0.0);
  projections.emplace_back(delta_x, delta_y, increments);
  projections.emplace_back(delta_x, -delta_y, -increments);
  projections.emplace_back(-hypotf(delta_x, delta_y), 0.0, 0.0);
  projections.emplace_back(-delta_x, delta_y, -increments);
  projections.emplace_back(-delta_x, -delta_y, increments);


  state_space = std::make_unique<ompl::base::ReedsSheppStateSpace>(min_turning_radius);


  delta_xs.resize(projections.size());
  delta_ys.resize(projections.size());
  trig_values.resize(num_angle_quantization);

  for (unsigned int i = 0; i != projections.size(); i++) {
    delta_xs[i].resize(num_angle_quantization);
    delta_ys[i].resize(num_angle_quantization);

    for (unsigned int j = 0; j != num_angle_quantization; j++) {
      double cos_theta = cos(bin_size * j);
      double sin_theta = sin(bin_size * j);
      if (i == 0) {

        trig_values[j] = {cos_theta, sin_theta};
      }
      delta_xs[i][j] = projections[i]._x * cos_theta - projections[i]._y * sin_theta;
      delta_ys[i][j] = projections[i]._x * sin_theta + projections[i]._y * cos_theta;
    }
  }
}

MotionPoses HybridMotionTable::getProjections(const NodeHybrid * node)
{
  MotionPoses projection_list;
  projection_list.reserve(projections.size());

  for (unsigned int i = 0; i != projections.size(); i++) {
    const MotionPose & motion_model = projections[i];


    const float & node_heading = node->pose.theta;
    float new_heading = node_heading + motion_model._theta;

    if (new_heading < 0.0) {
      new_heading += num_angle_quantization_float;
    }

    if (new_heading >= num_angle_quantization_float) {
      new_heading -= num_angle_quantization_float;
    }

    projection_list.emplace_back(
      delta_xs[i][node_heading] + node->pose.x,
      delta_ys[i][node_heading] + node->pose.y,
      new_heading);
  }

  return projection_list;
}

unsigned int HybridMotionTable::getClosestAngularBin(const double & theta)
{
  return static_cast<unsigned int>(floor(theta / bin_size));
}

float HybridMotionTable::getAngleFromBin(const unsigned int & bin_idx)
{
  return bin_idx * bin_size;
}

NodeHybrid::NodeHybrid(const unsigned int index)
: parent(nullptr),
  pose(0.0f, 0.0f, 0.0f),
  _cell_cost(std::numeric_limits<float>::quiet_NaN()),
  _accumulated_cost(std::numeric_limits<float>::max()),
  _index(index),
  _was_visited(false),
  _motion_primitive_index(std::numeric_limits<unsigned int>::max())
{
}

NodeHybrid::~NodeHybrid()
{
  parent = nullptr;
}

void NodeHybrid::reset()
{
  parent = nullptr;
  _cell_cost = std::numeric_limits<float>::quiet_NaN();
  _accumulated_cost = std::numeric_limits<float>::max();
  _was_visited = false;
  _motion_primitive_index = std::numeric_limits<unsigned int>::max();
  pose.x = 0.0f;
  pose.y = 0.0f;
  pose.theta = 0.0f;
}

bool NodeHybrid::isNodeValid(
  const bool & traverse_unknown,
  GridCollisionChecker * collision_checker)
{
  if (collision_checker->inCollision(
      this->pose.x, this->pose.y, this->pose.theta , traverse_unknown))
  {
    return false;
  }

  _cell_cost = collision_checker->getCost();
  return true;
}

float NodeHybrid::getTraversalCost(const NodePtr & child)
{
  const float normalized_cost = child->getCost() / 252.0;
  if (std::isnan(normalized_cost)) {
    throw std::runtime_error(
            "Node attempted to get traversal "
            "cost without a known SE2 collision cost!");
  }


  if (getMotionPrimitiveIndex() == std::numeric_limits<unsigned int>::max()) {
    return NodeHybrid::travel_distance_cost;
  }

  float travel_cost = 0.0;
  float travel_cost_raw =
    NodeHybrid::travel_distance_cost *
    (motion_table.travel_distance_reward + motion_table.cost_penalty * normalized_cost);

  if (child->getMotionPrimitiveIndex() == 0 || child->getMotionPrimitiveIndex() == 3) {

    travel_cost = travel_cost_raw;
  } else {
    if (getMotionPrimitiveIndex() == child->getMotionPrimitiveIndex()) {

      travel_cost = travel_cost_raw * motion_table.non_straight_penalty;
    } else {

      travel_cost = travel_cost_raw *
        (motion_table.non_straight_penalty + motion_table.change_penalty);
    }
  }

  if (child->getMotionPrimitiveIndex() > 2) {

    travel_cost *= motion_table.reverse_penalty;
  }

  return travel_cost;
}

float NodeHybrid::getHeuristicCost(
  const Coordinates & node_coords,
  const Coordinates & goal_coords,
  const nav2_costmap_2d::Costmap2D * )
{
  const float obstacle_heuristic =
    getObstacleHeuristic(node_coords, goal_coords, motion_table.cost_penalty);
  const float dist_heuristic = getDistanceHeuristic(node_coords, goal_coords, obstacle_heuristic);
  return std::max(obstacle_heuristic, dist_heuristic);
}

void NodeHybrid::initMotionModel(
  const MotionModel & motion_model,
  unsigned int & size_x,
  unsigned int & size_y,
  unsigned int & num_angle_quantization,
  SearchInfo & search_info)
{

  switch (motion_model) {
    case MotionModel::DUBIN:
      motion_table.initDubin(size_x, size_y, num_angle_quantization, search_info);
      break;
    case MotionModel::REEDS_SHEPP:
      motion_table.initReedsShepp(size_x, size_y, num_angle_quantization, search_info);
      break;
    default:
      throw std::runtime_error(
              "Invalid motion model for Hybrid A*. Please select between"
              " Dubin (Ackermann forward only),"
              " Reeds-Shepp (Ackermann forward and back).");
  }

  travel_distance_cost = motion_table.projections[0]._x;
}

inline float distanceHeuristic2D(
  const unsigned int idx, const unsigned int size_x,
  const unsigned int target_x, const unsigned int target_y)
{
  int dx = static_cast<int>(idx % size_x) - static_cast<int>(target_x);
  int dy = static_cast<int>(idx / size_x) - static_cast<int>(target_y);
  return std::sqrt(dx * dx + dy * dy);
}

void NodeHybrid::resetObstacleHeuristic(
  nav2_costmap_2d::Costmap2D * costmap,
  const unsigned int & start_x, const unsigned int & start_y,
  const unsigned int & goal_x, const unsigned int & goal_y)
{




  std::weak_ptr<nav2_util::LifecycleNode> ptr;
  downsampler.on_configure(ptr, "fake_frame", "fake_topic", costmap, 2.0, true);
  downsampler.on_activate();
  sampled_costmap = downsampler.downsample(2.0);


  unsigned int size = sampled_costmap->getSizeInCellsX() * sampled_costmap->getSizeInCellsY();
  if (obstacle_heuristic_lookup_table.size() == size) {

    std::fill(
      obstacle_heuristic_lookup_table.begin(),
      obstacle_heuristic_lookup_table.end(), 0.0);
  } else {
    unsigned int obstacle_size = obstacle_heuristic_lookup_table.size();
    obstacle_heuristic_lookup_table.resize(size, 0.0);

    std::fill_n(
      obstacle_heuristic_lookup_table.begin(), obstacle_size, 0.0);
  }

  obstacle_heuristic_queue.clear();
  obstacle_heuristic_queue.reserve(
    sampled_costmap->getSizeInCellsX() * sampled_costmap->getSizeInCellsY());


  const unsigned int size_x = sampled_costmap->getSizeInCellsX();
  const unsigned int goal_index = floor(goal_y / 2.0) * size_x + floor(goal_x / 2.0);
  obstacle_heuristic_queue.emplace_back(
    distanceHeuristic2D(goal_index, size_x, start_x, start_y), goal_index);



  obstacle_heuristic_lookup_table[goal_index] = -0.00001f;
}

float NodeHybrid::getObstacleHeuristic(
  const Coordinates & node_coords,
  const Coordinates & goal_coords,
  const double & cost_penalty)
{

  const unsigned int size_x = sampled_costmap->getSizeInCellsX();

  const unsigned int start_y = floor(node_coords.y / 2.0);
  const unsigned int start_x = floor(node_coords.x / 2.0);
  const unsigned int start_index = start_y * size_x + start_x;
  const float & requested_node_cost = obstacle_heuristic_lookup_table[start_index];
  if (requested_node_cost > 0.0f) {

    return 2.0 * requested_node_cost;
  }









  for (auto & n : obstacle_heuristic_queue) {
    n.first = -obstacle_heuristic_lookup_table[n.second] +
      distanceHeuristic2D(n.second, size_x, start_x, start_y);
  }
  std::make_heap(
    obstacle_heuristic_queue.begin(), obstacle_heuristic_queue.end(),
    ObstacleHeuristicComparator{});

  const int size_x_int = static_cast<int>(size_x);
  const unsigned int size_y = sampled_costmap->getSizeInCellsY();
  const float sqrt_2 = sqrt(2);
  float c_cost, cost, travel_cost, new_cost, existing_cost;
  unsigned int idx, mx, my, mx_idx, my_idx;
  unsigned int new_idx = 0;

  const std::vector<int> neighborhood = {1, -1,
    size_x_int, -size_x_int,
    size_x_int + 1, size_x_int - 1,
    -size_x_int + 1, -size_x_int - 1};

  while (!obstacle_heuristic_queue.empty()) {
    idx = obstacle_heuristic_queue.front().second;
    std::pop_heap(
      obstacle_heuristic_queue.begin(), obstacle_heuristic_queue.end(),
      ObstacleHeuristicComparator{});
    obstacle_heuristic_queue.pop_back();
    c_cost = obstacle_heuristic_lookup_table[idx];
    if (c_cost > 0.0f) {


      continue;
    }
    c_cost = -c_cost;
    obstacle_heuristic_lookup_table[idx] = c_cost;

    my_idx = idx / size_x;
    mx_idx = idx - (my_idx * size_x);


    for (unsigned int i = 0; i != neighborhood.size(); i++) {
      new_idx = static_cast<unsigned int>(static_cast<int>(idx) + neighborhood[i]);


      if (new_idx < size_x * size_y) {
        cost = static_cast<float>(sampled_costmap->getCost(new_idx));
        if (cost >= INSCRIBED) {
          continue;
        }

        my = new_idx / size_x;
        mx = new_idx - (my * size_x);

        if (mx == 0 && mx_idx >= size_x - 1 || mx >= size_x - 1 && mx_idx == 0) {
          continue;
        }
        if (my == 0 && my_idx >= size_y - 1 || my >= size_y - 1 && my_idx == 0) {
          continue;
        }

        existing_cost = obstacle_heuristic_lookup_table[new_idx];
        if (existing_cost <= 0.0f) {
          travel_cost =
            ((i <= 3) ? 1.0f : sqrt_2) * (1.0f + (cost_penalty * cost / 252.0f));
          new_cost = c_cost + travel_cost;
          if (existing_cost == 0.0f || -existing_cost > new_cost) {

            obstacle_heuristic_lookup_table[new_idx] = -new_cost;
            obstacle_heuristic_queue.emplace_back(
              new_cost + distanceHeuristic2D(new_idx, size_x, start_x, start_y), new_idx);
            std::push_heap(
              obstacle_heuristic_queue.begin(), obstacle_heuristic_queue.end(),
              ObstacleHeuristicComparator{});
          }
        }
      }
    }

    if (idx == start_index) {
      break;
    }
  }



  return 2.0 * requested_node_cost;
}

float NodeHybrid::getDistanceHeuristic(
  const Coordinates & node_coords,
  const Coordinates & goal_coords,
  const float & obstacle_heuristic)
{







  const TrigValues & trig_vals = motion_table.trig_values[goal_coords.theta];
  const float cos_th = trig_vals.first;
  const float sin_th = -trig_vals.second;
  const float dx = node_coords.x - goal_coords.x;
  const float dy = node_coords.y - goal_coords.y;

  double dtheta_bin = node_coords.theta - goal_coords.theta;
  if (dtheta_bin < 0) {
    dtheta_bin += motion_table.num_angle_quantization;
  }
  if (dtheta_bin > motion_table.num_angle_quantization) {
    dtheta_bin -= motion_table.num_angle_quantization;
  }

  Coordinates node_coords_relative(
    round(dx * cos_th - dy * sin_th),
    round(dx * sin_th + dy * cos_th),
    round(dtheta_bin));




  float motion_heuristic = 0.0;
  const int floored_size = floor(size_lookup / 2.0);
  const int ceiling_size = ceil(size_lookup / 2.0);
  const float mirrored_relative_y = abs(node_coords_relative.y);
  if (abs(node_coords_relative.x) < floored_size && mirrored_relative_y < floored_size) {

    int theta_pos;
    if (node_coords_relative.y < 0.0) {
      theta_pos = motion_table.num_angle_quantization - node_coords_relative.theta;
    } else {
      theta_pos = node_coords_relative.theta;
    }
    const int x_pos = node_coords_relative.x + floored_size;
    const int y_pos = static_cast<int>(mirrored_relative_y);
    const int index =
      x_pos * ceiling_size * motion_table.num_angle_quantization +
      y_pos * motion_table.num_angle_quantization +
      theta_pos;
    motion_heuristic = dist_heuristic_lookup_table[index];
  } else if (obstacle_heuristic <= 0.0) {


    static ompl::base::ScopedState<> from(motion_table.state_space), to(motion_table.state_space);
    to[0] = goal_coords.x;
    to[1] = goal_coords.y;
    to[2] = goal_coords.theta * motion_table.num_angle_quantization;
    from[0] = node_coords.x;
    from[1] = node_coords.y;
    from[2] = node_coords.theta * motion_table.num_angle_quantization;
    motion_heuristic = motion_table.state_space->distance(from(), to());
  }

  return motion_heuristic;
}

void NodeHybrid::precomputeDistanceHeuristic(
  const float & lookup_table_dim,
  const MotionModel & motion_model,
  const unsigned int & dim_3_size,
  const SearchInfo & search_info)
{

  if (motion_model == MotionModel::DUBIN) {
    motion_table.state_space = std::make_unique<ompl::base::DubinsStateSpace>(
      search_info.minimum_turning_radius);
  } else if (motion_model == MotionModel::REEDS_SHEPP) {
    motion_table.state_space = std::make_unique<ompl::base::ReedsSheppStateSpace>(
      search_info.minimum_turning_radius);
  } else {
    throw std::runtime_error(
            "Node attempted to precompute distance heuristics "
            "with invalid motion model!");
  }

  ompl::base::ScopedState<> from(motion_table.state_space), to(motion_table.state_space);
  to[0] = 0.0;
  to[1] = 0.0;
  to[2] = 0.0;
  size_lookup = lookup_table_dim;
  float motion_heuristic = 0.0;
  unsigned int index = 0;
  int dim_3_size_int = static_cast<int>(dim_3_size);
  float angular_bin_size = 2 * M_PI / static_cast<float>(dim_3_size);






  dist_heuristic_lookup_table.resize(size_lookup * ceil(size_lookup / 2.0) * dim_3_size_int);
  for (float x = ceil(-size_lookup / 2.0); x <= floor(size_lookup / 2.0); x += 1.0) {
    for (float y = 0.0; y <= floor(size_lookup / 2.0); y += 1.0) {
      for (int heading = 0; heading != dim_3_size_int; heading++) {
        from[0] = x;
        from[1] = y;
        from[2] = heading * angular_bin_size;
        motion_heuristic = motion_table.state_space->distance(from(), to());
        dist_heuristic_lookup_table[index] = motion_heuristic;
        index++;
      }
    }
  }
}

void NodeHybrid::getNeighbors(
  std::function<bool(const unsigned int &, nav2_smac_planner::NodeHybrid * &)> & NeighborGetter,
  GridCollisionChecker * collision_checker,
  const bool & traverse_unknown,
  NodeVector & neighbors)
{
  unsigned int index = 0;
  NodePtr neighbor = nullptr;
  Coordinates initial_node_coords;
  const MotionPoses motion_projections = motion_table.getProjections(this);

  for (unsigned int i = 0; i != motion_projections.size(); i++) {
    index = NodeHybrid::getIndex(
      static_cast<unsigned int>(motion_projections[i]._x),
      static_cast<unsigned int>(motion_projections[i]._y),
      static_cast<unsigned int>(motion_projections[i]._theta),
      motion_table.size_x, motion_table.num_angle_quantization);

    if (NeighborGetter(index, neighbor) && !neighbor->wasVisited()) {


      initial_node_coords = neighbor->pose;
      neighbor->setPose(
        Coordinates(
          motion_projections[i]._x,
          motion_projections[i]._y,
          motion_projections[i]._theta));
      if (neighbor->isNodeValid(traverse_unknown, collision_checker)) {
        neighbor->setMotionPrimitiveIndex(i);
        neighbors.push_back(neighbor);
      } else {
        neighbor->setPose(initial_node_coords);
      }
    }
  }
}

bool NodeHybrid::backtracePath(CoordinateVector & path)
{
  if (!this->parent) {
    return false;
  }

  NodePtr current_node = this;

  while (current_node->parent) {
    path.push_back(current_node->pose);

    path.back().theta = NodeHybrid::motion_table.getAngleFromBin(path.back().theta);
    current_node = current_node->parent;
  }


  path.push_back(current_node->pose);

  path.back().theta = NodeHybrid::motion_table.getAngleFromBin(path.back().theta);

  return true;
}

}
