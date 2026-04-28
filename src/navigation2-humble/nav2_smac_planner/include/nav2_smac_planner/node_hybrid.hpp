













#ifndef NAV2_SMAC_PLANNER__NODE_HYBRID_HPP_
#define NAV2_SMAC_PLANNER__NODE_HYBRID_HPP_

#include <math.h>
#include <vector>
#include <cmath>
#include <iostream>
#include <functional>
#include <queue>
#include <memory>
#include <utility>
#include <limits>

#include "ompl/base/StateSpace.h"

#include "nav2_smac_planner/constants.hpp"
#include "nav2_smac_planner/types.hpp"
#include "nav2_smac_planner/collision_checker.hpp"
#include "nav2_smac_planner/costmap_downsampler.hpp"

namespace nav2_smac_planner
{

typedef std::vector<float> LookupTable;
typedef std::pair<double, double> TrigValues;

typedef std::pair<float, unsigned int> ObstacleHeuristicElement;
struct ObstacleHeuristicComparator
{
  bool operator()(const ObstacleHeuristicElement & a, const ObstacleHeuristicElement & b) const
  {
    return a.first > b.first;
  }
};

typedef std::vector<ObstacleHeuristicElement> ObstacleHeuristicQueue;


class NodeHybrid;



struct HybridMotionTable
{
  

  HybridMotionTable() {}

  

  void initDubin(
    unsigned int & size_x_in,
    unsigned int & size_y_in,
    unsigned int & angle_quantization_in,
    SearchInfo & search_info);

  

  void initReedsShepp(
    unsigned int & size_x_in,
    unsigned int & size_y_in,
    unsigned int & angle_quantization_in,
    SearchInfo & search_info);

  

  MotionPoses getProjections(const NodeHybrid * node);

  

  unsigned int getClosestAngularBin(const double & theta);

  

  float getAngleFromBin(const unsigned int & bin_idx);

  MotionModel motion_model = MotionModel::UNKNOWN;
  MotionPoses projections;
  unsigned int size_x;
  unsigned int num_angle_quantization;
  float num_angle_quantization_float;
  float min_turning_radius;
  float bin_size;
  float change_penalty;
  float non_straight_penalty;
  float cost_penalty;
  float reverse_penalty;
  float travel_distance_reward;
  ompl::base::StateSpacePtr state_space;
  std::vector<std::vector<double>> delta_xs;
  std::vector<std::vector<double>> delta_ys;
  std::vector<TrigValues> trig_values;
};



class NodeHybrid
{
public:
  typedef NodeHybrid * NodePtr;
  typedef std::unique_ptr<std::vector<NodeHybrid>> Graph;
  typedef std::vector<NodePtr> NodeVector;

  

  struct Coordinates
  {
    

    Coordinates() {}

    

    Coordinates(const float & x_in, const float & y_in, const float & theta_in)
    : x(x_in), y(y_in), theta(theta_in)
    {}

    inline bool operator==(const Coordinates & rhs)
    {
      return this->x == rhs.x && this->y == rhs.y && this->theta == rhs.theta;
    }

    inline bool operator!=(const Coordinates & rhs)
    {
      return !(*this == rhs);
    }

    float x, y, theta;
  };

  typedef std::vector<Coordinates> CoordinateVector;

  

  explicit NodeHybrid(const unsigned int index);

  

  ~NodeHybrid();

  

  bool operator==(const NodeHybrid & rhs)
  {
    return this->_index == rhs._index;
  }

  

  inline void setPose(const Coordinates & pose_in)
  {
    pose = pose_in;
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

  

  inline void setMotionPrimitiveIndex(const unsigned int & idx)
  {
    _motion_primitive_index = idx;
  }

  

  inline unsigned int & getMotionPrimitiveIndex()
  {
    return _motion_primitive_index;
  }

  

  inline float & getCost()
  {
    return _cell_cost;
  }

  

  inline bool & wasVisited()
  {
    return _was_visited;
  }

  

  inline void visited()
  {
    _was_visited = true;
  }

  

  inline unsigned int & getIndex()
  {
    return _index;
  }

  

  bool isNodeValid(const bool & traverse_unknown, GridCollisionChecker * collision_checker);

  

  float getTraversalCost(const NodePtr & child);

  

  static inline unsigned int getIndex(
    const unsigned int & x, const unsigned int & y, const unsigned int & angle,
    const unsigned int & width, const unsigned int & angle_quantization)
  {
    return angle + x * angle_quantization + y * width * angle_quantization;
  }

  

  static inline unsigned int getIndex(
    const unsigned int & x, const unsigned int & y, const unsigned int & angle)
  {
    return getIndex(
      x, y, angle, motion_table.size_x,
      motion_table.num_angle_quantization);
  }

  

  static inline Coordinates getCoords(
    const unsigned int & index,
    const unsigned int & width, const unsigned int & angle_quantization)
  {
    return Coordinates(
      (index / angle_quantization) % width,
      index / (angle_quantization * width),
      index % angle_quantization);
  }

  

  static float getHeuristicCost(
    const Coordinates & node_coords,
    const Coordinates & goal_coordinates,
    const nav2_costmap_2d::Costmap2D * costmap);

  

  static void initMotionModel(
    const MotionModel & motion_model,
    unsigned int & size_x,
    unsigned int & size_y,
    unsigned int & angle_quantization,
    SearchInfo & search_info);

  

  static void precomputeDistanceHeuristic(
    const float & lookup_table_dim,
    const MotionModel & motion_model,
    const unsigned int & dim_3_size,
    const SearchInfo & search_info);

  

  static float getObstacleHeuristic(
    const Coordinates & node_coords,
    const Coordinates & goal_coords,
    const double & cost_penalty);

  

  static float getDistanceHeuristic(
    const Coordinates & node_coords,
    const Coordinates & goal_coords,
    const float & obstacle_heuristic);

  

  static void resetObstacleHeuristic(
    nav2_costmap_2d::Costmap2D * costmap,
    const unsigned int & start_x, const unsigned int & start_y,
    const unsigned int & goal_x, const unsigned int & goal_y);

  

  void getNeighbors(
    std::function<bool(const unsigned int &, nav2_smac_planner::NodeHybrid * &)> & validity_checker,
    GridCollisionChecker * collision_checker,
    const bool & traverse_unknown,
    NodeVector & neighbors);

  

  bool backtracePath(CoordinateVector & path);

  NodeHybrid * parent;
  Coordinates pose;


  static double travel_distance_cost;
  static HybridMotionTable motion_table;

  static LookupTable obstacle_heuristic_lookup_table;
  static ObstacleHeuristicQueue obstacle_heuristic_queue;

  static nav2_costmap_2d::Costmap2D * sampled_costmap;
  static CostmapDownsampler downsampler;

  static LookupTable dist_heuristic_lookup_table;
  static float size_lookup;

private:
  float _cell_cost;
  float _accumulated_cost;
  unsigned int _index;
  bool _was_visited;
  unsigned int _motion_primitive_index;
};

}

#endif
