













#ifndef NAV2_SMAC_PLANNER__NODE_LATTICE_HPP_
#define NAV2_SMAC_PLANNER__NODE_LATTICE_HPP_

#include <math.h>

#include <vector>
#include <cmath>
#include <iostream>
#include <functional>
#include <queue>
#include <memory>
#include <utility>
#include <limits>
#include <string>

#include "nlohmann/json.hpp"
#include "ompl/base/StateSpace.h"
#include "angles/angles.h"

#include "nav2_smac_planner/constants.hpp"
#include "nav2_smac_planner/types.hpp"
#include "nav2_smac_planner/collision_checker.hpp"
#include "nav2_smac_planner/node_hybrid.hpp"
#include "nav2_smac_planner/utils.hpp"

namespace nav2_smac_planner
{


class NodeLattice;
class NodeHybrid;



struct LatticeMotionTable
{
  

  LatticeMotionTable() {}

  

  void initMotionModel(
    unsigned int & size_x_in,
    SearchInfo & search_info);

  

  MotionPrimitivePtrs getMotionPrimitives(const NodeLattice * node);

  

  static LatticeMetadata getLatticeMetadata(const std::string & lattice_filepath);

  

  unsigned int getClosestAngularBin(const double & theta);

  

  float & getAngleFromBin(const unsigned int & bin_idx);

  unsigned int size_x;
  unsigned int num_angle_quantization;
  float change_penalty;
  float non_straight_penalty;
  float cost_penalty;
  float reverse_penalty;
  float travel_distance_reward;
  float rotation_penalty;
  bool allow_reverse_expansion;
  std::vector<std::vector<MotionPrimitive>> motion_primitives;
  ompl::base::StateSpacePtr state_space;
  std::vector<TrigValues> trig_values;
  std::string current_lattice_filepath;
  LatticeMetadata lattice_metadata;
};



class NodeLattice
{
public:
  typedef NodeLattice * NodePtr;
  typedef std::unique_ptr<std::vector<NodeLattice>> Graph;
  typedef std::vector<NodePtr> NodeVector;
  typedef NodeHybrid::Coordinates Coordinates;
  typedef NodeHybrid::CoordinateVector CoordinateVector;

  

  explicit NodeLattice(const unsigned int index);

  

  ~NodeLattice();

  

  bool operator==(const NodeLattice & rhs)
  {
    return this->_index == rhs._index;
  }

  

  inline void setPose(const Coordinates & pose_in)
  {
    pose = pose_in;
  }

  

  void reset();

  

  inline void setMotionPrimitive(MotionPrimitive * prim)
  {
    _motion_primitive = prim;
  }

  

  inline MotionPrimitive * & getMotionPrimitive()
  {
    return _motion_primitive;
  }

  

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

  

  inline void backwards(bool back = true)
  {
    _backwards = back;
  }

  

  inline bool isBackward()
  {
    return _backwards;
  }

  

  bool isNodeValid(
    const bool & traverse_unknown,
    GridCollisionChecker * collision_checker,
    MotionPrimitive * primitive = nullptr,
    bool is_backwards = false);

  

  float getTraversalCost(const NodePtr & child);

  

  static inline unsigned int getIndex(
    const unsigned int & x, const unsigned int & y, const unsigned int & angle)
  {

    return NodeHybrid::getIndex(
      x, y, angle, motion_table.size_x,
      motion_table.num_angle_quantization);
  }

  

  static inline Coordinates getCoords(
    const unsigned int & index,
    const unsigned int & width, const unsigned int & angle_quantization)
  {

    return NodeHybrid::Coordinates(
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

  

  static void resetObstacleHeuristic(
    nav2_costmap_2d::Costmap2D * costmap,
    const unsigned int & start_x, const unsigned int & start_y,
    const unsigned int & goal_x, const unsigned int & goal_y)
  {

    NodeHybrid::resetObstacleHeuristic(costmap, start_x, start_y, goal_x, goal_y);
  }

  

  static float getObstacleHeuristic(
    const Coordinates & node_coords,
    const Coordinates & goal_coords,
    const double & cost_penalty)
  {
    return NodeHybrid::getObstacleHeuristic(node_coords, goal_coords, cost_penalty);
  }

  

  static float getDistanceHeuristic(
    const Coordinates & node_coords,
    const Coordinates & goal_coords,
    const float & obstacle_heuristic);

  

  void getNeighbors(
    std::function<bool(const unsigned int &,
    nav2_smac_planner::NodeLattice * &)> & validity_checker,
    GridCollisionChecker * collision_checker,
    const bool & traverse_unknown,
    NodeVector & neighbors);

  

  bool backtracePath(CoordinateVector & path);

  

  void addNodeToPath(NodePtr current_node, CoordinateVector & path);

  NodeLattice * parent;
  Coordinates pose;
  static LatticeMotionTable motion_table;

  static LookupTable dist_heuristic_lookup_table;
  static float size_lookup;

private:
  float _cell_cost;
  float _accumulated_cost;
  unsigned int _index;
  bool _was_visited;
  MotionPrimitive * _motion_primitive;
  bool _backwards;
};

}

#endif
