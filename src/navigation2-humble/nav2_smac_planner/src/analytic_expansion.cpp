













#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>

#include <algorithm>
#include <vector>
#include <memory>

#include "nav2_smac_planner/analytic_expansion.hpp"

namespace nav2_smac_planner
{

template<typename NodeT>
AnalyticExpansion<NodeT>::AnalyticExpansion(
  const MotionModel & motion_model,
  const SearchInfo & search_info,
  const bool & traverse_unknown,
  const unsigned int & dim_3_size)
: _motion_model(motion_model),
  _search_info(search_info),
  _traverse_unknown(traverse_unknown),
  _dim_3_size(dim_3_size),
  _collision_checker(nullptr)
{
}

template<typename NodeT>
void AnalyticExpansion<NodeT>::setCollisionChecker(
  GridCollisionChecker * collision_checker)
{
  _collision_checker = collision_checker;
}

template<typename NodeT>
typename AnalyticExpansion<NodeT>::NodePtr AnalyticExpansion<NodeT>::tryAnalyticExpansion(
  const NodePtr & current_node, const NodePtr & goal_node,
  const NodeGetter & getter, int & analytic_iterations,
  int & closest_distance)
{

  if (_motion_model == MotionModel::DUBIN || _motion_model == MotionModel::REEDS_SHEPP ||
    _motion_model == MotionModel::STATE_LATTICE)
  {

    auto costmap = _collision_checker->getCostmap();
    const Coordinates node_coords =
      NodeT::getCoords(current_node->getIndex(), costmap->getSizeInCellsX(), _dim_3_size);
    closest_distance = std::min(
      closest_distance,
      static_cast<int>(NodeT::getHeuristicCost(node_coords, goal_node->pose, costmap)));




    int desired_iterations = std::max(
      static_cast<int>(closest_distance / _search_info.analytic_expansion_ratio),
      static_cast<int>(std::ceil(_search_info.analytic_expansion_ratio)));


    analytic_iterations =
      std::min(analytic_iterations, desired_iterations);



    if (analytic_iterations <= 0) {

      analytic_iterations = desired_iterations;
      AnalyticExpansionNodes analytic_nodes = getAnalyticPath(current_node, goal_node, getter);
      if (!analytic_nodes.empty()) {

        NodePtr node = current_node;
        NodePtr test_node = current_node;
        AnalyticExpansionNodes refined_analytic_nodes;
        for (int i = 0; i < 8; i++) {


          if (test_node->parent && test_node->parent->parent && test_node->parent->parent->parent &&
            test_node->parent->parent->parent->parent &&
            test_node->parent->parent->parent->parent->parent)
          {
            test_node = test_node->parent->parent->parent->parent->parent;
            refined_analytic_nodes = getAnalyticPath(test_node, goal_node, getter);
            if (refined_analytic_nodes.empty()) {
              break;
            }
            analytic_nodes = refined_analytic_nodes;
            node = test_node;
          } else {
            break;
          }
        }

        return setAnalyticPath(node, goal_node, analytic_nodes);
      }
    }

    analytic_iterations--;
  }


  return NodePtr(nullptr);
}

template<typename NodeT>
typename AnalyticExpansion<NodeT>::AnalyticExpansionNodes AnalyticExpansion<NodeT>::getAnalyticPath(
  const NodePtr & node,
  const NodePtr & goal,
  const NodeGetter & node_getter)
{
  static ompl::base::ScopedState<> from(node->motion_table.state_space), to(
    node->motion_table.state_space), s(node->motion_table.state_space);
  from[0] = node->pose.x;
  from[1] = node->pose.y;
  from[2] = node->motion_table.getAngleFromBin(node->pose.theta);
  to[0] = goal->pose.x;
  to[1] = goal->pose.y;
  to[2] = node->motion_table.getAngleFromBin(goal->pose.theta);

  float d = node->motion_table.state_space->distance(from(), to());





  if (d > _search_info.analytic_expansion_max_length) {
    return AnalyticExpansionNodes();
  }


  static const float sqrt_2 = std::sqrt(2.);
  unsigned int num_intervals = std::floor(d / sqrt_2);

  AnalyticExpansionNodes possible_nodes;


  possible_nodes.reserve(num_intervals);
  std::vector<double> reals;
  double theta;


  NodePtr prev(node);
  unsigned int index = 0;
  NodePtr next(nullptr);
  float angle = 0.0;
  Coordinates proposed_coordinates;
  bool failure = false;


  for (float i = 1; i < num_intervals; i++) {
    node->motion_table.state_space->interpolate(from(), to(), i / num_intervals, s());
    reals = s.reals();

    theta = (reals[2] < 0.0) ? (reals[2] + 2.0 * M_PI) : reals[2];
    theta = (theta > 2.0 * M_PI) ? (theta - 2.0 * M_PI) : theta;
    angle = node->motion_table.getClosestAngularBin(theta);


    index = NodeT::getIndex(
      static_cast<unsigned int>(reals[0]),
      static_cast<unsigned int>(reals[1]),
      static_cast<unsigned int>(angle));

    if (node_getter(index, next)) {
      Coordinates initial_node_coords = next->pose;
      proposed_coordinates = {static_cast<float>(reals[0]), static_cast<float>(reals[1]), angle};
      next->setPose(proposed_coordinates);
      if (next->isNodeValid(_traverse_unknown, _collision_checker) && next != prev) {

        possible_nodes.emplace_back(next, initial_node_coords, proposed_coordinates);
        prev = next;
      } else {

        next->setPose(initial_node_coords);
        failure = true;
        break;
      }
    } else {

      failure = true;
      break;
    }
  }


  for (const auto & node_pose : possible_nodes) {
    const auto & n = node_pose.node;
    n->setPose(node_pose.initial_coords);
  }

  if (failure) {
    return AnalyticExpansionNodes();
  }

  return possible_nodes;
}

template<typename NodeT>
typename AnalyticExpansion<NodeT>::NodePtr AnalyticExpansion<NodeT>::setAnalyticPath(
  const NodePtr & node,
  const NodePtr & goal_node,
  const AnalyticExpansionNodes & expanded_nodes)
{
  _detached_nodes.clear();

  NodePtr prev = node;
  for (const auto & node_pose : expanded_nodes) {
    auto n = node_pose.node;
    cleanNode(n);
    if (n->getIndex() != goal_node->getIndex()) {
      if (n->wasVisited()) {
        _detached_nodes.push_back(std::make_unique<NodeT>(-1));
        n = _detached_nodes.back().get();
      }
      n->parent = prev;
      n->pose = node_pose.proposed_coords;
      n->visited();
      prev = n;
    }
  }
  if (goal_node != prev) {
    goal_node->parent = prev;
    cleanNode(goal_node);
    goal_node->visited();
  }
  return goal_node;
}

template<>
void AnalyticExpansion<NodeLattice>::cleanNode(const NodePtr & node)
{
  node->setMotionPrimitive(nullptr);
}

template<typename NodeT>
void AnalyticExpansion<NodeT>::cleanNode(const NodePtr & )
{
}

template<>
typename AnalyticExpansion<Node2D>::AnalyticExpansionNodes AnalyticExpansion<Node2D>::
getAnalyticPath(
  const NodePtr & node,
  const NodePtr & goal,
  const NodeGetter & node_getter)
{
  return AnalyticExpansionNodes();
}

template<>
typename AnalyticExpansion<Node2D>::NodePtr AnalyticExpansion<Node2D>::setAnalyticPath(
  const NodePtr & node,
  const NodePtr & goal_node,
  const AnalyticExpansionNodes & expanded_nodes)
{
  return NodePtr(nullptr);
}

template<>
typename AnalyticExpansion<Node2D>::NodePtr AnalyticExpansion<Node2D>::tryAnalyticExpansion(
  const NodePtr & current_node, const NodePtr & goal_node,
  const NodeGetter & getter, int & analytic_iterations,
  int & closest_distance)
{
  return NodePtr(nullptr);
}

template class AnalyticExpansion<Node2D>;
template class AnalyticExpansion<NodeHybrid>;
template class AnalyticExpansion<NodeLattice>;

}
