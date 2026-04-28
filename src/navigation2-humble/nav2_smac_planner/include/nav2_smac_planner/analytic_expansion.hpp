













#ifndef NAV2_SMAC_PLANNER__ANALYTIC_EXPANSION_HPP_
#define NAV2_SMAC_PLANNER__ANALYTIC_EXPANSION_HPP_

#include <string>
#include <vector>
#include <list>
#include <memory>

#include "nav2_smac_planner/node_2d.hpp"
#include "nav2_smac_planner/node_hybrid.hpp"
#include "nav2_smac_planner/node_lattice.hpp"
#include "nav2_smac_planner/types.hpp"
#include "nav2_smac_planner/constants.hpp"

namespace nav2_smac_planner
{

template<typename NodeT>
class AnalyticExpansion
{
public:
  typedef NodeT * NodePtr;
  typedef typename NodeT::Coordinates Coordinates;
  typedef std::function<bool (const unsigned int &, NodeT * &)> NodeGetter;

  

  struct AnalyticExpansionNode
  {
    AnalyticExpansionNode(
      NodePtr & node_in,
      Coordinates & initial_coords_in,
      Coordinates & proposed_coords_in)
    : node(node_in),
      initial_coords(initial_coords_in),
      proposed_coords(proposed_coords_in)
    {
    }

    NodePtr node;
    Coordinates initial_coords;
    Coordinates proposed_coords;
  };

  typedef std::vector<AnalyticExpansionNode> AnalyticExpansionNodes;

  

  AnalyticExpansion(
    const MotionModel & motion_model,
    const SearchInfo & search_info,
    const bool & traverse_unknown,
    const unsigned int & dim_3_size);

  

  void setCollisionChecker(GridCollisionChecker * collision_checker);

  

  NodePtr tryAnalyticExpansion(
    const NodePtr & current_node,
    const NodePtr & goal_node,
    const NodeGetter & getter, int & iterations, int & best_cost);

  

  AnalyticExpansionNodes getAnalyticPath(
    const NodePtr & node, const NodePtr & goal,
    const NodeGetter & getter);

  

  NodePtr setAnalyticPath(
    const NodePtr & node, const NodePtr & goal,
    const AnalyticExpansionNodes & expanded_nodes);

  

  void cleanNode(const NodePtr & nodes);

protected:
  MotionModel _motion_model;
  SearchInfo _search_info;
  bool _traverse_unknown;
  unsigned int _dim_3_size;
  GridCollisionChecker * _collision_checker;
  std::list<std::unique_ptr<NodeT>> _detached_nodes;
};

}

#endif
