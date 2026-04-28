













#ifndef NAV2_SMAC_PLANNER__NODE_BASIC_HPP_
#define NAV2_SMAC_PLANNER__NODE_BASIC_HPP_

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
#include "nav2_smac_planner/node_hybrid.hpp"
#include "nav2_smac_planner/node_lattice.hpp"
#include "nav2_smac_planner/node_2d.hpp"
#include "nav2_smac_planner/types.hpp"
#include "nav2_smac_planner/collision_checker.hpp"

namespace nav2_smac_planner
{



template<typename NodeT>
class NodeBasic
{
public:
  

  explicit NodeBasic(const unsigned int index)
  : index(index),
    graph_node_ptr(nullptr)
  {
  }

  

  void populateSearchNode(NodeT * & node);

  

  void processSearchNode();

  typename NodeT::Coordinates pose;
  NodeT * graph_node_ptr;
  MotionPrimitive * prim_ptr;
  unsigned int index, motion_index;
  bool backward;
};

}

#endif
