













#include "nav2_smac_planner/node_basic.hpp"

namespace nav2_smac_planner
{

template<typename Node2D>
void NodeBasic<Node2D>::processSearchNode()
{
}

template<>
void NodeBasic<NodeHybrid>::processSearchNode()
{



  if (!this->graph_node_ptr->wasVisited()) {
    this->graph_node_ptr->pose = this->pose;
    this->graph_node_ptr->setMotionPrimitiveIndex(this->motion_index);
  }
}

template<>
void NodeBasic<NodeLattice>::processSearchNode()
{



  if (!this->graph_node_ptr->wasVisited()) {
    this->graph_node_ptr->pose = this->pose;
    this->graph_node_ptr->setMotionPrimitive(this->prim_ptr);
    this->graph_node_ptr->backwards(this->backward);
  }
}

template<>
void NodeBasic<Node2D>::populateSearchNode(Node2D * & node)
{
  this->graph_node_ptr = node;
}

template<>
void NodeBasic<NodeHybrid>::populateSearchNode(NodeHybrid * & node)
{
  this->pose = node->pose;
  this->graph_node_ptr = node;
  this->motion_index = node->getMotionPrimitiveIndex();
}

template<>
void NodeBasic<NodeLattice>::populateSearchNode(NodeLattice * & node)
{
  this->pose = node->pose;
  this->graph_node_ptr = node;
  this->prim_ptr = node->getMotionPrimitive();
  this->backward = node->isBackward();
}

template class NodeBasic<Node2D>;
template class NodeBasic<NodeHybrid>;
template class NodeBasic<NodeLattice>;

}
