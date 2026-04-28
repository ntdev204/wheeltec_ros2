













#include <math.h>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_smac_planner/node_basic.hpp"
#include "nav2_smac_planner/node_2d.hpp"
#include "nav2_smac_planner/node_hybrid.hpp"
#include "nav2_smac_planner/node_lattice.hpp"
#include "nav2_smac_planner/collision_checker.hpp"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

TEST(NodeBasicTest, test_node_basic)
{
  nav2_smac_planner::NodeBasic<nav2_smac_planner::NodeHybrid> node(50);

  EXPECT_EQ(node.index, 50u);
  EXPECT_EQ(node.graph_node_ptr, nullptr);

  nav2_smac_planner::NodeBasic<nav2_smac_planner::Node2D> node2(100);

  EXPECT_EQ(node2.index, 100u);
  EXPECT_EQ(node2.graph_node_ptr, nullptr);

  nav2_smac_planner::NodeBasic<nav2_smac_planner::NodeLattice> node3(200);

  EXPECT_EQ(node3.index, 200u);
  EXPECT_EQ(node3.graph_node_ptr, nullptr);
}
