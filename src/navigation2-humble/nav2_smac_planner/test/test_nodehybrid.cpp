













#include <math.h>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_smac_planner/node_hybrid.hpp"
#include "nav2_smac_planner/collision_checker.hpp"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

TEST(NodeHybridTest, test_node_hybrid)
{
  nav2_smac_planner::SearchInfo info;
  info.change_penalty = 0.1;
  info.non_straight_penalty = 1.1;
  info.reverse_penalty = 2.0;
  info.minimum_turning_radius = 8;
  info.cost_penalty = 1.7;
  info.retrospective_penalty = 0.1;
  unsigned int size_x = 10;
  unsigned int size_y = 10;
  unsigned int size_theta = 72;


  nav2_smac_planner::NodeHybrid testA(49);
  EXPECT_EQ(testA.travel_distance_cost, sqrt(2));

  nav2_smac_planner::NodeHybrid::initMotionModel(
    nav2_smac_planner::MotionModel::DUBIN, size_x, size_y, size_theta, info);

  nav2_costmap_2d::Costmap2D * costmapA = new nav2_costmap_2d::Costmap2D(
    10, 10, 0.05, 0.0, 0.0, 0);
  std::unique_ptr<nav2_smac_planner::GridCollisionChecker> checker =
    std::make_unique<nav2_smac_planner::GridCollisionChecker>(costmapA, 72);
  checker->setFootprint(nav2_costmap_2d::Footprint(), true, 0.0);


  nav2_smac_planner::NodeHybrid testB(49);
  EXPECT_TRUE(std::isnan(testA.getCost()));


  testA.pose.x = 5;
  testA.pose.y = 5;
  testA.pose.theta = 0;
  EXPECT_EQ(testA.isNodeValid(true, checker.get()), true);
  EXPECT_EQ(testA.isNodeValid(false, checker.get()), true);
  EXPECT_EQ(testA.getCost(), 0.0f);


  testA.reset();
  EXPECT_TRUE(std::isnan(testA.getCost()));


  EXPECT_NEAR(testA.travel_distance_cost, 2.08842, 0.1);


  EXPECT_EQ(testA.isNodeValid(false, checker.get()), true);



  EXPECT_NEAR(testB.getTraversalCost(&testA), 2.088, 0.1);


  testB.setMotionPrimitiveIndex(1);
  testA.setMotionPrimitiveIndex(0);
  EXPECT_NEAR(testB.getTraversalCost(&testA), 2.088 * 0.9, 0.1);

  testA.setMotionPrimitiveIndex(1);
  EXPECT_NEAR(testB.getTraversalCost(&testA), 2.297f * 0.9, 0.01);

  testA.setMotionPrimitiveIndex(2);
  EXPECT_NEAR(testB.getTraversalCost(&testA), 2.506f * 0.9, 0.01);


  EXPECT_THROW(testA.getTraversalCost(&testB), std::runtime_error);


  EXPECT_EQ(testA.getMotionPrimitiveIndex(), 2u);


  nav2_smac_planner::NodeHybrid testC(49);
  EXPECT_TRUE(testA == testC);


  testC.setAccumulatedCost(100);
  EXPECT_EQ(testC.getAccumulatedCost(), 100.0f);


  EXPECT_EQ(testC.wasVisited(), false);
  testC.visited();
  EXPECT_EQ(testC.wasVisited(), true);


  EXPECT_EQ(testC.getIndex(), 49u);


  testC.setPose(nav2_smac_planner::NodeHybrid::Coordinates(10.0, 5.0, 4));
  EXPECT_EQ(testC.pose.x, 10.0);
  EXPECT_EQ(testC.pose.y, 5.0);
  EXPECT_EQ(testC.pose.theta, 4);


  EXPECT_EQ(nav2_smac_planner::NodeHybrid::getIndex(1u, 1u, 4u, 10u, 72u), 796u);
  EXPECT_EQ(nav2_smac_planner::NodeHybrid::getCoords(796u, 10u, 72u).x, 1u);
  EXPECT_EQ(nav2_smac_planner::NodeHybrid::getCoords(796u, 10u, 72u).y, 1u);
  EXPECT_EQ(nav2_smac_planner::NodeHybrid::getCoords(796u, 10u, 72u).theta, 4u);

  delete costmapA;
}

TEST(NodeHybridTest, test_obstacle_heuristic)
{
  nav2_smac_planner::SearchInfo info;
  info.change_penalty = 0.1;
  info.non_straight_penalty = 1.1;
  info.reverse_penalty = 2.0;
  info.minimum_turning_radius = 8;
  info.cost_penalty = 1.7;
  info.retrospective_penalty = 0.0;
  unsigned int size_x = 100;
  unsigned int size_y = 100;
  unsigned int size_theta = 72;

  nav2_smac_planner::NodeHybrid::initMotionModel(
    nav2_smac_planner::MotionModel::DUBIN, size_x, size_y, size_theta, info);

  nav2_costmap_2d::Costmap2D * costmapA = new nav2_costmap_2d::Costmap2D(
    100, 100, 0.1, 0.0, 0.0, 0);

  for (unsigned int i = 20; i <= 80; ++i) {
    for (unsigned int j = 40; j <= 60; ++j) {
      costmapA->setCost(i, j, 254);
    }
  }

  for (unsigned int i = 20; i <= 80; ++i) {
    for (unsigned int j = 61; j <= 70; ++j) {
      costmapA->setCost(i, j, 250);
    }
  }
  for (unsigned int i = 20; i <= 80; ++i) {
    for (unsigned int j = 71; j < 100; ++j) {
      costmapA->setCost(i, j, 254);
    }
  }
  std::unique_ptr<nav2_smac_planner::GridCollisionChecker> checker =
    std::make_unique<nav2_smac_planner::GridCollisionChecker>(costmapA, 72);
  checker->setFootprint(nav2_costmap_2d::Footprint(), true, 0.0);

  nav2_smac_planner::NodeHybrid testA(0);
  testA.pose.x = 10;
  testA.pose.y = 50;
  testA.pose.theta = 0;

  nav2_smac_planner::NodeHybrid testB(1);
  testB.pose.x = 90;
  testB.pose.y = 51;
  testB.pose.theta = 0;


  for (unsigned int j = 61; j <= 70; ++j) {
    costmapA->setCost(50, j, 254);
  }
  nav2_smac_planner::NodeHybrid::resetObstacleHeuristic(
    costmapA, testA.pose.x, testA.pose.y, testB.pose.x, testB.pose.y);
  float wide_passage_cost = nav2_smac_planner::NodeHybrid::getObstacleHeuristic(
    testA.pose,
    testB.pose,
    info.cost_penalty);

  EXPECT_NEAR(wide_passage_cost, 91.1f, 0.1f);




  for (unsigned int j = 61; j <= 70; ++j) {
    costmapA->setCost(50, j, 250);
  }
  nav2_smac_planner::NodeHybrid::resetObstacleHeuristic(
    costmapA,
    testA.pose.x, testA.pose.y, testB.pose.x, testB.pose.y);
  float two_passages_cost = nav2_smac_planner::NodeHybrid::getObstacleHeuristic(
    testA.pose,
    testB.pose,
    info.cost_penalty);

  EXPECT_EQ(wide_passage_cost, two_passages_cost);

  delete costmapA;
}

TEST(NodeHybridTest, test_node_debin_neighbors)
{
  nav2_smac_planner::SearchInfo info;
  info.change_penalty = 1.2;
  info.non_straight_penalty = 1.4;
  info.reverse_penalty = 2.1;
  info.minimum_turning_radius = 4;
  info.retrospective_penalty = 0.0;
  unsigned int size_x = 100;
  unsigned int size_y = 100;
  unsigned int size_theta = 72;
  nav2_smac_planner::NodeHybrid::initMotionModel(
    nav2_smac_planner::MotionModel::DUBIN, size_x, size_y, size_theta, info);


  EXPECT_EQ(nav2_smac_planner::NodeHybrid::motion_table.projections.size(), 3u);
  EXPECT_NEAR(nav2_smac_planner::NodeHybrid::motion_table.projections[0]._x, 1.731517, 0.01);
  EXPECT_NEAR(nav2_smac_planner::NodeHybrid::motion_table.projections[0]._y, 0, 0.01);
  EXPECT_NEAR(nav2_smac_planner::NodeHybrid::motion_table.projections[0]._theta, 0, 0.01);

  EXPECT_NEAR(nav2_smac_planner::NodeHybrid::motion_table.projections[1]._x, 1.69047, 0.01);
  EXPECT_NEAR(nav2_smac_planner::NodeHybrid::motion_table.projections[1]._y, 0.3747, 0.01);
  EXPECT_NEAR(nav2_smac_planner::NodeHybrid::motion_table.projections[1]._theta, 5, 0.01);

  EXPECT_NEAR(nav2_smac_planner::NodeHybrid::motion_table.projections[2]._x, 1.69047, 0.01);
  EXPECT_NEAR(nav2_smac_planner::NodeHybrid::motion_table.projections[2]._y, -0.3747, 0.01);
  EXPECT_NEAR(nav2_smac_planner::NodeHybrid::motion_table.projections[2]._theta, -5, 0.01);
}

TEST(NodeHybridTest, test_node_reeds_neighbors)
{
  nav2_smac_planner::SearchInfo info;
  info.change_penalty = 1.2;
  info.non_straight_penalty = 1.4;
  info.reverse_penalty = 2.1;
  info.minimum_turning_radius = 8;
  info.retrospective_penalty = 0.0;
  unsigned int size_x = 100;
  unsigned int size_y = 100;
  unsigned int size_theta = 72;
  nav2_smac_planner::NodeHybrid::initMotionModel(
    nav2_smac_planner::MotionModel::REEDS_SHEPP, size_x, size_y, size_theta, info);

  EXPECT_EQ(nav2_smac_planner::NodeHybrid::motion_table.projections.size(), 6u);
  EXPECT_NEAR(nav2_smac_planner::NodeHybrid::motion_table.projections[0]._x, 2.088, 0.01);
  EXPECT_NEAR(nav2_smac_planner::NodeHybrid::motion_table.projections[0]._y, 0, 0.01);
  EXPECT_NEAR(nav2_smac_planner::NodeHybrid::motion_table.projections[0]._theta, 0, 0.01);

  EXPECT_NEAR(nav2_smac_planner::NodeHybrid::motion_table.projections[1]._x, 2.070, 0.01);
  EXPECT_NEAR(nav2_smac_planner::NodeHybrid::motion_table.projections[1]._y, 0.272, 0.01);
  EXPECT_NEAR(nav2_smac_planner::NodeHybrid::motion_table.projections[1]._theta, 3, 0.01);

  EXPECT_NEAR(nav2_smac_planner::NodeHybrid::motion_table.projections[2]._x, 2.070, 0.01);
  EXPECT_NEAR(nav2_smac_planner::NodeHybrid::motion_table.projections[2]._y, -0.272, 0.01);
  EXPECT_NEAR(nav2_smac_planner::NodeHybrid::motion_table.projections[2]._theta, -3, 0.01);

  EXPECT_NEAR(nav2_smac_planner::NodeHybrid::motion_table.projections[3]._x, -2.088, 0.01);
  EXPECT_NEAR(nav2_smac_planner::NodeHybrid::motion_table.projections[3]._y, 0, 0.01);
  EXPECT_NEAR(nav2_smac_planner::NodeHybrid::motion_table.projections[3]._theta, 0, 0.01);

  EXPECT_NEAR(nav2_smac_planner::NodeHybrid::motion_table.projections[4]._x, -2.07, 0.01);
  EXPECT_NEAR(nav2_smac_planner::NodeHybrid::motion_table.projections[4]._y, 0.272, 0.01);
  EXPECT_NEAR(nav2_smac_planner::NodeHybrid::motion_table.projections[4]._theta, -3, 0.01);

  EXPECT_NEAR(nav2_smac_planner::NodeHybrid::motion_table.projections[5]._x, -2.07, 0.01);
  EXPECT_NEAR(nav2_smac_planner::NodeHybrid::motion_table.projections[5]._y, -0.272, 0.01);
  EXPECT_NEAR(nav2_smac_planner::NodeHybrid::motion_table.projections[5]._theta, 3, 0.01);

  nav2_costmap_2d::Costmap2D costmapA(100, 100, 0.05, 0.0, 0.0, 0);
  std::unique_ptr<nav2_smac_planner::GridCollisionChecker> checker =
    std::make_unique<nav2_smac_planner::GridCollisionChecker>(&costmapA, 72);
  checker->setFootprint(nav2_costmap_2d::Footprint(), true, 0.0);
  nav2_smac_planner::NodeHybrid * node = new nav2_smac_planner::NodeHybrid(49);
  std::function<bool(const unsigned int &, nav2_smac_planner::NodeHybrid * &)> neighborGetter =
    [&, this](const unsigned int & index, nav2_smac_planner::NodeHybrid * & neighbor_rtn) -> bool
    {

      return false;
    };

  nav2_smac_planner::NodeHybrid::NodeVector neighbors;
  node->getNeighbors(neighborGetter, checker.get(), false, neighbors);
  delete node;


  EXPECT_EQ(neighbors.size(), 0u);
}
