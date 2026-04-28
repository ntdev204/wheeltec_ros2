














#include <fstream>
#include <string>
#include <vector>
#include <memory>
#include <unordered_map>
#include <limits>
#include "nav2_smac_planner/node_lattice.hpp"
#include "gtest/gtest.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

using json = nlohmann::json;

TEST(NodeLatticeTest, parser_test)
{
  std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("nav2_smac_planner");
  std::string filePath =
    pkg_share_dir +
    "/sample_primitives/5cm_resolution/0.5m_turning_radius/ackermann" +
    "/output.json";
  std::ifstream myJsonFile(filePath);

  ASSERT_TRUE(myJsonFile.is_open());

  json j;
  myJsonFile >> j;

  nav2_smac_planner::LatticeMetadata metaData;
  nav2_smac_planner::MotionPrimitive myPrimitive;
  nav2_smac_planner::MotionPose pose;

  json jsonMetaData = j["lattice_metadata"];
  json jsonPrimatives = j["primitives"];
  json jsonPose = jsonPrimatives[0]["poses"][0];

  nav2_smac_planner::fromJsonToMetaData(jsonMetaData, metaData);


  EXPECT_NEAR(metaData.min_turning_radius, 0.5, 0.001);
  EXPECT_NEAR(metaData.grid_resolution, 0.05, 0.001);
  EXPECT_NEAR(metaData.number_of_headings, 16, 0.01);
  EXPECT_NEAR(metaData.heading_angles[0], 0.0, 0.01);
  EXPECT_EQ(metaData.number_of_trajectories, 80u);
  EXPECT_EQ(metaData.motion_model, std::string("ackermann"));

  std::vector<nav2_smac_planner::MotionPrimitive> myPrimitives;
  for (unsigned int i = 0; i < jsonPrimatives.size(); ++i) {
    nav2_smac_planner::MotionPrimitive newPrimative;
    nav2_smac_planner::fromJsonToMotionPrimitive(jsonPrimatives[i], newPrimative);
    myPrimitives.push_back(newPrimative);
  }


  EXPECT_EQ(myPrimitives.size(), 80u);
  EXPECT_NEAR(myPrimitives[0].trajectory_id, 0, 0.01);
  EXPECT_NEAR(myPrimitives[0].start_angle, 0.0, 0.01);
  EXPECT_NEAR(myPrimitives[0].end_angle, 13, 0.01);
  EXPECT_NEAR(myPrimitives[0].turning_radius, 0.5259, 0.01);
  EXPECT_NEAR(myPrimitives[0].trajectory_length, 0.64856, 0.01);
  EXPECT_NEAR(myPrimitives[0].arc_length, 0.58225, 0.01);
  EXPECT_NEAR(myPrimitives[0].straight_length, 0.06631, 0.01);

  EXPECT_NEAR(myPrimitives[0].poses[0]._x, 0.04981, 0.01);
  EXPECT_NEAR(myPrimitives[0].poses[0]._y, -0.00236, 0.01);
  EXPECT_NEAR(myPrimitives[0].poses[0]._theta, 6.1883, 0.01);

  EXPECT_NEAR(myPrimitives[0].poses[1]._x, 0.09917, 0.01);
  EXPECT_NEAR(myPrimitives[0].poses[1]._y, -0.00944, 0.01);
  EXPECT_NEAR(myPrimitives[0].poses[1]._theta, 6.09345, 0.015);
}

TEST(NodeLatticeTest, test_node_lattice_neighbors_and_parsing)
{
  std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("nav2_smac_planner");
  std::string filePath =
    pkg_share_dir +
    "/sample_primitives/5cm_resolution/0.5m_turning_radius/ackermann" +
    "/output.json";

  nav2_smac_planner::SearchInfo info;
  info.minimum_turning_radius = 1.1;
  info.non_straight_penalty = 1;
  info.change_penalty = 1;
  info.reverse_penalty = 1;
  info.cost_penalty = 1;
  info.retrospective_penalty = 0.0;
  info.analytic_expansion_ratio = 1;
  info.lattice_filepath = filePath;
  info.cache_obstacle_heuristic = true;
  info.allow_reverse_expansion = true;

  unsigned int x = 100;
  unsigned int y = 100;
  unsigned int angle_quantization = 16;

  nav2_smac_planner::NodeLattice::initMotionModel(
    nav2_smac_planner::MotionModel::STATE_LATTICE, x, y, angle_quantization, info);

  nav2_smac_planner::NodeLattice aNode(0);
  aNode.setPose(nav2_smac_planner::NodeHybrid::Coordinates(0, 0, 0));
  nav2_smac_planner::MotionPrimitivePtrs projections =
    nav2_smac_planner::NodeLattice::motion_table.getMotionPrimitives(&aNode);

  EXPECT_NEAR(projections[0]->poses.back()._x, 0.5, 0.01);
  EXPECT_NEAR(projections[0]->poses.back()._y, -0.35, 0.01);
  EXPECT_NEAR(projections[0]->poses.back()._theta, 5.176, 0.01);

  EXPECT_NEAR(
    nav2_smac_planner::NodeLattice::motion_table.getLatticeMetadata(
      filePath).grid_resolution, 0.05, 0.005);
}

TEST(NodeLatticeTest, test_node_lattice_conversions)
{
  std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("nav2_smac_planner");
  std::string filePath =
    pkg_share_dir +
    "/sample_primitives/5cm_resolution/0.5m_turning_radius/ackermann" +
    "/output.json";

  nav2_smac_planner::SearchInfo info;
  info.minimum_turning_radius = 1.1;
  info.non_straight_penalty = 1;
  info.change_penalty = 1;
  info.reverse_penalty = 1;
  info.cost_penalty = 1;
  info.retrospective_penalty = 0.0;
  info.analytic_expansion_ratio = 1;
  info.lattice_filepath = filePath;
  info.cache_obstacle_heuristic = true;

  unsigned int x = 100;
  unsigned int y = 100;
  unsigned int angle_quantization = 16;

  nav2_smac_planner::NodeLattice::initMotionModel(
    nav2_smac_planner::MotionModel::STATE_LATTICE, x, y, angle_quantization, info);

  nav2_smac_planner::NodeLattice aNode(0);
  aNode.setPose(nav2_smac_planner::NodeHybrid::Coordinates(0, 0, 0));

  EXPECT_NEAR(aNode.motion_table.getAngleFromBin(0u), 0.0, 0.005);
  EXPECT_NEAR(aNode.motion_table.getAngleFromBin(1u), 0.46364, 0.005);
  EXPECT_NEAR(aNode.motion_table.getAngleFromBin(2u), 0.78539, 0.005);

  EXPECT_EQ(aNode.motion_table.getClosestAngularBin(0.0), 0u);
  EXPECT_EQ(aNode.motion_table.getClosestAngularBin(0.5), 1u);
  EXPECT_EQ(aNode.motion_table.getClosestAngularBin(1.5), 4u);
}

TEST(NodeLatticeTest, test_node_lattice)
{
  std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("nav2_smac_planner");
  std::string filePath =
    pkg_share_dir +
    "/sample_primitives/5cm_resolution/0.5m_turning_radius/ackermann" +
    "/output.json";

  nav2_smac_planner::SearchInfo info;
  info.minimum_turning_radius = 1.1;
  info.non_straight_penalty = 1;
  info.change_penalty = 1;
  info.reverse_penalty = 1;
  info.cost_penalty = 1;
  info.retrospective_penalty = 0.1;
  info.analytic_expansion_ratio = 1;
  info.lattice_filepath = filePath;
  info.cache_obstacle_heuristic = true;
  info.allow_reverse_expansion = true;

  unsigned int x = 100;
  unsigned int y = 100;
  unsigned int angle_quantization = 16;

  nav2_smac_planner::NodeLattice::initMotionModel(
    nav2_smac_planner::MotionModel::STATE_LATTICE, x, y, angle_quantization, info);


  nav2_smac_planner::NodeLattice aNode(0);
  nav2_smac_planner::NodeLattice testA(49);
  EXPECT_EQ(testA.getIndex(), 49u);
  EXPECT_EQ(testA.getAccumulatedCost(), std::numeric_limits<float>::max());
  EXPECT_TRUE(std::isnan(testA.getCost()));
  EXPECT_EQ(testA.getMotionPrimitive(), nullptr);


  EXPECT_EQ(testA.wasVisited(), false);
  testA.visited();
  EXPECT_EQ(testA.wasVisited(), true);
  testA.reset();
  EXPECT_EQ(testA.wasVisited(), false);

  nav2_costmap_2d::Costmap2D * costmapA = new nav2_costmap_2d::Costmap2D(
    10, 10, 0.05, 0.0, 0.0, 0);
  std::unique_ptr<nav2_smac_planner::GridCollisionChecker> checker =
    std::make_unique<nav2_smac_planner::GridCollisionChecker>(costmapA, 72);
  checker->setFootprint(nav2_costmap_2d::Footprint(), true, 0.0);


  testA.pose.x = 5;
  testA.pose.y = 5;
  testA.pose.theta = 0;
  EXPECT_EQ(testA.isNodeValid(true, checker.get()), true);
  EXPECT_EQ(testA.isNodeValid(false, checker.get()), true);
  EXPECT_EQ(testA.getCost(), 0.0f);


  EXPECT_EQ(testA.isNodeValid(false, checker.get()), true);


  nav2_smac_planner::NodeLattice testC(49);
  EXPECT_TRUE(testA == testC);


  testC.setAccumulatedCost(100);
  EXPECT_EQ(testC.getAccumulatedCost(), 100.0f);


  testC.setPose(nav2_smac_planner::NodeLattice::Coordinates(10.0, 5.0, 4));
  EXPECT_EQ(testC.pose.x, 10.0);
  EXPECT_EQ(testC.pose.y, 5.0);
  EXPECT_EQ(testC.pose.theta, 4);

  delete costmapA;
}


TEST(NodeLatticeTest, test_get_neighbors)
{
  std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("nav2_smac_planner");
  std::string filePath =
    pkg_share_dir +
    "/sample_primitives/5cm_resolution/0.5m_turning_radius/ackermann" +
    "/output.json";

  nav2_smac_planner::SearchInfo info;
  info.minimum_turning_radius = 1.1;
  info.non_straight_penalty = 1;
  info.change_penalty = 1;
  info.reverse_penalty = 1;
  info.cost_penalty = 1;
  info.analytic_expansion_ratio = 1;
  info.retrospective_penalty = 0.0;
  info.lattice_filepath = filePath;
  info.cache_obstacle_heuristic = true;
  info.allow_reverse_expansion = true;

  unsigned int x = 100;
  unsigned int y = 100;
  unsigned int angle_quantization = 16;

  nav2_smac_planner::NodeLattice::initMotionModel(
    nav2_smac_planner::MotionModel::STATE_LATTICE, x, y, angle_quantization, info);

  nav2_smac_planner::NodeLattice node(49);

  nav2_costmap_2d::Costmap2D * costmapA = new nav2_costmap_2d::Costmap2D(
    10, 10, 0.05, 0.0, 0.0, 0);
  std::unique_ptr<nav2_smac_planner::GridCollisionChecker> checker =
    std::make_unique<nav2_smac_planner::GridCollisionChecker>(costmapA, 72);
  checker->setFootprint(nav2_costmap_2d::Footprint(), true, 0.0);

  std::function<bool(const unsigned int &, nav2_smac_planner::NodeLattice * &)> neighborGetter =
    [&, this](const unsigned int & index, nav2_smac_planner::NodeLattice * & neighbor_rtn) -> bool
    {

      return false;
    };

  nav2_smac_planner::NodeLattice::NodeVector neighbors;
  node.getNeighbors(neighborGetter, checker.get(), false, neighbors);

  EXPECT_EQ(neighbors.size(), 0u);

  delete costmapA;
}
