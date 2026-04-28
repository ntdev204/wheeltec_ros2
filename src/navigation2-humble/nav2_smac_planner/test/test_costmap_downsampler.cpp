













#include <memory>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_smac_planner/costmap_downsampler.hpp"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

TEST(CostmapDownsampler, costmap_downsample_test)
{
  nav2_util::LifecycleNode::SharedPtr node = std::make_shared<nav2_util::LifecycleNode>(
    "CostmapDownsamplerTest");
  nav2_smac_planner::CostmapDownsampler downsampler;


  nav2_costmap_2d::Costmap2D costmapA(10, 10, 0.05, 0.0, 0.0, 0);
  costmapA.setCost(0, 0, 100);
  costmapA.setCost(5, 5, 50);


  downsampler.on_configure(node, "map", "unused_topic", &costmapA, 2);
  nav2_costmap_2d::Costmap2D * downsampledCostmapA = downsampler.downsample(2);


  EXPECT_EQ(downsampledCostmapA->getCost(0, 0), 100);
  EXPECT_EQ(downsampledCostmapA->getCost(2, 2), 50);
  EXPECT_EQ(downsampledCostmapA->getSizeInCellsX(), 5u);
  EXPECT_EQ(downsampledCostmapA->getSizeInCellsY(), 5u);


  nav2_costmap_2d::Costmap2D costmapB(4, 4, 0.10, 0.0, 0.0, 0);


  downsampler.on_configure(node, "map", "unused_topic", &costmapB, 4);
  downsampler.on_activate();
  nav2_costmap_2d::Costmap2D * downsampledCostmapB = downsampler.downsample(4);
  downsampler.on_deactivate();


  EXPECT_EQ(downsampledCostmapB->getSizeInCellsX(), 1u);
  EXPECT_EQ(downsampledCostmapB->getSizeInCellsY(), 1u);

  downsampler.resizeCostmap();
}
