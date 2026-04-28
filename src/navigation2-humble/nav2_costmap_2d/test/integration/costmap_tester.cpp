

#include <memory>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

namespace nav2_costmap_2d
{

std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

class CostmapTester : public testing::Test
{
public:
  explicit CostmapTester(tf2_ros::Buffer & tf);
  void checkConsistentCosts();
  void compareCellToNeighbors(
    nav2_costmap_2d::Costmap2D & costmap,
    unsigned int x, unsigned int y);
  void compareCells(
    nav2_costmap_2d::Costmap2D & costmap,
    unsigned int x, unsigned int y, unsigned int nx, unsigned int ny);
  virtual void TestBody() {}
};

CostmapTester::CostmapTester(tf2_ros::Buffer & tf)
{
  costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap", tf);
}

void CostmapTester::checkConsistentCosts()
{
  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();


  costmap->saveMap("costmap_test.pgm");


  for (unsigned int i = 0; i < costmap->getSizeInCellsX(); ++i) {
    for (unsigned int j = 0; j < costmap->getSizeInCellsY(); ++j) {
      compareCellToNeighbors(*costmap, i, j);
    }
  }
}

void CostmapTester::compareCellToNeighbors(
  nav2_costmap_2d::Costmap2D & costmap,
  unsigned int x, unsigned int y)
{


  for (int offset_x = -1; offset_x <= 1; ++offset_x) {
    for (int offset_y = -1; offset_y <= 1; ++offset_y) {
      int nx = x + offset_x;
      int ny = y + offset_y;


      if (nx >= 0 && nx < static_cast<int>(costmap.getSizeInCellsX()) && ny >= 0 &&
        ny < static_cast<int>(costmap.getSizeInCellsY()))
      {
        compareCells(costmap, x, y, nx, ny);
      }
    }
  }
}



void CostmapTester::compareCells(
  nav2_costmap_2d::Costmap2D & costmap,
  unsigned int x, unsigned int y, unsigned int nx, unsigned int ny)
{
  double cell_distance = hypot(static_cast<int>(x - nx), static_cast<int>(y - ny));

  unsigned char cell_cost = costmap.getCost(x, y);
  unsigned char neighbor_cost = costmap.getCost(nx, ny);

  if (cell_cost == nav2_costmap_2d::LETHAL_OBSTACLE) {


    unsigned char expected_lowest_cost = 0;
    EXPECT_TRUE(
      neighbor_cost >= expected_lowest_cost ||
      (cell_distance > 0 && neighbor_cost == nav2_costmap_2d::FREE_SPACE));
  } else if (cell_cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {


    double furthest_valid_distance = 0;
    unsigned char expected_lowest_cost = 0;
    if (neighbor_cost < expected_lowest_cost) {
      RCLCPP_ERROR(
        rclcpp::get_logger(
          "costmap_tester"),
        "Cell cost (%d, %d): %d, neighbor cost (%d, %d): %d, expected lowest cost: %d, cell distance: %.2f, furthest valid distance: %.2f",
        x, y, cell_cost, nx, ny, neighbor_cost, expected_lowest_cost,
        cell_distance, furthest_valid_distance);
      RCLCPP_ERROR(
        rclcpp::get_logger("costmap_tester"), "Cell: (%d, %d), Neighbor: (%d, %d)",
        x, y, nx, ny);
      costmap.saveMap("failing_costmap.pgm");
    }
    EXPECT_TRUE(
      neighbor_cost >= expected_lowest_cost ||
      (furthest_valid_distance > 0 && neighbor_cost == nav2_costmap_2d::FREE_SPACE));
  }
}
}

nav2_costmap_2d::CostmapTester * map_tester = NULL;
tf2_ros::TransformListener * tfl_;
tf2_ros::Buffer * tf_;

TEST(CostmapTester, checkConsistentCosts) {
  map_tester->checkConsistentCosts();
}

void testCallback()
{
  int test_result = RUN_ALL_TESTS();
  RCLCPP_INFO(rclcpp::get_logger("costmap_tester"), "gtest return value: %d", test_result);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = nav2_util::LifecycleNode::make_shared("costmap_tester");
  testing::InitGoogleTest(&argc, argv);

  tf_ = new tf2_ros::Buffer(node->get_clock());
  tfl_ = new tf2_ros::TransformListener(*tf_);
  map_tester = new nav2_costmap_2d::CostmapTester(*tf_);
  rclcpp::TimerBase::SharedPtr timer = node->create_wall_timer(30000ms, testCallback);
  rclcpp::spin(costmap_ros_);
  rclcpp::shutdown();

  return 0;
}
