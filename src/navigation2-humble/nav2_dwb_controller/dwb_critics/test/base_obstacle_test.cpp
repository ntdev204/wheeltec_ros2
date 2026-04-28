


#include <vector>
#include <memory>
#include <string>
#include <utility>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "dwb_critics/obstacle_footprint.hpp"
#include "dwb_core/exceptions.hpp"

TEST(BaseObstacle, IsValidCost)
{
  std::shared_ptr<dwb_critics::BaseObstacleCritic> critic =
    std::make_shared<dwb_critics::BaseObstacleCritic>();

  for (int i = 0; i < 256; i++) {

    if (i == nav2_costmap_2d::LETHAL_OBSTACLE ||
      i == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||
      i == nav2_costmap_2d::NO_INFORMATION)
    {
      ASSERT_FALSE(critic->isValidCost(i));
    } else {
      ASSERT_TRUE(critic->isValidCost(i));
    }
  }
}

TEST(BaseObstacle, ScorePose)
{
  std::shared_ptr<dwb_critics::BaseObstacleCritic> critic =
    std::make_shared<dwb_critics::BaseObstacleCritic>();

  auto node = nav2_util::LifecycleNode::make_shared("base_obstacle_critic_tester");

  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_global_costmap");
  costmap_ros->configure();

  std::string name = "name";
  std::string ns = "ns";

  critic->initialize(node, name, ns, costmap_ros);

  costmap_ros->getCostmap()->setCost(0, 0, nav2_costmap_2d::LETHAL_OBSTACLE);
  costmap_ros->getCostmap()->setCost(0, 1, nav2_costmap_2d::NO_INFORMATION);
  const int some_other_cost = 128;
  costmap_ros->getCostmap()->setCost(0, 2, some_other_cost);


  geometry_msgs::msg::Pose2D pose;
  pose.x = 0;
  pose.y = 0;

  ASSERT_THROW(critic->scorePose(pose), dwb_core::IllegalTrajectoryException);

  pose.x = 0;
  pose.y = 0.15;
  ASSERT_THROW(critic->scorePose(pose), dwb_core::IllegalTrajectoryException);

  pose.y = 0.25;
  pose.x = 0.05;
  ASSERT_EQ(critic->scorePose(pose), some_other_cost);


  for (int i = -50; i < 150; i++) {
    pose.theta = (1.0 / 50) * i * M_PI;
    ASSERT_EQ(critic->scorePose(pose), some_other_cost);
  }


  pose.x = 1.0;
  pose.y = -0.1;
  ASSERT_THROW(critic->scorePose(pose), dwb_core::IllegalTrajectoryException);

  pose.x = costmap_ros->getCostmap()->getSizeInMetersX() + 0.1;
  pose.y = 1.0;
  ASSERT_THROW(critic->scorePose(pose), dwb_core::IllegalTrajectoryException);

  pose.x = 1.0;
  pose.y = costmap_ros->getCostmap()->getSizeInMetersY() + 0.1;
  ASSERT_THROW(critic->scorePose(pose), dwb_core::IllegalTrajectoryException);

  pose.x = -0.1;
  pose.y = 1.0;
  ASSERT_THROW(critic->scorePose(pose), dwb_core::IllegalTrajectoryException);
}

TEST(BaseObstacle, CriticVisualization)
{
  std::shared_ptr<dwb_critics::BaseObstacleCritic> critic =
    std::make_shared<dwb_critics::BaseObstacleCritic>();

  auto node = nav2_util::LifecycleNode::make_shared("base_obstacle_critic_tester");

  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_global_costmap");
  costmap_ros->configure();

  std::string name = "name";
  std::string ns = "ns";

  critic->initialize(node, name, ns, costmap_ros);

  costmap_ros->getCostmap()->setCost(0, 0, nav2_costmap_2d::LETHAL_OBSTACLE);
  costmap_ros->getCostmap()->setCost(0, 1, nav2_costmap_2d::NO_INFORMATION);

  costmap_ros->getCostmap()->setCost(3, 2, 64);
  costmap_ros->getCostmap()->setCost(30, 12, 85);
  costmap_ros->getCostmap()->setCost(10, 49, 24);
  costmap_ros->getCostmap()->setCost(45, 2, 12);

  std::vector<std::pair<std::string, std::vector<float>>> cost_channels;
  critic->addCriticVisualization(cost_channels);

  unsigned int size_x = costmap_ros->getCostmap()->getSizeInCellsX();
  unsigned int size_y = costmap_ros->getCostmap()->getSizeInCellsY();


  for (unsigned int y = 0; y < size_y; y++) {
    for (unsigned int x = 0; x < size_x; x++) {
      float pointValue = cost_channels[0].second[y * size_y + x];
      ASSERT_EQ(static_cast<int>(pointValue), costmap_ros->getCostmap()->getCost(x, y));
    }
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);


  rclcpp::init(argc, argv);

  bool all_successful = RUN_ALL_TESTS();


  rclcpp::shutdown();

  return all_successful;
}
