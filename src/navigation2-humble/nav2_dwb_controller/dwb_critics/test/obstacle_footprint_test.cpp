


#include <vector>
#include <memory>
#include <string>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "dwb_critics/obstacle_footprint.hpp"
#include "dwb_core/exceptions.hpp"

class OpenObstacleFootprintCritic : public dwb_critics::ObstacleFootprintCritic
{
public:
  double pointCost(int x, int y)
  {
    return dwb_critics::ObstacleFootprintCritic::pointCost(x, y);
  }

  double lineCost(int x0, int x1, int y0, int y1)
  {
    return dwb_critics::ObstacleFootprintCritic::lineCost(x0, x1, y0, y1);
  }
};


geometry_msgs::msg::Point rotate_origin(geometry_msgs::msg::Point p, double angle)
{
  double s = sin(angle);
  double c = cos(angle);


  double xnew = p.x * c - p.y * s;
  double ynew = p.x * s + p.y * c;

  p.x = xnew;
  p.y = ynew;

  return p;
}


geometry_msgs::msg::Point getPoint(double x, double y)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  return p;
}


double footprint_size_x_half = 1.8;
double footprint_size_y_half = 1.6;

std::vector<geometry_msgs::msg::Point> getFootprint()
{
  std::vector<geometry_msgs::msg::Point> footprint;
  footprint.push_back(getPoint(footprint_size_x_half, footprint_size_y_half));
  footprint.push_back(getPoint(footprint_size_x_half, -footprint_size_y_half));
  footprint.push_back(getPoint(-footprint_size_x_half, -footprint_size_y_half));
  footprint.push_back(getPoint(-footprint_size_x_half, footprint_size_y_half));
  return footprint;
}

TEST(ObstacleFootprint, GetOrientedFootprint)
{
  double theta = 0.1234;

  std::vector<geometry_msgs::msg::Point> footprint_before = getFootprint();

  std::vector<geometry_msgs::msg::Point> footprint_after;
  geometry_msgs::msg::Pose2D pose;
  pose.theta = theta;
  footprint_after = dwb_critics::getOrientedFootprint(pose, footprint_before);

  uint i;
  for (i = 0; i < footprint_before.size(); i++) {
    ASSERT_EQ(rotate_origin(footprint_before[i], theta), footprint_after[i]);
  }

  theta = 5.123;
  pose.theta = theta;
  footprint_after = dwb_critics::getOrientedFootprint(pose, footprint_before);

  for (unsigned int i = 0; i < footprint_before.size(); i++) {
    ASSERT_EQ(rotate_origin(footprint_before[i], theta), footprint_after[i]);
  }
}

TEST(ObstacleFootprint, Prepare)
{
  std::shared_ptr<dwb_critics::ObstacleFootprintCritic> critic =
    std::make_shared<dwb_critics::ObstacleFootprintCritic>();

  auto node = nav2_util::LifecycleNode::make_shared("costmap_tester");

  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_global_costmap");
  costmap_ros->configure();

  std::string name = "name";
  std::string ns = "ns";
  critic->initialize(node, name, ns, costmap_ros);

  geometry_msgs::msg::Pose2D pose;
  nav_2d_msgs::msg::Twist2D vel;
  geometry_msgs::msg::Pose2D goal;
  nav_2d_msgs::msg::Path2D global_plan;


  std::vector<geometry_msgs::msg::Point> footprint;
  costmap_ros->setRobotFootprint(footprint);
  ASSERT_FALSE(critic->prepare(pose, vel, goal, global_plan));

  costmap_ros->setRobotFootprint(getFootprint());
  ASSERT_TRUE(critic->prepare(pose, vel, goal, global_plan));

  double epsilon = 0.01;



  pose.x = footprint_size_x_half;
  pose.y = footprint_size_y_half + epsilon;
  ASSERT_THROW(critic->scorePose(pose), dwb_core::IllegalTrajectoryException);

  pose.x = footprint_size_x_half + epsilon;
  pose.y = footprint_size_y_half;
  ASSERT_THROW(critic->scorePose(pose), dwb_core::IllegalTrajectoryException);

  pose.x = costmap_ros->getCostmap()->getSizeInMetersX() - footprint_size_x_half;
  pose.y = costmap_ros->getCostmap()->getSizeInMetersY() + footprint_size_y_half - epsilon;
  ASSERT_THROW(critic->scorePose(pose), dwb_core::IllegalTrajectoryException);

  pose.x = costmap_ros->getCostmap()->getSizeInMetersX() - footprint_size_x_half - epsilon;
  pose.y = costmap_ros->getCostmap()->getSizeInMetersY() + footprint_size_y_half;
  ASSERT_THROW(critic->scorePose(pose), dwb_core::IllegalTrajectoryException);

  pose.x = footprint_size_x_half + epsilon;
  pose.y = footprint_size_y_half + epsilon;
  ASSERT_EQ(critic->scorePose(pose), 0.0);

  for (unsigned int i = 1; i < costmap_ros->getCostmap()->getSizeInCellsX(); i++) {
    costmap_ros->getCostmap()->setCost(i, 10, nav2_costmap_2d::LETHAL_OBSTACLE);
  }

  ASSERT_THROW(critic->scorePose(pose), dwb_core::IllegalTrajectoryException);
}



TEST(ObstacleFootprint, PointCost)
{
  std::shared_ptr<OpenObstacleFootprintCritic> critic =
    std::make_shared<OpenObstacleFootprintCritic>();

  auto node = nav2_util::LifecycleNode::make_shared("costmap_tester");

  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_global_costmap");
  costmap_ros->configure();

  std::string name = "name";
  std::string ns = "ns";
  critic->initialize(node, name, ns, costmap_ros);

  costmap_ros->getCostmap()->setCost(0, 0, nav2_costmap_2d::LETHAL_OBSTACLE);
  costmap_ros->getCostmap()->setCost(0, 1, nav2_costmap_2d::NO_INFORMATION);
  costmap_ros->getCostmap()->setCost(0, 2, 128);

  ASSERT_THROW(critic->pointCost(0, 0), dwb_core::IllegalTrajectoryException);
  ASSERT_THROW(critic->pointCost(0, 1), dwb_core::IllegalTrajectoryException);
  ASSERT_EQ(critic->pointCost(0, 2), 128);
}

TEST(ObstacleFootprint, LineCost)
{
  std::shared_ptr<OpenObstacleFootprintCritic> critic =
    std::make_shared<OpenObstacleFootprintCritic>();

  auto node = nav2_util::LifecycleNode::make_shared("costmap_tester");

  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_global_costmap");
  costmap_ros->configure();

  std::string name = "name";
  std::string ns = "ns";
  critic->initialize(node, name, ns, costmap_ros);

  costmap_ros->getCostmap()->setCost(3, 3, nav2_costmap_2d::LETHAL_OBSTACLE);
  costmap_ros->getCostmap()->setCost(3, 4, nav2_costmap_2d::LETHAL_OBSTACLE);
  costmap_ros->getCostmap()->setCost(4, 3, nav2_costmap_2d::LETHAL_OBSTACLE);
  costmap_ros->getCostmap()->setCost(4, 4, nav2_costmap_2d::LETHAL_OBSTACLE);

  ASSERT_THROW(critic->lineCost(0, 5, 2, 6), dwb_core::IllegalTrajectoryException);
  ASSERT_THROW(critic->lineCost(5, 0, 6, 2), dwb_core::IllegalTrajectoryException);

  ASSERT_THROW(critic->lineCost(2, 4, 0, 10), dwb_core::IllegalTrajectoryException);
  ASSERT_THROW(critic->lineCost(4, 2, 10, 0), dwb_core::IllegalTrajectoryException);


  ASSERT_EQ(critic->lineCost(2, 2, 0, 10), 0.0);
  ASSERT_EQ(critic->lineCost(2, 2, 10, 0), 0.0);
  ASSERT_EQ(critic->lineCost(5, 5, 0, 10), 0.0);
  ASSERT_EQ(critic->lineCost(5, 5, 10, 0), 0.0);
  ASSERT_EQ(critic->lineCost(0, 50, 2, 2), 0.0);
  ASSERT_EQ(critic->lineCost(50, 0, 2, 2), 0.0);
  ASSERT_EQ(critic->lineCost(0, 50, 5, 5), 0.0);
  ASSERT_EQ(critic->lineCost(50, 0, 5, 5), 0.0);


  costmap_ros->getCostmap()->setCost(3, 3, 50);
  costmap_ros->getCostmap()->setCost(3, 4, 50);
  costmap_ros->getCostmap()->setCost(4, 3, 100);
  costmap_ros->getCostmap()->setCost(4, 4, 100);

  ASSERT_EQ(critic->lineCost(3, 3, 0, 50), 50);
  ASSERT_EQ(critic->lineCost(4, 4, 0, 10), 100);
  ASSERT_EQ(critic->lineCost(0, 50, 3, 3), 100);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);


  rclcpp::init(argc, argv);

  bool all_successful = RUN_ALL_TESTS();


  rclcpp::shutdown();

  return all_successful;
}
