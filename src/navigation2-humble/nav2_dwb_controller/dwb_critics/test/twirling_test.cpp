


#include <vector>
#include <memory>
#include <string>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "dwb_critics/twirling.hpp"
#include "dwb_core/exceptions.hpp"

TEST(TwirlingTests, Scoring)
{
  std::shared_ptr<dwb_critics::TwirlingCritic> critic =
    std::make_shared<dwb_critics::TwirlingCritic>();

  auto node = nav2_util::LifecycleNode::make_shared("costmap_tester");

  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_global_costmap");
  costmap_ros->configure();

  std::string name = "name";
  std::string ns = "ns";
  critic->initialize(node, name, ns, costmap_ros);

  dwb_msgs::msg::Trajectory2D traj;
  traj.velocity.theta = 1.0;
  EXPECT_EQ(critic->scoreTrajectory(traj), 1.0);
  traj.velocity.theta = -1.0;
  EXPECT_EQ(critic->scoreTrajectory(traj), 1.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);


  rclcpp::init(argc, argv);

  bool all_successful = RUN_ALL_TESTS();


  rclcpp::shutdown();

  return all_successful;
}
