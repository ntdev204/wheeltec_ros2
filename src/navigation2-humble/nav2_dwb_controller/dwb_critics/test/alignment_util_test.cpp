


#include <vector>
#include <memory>
#include <string>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "dwb_critics/alignment_util.hpp"
#include "dwb_core/exceptions.hpp"

TEST(AlignmentUtil, TestProjection)
{
  geometry_msgs::msg::Pose2D pose, pose_out;
  pose.x = 1.0;
  pose.y = -1.0;
  double distance = 1.0;
  pose_out = dwb_critics::getForwardPose(pose, distance);
  EXPECT_EQ(pose_out.x, 2.0);
  EXPECT_EQ(pose_out.y, -1.0);
  EXPECT_EQ(pose_out.theta, pose.theta);

  pose.x = 2.0;
  pose.y = -10.0;
  pose.theta = 0.54;
  pose_out = dwb_critics::getForwardPose(pose, distance);
  EXPECT_NEAR(pose_out.x, 2.8577, 0.01);
  EXPECT_NEAR(pose_out.y, -9.4858, 0.01);
  EXPECT_EQ(pose_out.theta, pose.theta);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);


  rclcpp::init(argc, argv);

  bool all_successful = RUN_ALL_TESTS();


  rclcpp::shutdown();

  return all_successful;
}
