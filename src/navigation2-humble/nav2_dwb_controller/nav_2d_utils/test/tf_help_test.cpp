


#include <memory>
#include <string>

#include "gtest/gtest.h"
#include "nav_2d_utils/tf_help.hpp"

TEST(TF_Help, TransformToSelf) {
  bool result;

  std::shared_ptr<tf2_ros::Buffer> tf;
  std::string frame = "frame_id";
  geometry_msgs::msg::PoseStamped in_pose;
  in_pose.header.frame_id = "frame_id";
  in_pose.pose.position.x = 1.0;
  in_pose.pose.position.y = 2.0;
  in_pose.pose.position.z = 3.0;
  tf2::Quaternion qt;
  qt.setRPY(0.5, 1.0, 1.5);
  in_pose.pose.orientation.w = qt.w();
  in_pose.pose.orientation.x = qt.x();
  in_pose.pose.orientation.y = qt.y();
  in_pose.pose.orientation.z = qt.z();

  geometry_msgs::msg::PoseStamped out_pose;
  rclcpp::Duration transform_tolerance(0, 500);

  result = nav_2d_utils::transformPose(tf, frame, in_pose, out_pose, transform_tolerance);

  EXPECT_TRUE(result);
  EXPECT_EQ(out_pose.header.frame_id, "frame_id");
  EXPECT_EQ(out_pose.pose.position.x, 1.0);
  EXPECT_EQ(out_pose.pose.position.y, 2.0);
  EXPECT_EQ(out_pose.pose.position.z, 3.0);
  EXPECT_EQ(out_pose.pose.orientation.w, qt.w());
  EXPECT_EQ(out_pose.pose.orientation.x, qt.x());
  EXPECT_EQ(out_pose.pose.orientation.y, qt.y());
  EXPECT_EQ(out_pose.pose.orientation.z, qt.z());
}

TEST(TF_Help, EmptyBuffer) {
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  auto buffer = std::make_shared<tf2_ros::Buffer>(clock);

  std::string frame = "frame_id";
  geometry_msgs::msg::PoseStamped in_pose;
  in_pose.header.frame_id = "other_frame_id";
  in_pose.pose.position.x = 1.0;
  in_pose.pose.position.y = 2.0;
  in_pose.pose.position.z = 3.0;
  tf2::Quaternion qt;
  qt.setRPY(0.5, 1.0, 1.5);
  in_pose.pose.orientation.w = qt.w();
  in_pose.pose.orientation.x = qt.x();
  in_pose.pose.orientation.y = qt.y();
  in_pose.pose.orientation.z = qt.z();

  geometry_msgs::msg::PoseStamped out_pose;
  rclcpp::Duration transform_tolerance(0, 500);

  bool result;
  result = nav_2d_utils::transformPose(buffer, frame, in_pose, out_pose, transform_tolerance);

  EXPECT_FALSE(result);
}
