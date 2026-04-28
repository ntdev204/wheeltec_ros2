


#include <cmath>
#include "gtest/gtest.h"
#include "nav_2d_utils/path_ops.hpp"

using std::sqrt;
using nav_2d_utils::adjustPlanResolution;

TEST(path_ops_test, AdjustResolutionEmpty)
{
  nav_2d_msgs::msg::Path2D in;
  nav_2d_msgs::msg::Path2D out = adjustPlanResolution(in, 2.0);
  EXPECT_EQ(out.poses.size(), 0ul);
}

TEST(path_ops_test, AdjustResolutionSimple)
{
  nav_2d_msgs::msg::Path2D in;
  const float RESOLUTION = 20.0;

  geometry_msgs::msg::Pose2D pose1;
  pose1.x = 0.0;
  pose1.y = 0.0;
  geometry_msgs::msg::Pose2D pose2;
  pose2.x = 100.0;
  pose2.y = 0.0;

  in.poses.push_back(pose1);
  in.poses.push_back(pose2);

  nav_2d_msgs::msg::Path2D out = adjustPlanResolution(in, RESOLUTION);
  float length = 100;
  ulong number_of_points = ceil(length / (2 * RESOLUTION));
  EXPECT_EQ(out.poses.size(), number_of_points);
  float max_length = length / (number_of_points - 1);

  for (unsigned int i = 1; i < out.poses.size(); i++) {
    pose1 = out.poses[i - 1];
    pose2 = out.poses[i];

    double sq_dist = (pose1.x - pose2.x) * (pose1.x - pose2.x) +
      (pose1.y - pose2.y) * (pose1.y - pose2.y);

    EXPECT_TRUE(sqrt(sq_dist) <= max_length);
  }
}
