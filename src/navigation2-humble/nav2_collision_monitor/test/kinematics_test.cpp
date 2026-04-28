













#include <gtest/gtest.h>

#include <math.h>
#include <cmath>
#include <chrono>
#include <vector>
#include <limits>

#include "rclcpp/rclcpp.hpp"

#include "nav2_collision_monitor/types.hpp"
#include "nav2_collision_monitor/kinematics.hpp"

using namespace std::chrono_literals;

static constexpr double EPSILON = std::numeric_limits<float>::epsilon();

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

TEST(KinematicsTest, testTransformPoints)
{

  const nav2_collision_monitor::Pose tf{2.0, 1.0, M_PI / 6.0};

  std::vector<nav2_collision_monitor::Point> points;
  points.push_back({3.0, 2.0});
  points.push_back({0.0, 0.0});


  nav2_collision_monitor::transformPoints(tf, points);



  double new_point_distance = std::sqrt(1.0 + 1.0);


  double new_point_angle = M_PI / 4.0 - M_PI / 6.0;
  EXPECT_NEAR(points[0].x, new_point_distance * std::cos(new_point_angle), EPSILON);
  EXPECT_NEAR(points[0].y, new_point_distance * std::sin(new_point_angle), EPSILON);

  new_point_distance = std::sqrt(1.0 + 4.0);
  new_point_angle = M_PI + std::atan(1.0 / 2.0) - M_PI / 6.0;
  EXPECT_NEAR(points[1].x, new_point_distance * std::cos(new_point_angle), EPSILON);
  EXPECT_NEAR(points[1].y, new_point_distance * std::sin(new_point_angle), EPSILON);
}

TEST(KinematicsTest, testProjectState)
{










  nav2_collision_monitor::Pose pose{2.0, 1.0, M_PI / 4.0};

  nav2_collision_monitor::Velocity vel{0.0, 1.0, M_PI / 4.0};
  const double dt = 1.0;


  nav2_collision_monitor::projectState(dt, pose, vel);


  EXPECT_NEAR(pose.x, 2.0, EPSILON);
  EXPECT_NEAR(pose.y, 2.0, EPSILON);
  EXPECT_NEAR(pose.theta, M_PI / 2, EPSILON);



  const double rotated_vel_angle = M_PI / 2.0 + M_PI / 4.0;
  EXPECT_NEAR(vel.x, std::cos(rotated_vel_angle), EPSILON);
  EXPECT_NEAR(vel.y, std::sin(rotated_vel_angle), EPSILON);
  EXPECT_NEAR(vel.tw, M_PI / 4.0, EPSILON);
}
