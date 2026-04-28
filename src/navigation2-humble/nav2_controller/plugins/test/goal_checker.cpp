


#include <memory>
#include <string>

#include "gtest/gtest.h"
#include "nav2_controller/plugins/simple_goal_checker.hpp"
#include "nav2_controller/plugins/stopped_goal_checker.hpp"
#include "nav_2d_utils/conversions.hpp"
#include "nav2_util/lifecycle_node.hpp"

using nav2_controller::SimpleGoalChecker;
using nav2_controller::StoppedGoalChecker;

void checkMacro(
  nav2_core::GoalChecker & gc,
  double x0, double y0, double theta0,
  double x1, double y1, double theta1,
  double xv, double yv, double thetav,
  bool expected_result)
{
  gc.reset();
  geometry_msgs::msg::Pose2D pose0, pose1;
  pose0.x = x0;
  pose0.y = y0;
  pose0.theta = theta0;
  pose1.x = x1;
  pose1.y = y1;
  pose1.theta = theta1;
  nav_2d_msgs::msg::Twist2D v;
  v.x = xv;
  v.y = yv;
  v.theta = thetav;
  if (expected_result) {
    EXPECT_TRUE(
      gc.isGoalReached(
        nav_2d_utils::pose2DToPose(pose0),
        nav_2d_utils::pose2DToPose(pose1), nav_2d_utils::twist2Dto3D(v)));
  } else {
    EXPECT_FALSE(
      gc.isGoalReached(
        nav_2d_utils::pose2DToPose(pose0),
        nav_2d_utils::pose2DToPose(pose1), nav_2d_utils::twist2Dto3D(v)));
  }
}

void sameResult(
  nav2_core::GoalChecker & gc0, nav2_core::GoalChecker & gc1,
  double x0, double y0, double theta0,
  double x1, double y1, double theta1,
  double xv, double yv, double thetav,
  bool expected_result)
{
  checkMacro(gc0, x0, y0, theta0, x1, y1, theta1, xv, yv, thetav, expected_result);
  checkMacro(gc1, x0, y0, theta0, x1, y1, theta1, xv, yv, thetav, expected_result);
}

void trueFalse(
  nav2_core::GoalChecker & gc0, nav2_core::GoalChecker & gc1,
  double x0, double y0, double theta0,
  double x1, double y1, double theta1,
  double xv, double yv, double thetav)
{
  checkMacro(gc0, x0, y0, theta0, x1, y1, theta1, xv, yv, thetav, true);
  checkMacro(gc1, x0, y0, theta0, x1, y1, theta1, xv, yv, thetav, false);
}
class TestLifecycleNode : public nav2_util::LifecycleNode
{
public:
  explicit TestLifecycleNode(const std::string & name)
  : nav2_util::LifecycleNode(name)
  {
  }

  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn onShutdown(const rclcpp_lifecycle::State &)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn onError(const rclcpp_lifecycle::State &)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }
};

TEST(VelocityIterator, goal_checker_reset)
{
  auto x = std::make_shared<TestLifecycleNode>("goal_checker");

  nav2_core::GoalChecker * gc = new SimpleGoalChecker;
  gc->reset();
  delete gc;
  EXPECT_TRUE(true);
}

TEST(VelocityIterator, two_checks)
{
  auto x = std::make_shared<TestLifecycleNode>("goal_checker");

  SimpleGoalChecker gc;
  StoppedGoalChecker sgc;
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");

  gc.initialize(x, "nav2_controller", costmap);
  sgc.initialize(x, "nav2_controller", costmap);
  sameResult(gc, sgc, 0, 0, 0, 0, 0, 0, 0, 0, 0, true);
  sameResult(gc, sgc, 0, 0, 0, 1, 0, 0, 0, 0, 0, false);
  sameResult(gc, sgc, 0, 0, 0, 0, 1, 0, 0, 0, 0, false);
  sameResult(gc, sgc, 0, 0, 0, 0, 0, 1, 0, 0, 0, false);
  sameResult(gc, sgc, 0, 0, 3.14, 0, 0, -3.14, 0, 0, 0, true);
  trueFalse(gc, sgc, 0, 0, 3.14, 0, 0, -3.14, 1, 0, 0);
  trueFalse(gc, sgc, 0, 0, 0, 0, 0, 0, 1, 0, 0);
  trueFalse(gc, sgc, 0, 0, 0, 0, 0, 0, 0, 1, 0);
  trueFalse(gc, sgc, 0, 0, 0, 0, 0, 0, 0, 0, 1);
}

TEST(StoppedGoalChecker, get_tol_and_dynamic_params)
{
  auto x = std::make_shared<TestLifecycleNode>("goal_checker");

  SimpleGoalChecker gc;
  StoppedGoalChecker sgc;
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");

  sgc.initialize(x, "test", costmap);
  gc.initialize(x, "test2", costmap);
  geometry_msgs::msg::Pose pose_tol;
  geometry_msgs::msg::Twist vel_tol;


  EXPECT_TRUE(sgc.getTolerances(pose_tol, vel_tol));
  EXPECT_EQ(vel_tol.linear.x, 0.25);
  EXPECT_EQ(vel_tol.linear.y, 0.25);
  EXPECT_EQ(vel_tol.angular.z, 0.25);


  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
    x->get_node_base_interface(), x->get_node_topics_interface(),
    x->get_node_graph_interface(),
    x->get_node_services_interface());

  auto results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("test.rot_stopped_velocity", 100.0),
      rclcpp::Parameter("test.trans_stopped_velocity", 100.0)});

  rclcpp::spin_until_future_complete(
    x->get_node_base_interface(),
    results);

  EXPECT_EQ(x->get_parameter("test.rot_stopped_velocity").as_double(), 100.0);
  EXPECT_EQ(x->get_parameter("test.trans_stopped_velocity").as_double(), 100.0);


  results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("test2.xy_goal_tolerance", 200.0),
      rclcpp::Parameter("test2.yaw_goal_tolerance", 200.0),
      rclcpp::Parameter("test2.stateful", true)});

  rclcpp::spin_until_future_complete(
    x->get_node_base_interface(),
    results);

  EXPECT_EQ(x->get_parameter("test2.xy_goal_tolerance").as_double(), 200.0);
  EXPECT_EQ(x->get_parameter("test2.yaw_goal_tolerance").as_double(), 200.0);
  EXPECT_EQ(x->get_parameter("test2.stateful").as_bool(), true);


  EXPECT_TRUE(sgc.getTolerances(pose_tol, vel_tol));
  EXPECT_EQ(vel_tol.linear.x, 100.0);
  EXPECT_EQ(vel_tol.linear.y, 100.0);
  EXPECT_EQ(vel_tol.angular.z, 100.0);

  EXPECT_TRUE(gc.getTolerances(pose_tol, vel_tol));
  EXPECT_EQ(pose_tol.position.x, 200.0);
  EXPECT_EQ(pose_tol.position.y, 200.0);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
