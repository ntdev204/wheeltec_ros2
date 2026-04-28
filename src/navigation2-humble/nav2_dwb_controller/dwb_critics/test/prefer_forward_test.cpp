


#include <math.h>
#include <vector>

#include <memory>
#include <string>
#include <limits>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "dwb_critics/prefer_forward.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"

static constexpr double default_penalty = 1.0;
static constexpr double default_strafe_x = 0.1;
static constexpr double default_strafe_theta = 0.2;
static constexpr double default_theta_scale = 10.0;

TEST(PreferForward, StartNode)
{
  auto critic = std::make_shared<dwb_critics::PreferForwardCritic>();
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_global_costmap");
  auto node = nav2_util::LifecycleNode::make_shared("costmap_tester");
  node->configure();
  node->activate();

  std::string name = "test";
  std::string ns = "ns";
  critic->initialize(node, name, ns, costmap_ros);

  EXPECT_EQ(node->get_parameter(ns + "." + name + ".penalty").as_double(), default_penalty);
  EXPECT_EQ(node->get_parameter(ns + "." + name + ".strafe_x").as_double(), default_strafe_x);
  EXPECT_EQ(
    node->get_parameter(ns + "." + name + ".strafe_theta").as_double(), default_strafe_theta);
  EXPECT_EQ(node->get_parameter(ns + "." + name + ".theta_scale").as_double(), default_theta_scale);
}

TEST(PreferForward, NegativeVelocityX)
{
  auto critic = std::make_shared<dwb_critics::PreferForwardCritic>();
  dwb_msgs::msg::Trajectory2D trajectory;


  trajectory.velocity.x = -1.0;
  EXPECT_EQ(critic->scoreTrajectory(trajectory), default_penalty);

  trajectory.velocity.x = -0.00001;
  EXPECT_EQ(critic->scoreTrajectory(trajectory), default_penalty);

  trajectory.velocity.x = -0.1;
  EXPECT_EQ(critic->scoreTrajectory(trajectory), default_penalty);

  trajectory.velocity.x = std::numeric_limits<double>::lowest();
  EXPECT_EQ(critic->scoreTrajectory(trajectory), default_penalty);

  trajectory.velocity.x = -std::numeric_limits<double>::min();
  EXPECT_EQ(critic->scoreTrajectory(trajectory), default_penalty);
}

TEST(PreferForward, Strafe)
{
  auto critic = std::make_shared<dwb_critics::PreferForwardCritic>();
  dwb_msgs::msg::Trajectory2D trajectory;



  trajectory.velocity.x = 0.05;
  trajectory.velocity.theta = -0.1;
  EXPECT_EQ(critic->scoreTrajectory(trajectory), default_penalty);

  trajectory.velocity.x = 0.0999999;
  EXPECT_EQ(critic->scoreTrajectory(trajectory), default_penalty);

  trajectory.velocity.x = 0.000001;
  EXPECT_EQ(critic->scoreTrajectory(trajectory), default_penalty);

  trajectory.velocity.theta = -0.19;
  EXPECT_EQ(critic->scoreTrajectory(trajectory), default_penalty);

  trajectory.velocity.theta = 0.19;
  EXPECT_EQ(critic->scoreTrajectory(trajectory), default_penalty);
}

TEST(PreferForward, Normal)
{
  auto critic = std::make_shared<dwb_critics::PreferForwardCritic>();
  dwb_msgs::msg::Trajectory2D trajectory;


  trajectory.velocity.x = 0.2;
  trajectory.velocity.theta = -0.1;
  EXPECT_EQ(critic->scoreTrajectory(trajectory), 0.1 * default_theta_scale);

  trajectory.velocity.theta = 0.1;
  EXPECT_EQ(critic->scoreTrajectory(trajectory), 0.1 * default_theta_scale);

  trajectory.velocity.theta = -0.2;
  EXPECT_EQ(critic->scoreTrajectory(trajectory), 0.2 * default_theta_scale);

  trajectory.velocity.theta = 0.2;
  EXPECT_EQ(critic->scoreTrajectory(trajectory), 0.2 * default_theta_scale);

  trajectory.velocity.theta = 1.5;
  EXPECT_EQ(critic->scoreTrajectory(trajectory), 1.5 * default_theta_scale);
}

TEST(PreferForward, NoneDefaultValues)
{
  auto critic = std::make_shared<dwb_critics::PreferForwardCritic>();
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_global_costmap");
  auto node = nav2_util::LifecycleNode::make_shared("costmap_tester");
  node->configure();
  node->activate();

  double penalty = 18.3;
  double strafe_x = 0.5;
  double strafe_theta = 0.4;
  double theta_scale = 15.0;

  std::string name = "test";
  std::string ns = "ns";

  nav2_util::declare_parameter_if_not_declared(
    node, ns + "." + name + ".penalty",
    rclcpp::ParameterValue(penalty));
  nav2_util::declare_parameter_if_not_declared(
    node, ns + "." + name + ".strafe_x",
    rclcpp::ParameterValue(strafe_x));
  nav2_util::declare_parameter_if_not_declared(
    node, ns + "." + name + ".strafe_theta",
    rclcpp::ParameterValue(strafe_theta));
  nav2_util::declare_parameter_if_not_declared(
    node, ns + "." + name + ".theta_scale",
    rclcpp::ParameterValue(theta_scale));

  critic->initialize(node, name, ns, costmap_ros);
  critic->onInit();

  dwb_msgs::msg::Trajectory2D trajectory;
  trajectory.velocity.x = 0.05;
  trajectory.velocity.theta = -0.1;

  EXPECT_EQ(critic->scoreTrajectory(trajectory), penalty);



  trajectory.velocity.x = 0.4;
  trajectory.velocity.theta = -0.39;
  EXPECT_EQ(critic->scoreTrajectory(trajectory), penalty);

  trajectory.velocity.x = 0.0999999;
  EXPECT_EQ(critic->scoreTrajectory(trajectory), penalty);

  trajectory.velocity.x = 0.000001;
  EXPECT_EQ(critic->scoreTrajectory(trajectory), penalty);

  trajectory.velocity.theta = -0.09999;
  EXPECT_EQ(critic->scoreTrajectory(trajectory), penalty);

  trajectory.velocity.theta = 0.09999;
  EXPECT_EQ(critic->scoreTrajectory(trajectory), penalty);


  trajectory.velocity.x = 0.5;
  trajectory.velocity.theta = -0.1;
  EXPECT_EQ(critic->scoreTrajectory(trajectory), 0.1 * theta_scale);

  trajectory.velocity.theta = 0.1;
  EXPECT_EQ(critic->scoreTrajectory(trajectory), 0.1 * theta_scale);

  trajectory.velocity.theta = -0.2;
  EXPECT_EQ(critic->scoreTrajectory(trajectory), 0.2 * theta_scale);

  trajectory.velocity.theta = 0.2;
  EXPECT_EQ(critic->scoreTrajectory(trajectory), 0.2 * theta_scale);

  trajectory.velocity.theta = 1.5;
  EXPECT_EQ(critic->scoreTrajectory(trajectory), 1.5 * theta_scale);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);


  rclcpp::init(argc, argv);

  bool all_successful = RUN_ALL_TESTS();


  rclcpp::shutdown();

  return all_successful;
}
