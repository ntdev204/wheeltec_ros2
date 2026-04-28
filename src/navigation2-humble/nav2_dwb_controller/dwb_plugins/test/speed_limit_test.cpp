


#include <gtest/gtest.h>

#include <string>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

#include "dwb_plugins/kinematic_parameters.hpp"

using namespace std::chrono_literals;

static constexpr double EPSILON = 1e-5;

static const char NODE_NAME[] = "test_node";
static const double MAX_VEL_X = 40.0;
static const double MAX_VEL_Y = 30.0;
static const double MAX_VEL_THETA = 15.0;
static const double MAX_VEL_LINEAR = 50.0;

class TestNode : public ::testing::Test
{
public:
  TestNode()
  {
    const std::string node_name = NODE_NAME;
    node_ = nav2_util::LifecycleNode::make_shared(node_name);

    node_->declare_parameter(
      node_name + ".max_vel_x", rclcpp::ParameterValue(MAX_VEL_X));
    node_->set_parameter(
      rclcpp::Parameter(node_name + ".max_vel_x", MAX_VEL_X));

    node_->declare_parameter(
      node_name + ".max_vel_y", rclcpp::ParameterValue(MAX_VEL_Y));
    node_->set_parameter(
      rclcpp::Parameter(node_name + ".max_vel_y", MAX_VEL_Y));

    node_->declare_parameter(
      node_name + ".max_vel_theta", rclcpp::ParameterValue(MAX_VEL_THETA));
    node_->set_parameter(
      rclcpp::Parameter(node_name + ".max_vel_theta", MAX_VEL_THETA));

    node_->declare_parameter(
      node_name + ".max_speed_xy", rclcpp::ParameterValue(MAX_VEL_LINEAR));
    node_->set_parameter(
      rclcpp::Parameter(node_name + ".max_speed_xy", MAX_VEL_LINEAR));
  }

  ~TestNode() {}

protected:
  nav2_util::LifecycleNode::SharedPtr node_;
};

TEST_F(TestNode, TestPercentLimit)
{
  dwb_plugins::KinematicsHandler kh;
  kh.initialize(node_, NODE_NAME);

  dwb_plugins::KinematicParameters kp = kh.getKinematics();
  EXPECT_NEAR(kp.getMaxX(), MAX_VEL_X, EPSILON);
  EXPECT_NEAR(kp.getMaxY(), MAX_VEL_Y, EPSILON);
  EXPECT_NEAR(kp.getMaxTheta(), MAX_VEL_THETA, EPSILON);
  EXPECT_NEAR(kp.getMaxSpeedXY(), MAX_VEL_LINEAR, EPSILON);


  kh.setSpeedLimit(30, true);


  kp = kh.getKinematics();
  EXPECT_NEAR(kp.getMaxX(), MAX_VEL_X * 0.3, EPSILON);
  EXPECT_NEAR(kp.getMaxY(), MAX_VEL_Y * 0.3, EPSILON);
  EXPECT_NEAR(kp.getMaxTheta(), MAX_VEL_THETA * 0.3, EPSILON);
  EXPECT_NEAR(kp.getMaxSpeedXY(), MAX_VEL_LINEAR * 0.3, EPSILON);


  kh.setSpeedLimit(nav2_costmap_2d::NO_SPEED_LIMIT, true);


  kp = kh.getKinematics();
  EXPECT_NEAR(kp.getMaxX(), MAX_VEL_X, EPSILON);
  EXPECT_NEAR(kp.getMaxY(), MAX_VEL_Y, EPSILON);
  EXPECT_NEAR(kp.getMaxTheta(), MAX_VEL_THETA, EPSILON);
  EXPECT_NEAR(kp.getMaxSpeedXY(), MAX_VEL_LINEAR, EPSILON);
}

TEST_F(TestNode, TestAbsoluteLimit)
{
  dwb_plugins::KinematicsHandler kh;
  kh.initialize(node_, NODE_NAME);

  dwb_plugins::KinematicParameters kp = kh.getKinematics();
  EXPECT_NEAR(kp.getMaxX(), MAX_VEL_X, EPSILON);
  EXPECT_NEAR(kp.getMaxY(), MAX_VEL_Y, EPSILON);
  EXPECT_NEAR(kp.getMaxTheta(), MAX_VEL_THETA, EPSILON);
  EXPECT_NEAR(kp.getMaxSpeedXY(), MAX_VEL_LINEAR, EPSILON);


  kh.setSpeedLimit(35.0, false);


  kp = kh.getKinematics();
  EXPECT_NEAR(kp.getMaxX(), MAX_VEL_X * 35.0 / MAX_VEL_LINEAR, EPSILON);
  EXPECT_NEAR(kp.getMaxY(), MAX_VEL_Y * 35.0 / MAX_VEL_LINEAR, EPSILON);
  EXPECT_NEAR(kp.getMaxTheta(), MAX_VEL_THETA * 35.0 / MAX_VEL_LINEAR, EPSILON);
  EXPECT_NEAR(kp.getMaxSpeedXY(), 35.0, EPSILON);


  kh.setSpeedLimit(nav2_costmap_2d::NO_SPEED_LIMIT, false);


  kp = kh.getKinematics();
  EXPECT_NEAR(kp.getMaxX(), MAX_VEL_X, EPSILON);
  EXPECT_NEAR(kp.getMaxY(), MAX_VEL_Y, EPSILON);
  EXPECT_NEAR(kp.getMaxTheta(), MAX_VEL_THETA, EPSILON);
  EXPECT_NEAR(kp.getMaxSpeedXY(), MAX_VEL_LINEAR, EPSILON);
}

int main(int argc, char ** argv)
{

  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);


  bool test_result = RUN_ALL_TESTS();


  rclcpp::shutdown();

  return test_result;
}
