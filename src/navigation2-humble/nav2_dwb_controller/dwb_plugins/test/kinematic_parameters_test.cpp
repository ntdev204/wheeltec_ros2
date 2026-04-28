


#include <string>
#include <memory>
#include <vector>

#include "gtest/gtest.h"
#include "dwb_plugins/kinematic_parameters.hpp"

using rcl_interfaces::msg::Parameter;
using rcl_interfaces::msg::ParameterType;
using rcl_interfaces::msg::ParameterEvent;

class KinematicsHandlerTest : public dwb_plugins::KinematicsHandler
{
public:
  void simulate_event(
    std::vector<rclcpp::Parameter> parameters)
  {
    dynamicParametersCallback(parameters);
  }
};

TEST(KinematicParameters, SetAllParameters) {
  std::string nodeName = "test_node";
  auto node = nav2_util::LifecycleNode::make_shared(nodeName);
  KinematicsHandlerTest kh;
  kh.initialize(node, nodeName);

  std::vector<rclcpp::Parameter> parameters;
  rclcpp::Parameter
    p_minX(nodeName + ".min_vel_x", 12.34),
  p_maxX(nodeName + ".max_vel_x", 23.45),
  p_minY(nodeName + ".min_vel_y", 34.56),
  p_maxY(nodeName + ".max_vel_y", 45.67),
  p_accX(nodeName + ".acc_lim_x", 56.78),
  p_decelX(nodeName + ".acc_lim_y", 67.89),
  p_accY(nodeName + ".decel_lim_x", 78.90),
  p_decelY(nodeName + ".decel_lim_y", 89.01),
  p_minSpeedXY(nodeName + ".min_speed_xy", 90.12),
  p_maxSpeedXY(nodeName + ".max_speed_xy", 123.456),
  p_maxTheta(nodeName + ".max_vel_theta", 345.678),
  p_accTheta(nodeName + ".acc_lim_theta", 234.567),
  p_decelTheta(nodeName + ".decel_lim_theta", 456.789),
  p_minSpeedTheta(nodeName + ".min_speed_theta", 567.890);

  parameters.push_back(p_minX);
  parameters.push_back(p_minX);
  parameters.push_back(p_maxX);
  parameters.push_back(p_minY);
  parameters.push_back(p_maxY);
  parameters.push_back(p_accX);
  parameters.push_back(p_accY);
  parameters.push_back(p_decelX);
  parameters.push_back(p_decelY);
  parameters.push_back(p_minSpeedXY);
  parameters.push_back(p_maxSpeedXY);
  parameters.push_back(p_maxTheta);
  parameters.push_back(p_accTheta);
  parameters.push_back(p_decelTheta);
  parameters.push_back(p_minSpeedTheta);

  kh.simulate_event(parameters);

  dwb_plugins::KinematicParameters kp = kh.getKinematics();

  EXPECT_EQ(kp.getMinX(), 12.34);
  EXPECT_EQ(kp.getMaxX(), 23.45);
  EXPECT_EQ(kp.getMinY(), 34.56);
  EXPECT_EQ(kp.getMaxY(), 45.67);
  EXPECT_EQ(kp.getAccX(), 56.78);
  EXPECT_EQ(kp.getAccY(), 67.89);
  EXPECT_EQ(kp.getDecelX(), 78.90);
  EXPECT_EQ(kp.getDecelY(), 89.01);
  EXPECT_EQ(kp.getMinSpeedXY(), 90.12);
  EXPECT_EQ(kp.getMaxSpeedXY(), 123.456);
  EXPECT_EQ(kp.getAccTheta(), 234.567);
  EXPECT_EQ(kp.getMaxTheta(), 345.678);
  EXPECT_EQ(kp.getDecelTheta(), 456.789);
  EXPECT_EQ(kp.getMinSpeedTheta(), 567.890);
}


int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);


  rclcpp::init(argc, argv);

  bool all_successful = RUN_ALL_TESTS();


  rclcpp::shutdown();

  return all_successful;
}
