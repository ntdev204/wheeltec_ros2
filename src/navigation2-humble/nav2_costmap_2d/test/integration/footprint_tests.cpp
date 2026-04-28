

#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_listener.h"
#include "nav2_costmap_2d/footprint.hpp"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

class FootprintTestNode
{
public:
  FootprintTestNode()
  {

    testFootprint(0.01f, 0.1);
  }

  ~FootprintTestNode() {}

  void testFootprint(double footprint_padding, std::string footprint)
  {
    footprint_padding_ = footprint_padding;
    if (footprint != "" && footprint != "[]") {
      std::vector<geometry_msgs::msg::Point> new_footprint;
      if (nav2_costmap_2d::makeFootprintFromString(footprint, new_footprint)) {
        setRobotFootprint(new_footprint);
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("footprint_tester"), "Invalid footprint string");
      }
    }
  }

  void testFootprint(double footprint_padding, double robot_radius)
  {
    footprint_padding_ = footprint_padding;
    setRobotFootprint(nav2_costmap_2d::makeFootprintFromRadius(robot_radius));
  }

  std::vector<geometry_msgs::msg::Point> getRobotFootprint()
  {
    return footprint_;
  }

protected:
  void setRobotFootprint(const std::vector<geometry_msgs::msg::Point> & points)
  {
    footprint_ = points;
    nav2_costmap_2d::padFootprint(footprint_, footprint_padding_);
  }

  double footprint_padding_;
  std::vector<geometry_msgs::msg::Point> footprint_;
};

class TestNode : public ::testing::Test
{
public:
  TestNode()
  {
    footprint_tester_ = std::make_shared<FootprintTestNode>();
  }

  ~TestNode() {}

protected:
  std::shared_ptr<FootprintTestNode> footprint_tester_;
};


TEST_F(TestNode, footprint_empty)
{

  std::vector<geometry_msgs::msg::Point> footprint = footprint_tester_->getRobotFootprint();


  EXPECT_EQ(16u, footprint.size());

  EXPECT_NEAR(0.11f, footprint[0].x, 0.0001);
  EXPECT_NEAR(0.0f, footprint[0].y, 0.0001);
  EXPECT_EQ(0.0f, footprint[0].z);
}

TEST_F(TestNode, unpadded_footprint_from_string_param)
{
  footprint_tester_->testFootprint(0.0, "[[1, 1], [-1, 1], [-1, -1]]");

  std::vector<geometry_msgs::msg::Point> footprint = footprint_tester_->getRobotFootprint();
  EXPECT_EQ(3u, footprint.size());

  EXPECT_EQ(1.0f, footprint[0].x);
  EXPECT_EQ(1.0f, footprint[0].y);
  EXPECT_EQ(0.0f, footprint[0].z);

  EXPECT_EQ(-1.0f, footprint[1].x);
  EXPECT_EQ(1.0f, footprint[1].y);
  EXPECT_EQ(0.0f, footprint[1].z);

  EXPECT_EQ(-1.0f, footprint[2].x);
  EXPECT_EQ(-1.0f, footprint[2].y);
  EXPECT_EQ(0.0f, footprint[2].z);
}

TEST_F(TestNode, padded_footprint_from_string_param)
{
  footprint_tester_->testFootprint(0.5, "[[1, 1], [-1, 1], [-1, -1]]");

  std::vector<geometry_msgs::msg::Point> footprint = footprint_tester_->getRobotFootprint();
  EXPECT_EQ(3u, footprint.size());

  EXPECT_EQ(1.5f, footprint[0].x);
  EXPECT_EQ(1.5f, footprint[0].y);
  EXPECT_EQ(0.0f, footprint[0].z);

  EXPECT_EQ(-1.5f, footprint[1].x);
  EXPECT_EQ(1.5f, footprint[1].y);
  EXPECT_EQ(0.0f, footprint[1].z);

  EXPECT_EQ(-1.5f, footprint[2].x);
  EXPECT_EQ(-1.5f, footprint[2].y);
  EXPECT_EQ(0.0f, footprint[2].z);
}

TEST_F(TestNode, radius_param)
{
  footprint_tester_->testFootprint(0, 10.0);
  std::vector<geometry_msgs::msg::Point> footprint = footprint_tester_->getRobotFootprint();

  EXPECT_EQ(16u, footprint.size());


  EXPECT_EQ(10.0f, footprint[0].x);
  EXPECT_EQ(0.0f, footprint[0].y);
  EXPECT_EQ(0.0f, footprint[0].z);


  EXPECT_NEAR(0.0f, footprint[4].x, 0.0001);
  EXPECT_NEAR(10.0f, footprint[4].y, 0.0001);
  EXPECT_EQ(0.0f, footprint[4].z);
}

TEST_F(TestNode, footprint_from_same_level_param)
{
  footprint_tester_->testFootprint(0.0, "[[1, 2], [3, 4], [5, 6]]");
  std::vector<geometry_msgs::msg::Point> footprint = footprint_tester_->getRobotFootprint();
  EXPECT_EQ(3u, footprint.size());

  EXPECT_EQ(1.0f, footprint[0].x);
  EXPECT_EQ(2.0f, footprint[0].y);
  EXPECT_EQ(0.0f, footprint[0].z);

  EXPECT_EQ(3.0f, footprint[1].x);
  EXPECT_EQ(4.0f, footprint[1].y);
  EXPECT_EQ(0.0f, footprint[1].z);

  EXPECT_EQ(5.0f, footprint[2].x);
  EXPECT_EQ(6.0f, footprint[2].y);
  EXPECT_EQ(0.0f, footprint[2].z);
}
