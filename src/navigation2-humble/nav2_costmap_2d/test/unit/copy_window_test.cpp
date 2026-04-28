













#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

TEST(CopyWindow, copyValidWindow)
{
  nav2_costmap_2d::Costmap2D src(10, 10, 0.1, 0.0, 0.0);
  nav2_costmap_2d::Costmap2D dst(5, 5, 0.2, 100.0, 100.0);

  src.setCost(2, 2, 100);
  src.setCost(5, 5, 200);

  ASSERT_TRUE(dst.copyWindow(src, 2, 2, 6, 6, 0, 0));

  ASSERT_EQ(dst.getCost(0, 0), 100);
  ASSERT_EQ(dst.getCost(3, 3), 200);
}

TEST(CopyWindow, copyInvalidWindow)
{
  nav2_costmap_2d::Costmap2D src(10, 10, 0.1, 0.0, 0.0);
  nav2_costmap_2d::Costmap2D dst(5, 5, 0.2, 100.0, 100.0);


  ASSERT_FALSE(dst.copyWindow(src, 9, 9, 11, 11, 0, 0));

  ASSERT_FALSE(dst.copyWindow(src, 0, 0, 1, 1, 5, 5));
  ASSERT_FALSE(dst.copyWindow(src, 0, 0, 6, 6, 0, 0));
}
