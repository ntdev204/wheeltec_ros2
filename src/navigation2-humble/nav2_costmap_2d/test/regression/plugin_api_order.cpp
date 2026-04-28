













#include <string>
#include <vector>
#include <memory>

#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <gtest/gtest.h>

TEST(CostmapPluginsTester, checkPluginAPIOrder)
{
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros =
    std::make_shared<nav2_costmap_2d::Costmap2DROS>("costmap_ros");


  costmap_ros->set_parameter(rclcpp::Parameter("robot_base_frame", "map"));

  std::vector<std::string> plugins_str;
  plugins_str.push_back("order_layer");
  costmap_ros->set_parameter(rclcpp::Parameter("plugins", plugins_str));
  costmap_ros->declare_parameter(
    "order_layer.plugin",
    rclcpp::ParameterValue(std::string("nav2_costmap_2d::OrderLayer")));



  costmap_ros->on_configure(costmap_ros->get_current_state());
  costmap_ros->on_activate(costmap_ros->get_current_state());


  costmap_ros->on_deactivate(costmap_ros->get_current_state());
  costmap_ros->on_cleanup(costmap_ros->get_current_state());
  costmap_ros->on_shutdown(costmap_ros->get_current_state());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
