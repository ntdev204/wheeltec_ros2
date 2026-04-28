













#ifndef NAV2_UTIL__COSTMAP_HPP_
#define NAV2_UTIL__COSTMAP_HPP_

#include <vector>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_msgs/msg/costmap_meta_data.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace nav2_util
{

enum class TestCostmap
{
  open_space,
  bounded,
  bottom_left_obstacle,
  top_left_obstacle,
  maze1,
  maze2
};



class Costmap
{
public:
  typedef uint8_t CostValue;

  

  Costmap(
    rclcpp::Node * node, bool trinary_costmap = true, bool track_unknown_space = true,
    int lethal_threshold = 100, int unknown_cost_value = -1);
  Costmap() = delete;
  ~Costmap();

  

  void set_static_map(const nav_msgs::msg::OccupancyGrid & occupancy_grid);

  

  void set_test_costmap(const TestCostmap & testCostmapType);

  

  nav2_msgs::msg::Costmap get_costmap(const nav2_msgs::msg::CostmapMetaData & specifications);

  

  nav2_msgs::msg::CostmapMetaData get_properties() {return costmap_properties_;}

  

  bool is_free(const unsigned int x_coordinate, const unsigned int y_coordinate) const;

  

  bool is_free(const unsigned int index) const;


  static const CostValue no_information;
  static const CostValue lethal_obstacle;
  static const CostValue inscribed_inflated_obstacle;
  static const CostValue medium_cost;
  static const CostValue free_space;

private:
  

  std::vector<uint8_t> get_test_data(const TestCostmap configuration);

  

  uint8_t interpret_value(const int8_t value) const;


  rclcpp::Node * node_;


  nav2_msgs::msg::CostmapMetaData costmap_properties_;
  std::vector<uint8_t> costs_;


  bool trinary_costmap_;
  bool track_unknown_space_;
  int lethal_threshold_;
  int unknown_cost_value_;


  bool map_provided_;
  bool using_test_map_;
};

}

#endif
