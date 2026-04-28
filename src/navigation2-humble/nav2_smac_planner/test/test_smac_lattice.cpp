













#include <math.h>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_smac_planner/node_hybrid.hpp"
#include "nav2_smac_planner/a_star.hpp"
#include "nav2_smac_planner/collision_checker.hpp"
#include "nav2_smac_planner/smac_planner_lattice.hpp"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;


class LatticeWrap : public nav2_smac_planner::SmacPlannerLattice
{
public:
  void callDynamicParams(std::vector<rclcpp::Parameter> parameters)
  {
    dynamicParametersCallback(parameters);
  }
};





TEST(SmacTest, test_smac_lattice)
{
  rclcpp_lifecycle::LifecycleNode::SharedPtr nodeLattice =
    std::make_shared<rclcpp_lifecycle::LifecycleNode>("SmacLatticeTest");

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros =
    std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap");
  costmap_ros->on_configure(rclcpp_lifecycle::State());

  geometry_msgs::msg::PoseStamped start, goal;
  start.pose.position.x = 0.0;
  start.pose.position.y = 0.0;
  start.pose.orientation.w = 1.0;
  goal.pose.position.x = 1.0;
  goal.pose.position.y = 1.0;
  goal.pose.orientation.w = 1.0;
  auto planner = std::make_unique<nav2_smac_planner::SmacPlannerLattice>();
  try {

    planner->configure(nodeLattice, "test", nullptr, costmap_ros);
  } catch (...) {
  }
  planner->activate();

  try {
    planner->createPlan(start, goal);
  } catch (...) {
  }

  planner->deactivate();
  planner->cleanup();

  planner.reset();
  costmap_ros->on_cleanup(rclcpp_lifecycle::State());
  costmap_ros.reset();
  nodeLattice.reset();
}

TEST(SmacTest, test_smac_lattice_reconfigure)
{
  rclcpp_lifecycle::LifecycleNode::SharedPtr nodeLattice =
    std::make_shared<rclcpp_lifecycle::LifecycleNode>("SmacLatticeTest");

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros =
    std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap");
  costmap_ros->on_configure(rclcpp_lifecycle::State());

  auto planner = std::make_unique<LatticeWrap>();
  try {

    planner->configure(nodeLattice, "test", nullptr, costmap_ros);
  } catch (...) {
  }
  planner->activate();

  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
    nodeLattice->get_node_base_interface(), nodeLattice->get_node_topics_interface(),
    nodeLattice->get_node_graph_interface(),
    nodeLattice->get_node_services_interface());

  auto results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("test.allow_unknown", false),
      rclcpp::Parameter("test.max_iterations", -1),
      rclcpp::Parameter("test.cache_obstacle_heuristic", true),
      rclcpp::Parameter("test.reverse_penalty", 5.0),
      rclcpp::Parameter("test.change_penalty", 1.0),
      rclcpp::Parameter("test.non_straight_penalty", 2.0),
      rclcpp::Parameter("test.cost_penalty", 2.0),
      rclcpp::Parameter("test.retrospective_penalty", 0.2),
      rclcpp::Parameter("test.analytic_expansion_ratio", 4.0),
      rclcpp::Parameter("test.max_planning_time", 10.0),
      rclcpp::Parameter("test.lookup_table_size", 30.0),
      rclcpp::Parameter("test.smooth_path", false),
      rclcpp::Parameter("test.analytic_expansion_max_length", 42.0),
      rclcpp::Parameter("test.tolerance", 42.0),
      rclcpp::Parameter("test.rotation_penalty", 42.0),
      rclcpp::Parameter("test.max_on_approach_iterations", 42),
      rclcpp::Parameter("test.allow_reverse_expansion", true)});

  try {



    rclcpp::spin_until_future_complete(
      nodeLattice->get_node_base_interface(),
      results);
  } catch (...) {
  }


  std::vector<rclcpp::Parameter> parameters;
  parameters.push_back(rclcpp::Parameter("test.lattice_filepath", std::string("HI")));
  EXPECT_THROW(planner->callDynamicParams(parameters), std::runtime_error);
}
