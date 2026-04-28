













#include <chrono>
#include <thread>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_mppi_controller/tools/trajectory_visualizer.hpp"



class RosLockGuard
{
public:
  RosLockGuard() {rclcpp::init(0, nullptr);}
  ~RosLockGuard() {rclcpp::shutdown();}
};
RosLockGuard g_rclcpp;

using namespace mppi;

TEST(TrajectoryVisualizerTests, StateTransition)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  auto parameters_handler = std::make_unique<ParametersHandler>(node);

  TrajectoryVisualizer vis;
  vis.on_configure(node, "my_name", "map", parameters_handler.get());
  vis.on_activate();
  vis.on_deactivate();
  vis.on_cleanup();
}

TEST(TrajectoryVisualizerTests, VisPathRepub)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  auto parameters_handler = std::make_unique<ParametersHandler>(node);
  nav_msgs::msg::Path recieved_path;
  nav_msgs::msg::Path pub_path;
  pub_path.header.frame_id = "fake_frame";
  pub_path.poses.resize(5);

  auto my_sub = node->create_subscription<nav_msgs::msg::Path>(
    "transformed_global_plan", 10,
    [&](const nav_msgs::msg::Path msg) {recieved_path = msg;});

  TrajectoryVisualizer vis;
  vis.on_configure(node, "my_name", "map", parameters_handler.get());
  vis.on_activate();
  vis.visualize(pub_path);

  rclcpp::spin_some(node->get_node_base_interface());
  EXPECT_EQ(recieved_path.poses.size(), 5u);
  EXPECT_EQ(recieved_path.header.frame_id, "fake_frame");
}

TEST(TrajectoryVisualizerTests, VisOptimalTrajectory)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  auto parameters_handler = std::make_unique<ParametersHandler>(node);

  visualization_msgs::msg::MarkerArray recieved_msg;
  auto my_sub = node->create_subscription<visualization_msgs::msg::MarkerArray>(
    "/trajectories", 10,
    [&](const visualization_msgs::msg::MarkerArray msg) {recieved_msg = msg;});


  xt::xtensor<float, 2> optimal_trajectory;
  TrajectoryVisualizer vis;
  vis.on_configure(node, "my_name", "fkmap", parameters_handler.get());
  vis.on_activate();
  vis.add(optimal_trajectory, "Optimal Trajectory");
  nav_msgs::msg::Path bogus_path;
  vis.visualize(bogus_path);

  rclcpp::spin_some(node->get_node_base_interface());
  EXPECT_EQ(recieved_msg.markers.size(), 0u);


  optimal_trajectory = xt::ones<float>({20, 2});
  vis.add(optimal_trajectory, "Optimal Trajectory");
  vis.visualize(bogus_path);

  rclcpp::spin_some(node->get_node_base_interface());


  EXPECT_EQ(recieved_msg.markers.size(), 20u);
  EXPECT_EQ(recieved_msg.markers[0].header.frame_id, "fkmap");


  EXPECT_EQ(recieved_msg.markers[0].id, 0);
  EXPECT_EQ(recieved_msg.markers[1].id, 1);
  EXPECT_EQ(recieved_msg.markers[10].id, 10);


  EXPECT_EQ(recieved_msg.markers[0].pose.position.x, 1);
  EXPECT_EQ(recieved_msg.markers[0].pose.position.y, 1);
  EXPECT_EQ(recieved_msg.markers[0].pose.position.z, 0.06);


  EXPECT_EQ(recieved_msg.markers[0].scale.x, 0.03);
  EXPECT_EQ(recieved_msg.markers[0].scale.y, 0.03);
  EXPECT_EQ(recieved_msg.markers[0].scale.z, 0.07);

  EXPECT_EQ(recieved_msg.markers[19].scale.x, 0.07);
  EXPECT_EQ(recieved_msg.markers[19].scale.y, 0.07);
  EXPECT_EQ(recieved_msg.markers[19].scale.z, 0.09);


  for (unsigned int i = 0; i != recieved_msg.markers.size() - 1; i++) {
    EXPECT_LT(recieved_msg.markers[i].color.g, recieved_msg.markers[i + 1].color.g);
    EXPECT_LT(recieved_msg.markers[i].color.b, recieved_msg.markers[i + 1].color.b);
    EXPECT_EQ(recieved_msg.markers[i].color.r, recieved_msg.markers[i + 1].color.r);
    EXPECT_EQ(recieved_msg.markers[i].color.a, recieved_msg.markers[i + 1].color.a);
  }
}

TEST(TrajectoryVisualizerTests, VisCandidateTrajectories)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  auto parameters_handler = std::make_unique<ParametersHandler>(node);

  visualization_msgs::msg::MarkerArray recieved_msg;
  auto my_sub = node->create_subscription<visualization_msgs::msg::MarkerArray>(
    "/trajectories", 10,
    [&](const visualization_msgs::msg::MarkerArray msg) {recieved_msg = msg;});

  models::Trajectories candidate_trajectories;
  candidate_trajectories.x = xt::ones<float>({200, 12});
  candidate_trajectories.y = xt::ones<float>({200, 12});
  candidate_trajectories.yaws = xt::ones<float>({200, 12});

  TrajectoryVisualizer vis;
  vis.on_configure(node, "my_name", "fkmap", parameters_handler.get());
  vis.on_activate();
  vis.add(candidate_trajectories, "Candidate Trajectories");
  nav_msgs::msg::Path bogus_path;
  vis.visualize(bogus_path);

  rclcpp::spin_some(node->get_node_base_interface());

  EXPECT_EQ(recieved_msg.markers.size(), 160u);
}
