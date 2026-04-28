













#include <chrono>
#include <thread>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"
#include "nav2_mppi_controller/motion_models.hpp"
#include "nav2_mppi_controller/critics/constraint_critic.hpp"
#include "nav2_mppi_controller/critics/goal_angle_critic.hpp"
#include "nav2_mppi_controller/critics/goal_critic.hpp"
#include "nav2_mppi_controller/critics/obstacles_critic.hpp"
#include "nav2_mppi_controller/critics/path_align_critic.hpp"
#include "nav2_mppi_controller/critics/path_angle_critic.hpp"
#include "nav2_mppi_controller/critics/path_follow_critic.hpp"
#include "nav2_mppi_controller/critics/prefer_forward_critic.hpp"
#include "nav2_mppi_controller/critics/twirling_critic.hpp"
#include "utils_test.cpp"





using namespace mppi;
using namespace mppi::critics;
using namespace mppi::utils;
using xt::evaluation_strategy::immediate;

TEST(CriticTests, ConstraintsCritic)
{

  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap");
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);

  models::State state;
  models::ControlSequence control_sequence;
  models::Trajectories generated_trajectories;
  models::Path path;
  xt::xtensor<float, 1> costs = xt::zeros<float>({1000});
  float model_dt = 0.1;
  CriticData data =
  {state, generated_trajectories, path, costs, model_dt, false, nullptr, nullptr, std::nullopt,
    std::nullopt};
  data.motion_model = std::make_shared<DiffDriveMotionModel>();




  ConstraintCritic critic;
  critic.on_configure(node, "mppi", "critic", costmap_ros, &param_handler);
  EXPECT_EQ(critic.getName(), "critic");
  EXPECT_TRUE(critic.getMaxVelConstraint() > 0.0);
  EXPECT_TRUE(critic.getMinVelConstraint() < 0.0);




  state.vx = 0.40 * xt::ones<float>({1000, 30});
  state.vy = xt::zeros<float>({1000, 30});
  state.wz = xt::ones<float>({1000, 30});
  critic.score(data);
  EXPECT_NEAR(xt::sum(costs, immediate)(), 0, 1e-6);


  auto last_batch_traj_in_full = xt::view(state.vx, -1, xt::all());
  last_batch_traj_in_full = 0.60 * xt::ones<float>({30});
  critic.score(data);
  EXPECT_GT(xt::sum(costs, immediate)(), 0);

  EXPECT_NEAR(costs(999), 1.2, 0.01);
  costs = xt::zeros<float>({1000});


  auto first_batch_traj_in_full = xt::view(state.vx, 1, xt::all());
  first_batch_traj_in_full = -0.45 * xt::ones<float>({30});
  critic.score(data);
  EXPECT_GT(xt::sum(costs, immediate)(), 0);

  EXPECT_NEAR(costs(1), 1.2, 0.01);
  costs = xt::zeros<float>({1000});


  state.vx = 0.40 * xt::ones<float>({1000, 30});
  state.wz = 1.5 * xt::ones<float>({1000, 30});
  data.motion_model = std::make_shared<AckermannMotionModel>(&param_handler);
  critic.score(data);
  EXPECT_NEAR(xt::sum(costs, immediate)(), 0, 1e-6);


  state.wz = 2.5 * xt::ones<float>({1000, 30});
  critic.score(data);
  EXPECT_GT(xt::sum(costs, immediate)(), 0);

  EXPECT_NEAR(costs(1), 0.48, 0.01);
}

TEST(CriticTests, GoalAngleCritic)
{

  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap");
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);

  models::State state;
  models::ControlSequence control_sequence;
  models::Trajectories generated_trajectories;
  generated_trajectories.reset(1000, 30);
  models::Path path;
  xt::xtensor<float, 1> costs = xt::zeros<float>({1000});
  float model_dt = 0.1;
  CriticData data =
  {state, generated_trajectories, path, costs, model_dt, false, nullptr, nullptr, std::nullopt,
    std::nullopt};
  data.motion_model = std::make_shared<DiffDriveMotionModel>();




  GoalAngleCritic critic;
  critic.on_configure(node, "mppi", "critic", costmap_ros, &param_handler);
  EXPECT_EQ(critic.getName(), "critic");




  state.pose.pose.position.x = 1.0;
  path.reset(10);
  path.x(9) = 10.0;
  path.y(9) = 0.0;
  path.yaws(9) = 3.14;
  critic.score(data);
  EXPECT_NEAR(xt::sum(costs, immediate)(), 0, 1e-6);


  state.pose.pose.position.x = 9.2;
  critic.score(data);
  EXPECT_NEAR(xt::sum(costs, immediate)(), 0, 1e-6);


  state.pose.pose.position.x = 9.7;
  critic.score(data);
  EXPECT_GT(xt::sum(costs, immediate)(), 0);
  EXPECT_NEAR(costs(0), 9.42, 0.02);
}

TEST(CriticTests, GoalCritic)
{

  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap");
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);

  models::State state;
  models::ControlSequence control_sequence;
  models::Trajectories generated_trajectories;
  generated_trajectories.reset(1000, 30);
  models::Path path;
  xt::xtensor<float, 1> costs = xt::zeros<float>({1000});
  float model_dt = 0.1;
  CriticData data =
  {state, generated_trajectories, path, costs, model_dt, false, nullptr, nullptr, std::nullopt,
    std::nullopt};
  data.motion_model = std::make_shared<DiffDriveMotionModel>();




  GoalCritic critic;
  critic.on_configure(node, "mppi", "critic", costmap_ros, &param_handler);
  EXPECT_EQ(critic.getName(), "critic");




  state.pose.pose.position.x = 1.0;
  path.reset(10);
  path.x(9) = 10.0;
  path.y(9) = 0.0;
  critic.score(data);
  EXPECT_NEAR(costs(2), 0.0, 1e-6);
  EXPECT_NEAR(xt::sum(costs, immediate)(), 0.0, 1e-6);
  costs = xt::zeros<float>({1000});


  path.x(9) = 0.5;
  path.y(9) = 0.0;
  critic.score(data);
  EXPECT_NEAR(costs(2), 2.5, 1e-6);
  EXPECT_NEAR(xt::sum(costs, immediate)(), 2500.0, 1e-6);
}

TEST(CriticTests, PathAngleCritic)
{

  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap");
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);

  models::State state;
  state.reset(1000, 30);
  models::ControlSequence control_sequence;
  models::Trajectories generated_trajectories;
  generated_trajectories.reset(1000, 30);
  models::Path path;
  xt::xtensor<float, 1> costs = xt::zeros<float>({1000});
  float model_dt = 0.1;
  CriticData data =
  {state, generated_trajectories, path, costs, model_dt, false, nullptr, nullptr, std::nullopt,
    std::nullopt};
  data.motion_model = std::make_shared<DiffDriveMotionModel>();
  TestGoalChecker goal_checker;




  PathAngleCritic critic;
  critic.on_configure(node, "mppi", "critic", costmap_ros, &param_handler);
  EXPECT_EQ(critic.getName(), "critic");




  state.pose.pose.position.x = 0.0;
  state.pose.pose.position.y = 0.0;
  path.reset(10);
  path.x(9) = 0.15;
  critic.score(data);
  EXPECT_NEAR(xt::sum(costs, immediate)(), 0.0, 1e-6);


  path.x(9) = 0.95;
  data.furthest_reached_path_point = 2;
  path.x(6) = 1.0;
  path.y(6) = 0.0;
  critic.score(data);
  EXPECT_NEAR(xt::sum(costs, immediate)(), 0.0, 1e-6);


  path.x(6) = -1.0;
  path.y(6) = 4.0;
  critic.score(data);
  EXPECT_GT(xt::sum(costs, immediate)(), 0.0);
  EXPECT_NEAR(costs(0), 3.6315, 1e-2);
}

TEST(CriticTests, PreferForwardCritic)
{

  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap");
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);

  models::State state;
  state.reset(1000, 30);
  models::ControlSequence control_sequence;
  models::Trajectories generated_trajectories;
  generated_trajectories.reset(1000, 30);
  models::Path path;
  xt::xtensor<float, 1> costs = xt::zeros<float>({1000});
  float model_dt = 0.1;
  CriticData data =
  {state, generated_trajectories, path, costs, model_dt, false, nullptr, nullptr, std::nullopt,
    std::nullopt};
  data.motion_model = std::make_shared<DiffDriveMotionModel>();
  TestGoalChecker goal_checker;




  PreferForwardCritic critic;
  critic.on_configure(node, "mppi", "critic", costmap_ros, &param_handler);
  EXPECT_EQ(critic.getName(), "critic");




  state.pose.pose.position.x = 1.0;
  path.reset(10);
  path.x(9) = 10.0;
  critic.score(data);
  EXPECT_NEAR(xt::sum(costs, immediate)(), 0.0, 1e-6);


  path.x(9) = 0.15;
  state.vx = xt::ones<float>({1000, 30});
  critic.score(data);
  EXPECT_NEAR(xt::sum(costs, immediate)(), 0.0, 1e-6);


  state.vx = -1.0 * xt::ones<float>({1000, 30});
  critic.score(data);
  EXPECT_GT(xt::sum(costs, immediate)(), 0.0);
  EXPECT_NEAR(costs(0), 15.0, 1e-6);
}

TEST(CriticTests, TwirlingCritic)
{

  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap");
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);

  models::State state;
  state.reset(1000, 30);
  models::ControlSequence control_sequence;
  models::Trajectories generated_trajectories;
  generated_trajectories.reset(1000, 30);
  models::Path path;
  xt::xtensor<float, 1> costs = xt::zeros<float>({1000});
  float model_dt = 0.1;
  CriticData data =
  {state, generated_trajectories, path, costs, model_dt, false, nullptr, nullptr, std::nullopt,
    std::nullopt};
  data.motion_model = std::make_shared<DiffDriveMotionModel>();
  TestGoalChecker goal_checker;
  data.goal_checker = &goal_checker;




  TwirlingCritic critic;
  critic.on_configure(node, "mppi", "critic", costmap_ros, &param_handler);
  EXPECT_EQ(critic.getName(), "critic");




  state.pose.pose.position.x = 1.0;
  path.reset(10);
  path.x(9) = 10.0;
  critic.score(data);
  EXPECT_NEAR(xt::sum(costs, immediate)(), 0.0, 1e-6);


  path.x(9) = 0.15;
  state.wz = xt::zeros<float>({1000, 30});
  critic.score(data);
  EXPECT_NEAR(xt::sum(costs, immediate)(), 0.0, 1e-6);


  auto traj_view = xt::view(state.wz, 0, xt::all());
  traj_view = 10.0;
  critic.score(data);
  EXPECT_NEAR(costs(0), 100.0, 1e-6);
  costs = xt::zeros<float>({1000});


  traj_view = xt::random::randn<float>({30}, 0.0, 0.5);
  critic.score(data);
  EXPECT_NEAR(costs(0), 3.3, 4e-1);
}

TEST(CriticTests, PathFollowCritic)
{

  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap");
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);

  models::State state;
  state.reset(1000, 30);
  models::ControlSequence control_sequence;
  models::Trajectories generated_trajectories;
  generated_trajectories.reset(1000, 30);
  models::Path path;
  xt::xtensor<float, 1> costs = xt::zeros<float>({1000});
  float model_dt = 0.1;
  CriticData data =
  {state, generated_trajectories, path, costs, model_dt, false, nullptr, nullptr, std::nullopt,
    std::nullopt};
  data.motion_model = std::make_shared<DiffDriveMotionModel>();
  TestGoalChecker goal_checker;
  data.goal_checker = &goal_checker;




  PathFollowCritic critic;
  critic.on_configure(node, "mppi", "critic", costmap_ros, &param_handler);
  EXPECT_EQ(critic.getName(), "critic");




  state.pose.pose.position.x = 2.0;
  path.reset(6);
  path.x(5) = 1.7;
  critic.score(data);
  EXPECT_NEAR(xt::sum(costs, immediate)(), 0.0, 1e-6);



  path.x(5) = 0.15;
  critic.score(data);
  EXPECT_NEAR(xt::sum(costs, immediate)(), 750.0, 1e-2);
}

TEST(CriticTests, PathAlignCritic)
{

  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap");
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);

  models::State state;
  state.reset(1000, 30);
  models::ControlSequence control_sequence;
  models::Trajectories generated_trajectories;
  generated_trajectories.reset(1000, 30);
  models::Path path;
  xt::xtensor<float, 1> costs = xt::zeros<float>({1000});
  float model_dt = 0.1;
  CriticData data =
  {state, generated_trajectories, path, costs, model_dt, false, nullptr, nullptr, std::nullopt,
    std::nullopt};
  data.motion_model = std::make_shared<DiffDriveMotionModel>();
  TestGoalChecker goal_checker;
  data.goal_checker = &goal_checker;




  PathAlignCritic critic;
  critic.on_configure(node, "mppi", "critic", costmap_ros, &param_handler);
  EXPECT_EQ(critic.getName(), "critic");




  state.pose.pose.position.x = 1.0;
  path.reset(10);
  path.x(9) = 0.85;
  critic.score(data);
  EXPECT_NEAR(xt::sum(costs, immediate)(), 0.0, 1e-6);



  path.x(9) = 0.15;
  critic.score(data);
  EXPECT_NEAR(xt::sum(costs, immediate)(), 0.0, 1e-6);



  *data.furthest_reached_path_point = 21;
  path.x(9) = 0.15;
  critic.score(data);
  EXPECT_NEAR(xt::sum(costs, immediate)(), 0.0, 1e-6);



  state.pose.pose.position.x = 0.0;
  data.path_pts_valid.reset();
  path.reset(22);
  path.x(0) = 0;
  path.x(1) = 0.1;
  path.x(2) = 0.2;
  path.x(3) = 0.3;
  path.x(4) = 0.4;
  path.x(5) = 0.5;
  path.x(6) = 0.6;
  path.x(7) = 0.7;
  path.x(8) = 0.8;
  path.x(9) = 0.9;
  path.x(10) = 0.9;
  path.x(11) = 0.9;
  path.x(12) = 0.9;
  path.x(13) = 0.9;
  path.x(14) = 0.9;
  path.x(15) = 0.9;
  path.x(16) = 0.9;
  path.x(17) = 0.9;
  path.x(18) = 0.9;
  path.x(19) = 0.9;
  path.x(20) = 0.9;
  path.x(21) = 0.9;
  generated_trajectories.x = 0.66 * xt::ones<float>({1000, 30});
  critic.score(data);

  EXPECT_NEAR(xt::sum(costs, immediate)(), 400.0, 1e-2);



  auto * costmap = costmap_ros->getCostmap();

  for (unsigned int i = 11; i <= 30; ++i) {
    for (unsigned int j = 11; j <= 30; ++j) {
      costmap->setCost(i, j, 254);
    }
  }

  data.path_pts_valid.reset();
  costs = xt::zeros<float>({1000});
  path.x = 1.5 * xt::ones<float>({22});
  path.y = 1.5 * xt::ones<float>({22});
  critic.score(data);
  EXPECT_NEAR(xt::sum(costs, immediate)(), 0.0, 1e-6);
}
