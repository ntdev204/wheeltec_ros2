













#include <chrono>
#include <thread>

#include <xtensor/xrandom.hpp>
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"
#include "nav2_mppi_controller/models/path.hpp"



class RosLockGuard
{
public:
  RosLockGuard() {rclcpp::init(0, nullptr);}
  ~RosLockGuard() {rclcpp::shutdown();}
};
RosLockGuard g_rclcpp;

using namespace mppi::utils;
using namespace mppi;

class TestGoalChecker : public nav2_core::GoalChecker
{
public:
  TestGoalChecker() {}

  virtual void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & ,
    const std::string & ,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) {}

  virtual void reset() {}

  virtual bool isGoalReached(
    const geometry_msgs::msg::Pose & ,
    const geometry_msgs::msg::Pose & ,
    const geometry_msgs::msg::Twist & ) {return false;}

  virtual bool getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance,
    geometry_msgs::msg::Twist & )
  {
    pose_tolerance.position.x = 0.25;
    pose_tolerance.position.y = 0.25;
    return true;
  }
};

TEST(UtilsTests, MarkerPopulationUtils)
{
  auto pose = createPose(1.0, 2.0, 3.0);
  EXPECT_EQ(pose.position.x, 1.0);
  EXPECT_EQ(pose.position.y, 2.0);
  EXPECT_EQ(pose.position.z, 3.0);
  EXPECT_EQ(pose.orientation.w, 1.0);

  auto scale = createScale(1.0, 2.0, 3.0);
  EXPECT_EQ(scale.x, 1.0);
  EXPECT_EQ(scale.y, 2.0);
  EXPECT_EQ(scale.z, 3.0);

  auto color = createColor(1.0, 2.0, 3.0, 0.0);
  EXPECT_EQ(color.r, 1.0);
  EXPECT_EQ(color.g, 2.0);
  EXPECT_EQ(color.b, 3.0);
  EXPECT_EQ(color.a, 0.0);

  auto marker = createMarker(999, pose, scale, color, "map", "ns");
  EXPECT_EQ(marker.header.frame_id, "map");
  EXPECT_EQ(marker.id, 999);
  EXPECT_EQ(marker.pose, pose);
  EXPECT_EQ(marker.scale, scale);
  EXPECT_EQ(marker.color, color);
  EXPECT_EQ(marker.ns, "ns");
}

TEST(UtilsTests, ConversionTests)
{
  geometry_msgs::msg::TwistStamped output;
  builtin_interfaces::msg::Time time;


  output = toTwistStamped(0.5, 0.3, time, "map");
  EXPECT_NEAR(output.twist.linear.x, 0.5, 1e-6);
  EXPECT_NEAR(output.twist.linear.y, 0.0, 1e-6);
  EXPECT_NEAR(output.twist.angular.z, 0.3, 1e-6);
  EXPECT_EQ(output.header.frame_id, "map");
  EXPECT_EQ(output.header.stamp, time);

  output = toTwistStamped(0.5, 0.4, 0.3, time, "map");
  EXPECT_NEAR(output.twist.linear.x, 0.5, 1e-6);
  EXPECT_NEAR(output.twist.linear.y, 0.4, 1e-6);
  EXPECT_NEAR(output.twist.angular.z, 0.3, 1e-6);
  EXPECT_EQ(output.header.frame_id, "map");
  EXPECT_EQ(output.header.stamp, time);

  nav_msgs::msg::Path path;
  path.poses.resize(5);
  path.poses[2].pose.position.x = 5;
  path.poses[2].pose.position.y = 50;
  models::Path path_t = toTensor(path);


  EXPECT_EQ(path_t.x.shape(0), 5u);
  EXPECT_EQ(path_t.y.shape(0), 5u);
  EXPECT_EQ(path_t.yaws.shape(0), 5u);
  EXPECT_EQ(path_t.x(2), 5);
  EXPECT_EQ(path_t.y(2), 50);
  EXPECT_NEAR(path_t.yaws(2), 0.0, 1e-6);
}

TEST(UtilsTests, WithTolTests)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = 10.0;
  pose.position.y = 1.0;

  nav2_core::GoalChecker * goal_checker = new TestGoalChecker;


  nav_msgs::msg::Path path;
  path.poses.resize(2);
  path.poses[1].pose.position.x = 0.0;
  path.poses[1].pose.position.y = 0.0;
  models::Path path_t = toTensor(path);
  EXPECT_FALSE(withinPositionGoalTolerance(goal_checker, pose, path_t));
  EXPECT_FALSE(withinPositionGoalTolerance(0.25, pose, path_t));


  path.poses[1].pose.position.x = 9.8;
  path.poses[1].pose.position.y = 0.95;
  path_t = toTensor(path);
  EXPECT_TRUE(withinPositionGoalTolerance(goal_checker, pose, path_t));
  EXPECT_TRUE(withinPositionGoalTolerance(0.25, pose, path_t));

  path.poses[1].pose.position.x = 10.0;
  path.poses[1].pose.position.y = 0.76;
  path_t = toTensor(path);
  EXPECT_TRUE(withinPositionGoalTolerance(goal_checker, pose, path_t));
  EXPECT_TRUE(withinPositionGoalTolerance(0.25, pose, path_t));

  path.poses[1].pose.position.x = 9.76;
  path.poses[1].pose.position.y = 1.0;
  path_t = toTensor(path);
  EXPECT_TRUE(withinPositionGoalTolerance(goal_checker, pose, path_t));
  EXPECT_TRUE(withinPositionGoalTolerance(0.25, pose, path_t));

  delete goal_checker;
  goal_checker = nullptr;
  EXPECT_FALSE(withinPositionGoalTolerance(goal_checker, pose, path_t));
}

TEST(UtilsTests, AnglesTests)
{

  xt::xtensor<float, 1> angles, zero_angles;
  angles = xt::ones<float>({100});
  for (unsigned int i = 0; i != angles.shape(0); i++) {
    angles(i) = i * i;
    if (i % 2 == 0) {
      angles(i) *= -1;
    }
  }

  auto norm_ang = normalize_angles(angles);
  for (unsigned int i = 0; i != norm_ang.shape(0); i++) {
    EXPECT_TRUE((norm_ang(i) >= -M_PI) && (norm_ang(i) <= M_PI));
  }


  zero_angles = xt::zeros<float>({100});
  auto ang_dist = shortest_angular_distance(angles, zero_angles);
  for (unsigned int i = 0; i != ang_dist.shape(0); i++) {
    EXPECT_TRUE((ang_dist(i) >= -M_PI) && (ang_dist(i) <= M_PI));
  }


  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.orientation.w = 1.0;
  double point_x = 1.0, point_y = 0.0;
  bool forward_preference = true;
  EXPECT_NEAR(posePointAngle(pose, point_x, point_y, forward_preference), 0.0, 1e-6);
  forward_preference = false;
  EXPECT_NEAR(posePointAngle(pose, point_x, point_y, forward_preference), 0.0, 1e-6);
  point_x = -1.0;
  EXPECT_NEAR(posePointAngle(pose, point_x, point_y, forward_preference), 0.0, 1e-6);
  forward_preference = true;
  EXPECT_NEAR(posePointAngle(pose, point_x, point_y, forward_preference), M_PI, 1e-6);
}

TEST(UtilsTests, FurthestAndClosestReachedPoint)
{
  models::State state;
  models::Trajectories generated_trajectories;
  models::Path path;
  xt::xtensor<float, 1> costs;
  float model_dt = 0.1;

  CriticData data =
  {state, generated_trajectories, path, costs, model_dt, false, nullptr, nullptr,
    std::nullopt, std::nullopt};


  data.furthest_reached_path_point = 99999;
  setPathFurthestPointIfNotSet(data);
  EXPECT_EQ(data.furthest_reached_path_point, 99999);


  CriticData data2 =
  {state, generated_trajectories, path, costs, model_dt, false, nullptr, nullptr,
    std::nullopt, std::nullopt};
  setPathFurthestPointIfNotSet(data2);
  EXPECT_EQ(data2.furthest_reached_path_point, 0);


  generated_trajectories.x = xt::ones<float>({100, 2});
  generated_trajectories.y = xt::zeros<float>({100, 2});
  generated_trajectories.yaws = xt::zeros<float>({100, 2});

  nav_msgs::msg::Path plan;
  plan.poses.resize(10);
  for (unsigned int i = 0; i != plan.poses.size(); i++) {
    plan.poses[i].pose.position.x = 0.2 * i;
    plan.poses[i].pose.position.y = 0.0;
  }
  path = toTensor(plan);

  CriticData data3 =
  {state, generated_trajectories, path, costs, model_dt, false, nullptr, nullptr,
    std::nullopt, std::nullopt};
  EXPECT_EQ(findPathFurthestReachedPoint(data3), 5u);
  EXPECT_EQ(findPathTrajectoryInitialPoint(data3), 5u);
}

TEST(UtilsTests, findPathCosts)
{
  models::State state;
  models::Trajectories generated_trajectories;
  models::Path path;
  xt::xtensor<float, 1> costs;
  float model_dt = 0.1;

  CriticData data =
  {state, generated_trajectories, path, costs, model_dt, false, nullptr, nullptr,
    std::nullopt, std::nullopt};


  data.path_pts_valid = std::vector<bool>(10, false);
  for (unsigned int i = 0; i != 10; i++) {
    (*data.path_pts_valid)[i] = false;
  }
  EXPECT_TRUE(data.path_pts_valid);
  setPathCostsIfNotSet(data, nullptr);
  EXPECT_EQ(data.path_pts_valid->size(), 10u);

  CriticData data3 =
  {state, generated_trajectories, path, costs, model_dt, false, nullptr, nullptr,
    std::nullopt, std::nullopt};

  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap");
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  auto * costmap = costmap_ros->getCostmap();

  for (unsigned int i = 10; i <= 30; ++i) {
    for (unsigned int j = 10; j <= 30; ++j) {
      costmap->setCost(i, j, 254);
    }
  }
  for (unsigned int i = 40; i <= 45; ++i) {
    for (unsigned int j = 45; j <= 45; ++j) {
      costmap->setCost(i, j, 253);
    }
  }

  path.reset(50);
  path.x(1) = 999999999;
  path.y(1) = 999999999;
  path.x(10) = 1.5;
  path.y(10) = 1.5;
  path.x(20) = 4.2;
  path.y(20) = 4.2;


  setPathCostsIfNotSet(data3, costmap_ros);
  EXPECT_TRUE(data3.path_pts_valid.has_value());
  for (unsigned int i = 0; i != path.x.shape(0) - 1; i++) {
    if (i == 1 || i == 10) {
      EXPECT_FALSE((*data3.path_pts_valid)[i]);
    } else {
      EXPECT_TRUE((*data3.path_pts_valid)[i]);
    }
  }
}

TEST(UtilsTests, SmootherTest)
{
  models::ControlSequence noisey_sequence, sequence_init;
  noisey_sequence.vx = 0.2 * xt::ones<float>({30});
  noisey_sequence.vy = 0.0 * xt::ones<float>({30});
  noisey_sequence.wz = 0.3 * xt::ones<float>({30});


  auto noises = xt::random::randn<float>({30}, 0.0, 0.2);
  noisey_sequence.vx += noises;
  noisey_sequence.vy += noises;
  noisey_sequence.wz += noises;
  sequence_init = noisey_sequence;

  std::array<mppi::models::Control, 4> history, history_init;
  history[3].vx = 0.1;
  history[3].vy = 0.0;
  history[3].wz = 0.3;
  history[2].vx = 0.1;
  history[2].vy = 0.0;
  history[2].wz = 0.3;
  history[1].vx = 0.1;
  history[1].vy = 0.0;
  history[1].wz = 0.3;
  history[0].vx = 0.0;
  history[0].vy = 0.0;
  history[0].wz = 0.0;
  history_init = history;

  models::OptimizerSettings settings;
  settings.shift_control_sequence = false;

  savitskyGolayFilter(noisey_sequence, history, settings);


  EXPECT_NEAR(history_init[3].vx, history[2].vx, 0.02);
  EXPECT_NEAR(history_init[3].vy, history[2].vy, 0.02);
  EXPECT_NEAR(history_init[3].wz, history[2].wz, 0.02);


  EXPECT_NEAR(history[3].vx, 0.2, 0.05);
  EXPECT_NEAR(history[3].vy, 0.0, 0.035);
  EXPECT_NEAR(history[3].wz, 0.23, 0.02);


  float smoothed_val, original_val;
  for (unsigned int i = 0; i != noisey_sequence.vx.shape(0); i++) {
    smoothed_val += fabs(noisey_sequence.vx(i) - 0.2);
    smoothed_val += fabs(noisey_sequence.vy(i) - 0.0);
    smoothed_val += fabs(noisey_sequence.wz(i) - 0.3);

    original_val += fabs(sequence_init.vx(i) - 0.2);
    original_val += fabs(sequence_init.vy(i) - 0.0);
    original_val += fabs(sequence_init.wz(i) - 0.3);
  }

  EXPECT_LT(smoothed_val, original_val);
}

TEST(UtilsTests, FindPathInversionTest)
{

  nav_msgs::msg::Path path;
  for (unsigned int i = 0; i != 10; i++) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = i;
    path.poses.push_back(pose);
  }
  EXPECT_EQ(utils::findFirstPathInversion(path), 10u);


  path.poses.erase(path.poses.begin(), path.poses.begin() + 7);
  EXPECT_EQ(utils::findFirstPathInversion(path), 3u);



  path.poses.clear();
  for (unsigned int i = 0; i != 10; i++) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = i;
    path.poses.push_back(pose);
  }
  for (unsigned int i = 0; i != 10; i++) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = 10 - i;
    path.poses.push_back(pose);
  }
  EXPECT_EQ(utils::findFirstPathInversion(path), 11u);
}

TEST(UtilsTests, RemovePosesAfterPathInversionTest)
{
  nav_msgs::msg::Path path;

  for (unsigned int i = 0; i != 10; i++) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = i;
    path.poses.push_back(pose);
  }
  EXPECT_EQ(utils::removePosesAfterFirstInversion(path), 0u);


  path.poses.clear();
  EXPECT_EQ(utils::removePosesAfterFirstInversion(path), 0u);


  for (unsigned int i = 0; i != 10; i++) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = i;
    path.poses.push_back(pose);
  }
  for (unsigned int i = 0; i != 10; i++) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = 10 - i;
    path.poses.push_back(pose);
  }
  EXPECT_EQ(utils::removePosesAfterFirstInversion(path), 11u);

  EXPECT_EQ(path.poses.size(), 11u);
  EXPECT_EQ(path.poses.back().pose.position.x, 10);
}
