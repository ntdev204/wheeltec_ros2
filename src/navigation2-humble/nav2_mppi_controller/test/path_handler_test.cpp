













#include <chrono>
#include <thread>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_mppi_controller/tools/path_handler.hpp"
#include "tf2_ros/transform_broadcaster.h"



class RosLockGuard
{
public:
  RosLockGuard() {rclcpp::init(0, nullptr);}
  ~RosLockGuard() {rclcpp::shutdown();}
};
RosLockGuard g_rclcpp;

using namespace mppi;

class PathHandlerWrapper : public PathHandler
{
public:
  PathHandlerWrapper()
  : PathHandler() {}

  void pruneGlobalPlanWrapper(nav_msgs::msg::Path & path, const PathIterator end)
  {
    return prunePlan(path, end);
  }

  double getMaxCostmapDistWrapper()
  {
    return getMaxCostmapDist();
  }

  std::pair<nav_msgs::msg::Path, PathIterator>
  getGlobalPlanConsideringBoundsInCostmapFrameWrapper(const geometry_msgs::msg::PoseStamped & pose)
  {
    return getGlobalPlanConsideringBoundsInCostmapFrame(pose);
  }

  bool transformPoseWrapper(
    const std::string & frame, const geometry_msgs::msg::PoseStamped & in_pose,
    geometry_msgs::msg::PoseStamped & out_pose) const
  {
    return transformPose(frame, in_pose, out_pose);
  }

  geometry_msgs::msg::PoseStamped transformToGlobalPlanFrameWrapper(
    const geometry_msgs::msg::PoseStamped & pose)
  {
    return transformToGlobalPlanFrame(pose);
  }

  void setGlobalPlanUpToInversion(const nav_msgs::msg::Path & path)
  {
    global_plan_up_to_inversion_ = path;
  }

  bool isWithinInversionTolerancesWrapper(const geometry_msgs::msg::PoseStamped & robot_pose)
  {
    return isWithinInversionTolerances(robot_pose);
  }

  nav_msgs::msg::Path & getInvertedPath()
  {
    return global_plan_up_to_inversion_;
  }
};

TEST(PathHandlerTests, GetAndPrunePath)
{
  nav_msgs::msg::Path path;
  PathHandlerWrapper handler;

  path.header.frame_id = "fkframe";
  path.poses.resize(11);

  handler.setPath(path);
  auto & rtn_path = handler.getPath();
  EXPECT_EQ(path.header.frame_id, rtn_path.header.frame_id);
  EXPECT_EQ(path.poses.size(), rtn_path.poses.size());

  PathIterator it = rtn_path.poses.begin() + 5;
  handler.pruneGlobalPlanWrapper(rtn_path, it);
  auto rtn2_path = handler.getPath();
  EXPECT_EQ(rtn2_path.poses.size(), 6u);
}

TEST(PathHandlerTests, TestBounds)
{
  PathHandlerWrapper handler;
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  node->declare_parameter("dummy.max_robot_pose_search_dist", rclcpp::ParameterValue(99999.9));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap");
  auto results = costmap_ros->set_parameters_atomically(
    {rclcpp::Parameter("global_frame", "odom"),
      rclcpp::Parameter("robot_base_frame", "base_link")});
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State state;
  costmap_ros->on_configure(state);


  handler.initialize(node, "dummy", costmap_ros, costmap_ros->getTfBuffer(), &param_handler);
  EXPECT_EQ(handler.getMaxCostmapDistWrapper(), 2.5);


  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ =
    std::make_unique<tf2_ros::TransformBroadcaster>(node);
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = "map";
  t.child_frame_id = "base_link";
  tf_broadcaster_->sendTransform(t);
  t.header.frame_id = "map";
  t.child_frame_id = "odom";
  tf_broadcaster_->sendTransform(t);


  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.poses.resize(100);
  for (unsigned int i = 0; i != path.poses.size(); i++) {
    path.poses[i].pose.position.x = i;
    path.poses[i].header.frame_id = "map";
  }
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = "odom";
  robot_pose.pose.position.x = 25.0;

  handler.setPath(path);
  auto [transformed_plan, closest] =
    handler.getGlobalPlanConsideringBoundsInCostmapFrameWrapper(robot_pose);
  auto & path_inverted = handler.getInvertedPath();
  EXPECT_EQ(closest - path_inverted.poses.begin(), 25);
  handler.pruneGlobalPlanWrapper(path_inverted, closest);
  auto & path_pruned = handler.getInvertedPath();
  EXPECT_EQ(path_pruned.poses.size(), 75u);
}

TEST(PathHandlerTests, TestTransforms)
{
  PathHandlerWrapper handler;
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  node->declare_parameter("dummy.max_robot_pose_search_dist", rclcpp::ParameterValue(99999.9));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap");
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State state;
  costmap_ros->on_configure(state);


  handler.initialize(node, "dummy", costmap_ros, costmap_ros->getTfBuffer(), &param_handler);


  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ =
    std::make_unique<tf2_ros::TransformBroadcaster>(node);
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = "map";
  t.child_frame_id = "base_link";
  tf_broadcaster_->sendTransform(t);
  t.header.frame_id = "map";
  t.child_frame_id = "odom";
  tf_broadcaster_->sendTransform(t);

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.poses.resize(100);
  for (unsigned int i = 0; i != path.poses.size(); i++) {
    path.poses[i].pose.position.x = i;
    path.poses[i].header.frame_id = "map";
  }

  geometry_msgs::msg::PoseStamped robot_pose, output_pose;
  robot_pose.header.frame_id = "odom";
  robot_pose.pose.position.x = 2.5;

  EXPECT_TRUE(handler.transformPoseWrapper("map", robot_pose, output_pose));
  EXPECT_EQ(output_pose.pose.position.x, 2.5);

  EXPECT_THROW(handler.transformToGlobalPlanFrameWrapper(robot_pose), std::runtime_error);
  handler.setPath(path);
  EXPECT_NO_THROW(handler.transformToGlobalPlanFrameWrapper(robot_pose));

  auto [path_out, closest] =
    handler.getGlobalPlanConsideringBoundsInCostmapFrameWrapper(robot_pose);


  auto final_path = handler.transformPath(robot_pose);
  EXPECT_EQ(final_path.poses.size(), path_out.poses.size());
}

TEST(PathHandlerTests, TestInversionToleranceChecks)
{
  nav_msgs::msg::Path path;
  for (unsigned int i = 0; i != 10; i++) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = static_cast<double>(i);
    path.poses.push_back(pose);
  }
  path.poses.back().pose.orientation.w = 1;

  PathHandlerWrapper handler;
  handler.setGlobalPlanUpToInversion(path);


  geometry_msgs::msg::PoseStamped robot_pose;
  EXPECT_FALSE(handler.isWithinInversionTolerancesWrapper(robot_pose));


  robot_pose.pose.position.x = 9;
  robot_pose.pose.orientation.w = 1.0;
  EXPECT_TRUE(handler.isWithinInversionTolerancesWrapper(robot_pose));


  robot_pose.pose.position.y = 9;
  EXPECT_FALSE(handler.isWithinInversionTolerancesWrapper(robot_pose));


  robot_pose.pose.position.y = 0;
  robot_pose.pose.orientation.z = 0.8509035;
  robot_pose.pose.orientation.w = 0.525322;
  EXPECT_FALSE(handler.isWithinInversionTolerancesWrapper(robot_pose));


  robot_pose.pose.position.y = 0;
  robot_pose.pose.orientation.w = 0.9961947;
  robot_pose.pose.orientation.z = 0.0871558;
  EXPECT_TRUE(handler.isWithinInversionTolerancesWrapper(robot_pose));


  robot_pose.pose.position.x = 9.10;
  EXPECT_TRUE(handler.isWithinInversionTolerancesWrapper(robot_pose));
}
