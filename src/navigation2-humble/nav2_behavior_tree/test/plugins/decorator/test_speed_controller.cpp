














#include <gtest/gtest.h>
#include <chrono>
#include <memory>
#include <set>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/robot_utils.hpp"

#include "../../test_behavior_tree_fixture.hpp"
#include "../../test_dummy_tree_node.hpp"
#include "nav2_behavior_tree/plugins/decorator/speed_controller.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;

class SpeedControllerTestFixture : public nav2_behavior_tree::BehaviorTreeTestFixture
{
public:
  void SetUp()
  {
    odom_smoother_ = std::make_shared<nav2_util::OdomSmoother>(node_);
    config_->blackboard->set<std::shared_ptr<nav2_util::OdomSmoother>>(
      "odom_smoother", odom_smoother_);

    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = node_->now();
    config_->blackboard->set("goal", goal);

    std::vector<geometry_msgs::msg::PoseStamped> fake_poses;
    config_->blackboard->set<std::vector<geometry_msgs::msg::PoseStamped>>("goals", fake_poses);

    bt_node_ = std::make_shared<nav2_behavior_tree::SpeedController>("speed_controller", *config_);
    dummy_node_ = std::make_shared<nav2_behavior_tree::DummyNode>();
    bt_node_->setChild(dummy_node_.get());
  }

  void TearDown()
  {
    dummy_node_.reset();
    bt_node_.reset();
    odom_smoother_.reset();
  }

protected:
  static std::shared_ptr<nav2_util::OdomSmoother> odom_smoother_;
  static std::shared_ptr<nav2_behavior_tree::SpeedController> bt_node_;
  static std::shared_ptr<nav2_behavior_tree::DummyNode> dummy_node_;
};

std::shared_ptr<nav2_util::OdomSmoother>
SpeedControllerTestFixture::odom_smoother_ = nullptr;
std::shared_ptr<nav2_behavior_tree::SpeedController>
SpeedControllerTestFixture::bt_node_ = nullptr;
std::shared_ptr<nav2_behavior_tree::DummyNode>
SpeedControllerTestFixture::dummy_node_ = nullptr;



TEST_F(SpeedControllerTestFixture, test_behavior)
{
  auto odom_pub = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 1);
  nav_msgs::msg::Odometry odom_msg;

  auto time = node_->now();
  odom_msg.header.stamp = time;
  odom_msg.twist.twist.linear.x = 0.223;
  odom_pub->publish(odom_msg);

  EXPECT_EQ(bt_node_->status(), BT::NodeStatus::IDLE);

  dummy_node_->changeStatus(BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->status(), BT::NodeStatus::IDLE);



  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);


  dummy_node_->changeStatus(BT::NodeStatus::SUCCESS);


  rclcpp::sleep_for(1s);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);


  odom_msg.header.stamp = time + rclcpp::Duration::from_seconds(0.5);
  odom_msg.twist.twist.linear.x = 0;
  odom_msg.twist.twist.linear.y = 0;
  odom_pub->publish(odom_msg);


  rclcpp::sleep_for(1s);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);

  dummy_node_->changeStatus(BT::NodeStatus::SUCCESS);
  rclcpp::sleep_for(1s);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);



  for (int i = 0; i < 9; ++i) {
    rclcpp::sleep_for(1s);
    EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);
  }


  dummy_node_->changeStatus(BT::NodeStatus::SUCCESS);


  rclcpp::sleep_for(1s);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);


  rclcpp::init(argc, argv);

  bool all_successful = RUN_ALL_TESTS();


  rclcpp::shutdown();

  return all_successful;
}
