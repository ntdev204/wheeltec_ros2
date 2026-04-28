













#include <gtest/gtest.h>
#include <chrono>
#include <memory>
#include <set>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/robot_utils.hpp"

#include "../../test_behavior_tree_fixture.hpp"
#include "nav2_behavior_tree/plugins/condition/path_expiring_timer_condition.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;

class PathExpiringTimerConditionTestFixture : public nav2_behavior_tree::BehaviorTreeTestFixture
{
public:
  void SetUp()
  {
    node_ = std::make_shared<rclcpp::Node>("test_path_expiring_condition");
    config_ = new BT::NodeConfiguration();
    config_->blackboard = BT::Blackboard::create();
    config_->blackboard->set<rclcpp::Node::SharedPtr>("node", node_);
    bt_node_ = std::make_shared<nav2_behavior_tree::PathExpiringTimerCondition>(
      "time_expired", *config_);
  }

  void TearDown()
  {
    delete config_;
    config_ = nullptr;
    node_.reset();
    bt_node_.reset();
  }

protected:
  static rclcpp::Node::SharedPtr node_;
  static std::shared_ptr<nav2_behavior_tree::PathExpiringTimerCondition> bt_node_;
  static BT::NodeConfiguration * config_;
};

rclcpp::Node::SharedPtr PathExpiringTimerConditionTestFixture::node_ = nullptr;
std::shared_ptr<nav2_behavior_tree::PathExpiringTimerCondition>
PathExpiringTimerConditionTestFixture::bt_node_ = nullptr;
BT::NodeConfiguration * PathExpiringTimerConditionTestFixture::config_ = nullptr;

TEST_F(PathExpiringTimerConditionTestFixture, test_behavior)
{
  EXPECT_EQ(bt_node_->status(), BT::NodeStatus::IDLE);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);

  for (int i = 0; i < 20; ++i) {
    rclcpp::sleep_for(500ms);
    if (i % 2) {
      EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
    } else {
      EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);
    }
  }


  nav_msgs::msg::Path path;
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = 1.0;
  path.poses.push_back(pose);

  config_->blackboard->set<nav_msgs::msg::Path>("path", path);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);
  rclcpp::sleep_for(1500ms);
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
