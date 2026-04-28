















#include <gtest/gtest.h>
#include <memory>
#include <set>
#include <string>

#include "nav_msgs/msg/path.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"

#include "../../test_action_server.hpp"
#include "nav2_behavior_tree/plugins/action/smooth_path_action.hpp"

class SmoothPathActionServer : public TestActionServer<nav2_msgs::action::SmoothPath>
{
public:
  SmoothPathActionServer()
  : TestActionServer("smooth_path")
  {}

protected:
  void execute(
    const typename std::shared_ptr<
      rclcpp_action::ServerGoalHandle<nav2_msgs::action::SmoothPath>> goal_handle)
  override
  {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<nav2_msgs::action::SmoothPath::Result>();
    goal_handle->succeed(result);
  }
};

class SmoothPathActionTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("smooth_path_action_test_fixture");
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();

    config_ = new BT::NodeConfiguration();


    config_->blackboard = BT::Blackboard::create();

    config_->blackboard->set<rclcpp::Node::SharedPtr>(
      "node",
      node_);
    config_->blackboard->set<std::chrono::milliseconds>(
      "server_timeout",
      std::chrono::milliseconds(20));
    config_->blackboard->set<std::chrono::milliseconds>(
      "bt_loop_duration",
      std::chrono::milliseconds(10));
    config_->blackboard->set<bool>("initial_pose_received", false);

    BT::NodeBuilder builder =
      [](const std::string & name, const BT::NodeConfiguration & config)
      {
        return std::make_unique<nav2_behavior_tree::SmoothPathAction>(
          name, "smooth_path", config);
      };

    factory_->registerBuilder<nav2_behavior_tree::SmoothPathAction>(
      "SmoothPath", builder);
  }

  static void TearDownTestCase()
  {
    delete config_;
    config_ = nullptr;
    node_.reset();
    action_server_.reset();
    factory_.reset();
  }

  void TearDown() override
  {
    tree_.reset();
  }

  static std::shared_ptr<SmoothPathActionServer> action_server_;

protected:
  static rclcpp::Node::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
};

rclcpp::Node::SharedPtr SmoothPathActionTestFixture::node_ = nullptr;
std::shared_ptr<SmoothPathActionServer>
SmoothPathActionTestFixture::action_server_ = nullptr;
BT::NodeConfiguration * SmoothPathActionTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> SmoothPathActionTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> SmoothPathActionTestFixture::tree_ = nullptr;

TEST_F(SmoothPathActionTestFixture, test_tick)
{

  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <SmoothPath unsmoothed_path="{unsmoothed_path}" />
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  nav_msgs::msg::Path path;


  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS) {
    tree_->rootNode()->executeTick();
  }


  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(action_server_->getCurrentGoal()->path, path);


  tree_->rootNode()->halt();
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::IDLE);


  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = -2.5;
  pose.pose.orientation.x = 1.0;
  path.poses.push_back(pose);
  config_->blackboard->set<nav_msgs::msg::Path>("unsmoothed_path", path);

  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS) {
    tree_->rootNode()->executeTick();
  }

  nav_msgs::msg::Path path_empty;
  EXPECT_NE(path_empty, path);
  EXPECT_EQ(action_server_->getCurrentGoal()->path, path);
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);


  rclcpp::init(argc, argv);


  SmoothPathActionTestFixture::action_server_ =
    std::make_shared<SmoothPathActionServer>();

  std::thread server_thread([]() {
      rclcpp::spin(SmoothPathActionTestFixture::action_server_);
    });

  int all_successful = RUN_ALL_TESTS();


  rclcpp::shutdown();
  server_thread.join();

  return all_successful;
}
