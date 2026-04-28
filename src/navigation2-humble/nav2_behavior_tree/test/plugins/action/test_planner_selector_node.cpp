














#include <gtest/gtest.h>

#include <memory>
#include <set>
#include <string>

#include "../../test_action_server.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav2_behavior_tree/plugins/action/planner_selector_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/string.hpp"

class PlannerSelectorTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("planner_selector_test_fixture");
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();

    config_ = new BT::NodeConfiguration();


    config_->blackboard = BT::Blackboard::create();

    config_->blackboard->set<rclcpp::Node::SharedPtr>("node", node_);

    BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
        return std::make_unique<nav2_behavior_tree::PlannerSelector>(name, config);
      };

    factory_->registerBuilder<nav2_behavior_tree::PlannerSelector>("PlannerSelector", builder);
  }

  static void TearDownTestCase()
  {
    delete config_;
    config_ = nullptr;
    node_.reset();
    factory_.reset();
  }

  void TearDown() override
  {
    tree_.reset();
  }

protected:
  static rclcpp::Node::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
};

rclcpp::Node::SharedPtr PlannerSelectorTestFixture::node_ = nullptr;

BT::NodeConfiguration * PlannerSelectorTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> PlannerSelectorTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> PlannerSelectorTestFixture::tree_ = nullptr;

TEST_F(PlannerSelectorTestFixture, test_custom_topic)
{

  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
          <PlannerSelector selected_planner="{selected_planner}" default_planner="GridBased" topic_name="planner_selector_custom_topic_name"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));


  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS) {
    tree_->rootNode()->executeTick();
  }


  std::string selected_planner_result;
  config_->blackboard->get("selected_planner", selected_planner_result);

  EXPECT_EQ(selected_planner_result, "GridBased");

  std_msgs::msg::String selected_planner_cmd;

  selected_planner_cmd.data = "RRT";

  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.transient_local().reliable();

  auto planner_selector_pub =
    node_->create_publisher<std_msgs::msg::String>("planner_selector_custom_topic_name", qos);


  auto start = node_->now();
  while ((node_->now() - start).seconds() < 0.5) {
    tree_->rootNode()->executeTick();
    planner_selector_pub->publish(selected_planner_cmd);

    rclcpp::spin_some(node_);
  }


  config_->blackboard->get("selected_planner", selected_planner_result);
  EXPECT_EQ("RRT", selected_planner_result);
}

TEST_F(PlannerSelectorTestFixture, test_default_topic)
{

  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
          <PlannerSelector selected_planner="{selected_planner}" default_planner="GridBased"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));


  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS) {
    tree_->rootNode()->executeTick();
  }


  std::string selected_planner_result;
  config_->blackboard->get("selected_planner", selected_planner_result);

  EXPECT_EQ(selected_planner_result, "GridBased");

  std_msgs::msg::String selected_planner_cmd;

  selected_planner_cmd.data = "RRT";

  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.transient_local().reliable();

  auto planner_selector_pub =
    node_->create_publisher<std_msgs::msg::String>("planner_selector", qos);


  auto start = node_->now();
  while ((node_->now() - start).seconds() < 0.5) {
    tree_->rootNode()->executeTick();
    planner_selector_pub->publish(selected_planner_cmd);

    rclcpp::spin_some(node_);
  }


  config_->blackboard->get("selected_planner", selected_planner_result);
  EXPECT_EQ("RRT", selected_planner_result);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);


  rclcpp::init(argc, argv);

  int all_successful = RUN_ALL_TESTS();


  rclcpp::shutdown();

  return all_successful;
}
