














#include <gtest/gtest.h>

#include <memory>
#include <set>
#include <string>

#include "../../test_action_server.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav2_behavior_tree/plugins/action/smoother_selector_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/string.hpp"

class SmootherSelectorTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("smoother_selector_test_fixture");
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();

    config_ = new BT::NodeConfiguration();


    config_->blackboard = BT::Blackboard::create();

    config_->blackboard->set<rclcpp::Node::SharedPtr>("node", node_);

    BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
        return std::make_unique<nav2_behavior_tree::SmootherSelector>(name, config);
      };

    factory_->registerBuilder<nav2_behavior_tree::SmootherSelector>(
      "SmootherSelector",
      builder);
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

rclcpp::Node::SharedPtr SmootherSelectorTestFixture::node_ = nullptr;

BT::NodeConfiguration * SmootherSelectorTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> SmootherSelectorTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> SmootherSelectorTestFixture::tree_ = nullptr;

TEST_F(SmootherSelectorTestFixture, test_custom_topic)
{

  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
          <SmootherSelector selected_smoother="{selected_smoother}" default_smoother="DWB" topic_name="smoother_selector_custom_topic_name"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));


  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS) {
    tree_->rootNode()->executeTick();
  }


  std::string selected_smoother_result;
  config_->blackboard->get("selected_smoother", selected_smoother_result);

  EXPECT_EQ(selected_smoother_result, "DWB");

  std_msgs::msg::String selected_smoother_cmd;

  selected_smoother_cmd.data = "DWC";

  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.transient_local().reliable();

  auto smoother_selector_pub =
    node_->create_publisher<std_msgs::msg::String>("smoother_selector_custom_topic_name", qos);


  auto start = node_->now();
  while ((node_->now() - start).seconds() < 0.5) {
    tree_->rootNode()->executeTick();
    smoother_selector_pub->publish(selected_smoother_cmd);

    rclcpp::spin_some(node_);
  }


  config_->blackboard->get("selected_smoother", selected_smoother_result);
  EXPECT_EQ("DWC", selected_smoother_result);
}

TEST_F(SmootherSelectorTestFixture, test_default_topic)
{

  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
          <SmootherSelector selected_smoother="{selected_smoother}" default_smoother="GridBased"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));


  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS) {
    tree_->rootNode()->executeTick();
  }


  std::string selected_smoother_result;
  config_->blackboard->get("selected_smoother", selected_smoother_result);

  EXPECT_EQ(selected_smoother_result, "GridBased");

  std_msgs::msg::String selected_smoother_cmd;

  selected_smoother_cmd.data = "RRT";

  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.transient_local().reliable();

  auto smoother_selector_pub =
    node_->create_publisher<std_msgs::msg::String>("smoother_selector", qos);


  auto start = node_->now();
  while ((node_->now() - start).seconds() < 0.5) {
    tree_->rootNode()->executeTick();
    smoother_selector_pub->publish(selected_smoother_cmd);

    rclcpp::spin_some(node_);
  }


  config_->blackboard->get("selected_smoother", selected_smoother_result);
  EXPECT_EQ("RRT", selected_smoother_result);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);


  rclcpp::init(argc, argv);

  int all_successful = RUN_ALL_TESTS();


  rclcpp::shutdown();

  return all_successful;
}
