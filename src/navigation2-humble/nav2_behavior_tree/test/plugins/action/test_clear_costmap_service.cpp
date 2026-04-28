














#include <gtest/gtest.h>
#include <memory>
#include <set>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"

#include "../../test_service.hpp"
#include "nav2_behavior_tree/plugins/action/clear_costmap_service.hpp"

class ClearEntireCostmapService : public TestService<nav2_msgs::srv::ClearEntireCostmap>
{
public:
  ClearEntireCostmapService()
  : TestService("clear_entire_costmap")
  {}
};

class ClearEntireCostmapServiceTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("clear_entire_costmap_test_fixture");
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
    config_->blackboard->set<int>("number_recoveries", 0);

    factory_->registerNodeType<nav2_behavior_tree::ClearEntireCostmapService>("ClearEntireCostmap");
  }

  static void TearDownTestCase()
  {
    delete config_;
    config_ = nullptr;
    node_.reset();
    server_.reset();
    factory_.reset();
  }

  void SetUp() override
  {
    config_->blackboard->set("number_recoveries", 0);
  }

  void TearDown() override
  {
    tree_.reset();
  }

  static std::shared_ptr<ClearEntireCostmapService> server_;

protected:
  static rclcpp::Node::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
};

rclcpp::Node::SharedPtr ClearEntireCostmapServiceTestFixture::node_ = nullptr;
std::shared_ptr<ClearEntireCostmapService> ClearEntireCostmapServiceTestFixture::server_ = nullptr;
BT::NodeConfiguration * ClearEntireCostmapServiceTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> ClearEntireCostmapServiceTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> ClearEntireCostmapServiceTestFixture::tree_ = nullptr;

TEST_F(ClearEntireCostmapServiceTestFixture, test_tick)
{
  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <ClearEntireCostmap service_name="clear_entire_costmap"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  EXPECT_EQ(config_->blackboard->get<int>("number_recoveries"), 0);
  EXPECT_EQ(tree_->rootNode()->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(config_->blackboard->get<int>("number_recoveries"), 1);
}

class ClearCostmapExceptRegionService : public TestService<nav2_msgs::srv::ClearCostmapExceptRegion>
{
public:
  ClearCostmapExceptRegionService()
  : TestService("clear_costmap_except_region")
  {}
};

class ClearCostmapExceptRegionServiceTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("clear_costmap_except_region_test_fixture");
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();

    config_ = new BT::NodeConfiguration();


    config_->blackboard = BT::Blackboard::create();

    config_->blackboard->set<rclcpp::Node::SharedPtr>(
      "node",
      node_);
    config_->blackboard->set<std::chrono::milliseconds>(
      "server_timeout",
      std::chrono::milliseconds(10));
    config_->blackboard->set<std::chrono::milliseconds>(
      "bt_loop_duration",
      std::chrono::milliseconds(10));
    config_->blackboard->set<bool>("initial_pose_received", false);
    config_->blackboard->set<int>("number_recoveries", 0);

    factory_->registerNodeType<nav2_behavior_tree::ClearCostmapExceptRegionService>(
      "ClearCostmapExceptRegion");
  }

  static void TearDownTestCase()
  {
    delete config_;
    config_ = nullptr;
    node_.reset();
    server_.reset();
    factory_.reset();
  }

  void SetUp() override
  {
    config_->blackboard->set("number_recoveries", 0);
  }

  void TearDown() override
  {
    tree_.reset();
  }

  static std::shared_ptr<ClearCostmapExceptRegionService> server_;

protected:
  static rclcpp::Node::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
};

rclcpp::Node::SharedPtr
ClearCostmapExceptRegionServiceTestFixture::node_ = nullptr;
std::shared_ptr<ClearCostmapExceptRegionService>
ClearCostmapExceptRegionServiceTestFixture::server_ = nullptr;
BT::NodeConfiguration
* ClearCostmapExceptRegionServiceTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory>
ClearCostmapExceptRegionServiceTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree>
ClearCostmapExceptRegionServiceTestFixture::tree_ = nullptr;

TEST_F(ClearCostmapExceptRegionServiceTestFixture, test_tick)
{
  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <ClearCostmapExceptRegion service_name="clear_costmap_except_region"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  EXPECT_EQ(config_->blackboard->get<int>("number_recoveries"), 0);
  EXPECT_EQ(tree_->rootNode()->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(config_->blackboard->get<int>("number_recoveries"), 1);
}
/