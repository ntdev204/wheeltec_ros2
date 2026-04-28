





#include <memory>
#include <string>
#include <algorithm>
#include <utility>

#include "gtest/gtest.h"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/observation_buffer.hpp"
#include "../testing_helper.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

using std::begin;
using std::end;
using std::for_each;
using std::all_of;
using std::none_of;
using std::pair;
using std::string;

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

class TestLifecycleNode : public nav2_util::LifecycleNode
{
public:
  explicit TestLifecycleNode(const string & name)
  : nav2_util::LifecycleNode(name)
  {
  }

  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn onShutdown(const rclcpp_lifecycle::State &)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn onError(const rclcpp_lifecycle::State &)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }
};

class TestNode : public ::testing::Test
{
public:
  TestNode()
  {
    node_ = std::make_shared<TestLifecycleNode>("obstacle_test_node");
    node_->declare_parameter("map_topic", rclcpp::ParameterValue(std::string("map")));
    node_->declare_parameter("track_unknown_space", rclcpp::ParameterValue(false));
    node_->declare_parameter("use_maximum", rclcpp::ParameterValue(false));
    node_->declare_parameter("lethal_cost_threshold", rclcpp::ParameterValue(100));
    node_->declare_parameter(
      "unknown_cost_value",
      rclcpp::ParameterValue(static_cast<unsigned char>(0xff)));
    node_->declare_parameter("trinary_costmap", rclcpp::ParameterValue(true));
    node_->declare_parameter("transform_tolerance", rclcpp::ParameterValue(0.3));
    node_->declare_parameter("observation_sources", rclcpp::ParameterValue(std::string("")));
  }

  ~TestNode() {}

protected:
  std::shared_ptr<TestLifecycleNode> node_;
};




#if (0)


TEST_F(TestNode, testRaytracing) {
  tf2_ros::Buffer tf(node_->get_clock());

  nav2_costmap_2d::LayeredCostmap layers("frame", false, false);
  addStaticLayer(layers, tf, node_);
  auto olayer = addObstacleLayer(layers, tf, node_);


  addObservation(olayer, 0.0, 0.0, MAX_Z / 2, 0, 0, MAX_Z / 2);


  layers.updateMap(0, 0, 0);


  int lethal_count = countValues(*(layers.getCostmap()), nav2_costmap_2d::LETHAL_OBSTACLE);


  ASSERT_EQ(lethal_count, 21);
}



TEST_F(TestNode, testRaytracing2) {
  tf2_ros::Buffer tf(node_->get_clock());
  nav2_costmap_2d::LayeredCostmap layers("frame", false, false);
  addStaticLayer(layers, tf, node_);
  auto olayer = addObstacleLayer(layers, tf, node_);





  layers.updateMap(0, 0, 0);





  int obs_before = countValues(*(layers.getCostmap()), nav2_costmap_2d::LETHAL_OBSTACLE);
  ASSERT_EQ(obs_before, 20);




  addObservation(olayer, 9.5, 9.5, MAX_Z / 2, 0.5, 0.5, MAX_Z / 2);
  layers.updateMap(0, 0, 0);



  int obs_after = countValues(*(layers.getCostmap()), nav2_costmap_2d::LETHAL_OBSTACLE);




  ASSERT_EQ(obs_after, obs_before + 1);


  for (int i = 0; i < olayer->getSizeInCellsY(); ++i) {
    olayer->setCost(i, i, nav2_costmap_2d::LETHAL_OBSTACLE);
  }


  layers.updateMap(0, 0, 0);



  int with_static = countValues(*(layers.getCostmap()), nav2_costmap_2d::LETHAL_OBSTACLE);


  ASSERT_EQ(with_static, obs_after);

  ASSERT_EQ(79, countValues(*(layers.getCostmap()), nav2_costmap_2d::FREE_SPACE));
}



TEST_F(TestNode, testWaveInterference) {
  tf2_ros::Buffer tf(node_->get_clock());
  node_->set_parameter(rclcpp::Parameter("track_unknown_space", true));

  nav2_costmap_2d::LayeredCostmap layers("frame", false, true);
  layers.resizeMap(10, 10, 1, 0, 0);
  auto olayer = addObstacleLayer(layers, tf, node_);





  addObservation(olayer, 3.0, 3.0, MAX_Z);
  addObservation(olayer, 5.0, 5.0, MAX_Z);
  addObservation(olayer, 7.0, 7.0, MAX_Z);
  layers.updateMap(0, 0, 0);

  nav2_costmap_2d::Costmap2D * costmap = layers.getCostmap();



  ASSERT_EQ(3, countValues(*costmap, nav2_costmap_2d::LETHAL_OBSTACLE));
  ASSERT_EQ(92, countValues(*costmap, nav2_costmap_2d::NO_INFORMATION));
  ASSERT_EQ(5, countValues(*costmap, nav2_costmap_2d::FREE_SPACE));
}



TEST_F(TestNode, testZThreshold) {
  tf2_ros::Buffer tf(node_->get_clock());

  nav2_costmap_2d::LayeredCostmap layers("frame", false, true);
  layers.resizeMap(10, 10, 1, 0, 0);

  auto olayer = addObstacleLayer(layers, tf, node_);


  addObservation(olayer, 0.0, 5.0, 0.4);
  addObservation(olayer, 1.0, 5.0, 2.2);

  layers.updateMap(0, 0, 0);

  nav2_costmap_2d::Costmap2D * costmap = layers.getCostmap();
  ASSERT_EQ(countValues(*costmap, nav2_costmap_2d::LETHAL_OBSTACLE), 1);
}



TEST_F(TestNode, testDynamicObstacles) {
  tf2_ros::Buffer tf(node_->get_clock());
  nav2_costmap_2d::LayeredCostmap layers("frame", false, false);
  addStaticLayer(layers, tf, node_);

  auto olayer = addObstacleLayer(layers, tf, node_);


  addObservation(olayer, 0.0, 0.0);
  addObservation(olayer, 0.0, 0.0);
  addObservation(olayer, 0.0, 0.0);

  layers.updateMap(0, 0, 0);

  nav2_costmap_2d::Costmap2D * costmap = layers.getCostmap();

  ASSERT_EQ(countValues(*costmap, nav2_costmap_2d::LETHAL_OBSTACLE), 21);


  ASSERT_EQ(countValues(*costmap, nav2_costmap_2d::LETHAL_OBSTACLE), 21);
}



TEST_F(TestNode, testMultipleAdditions) {
  tf2_ros::Buffer tf(node_->get_clock());
  nav2_costmap_2d::LayeredCostmap layers("frame", false, false);
  addStaticLayer(layers, tf, node_);

  auto olayer = addObstacleLayer(layers, tf, node_);


  addObservation(olayer, 9.5, 0.0);
  layers.updateMap(0, 0, 0);
  nav2_costmap_2d::Costmap2D * costmap = layers.getCostmap();


  ASSERT_EQ(countValues(*costmap, nav2_costmap_2d::LETHAL_OBSTACLE), 20);
}
#endif


TEST_F(TestNode, testRepeatedResets) {
  tf2_ros::Buffer tf(node_->get_clock());
  nav2_costmap_2d::LayeredCostmap layers("frame", false, false);

  std::shared_ptr<nav2_costmap_2d::StaticLayer> slayer = nullptr;
  addStaticLayer(layers, tf, node_, slayer);




  pair<string, string> node_dummy = {"node_dummy_param", "node_dummy_val"};
  node_->declare_parameter(node_dummy.first, rclcpp::ParameterValue(node_dummy.second));


  pair<string, string> layer_dummy = {"dummy_param", "dummy_val"};


  auto plugins = layers.getPlugins();
  for_each(
    begin(*plugins), end(*plugins), [&layer_dummy](const auto & plugin) {
      string layer_param = layer_dummy.first + "_" + plugin->getName();


      plugin->declareParameter(layer_param, rclcpp::ParameterValue(layer_dummy.second));
    });



  ASSERT_TRUE(node_->has_parameter(node_dummy.first));


  ASSERT_TRUE(
    all_of(
      begin(*plugins), end(*plugins), [&layer_dummy](const auto & plugin) {
        string layer_param = layer_dummy.first + "_" + plugin->getName();
        return plugin->hasParameter(layer_param);
      }));


  ASSERT_NO_THROW(
    for_each(
      begin(*plugins), end(*plugins), [](const auto & plugin) {
        plugin->reset();
      }));
}




TEST_F(TestNode, testRaytracing) {
  tf2_ros::Buffer tf(node_->get_clock());

  nav2_costmap_2d::LayeredCostmap layers("frame", false, false);
  layers.resizeMap(10, 10, 1, 0, 0);

  std::shared_ptr<nav2_costmap_2d::StaticLayer> slayer = nullptr;
  addStaticLayer(layers, tf, node_, slayer);
  std::shared_ptr<nav2_costmap_2d::ObstacleLayer> olayer = nullptr;
  addObstacleLayer(layers, tf, node_, olayer);

  addObservation(olayer, 0.0, 0.0, MAX_Z / 2, 0, 0, MAX_Z / 2);


  layers.updateMap(0, 0, 0);


  int lethal_count = countValues(*(layers.getCostmap()), nav2_costmap_2d::LETHAL_OBSTACLE);

  ASSERT_EQ(lethal_count, 1);

  addObservation(olayer, 1.0, 1.0, MAX_Z / 2, 0, 0, MAX_Z / 2, true, true, 100.0, 5.0, 100.0, 5.0);


  layers.updateMap(0, 0, 0);



  lethal_count = countValues(*(layers.getCostmap()), nav2_costmap_2d::LETHAL_OBSTACLE);

  ASSERT_EQ(lethal_count, 1);
}



TEST_F(TestNode, testDynParamsSetObstacle)
{
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");


  std::vector<std::string> plugins_str;
  plugins_str.push_back("obstacle_layer");
  costmap->set_parameter(rclcpp::Parameter("plugins", plugins_str));
  costmap->declare_parameter(
    "obstacle_layer.plugin",
    rclcpp::ParameterValue(std::string("nav2_costmap_2d::ObstacleLayer")));

  costmap->set_parameter(rclcpp::Parameter("global_frame", std::string("base_link")));
  costmap->on_configure(rclcpp_lifecycle::State());

  costmap->on_activate(rclcpp_lifecycle::State());

  auto parameter_client = std::make_shared<rclcpp::AsyncParametersClient>(
    costmap->get_node_base_interface(), costmap->get_node_topics_interface(),
    costmap->get_node_graph_interface(),
    costmap->get_node_services_interface());

  auto results = parameter_client->set_parameters_atomically(
  {
    rclcpp::Parameter("obstacle_layer.combination_method", 5),
    rclcpp::Parameter("obstacle_layer.max_obstacle_height", 4.0),
    rclcpp::Parameter("obstacle_layer.enabled", false),
    rclcpp::Parameter("obstacle_layer.footprint_clearing_enabled", false)
  });

  rclcpp::spin_until_future_complete(
    costmap->get_node_base_interface(),
    results);

  EXPECT_EQ(costmap->get_parameter("obstacle_layer.combination_method").as_int(), 5);
  EXPECT_EQ(costmap->get_parameter("obstacle_layer.max_obstacle_height").as_double(), 4.0);
  EXPECT_EQ(costmap->get_parameter("obstacle_layer.enabled").as_bool(), false);
  EXPECT_EQ(costmap->get_parameter("obstacle_layer.footprint_clearing_enabled").as_bool(), false);

  costmap->on_deactivate(rclcpp_lifecycle::State());
  costmap->on_cleanup(rclcpp_lifecycle::State());
  costmap->on_shutdown(rclcpp_lifecycle::State());
}



TEST_F(TestNode, testDynParamsSetVoxel)
{
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");


  std::vector<std::string> plugins_str;
  plugins_str.push_back("voxel_layer");
  costmap->set_parameter(rclcpp::Parameter("plugins", plugins_str));
  costmap->declare_parameter(
    "voxel_layer.plugin",
    rclcpp::ParameterValue(std::string("nav2_costmap_2d::VoxelLayer")));

  costmap->set_parameter(rclcpp::Parameter("global_frame", std::string("base_link")));
  costmap->on_configure(rclcpp_lifecycle::State());

  costmap->on_activate(rclcpp_lifecycle::State());

  auto parameter_client = std::make_shared<rclcpp::AsyncParametersClient>(
    costmap->get_node_base_interface(), costmap->get_node_topics_interface(),
    costmap->get_node_graph_interface(),
    costmap->get_node_services_interface());

  auto results = parameter_client->set_parameters_atomically(
  {
    rclcpp::Parameter("voxel_layer.combination_method", 0),
    rclcpp::Parameter("voxel_layer.mark_threshold", 1),
    rclcpp::Parameter("voxel_layer.unknown_threshold", 10),
    rclcpp::Parameter("voxel_layer.z_resolution", 0.4),
    rclcpp::Parameter("voxel_layer.origin_z", 1.0),
    rclcpp::Parameter("voxel_layer.z_voxels", 14),
    rclcpp::Parameter("voxel_layer.max_obstacle_height", 4.0),
    rclcpp::Parameter("voxel_layer.footprint_clearing_enabled", false),
    rclcpp::Parameter("voxel_layer.enabled", false),
    rclcpp::Parameter("voxel_layer.publish_voxel_map", true)
  });

  rclcpp::spin_until_future_complete(
    costmap->get_node_base_interface(),
    results);

  EXPECT_EQ(costmap->get_parameter("voxel_layer.combination_method").as_int(), 0);
  EXPECT_EQ(costmap->get_parameter("voxel_layer.mark_threshold").as_int(), 1);
  EXPECT_EQ(costmap->get_parameter("voxel_layer.unknown_threshold").as_int(), 10);
  EXPECT_EQ(costmap->get_parameter("voxel_layer.z_resolution").as_double(), 0.4);
  EXPECT_EQ(costmap->get_parameter("voxel_layer.origin_z").as_double(), 1.0);
  EXPECT_EQ(costmap->get_parameter("voxel_layer.z_voxels").as_int(), 14);
  EXPECT_EQ(costmap->get_parameter("voxel_layer.max_obstacle_height").as_double(), 4.0);
  EXPECT_EQ(costmap->get_parameter("voxel_layer.footprint_clearing_enabled").as_bool(), false);
  EXPECT_EQ(costmap->get_parameter("voxel_layer.enabled").as_bool(), false);
  EXPECT_EQ(costmap->get_parameter("voxel_layer.publish_voxel_map").as_bool(), true);

  costmap->on_deactivate(rclcpp_lifecycle::State());
  costmap->on_cleanup(rclcpp_lifecycle::State());
  costmap->on_shutdown(rclcpp_lifecycle::State());
}



TEST_F(TestNode, testDynParamsSetStatic)
{
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");

  costmap->set_parameter(rclcpp::Parameter("global_frame", std::string("base_link")));
  costmap->on_configure(rclcpp_lifecycle::State());

  costmap->on_activate(rclcpp_lifecycle::State());

  auto parameter_client = std::make_shared<rclcpp::AsyncParametersClient>(
    costmap->get_node_base_interface(), costmap->get_node_topics_interface(),
    costmap->get_node_graph_interface(),
    costmap->get_node_services_interface());

  auto results = parameter_client->set_parameters_atomically(
  {
    rclcpp::Parameter("static_layer.transform_tolerance", 1.0),
    rclcpp::Parameter("static_layer.enabled", false),
    rclcpp::Parameter("static_layer.map_subscribe_transient_local", false),
    rclcpp::Parameter("static_layer.map_topic", "dynamic_topic"),
    rclcpp::Parameter("static_layer.subscribe_to_updates", true)
  });

  rclcpp::spin_until_future_complete(
    costmap->get_node_base_interface(),
    results);

  EXPECT_EQ(costmap->get_parameter("static_layer.transform_tolerance").as_double(), 1.0);
  EXPECT_EQ(costmap->get_parameter("static_layer.enabled").as_bool(), false);
  EXPECT_EQ(costmap->get_parameter("static_layer.map_subscribe_transient_local").as_bool(), false);
  EXPECT_EQ(costmap->get_parameter("static_layer.map_topic").as_string(), "dynamic_topic");
  EXPECT_EQ(costmap->get_parameter("static_layer.subscribe_to_updates").as_bool(), true);

  costmap->on_deactivate(rclcpp_lifecycle::State());
  costmap->on_cleanup(rclcpp_lifecycle::State());
  costmap->on_shutdown(rclcpp_lifecycle::State());
}
