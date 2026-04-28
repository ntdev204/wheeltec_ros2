




#include <gtest/gtest.h>

#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/obstacle_layer.hpp"
#include "nav2_costmap_2d/inflation_layer.hpp"
#include "nav2_costmap_2d/observation_buffer.hpp"
#include "../testing_helper.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

using geometry_msgs::msg::Point;
using nav2_costmap_2d::CellData;

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

class TestNode : public ::testing::Test
{
public:
  TestNode() {}

  ~TestNode() {}

  std::vector<Point> setRadii(
    nav2_costmap_2d::LayeredCostmap & layers,
    double length, double width);

  void validatePointInflation(
    unsigned int mx, unsigned int my,
    nav2_costmap_2d::Costmap2D * costmap,
    std::shared_ptr<nav2_costmap_2d::InflationLayer> & ilayer,
    double inflation_radius);

  void initNode(std::vector<rclcpp::Parameter> parameters);
  void initNode(double inflation_radius);

  void waitForMap(std::shared_ptr<nav2_costmap_2d::StaticLayer> & slayer);

protected:
  nav2_util::LifecycleNode::SharedPtr node_;
};

std::vector<Point> TestNode::setRadii(
  nav2_costmap_2d::LayeredCostmap & layers,
  double length, double width)
{
  std::vector<Point> polygon;
  Point p;
  p.x = width;
  p.y = length;
  polygon.push_back(p);
  p.x = width;
  p.y = -length;
  polygon.push_back(p);
  p.x = -width;
  p.y = -length;
  polygon.push_back(p);
  p.x = -width;
  p.y = length;
  polygon.push_back(p);
  layers.setFootprint(polygon);

  return polygon;
}

void TestNode::waitForMap(std::shared_ptr<nav2_costmap_2d::StaticLayer> & slayer)
{
  while (!slayer->isCurrent()) {
    rclcpp::spin_some(node_->get_node_base_interface());
  }
}


void TestNode::validatePointInflation(
  unsigned int mx, unsigned int my,
  nav2_costmap_2d::Costmap2D * costmap,
  std::shared_ptr<nav2_costmap_2d::InflationLayer> & ilayer,
  double inflation_radius)
{
  bool * seen = new bool[costmap->getSizeInCellsX() * costmap->getSizeInCellsY()];
  memset(seen, false, costmap->getSizeInCellsX() * costmap->getSizeInCellsY() * sizeof(bool));
  std::map<double, std::vector<CellData>> m;
  CellData initial(costmap->getIndex(mx, my), mx, my, mx, my);
  m[0].push_back(initial);
  for (std::map<double, std::vector<CellData>>::iterator bin = m.begin();
    bin != m.end(); ++bin)
  {
    for (unsigned int i = 0; i < bin->second.size(); ++i) {
      const CellData cell = bin->second[i];
      if (!seen[cell.index_]) {
        seen[cell.index_] = true;
        unsigned int dx = (cell.x_ > cell.src_x_) ? cell.x_ - cell.src_x_ : cell.src_x_ - cell.x_;
        unsigned int dy = (cell.y_ > cell.src_y_) ? cell.y_ - cell.src_y_ : cell.src_y_ - cell.y_;
        double dist = std::hypot(dx, dy);

        unsigned char expected_cost = ilayer->computeCost(dist);
        ASSERT_TRUE(costmap->getCost(cell.x_, cell.y_) >= expected_cost);

        if (dist > inflation_radius) {
          continue;
        }

        if (dist == bin->first) {


          dist += 0.001;
        }

        if (cell.x_ > 0) {
          CellData data(costmap->getIndex(cell.x_ - 1, cell.y_),
            cell.x_ - 1, cell.y_, cell.src_x_, cell.src_y_);
          m[dist].push_back(data);
        }
        if (cell.y_ > 0) {
          CellData data(costmap->getIndex(cell.x_, cell.y_ - 1),
            cell.x_, cell.y_ - 1, cell.src_x_, cell.src_y_);
          m[dist].push_back(data);
        }
        if (cell.x_ < costmap->getSizeInCellsX() - 1) {
          CellData data(costmap->getIndex(cell.x_ + 1, cell.y_),
            cell.x_ + 1, cell.y_, cell.src_x_, cell.src_y_);
          m[dist].push_back(data);
        }
        if (cell.y_ < costmap->getSizeInCellsY() - 1) {
          CellData data(costmap->getIndex(cell.x_, cell.y_ + 1),
            cell.x_, cell.y_ + 1, cell.src_x_, cell.src_y_);
          m[dist].push_back(data);
        }
      }
    }
  }
  delete[] seen;
}

void TestNode::initNode(std::vector<rclcpp::Parameter> parameters)
{
  auto options = rclcpp::NodeOptions();
  options.parameter_overrides(parameters);

  node_ = std::make_shared<nav2_util::LifecycleNode>(
    "inflation_test_node", "", options);


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

void TestNode::initNode(double inflation_radius)
{
  std::vector<rclcpp::Parameter> parameters;

  parameters.push_back(rclcpp::Parameter("inflation.cost_scaling_factor", 1.0));
  parameters.push_back(rclcpp::Parameter("inflation.inflation_radius", inflation_radius));

  initNode(parameters);
}

TEST_F(TestNode, testAdjacentToObstacleCanStillMove)
{
  initNode(4.1);
  tf2_ros::Buffer tf(node_->get_clock());
  nav2_costmap_2d::LayeredCostmap layers("frame", false, false);
  layers.resizeMap(10, 10, 1, 0, 0);



  std::vector<Point> polygon = setRadii(layers, 2.1, 2.3);

  std::shared_ptr<nav2_costmap_2d::ObstacleLayer> olayer = nullptr;
  addObstacleLayer(layers, tf, node_, olayer);

  std::shared_ptr<nav2_costmap_2d::InflationLayer> ilayer = nullptr;
  addInflationLayer(layers, tf, node_, ilayer);

  layers.setFootprint(polygon);

  addObservation(olayer, 0, 0, MAX_Z);

  layers.updateMap(0, 0, 0);
  nav2_costmap_2d::Costmap2D * costmap = layers.getCostmap();

  EXPECT_EQ(nav2_costmap_2d::LETHAL_OBSTACLE, costmap->getCost(0, 0));
  EXPECT_EQ(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE, costmap->getCost(1, 0));
  EXPECT_EQ(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE, costmap->getCost(2, 0));
  EXPECT_TRUE(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE > costmap->getCost(3, 0));
  EXPECT_TRUE(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE > costmap->getCost(2, 1));
  EXPECT_EQ(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE, costmap->getCost(1, 1));
}

TEST_F(TestNode, testInflationShouldNotCreateUnknowns)
{
  initNode(4.1);
  tf2_ros::Buffer tf(node_->get_clock());
  nav2_costmap_2d::LayeredCostmap layers("frame", false, false);
  layers.resizeMap(10, 10, 1, 0, 0);



  std::vector<Point> polygon = setRadii(layers, 2.1, 2.3);

  std::shared_ptr<nav2_costmap_2d::ObstacleLayer> olayer = nullptr;
  addObstacleLayer(layers, tf, node_, olayer);

  std::shared_ptr<nav2_costmap_2d::InflationLayer> ilayer = nullptr;
  addInflationLayer(layers, tf, node_, ilayer);

  layers.setFootprint(polygon);

  addObservation(olayer, 0, 0, MAX_Z);

  layers.updateMap(0, 0, 0);
  nav2_costmap_2d::Costmap2D * costmap = layers.getCostmap();

  EXPECT_EQ(countValues(*costmap, nav2_costmap_2d::NO_INFORMATION), 0u);
}

TEST_F(TestNode, testInflationInUnkown)
{
  std::vector<rclcpp::Parameter> parameters;

  parameters.push_back(rclcpp::Parameter("inflation.cost_scaling_factor", 1.0));
  parameters.push_back(rclcpp::Parameter("inflation.inflation_radius", 4.1));
  parameters.push_back(rclcpp::Parameter("inflation.inflate_unknown", true));

  initNode(parameters);

  node_->set_parameter(rclcpp::Parameter("track_unknown_space", true));

  tf2_ros::Buffer tf(node_->get_clock());
  nav2_costmap_2d::LayeredCostmap layers("frame", false, true);
  layers.resizeMap(9, 9, 1, 0, 0);



  std::vector<Point> polygon = setRadii(layers, 2.1, 2.3);

  std::shared_ptr<nav2_costmap_2d::ObstacleLayer> olayer = nullptr;
  addObstacleLayer(layers, tf, node_, olayer);
  std::shared_ptr<nav2_costmap_2d::InflationLayer> ilayer = nullptr;
  addInflationLayer(layers, tf, node_, ilayer);
  layers.setFootprint(polygon);

  addObservation(olayer, 4, 4, MAX_Z, 0.0, 0.0, MAX_Z, true, false);

  layers.updateMap(0, 0, 0);
  nav2_costmap_2d::Costmap2D * costmap = layers.getCostmap();


  EXPECT_EQ(countValues(*costmap, nav2_costmap_2d::NO_INFORMATION), 4u);
}

TEST_F(TestNode, testInflationAroundUnkown)
{
  auto inflation_radius = 4.1;
  std::vector<rclcpp::Parameter> parameters;

  parameters.push_back(rclcpp::Parameter("inflation.cost_scaling_factor", 1.0));
  parameters.push_back(rclcpp::Parameter("inflation.inflation_radius", inflation_radius));
  parameters.push_back(rclcpp::Parameter("inflation.inflate_around_unknown", true));

  initNode(parameters);

  node_->set_parameter(rclcpp::Parameter("track_unknown_space", true));

  tf2_ros::Buffer tf(node_->get_clock());
  nav2_costmap_2d::LayeredCostmap layers("frame", false, false);
  layers.resizeMap(10, 10, 1, 0, 0);



  std::vector<Point> polygon = setRadii(layers, 2.1, 2.3);

  std::shared_ptr<nav2_costmap_2d::InflationLayer> ilayer = nullptr;
  addInflationLayer(layers, tf, node_, ilayer);
  layers.setFootprint(polygon);
  layers.updateMap(0, 0, 0);

  layers.getCostmap()->setCost(4, 4, nav2_costmap_2d::NO_INFORMATION);
  ilayer->updateCosts(*layers.getCostmap(), 0, 0, 10, 10);

  validatePointInflation(4, 4, layers.getCostmap(), ilayer, inflation_radius);
}



TEST_F(TestNode, testCostFunctionCorrectness)
{
  initNode(10.5);
  tf2_ros::Buffer tf(node_->get_clock());
  nav2_costmap_2d::LayeredCostmap layers("frame", false, false);

  layers.resizeMap(100, 100, 1, 0, 0);


  std::vector<Point> polygon = setRadii(layers, 5.0, 6.25);

  std::shared_ptr<nav2_costmap_2d::ObstacleLayer> olayer = nullptr;
  addObstacleLayer(layers, tf, node_, olayer);

  std::shared_ptr<nav2_costmap_2d::InflationLayer> ilayer = nullptr;
  addInflationLayer(layers, tf, node_, ilayer);

  layers.setFootprint(polygon);

  addObservation(olayer, 50, 50, MAX_Z);

  layers.updateMap(0, 0, 0);
  nav2_costmap_2d::Costmap2D * map = layers.getCostmap();





  for (unsigned int i = 0; i <= (unsigned int)ceil(5.0); i++) {

    ASSERT_EQ(map->getCost(50 + i, 50) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);
    ASSERT_EQ(map->getCost(50 + i, 50) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);

    ASSERT_EQ(map->getCost(50 - i, 50) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);
    ASSERT_EQ(map->getCost(50 - i, 50) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);

    ASSERT_EQ(map->getCost(50, 50 + i) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);
    ASSERT_EQ(map->getCost(50, 50 + i) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);

    ASSERT_EQ(map->getCost(50, 50 - i) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);
    ASSERT_EQ(map->getCost(50, 50 - i) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);
  }


  for (unsigned int i = (unsigned int)(ceil(5.0) + 1); i <= (unsigned int)ceil(10.5); i++) {
    unsigned char expectedValue = ilayer->computeCost(i / 1.0);
    ASSERT_EQ(map->getCost(50 + i, 50), expectedValue);
  }


  

}



TEST_F(TestNode, testInflationOrderCorrectness)
{
  const double inflation_radius = 4.1;
  initNode(inflation_radius);
  tf2_ros::Buffer tf(node_->get_clock());
  nav2_costmap_2d::LayeredCostmap layers("frame", false, false);
  layers.resizeMap(10, 10, 1, 0, 0);



  std::vector<Point> polygon = setRadii(layers, 2.1, 2.3);

  std::shared_ptr<nav2_costmap_2d::ObstacleLayer> olayer = nullptr;
  addObstacleLayer(layers, tf, node_, olayer);

  std::shared_ptr<nav2_costmap_2d::InflationLayer> ilayer = nullptr;
  addInflationLayer(layers, tf, node_, ilayer);

  layers.setFootprint(polygon);



  addObservation(olayer, 4, 4, MAX_Z);
  addObservation(olayer, 5, 5, MAX_Z);

  layers.updateMap(0, 0, 0);

  validatePointInflation(4, 4, layers.getCostmap(), ilayer, inflation_radius);
  validatePointInflation(5, 5, layers.getCostmap(), ilayer, inflation_radius);
}



TEST_F(TestNode, testInflation)
{
  initNode(1);
  tf2_ros::Buffer tf(node_->get_clock());
  nav2_costmap_2d::LayeredCostmap layers("frame", false, false);



  std::vector<Point> polygon = setRadii(layers, 1, 1);

  std::shared_ptr<nav2_costmap_2d::StaticLayer> slayer = nullptr;
  addStaticLayer(layers, tf, node_, slayer);

  std::shared_ptr<nav2_costmap_2d::ObstacleLayer> olayer = nullptr;
  addObstacleLayer(layers, tf, node_, olayer);

  std::shared_ptr<nav2_costmap_2d::InflationLayer> ilayer = nullptr;
  addInflationLayer(layers, tf, node_, ilayer);
  layers.setFootprint(polygon);

  nav2_costmap_2d::Costmap2D * costmap = layers.getCostmap();
  waitForMap(slayer);

  layers.updateMap(0, 0, 0);

  ASSERT_EQ(countValues(*costmap, nav2_costmap_2d::LETHAL_OBSTACLE), 20u);
  ASSERT_EQ(countValues(*costmap, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE), 28u);

   Iterate over all id's and verify they are obstacles
  for(std::vector<unsigned int>::const_iterator it = occupiedCells.begin(); it != occupiedCells.end(); ++it){
    unsigned int ind = *it;
    unsigned int x, y;
    map.indexToCells(ind, x, y);
    ASSERT_EQ(find(occupiedCells, map.getIndex(x, y)), true);
    ASSERT_EQ(map.getCost(x, y) == nav2_costmap_2d::LETHAL_OBSTACLE ||
      map.getCost(x, y) == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);
  }*/

  addObservation(olayer, 0, 0, 0.4);
  layers.updateMap(0, 0, 0);


  ASSERT_EQ(
    countValues(*costmap, nav2_costmap_2d::LETHAL_OBSTACLE) +
    countValues(*costmap, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE), 51u);



  addObservation(olayer, 2, 0);
  layers.updateMap(0, 0, 0);






  ASSERT_EQ(
    countValues(*costmap, nav2_costmap_2d::LETHAL_OBSTACLE) +
    countValues(*costmap, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE), 54u);


  addObservation(olayer, 1, 9);
  layers.updateMap(0, 0, 0);

  ASSERT_EQ(costmap->getCost(1, 9), nav2_costmap_2d::LETHAL_OBSTACLE);
  ASSERT_EQ(costmap->getCost(0, 9), nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  ASSERT_EQ(costmap->getCost(2, 9), nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);


  addObservation(olayer, 0, 9);
  layers.updateMap(0, 0, 0);

  ASSERT_EQ(costmap->getCost(0, 9), nav2_costmap_2d::LETHAL_OBSTACLE);
}



TEST_F(TestNode, testInflation2)
{
  initNode(1);
  tf2_ros::Buffer tf(node_->get_clock());
  nav2_costmap_2d::LayeredCostmap layers("frame", false, false);



  std::vector<Point> polygon = setRadii(layers, 1, 1);

  std::shared_ptr<nav2_costmap_2d::StaticLayer> slayer = nullptr;
  addStaticLayer(layers, tf, node_, slayer);

  std::shared_ptr<nav2_costmap_2d::ObstacleLayer> olayer = nullptr;
  addObstacleLayer(layers, tf, node_, olayer);

  std::shared_ptr<nav2_costmap_2d::InflationLayer> ilayer = nullptr;
  addInflationLayer(layers, tf, node_, ilayer);

  layers.setFootprint(polygon);

  waitForMap(slayer);


  addObservation(olayer, 1, 1, MAX_Z);
  addObservation(olayer, 2, 1, MAX_Z);
  addObservation(olayer, 2, 2, MAX_Z);
  layers.updateMap(0, 0, 0);

  nav2_costmap_2d::Costmap2D * costmap = layers.getCostmap();

  ASSERT_EQ(costmap->getCost(2, 3), nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  ASSERT_EQ(costmap->getCost(3, 3), nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
}



TEST_F(TestNode, testInflation3)
{
  initNode(3);
  tf2_ros::Buffer tf(node_->get_clock());
  nav2_costmap_2d::LayeredCostmap layers("frame", false, false);
  layers.resizeMap(10, 10, 1, 0, 0);


  std::vector<Point> polygon = setRadii(layers, 1, 1.75);

  std::shared_ptr<nav2_costmap_2d::ObstacleLayer> olayer = nullptr;
  addObstacleLayer(layers, tf, node_, olayer);

  std::shared_ptr<nav2_costmap_2d::InflationLayer> ilayer = nullptr;
  addInflationLayer(layers, tf, node_, ilayer);

  layers.setFootprint(polygon);


  nav2_costmap_2d::Costmap2D * costmap = layers.getCostmap();
  ASSERT_EQ(countValues(*costmap, nav2_costmap_2d::LETHAL_OBSTACLE), 0u);
  ASSERT_EQ(countValues(*costmap, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE), 0u);
  printMap(*costmap);

  addObservation(olayer, 5, 5, MAX_Z);
  layers.updateMap(0, 0, 0);
  printMap(*costmap);


  ASSERT_EQ(countValues(*costmap, nav2_costmap_2d::FREE_SPACE, false), 29u);
  ASSERT_EQ(countValues(*costmap, nav2_costmap_2d::LETHAL_OBSTACLE), 1u);
  ASSERT_EQ(countValues(*costmap, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE), 4u);


  layers.updateMap(0, 0, 0);

  ASSERT_EQ(countValues(*costmap, nav2_costmap_2d::FREE_SPACE, false), 29u);
  ASSERT_EQ(countValues(*costmap, nav2_costmap_2d::LETHAL_OBSTACLE), 1u);
  ASSERT_EQ(countValues(*costmap, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE), 4u);
}



TEST_F(TestNode, testDynParamsSet)
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
    rclcpp::Parameter("inflation_layer.inflation_radius", 0.0),
    rclcpp::Parameter("inflation_layer.cost_scaling_factor", 0.0),
    rclcpp::Parameter("inflation_layer.inflate_unknown", true),
    rclcpp::Parameter("inflation_layer.inflate_around_unknown", true),
    rclcpp::Parameter("inflation_layer.enabled", false)
  });

  rclcpp::spin_until_future_complete(
    costmap->get_node_base_interface(),
    results);

  EXPECT_EQ(costmap->get_parameter("inflation_layer.inflation_radius").as_double(), 0.0);
  EXPECT_EQ(costmap->get_parameter("inflation_layer.cost_scaling_factor").as_double(), 0.0);
  EXPECT_EQ(costmap->get_parameter("inflation_layer.inflate_unknown").as_bool(), true);
  EXPECT_EQ(costmap->get_parameter("inflation_layer.inflate_around_unknown").as_bool(), true);
  EXPECT_EQ(costmap->get_parameter("inflation_layer.enabled").as_bool(), false);

  costmap->on_deactivate(rclcpp_lifecycle::State());
  costmap->on_cleanup(rclcpp_lifecycle::State());
  costmap->on_shutdown(rclcpp_lifecycle::State());
}
