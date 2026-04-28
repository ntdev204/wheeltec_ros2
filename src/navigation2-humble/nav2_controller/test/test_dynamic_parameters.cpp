













#include <math.h>

#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_controller/controller_server.hpp"
#include "rclcpp/rclcpp.hpp"

class ControllerShim : public nav2_controller::ControllerServer
{
public:
  ControllerShim()
  : nav2_controller::ControllerServer(rclcpp::NodeOptions())
  {
  }



  void setDynamicCallback()
  {
    auto node = shared_from_this();

    dyn_params_handler_ = node->add_on_set_parameters_callback(
      std::bind(&ControllerShim::dynamicParamsShim, this, std::placeholders::_1));
  }

  rcl_interfaces::msg::SetParametersResult
  dynamicParamsShim(std::vector<rclcpp::Parameter> parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    dynamicParametersCallback(parameters);
    return result;
  }
};

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

TEST(WPTest, test_dynamic_parameters)
{
  auto controller = std::make_shared<ControllerShim>();
  controller->setDynamicCallback();

  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
    controller->get_node_base_interface(), controller->get_node_topics_interface(),
    controller->get_node_graph_interface(),
    controller->get_node_services_interface());

  auto results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("controller_frequency", 100.0),
      rclcpp::Parameter("min_x_velocity_threshold", 100.0),
      rclcpp::Parameter("min_y_velocity_threshold", 100.0),
      rclcpp::Parameter("min_theta_velocity_threshold", 100.0),
      rclcpp::Parameter("failure_tolerance", 5.0)});

  rclcpp::spin_until_future_complete(
    controller->get_node_base_interface(),
    results);

  EXPECT_EQ(controller->get_parameter("controller_frequency").as_double(), 100.0);
  EXPECT_EQ(controller->get_parameter("min_x_velocity_threshold").as_double(), 100.0);
  EXPECT_EQ(controller->get_parameter("min_y_velocity_threshold").as_double(), 100.0);
  EXPECT_EQ(controller->get_parameter("min_theta_velocity_threshold").as_double(), 100.0);
  EXPECT_EQ(controller->get_parameter("failure_tolerance").as_double(), 5.0);
}
