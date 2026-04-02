/**
 * Velocity Smoother Node for Wheeltec Mecanum Robot
 *
 * Replaces PID controller. Key difference:
 * - NO odometry feedback → NO fighting with MPPI
 * - Pure signal processing: low-pass filter + rate limiter + lateral suppression
 * - Very lightweight for Raspberry Pi 4
 *
 * Pipeline:
 *   cmd_vel_raw (MPPI) → [low-pass] → [lateral suppress] → [rate limit] → cmd_vel (STM32)
 */

#include <chrono>
#include <functional>
#include <memory>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "wheeltec_pid_controller/pid_controller.hpp"

using namespace std::chrono_literals;

class VelocitySmootherNode : public rclcpp::Node
{
public:
  VelocitySmootherNode()
  : Node("pid_controller")  // Keep node name for launch compatibility
  {
    declare_parameters();
    load_parameters();

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    debug_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("pid_debug", 10);

    cmd_vel_raw_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_raw", 10,
      std::bind(&VelocitySmootherNode::cmd_vel_raw_callback, this, std::placeholders::_1));

    // Control loop timer
    double freq = get_parameter("control_frequency").as_double();
    auto period = std::chrono::duration<double>(1.0 / freq);
    control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&VelocitySmootherNode::control_loop, this));

    param_cb_handle_ = add_on_set_parameters_callback(
      std::bind(&VelocitySmootherNode::on_parameter_change, this, std::placeholders::_1));

    cmd_timeout_ = get_parameter("cmd_timeout").as_double();
    last_cmd_time_ = now();
    last_control_time_ = now();

    RCLCPP_INFO(get_logger(),
      "Velocity Smoother started (NO PID, NO odom feedback) | freq=%.0fHz | lp=%.2f",
      freq, get_parameter("lowpass_alpha").as_double());
  }

private:
  void declare_parameters()
  {
    declare_parameter("control_frequency", 50.0);
    declare_parameter("cmd_timeout", 0.5);

    // Low-pass filter coefficient for incoming cmd_vel_raw
    // Lower = more filtering = smoother but slower response
    declare_parameter("lowpass_alpha", 0.3);

    // Lateral velocity suppression threshold
    // vy below this is zeroed — kills MPPI sampling noise
    declare_parameter("vy_suppress_threshold", 0.03);

    // Velocity limits
    declare_parameter("max_vx", 0.5);
    declare_parameter("max_vy", 0.15);
    declare_parameter("max_wz", 1.5);

    // Acceleration limits (rate limiter) — the primary smoothing mechanism
    declare_parameter("max_ax", 0.8);
    declare_parameter("max_ay", 0.3);
    declare_parameter("max_awz", 0.8);

    // Deadband per axis — zero commands below this
    declare_parameter("vx_deadband", 0.005);
    declare_parameter("vy_deadband", 0.02);
    declare_parameter("wz_deadband", 0.02);

    // Enable/disable (passthrough when false)
    declare_parameter("enabled", true);
  }

  void load_parameters()
  {
    lowpass_alpha_ = get_parameter("lowpass_alpha").as_double();
    vy_suppress_ = get_parameter("vy_suppress_threshold").as_double();
    enabled_ = get_parameter("enabled").as_bool();

    vx_smoother_.configure(
      get_parameter("max_vx").as_double(),
      get_parameter("max_ax").as_double(),
      get_parameter("vx_deadband").as_double());
    vy_smoother_.configure(
      get_parameter("max_vy").as_double(),
      get_parameter("max_ay").as_double(),
      get_parameter("vy_deadband").as_double());
    wz_smoother_.configure(
      get_parameter("max_wz").as_double(),
      get_parameter("max_awz").as_double(),
      get_parameter("wz_deadband").as_double());
  }

  void cmd_vel_raw_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    cmd_raw_ = *msg;
    last_cmd_time_ = now();
    cmd_received_ = true;
  }

  void control_loop()
  {
    auto current_time = now();
    double dt = (current_time - last_control_time_).seconds();
    last_control_time_ = current_time;

    if (dt <= 0.0 || dt > 1.0) return;

    // Timeout: stop if no command received
    double time_since_cmd = (current_time - last_cmd_time_).seconds();
    if (time_since_cmd > cmd_timeout_) {
      geometry_msgs::msg::Twist stop;
      cmd_vel_pub_->publish(stop);
      vx_smoother_.reset();
      vy_smoother_.reset();
      wz_smoother_.reset();
      filtered_vx_ = 0.0;
      filtered_vy_ = 0.0;
      filtered_wz_ = 0.0;
      return;
    }

    geometry_msgs::msg::Twist output;

    if (!enabled_ || !cmd_received_) {
      output = cmd_raw_;
    } else {
      // ═══ STAGE 1: Low-pass filter on raw input ═══
      // Removes high-frequency jitter from MPPI trajectory sampling
      filtered_vx_ = lowpass_alpha_ * cmd_raw_.linear.x +
                      (1.0 - lowpass_alpha_) * filtered_vx_;
      filtered_vy_ = lowpass_alpha_ * cmd_raw_.linear.y +
                      (1.0 - lowpass_alpha_) * filtered_vy_;
      filtered_wz_ = lowpass_alpha_ * cmd_raw_.angular.z +
                      (1.0 - lowpass_alpha_) * filtered_wz_;

      // ═══ STAGE 2: Lateral velocity suppression ═══
      // Small vy from MPPI Omni model is noise, not intentional strafe
      double vy_input = filtered_vy_;
      if (std::fabs(vy_input) < vy_suppress_) {
        vy_input = 0.0;
      }

      // ═══ STAGE 3: Rate limiting (acceleration clamp) ═══
      // This is the main smoother — limits how fast velocity can change
      output.linear.x = vx_smoother_.smooth(filtered_vx_, dt);
      output.linear.y = vy_smoother_.smooth(vy_input, dt);
      output.angular.z = wz_smoother_.smooth(filtered_wz_, dt);
    }

    cmd_vel_pub_->publish(output);

    // Debug output: [raw_vx, raw_vy, raw_wz, filtered_vx, filtered_vy, filtered_wz, out_vx, out_vy, out_wz]
    std_msgs::msg::Float64MultiArray debug_msg;
    debug_msg.data = {
      cmd_raw_.linear.x, cmd_raw_.linear.y, cmd_raw_.angular.z,
      filtered_vx_, filtered_vy_, filtered_wz_,
      output.linear.x, output.linear.y, output.angular.z
    };
    debug_pub_->publish(debug_msg);
  }

  rcl_interfaces::msg::SetParametersResult on_parameter_change(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & p : params) {
      RCLCPP_INFO(get_logger(), "Param changed: %s = %s",
        p.get_name().c_str(), p.value_to_string().c_str());
    }
    load_parameters();
    return result;
  }

  // ─── Members ───
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_raw_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // Per-axis smoothers (rate limiter + deadband)
  wheeltec_smoother::AxisSmoother vx_smoother_;
  wheeltec_smoother::AxisSmoother vy_smoother_;
  wheeltec_smoother::AxisSmoother wz_smoother_;

  // Raw command from MPPI
  geometry_msgs::msg::Twist cmd_raw_;

  // Low-pass filtered values
  double filtered_vx_ = 0.0;
  double filtered_vy_ = 0.0;
  double filtered_wz_ = 0.0;

  // Parameters
  double lowpass_alpha_ = 0.3;
  double vy_suppress_ = 0.03;
  double cmd_timeout_ = 0.5;
  bool enabled_ = true;
  bool cmd_received_ = false;

  rclcpp::Time last_cmd_time_;
  rclcpp::Time last_control_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VelocitySmootherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
