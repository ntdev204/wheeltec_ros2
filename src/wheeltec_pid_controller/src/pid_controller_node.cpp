#include <chrono>
#include <functional>
#include <memory>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "wheeltec_pid_controller/pid_controller.hpp"

using namespace std::chrono_literals;

class PidControllerNode : public rclcpp::Node
{
public:
  PidControllerNode()
  : Node("pid_controller")
  {
    declare_parameters();
    load_parameters();

    // Publishers
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    debug_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("pid_debug", 10);

    // Subscribers
    cmd_vel_raw_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_raw", 10,
      std::bind(&PidControllerNode::cmd_vel_raw_callback, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom_combined", 10,
      std::bind(&PidControllerNode::odom_callback, this, std::placeholders::_1));

    // Control loop timer
    double freq = get_parameter("control_frequency").as_double();
    auto period = std::chrono::duration<double>(1.0 / freq);
    control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&PidControllerNode::control_loop, this));

    // Parameter change callback for live tuning
    param_cb_handle_ = add_on_set_parameters_callback(
      std::bind(&PidControllerNode::on_parameter_change, this, std::placeholders::_1));

    // Timeout: if no cmd_vel_raw for this duration, stop
    cmd_timeout_ = std::chrono::duration<double>(
      get_parameter("cmd_timeout").as_double());

    last_cmd_time_ = now();
    last_control_time_ = now();

    RCLCPP_INFO(get_logger(),
      "PID Controller started | freq=%.0fHz | vx_kp=%.2f vy_kp=%.2f wz_kp=%.2f",
      freq,
      get_parameter("vx_kp").as_double(),
      get_parameter("vy_kp").as_double(),
      get_parameter("wz_kp").as_double());
  }

private:
  // ─── Parameter Declaration ───
  void declare_parameters()
  {
    declare_parameter("control_frequency", 50.0);
    declare_parameter("cmd_timeout", 0.5);

    // PID gains - Linear X
    declare_parameter("vx_kp", 1.0);
    declare_parameter("vx_ki", 0.1);
    declare_parameter("vx_kd", 0.05);
    declare_parameter("vx_max_integral", 0.3);
    declare_parameter("vx_deadband", 0.01);

    // PID gains - Linear Y (Mecanum strafe)
    declare_parameter("vy_kp", 1.0);
    declare_parameter("vy_ki", 0.1);
    declare_parameter("vy_kd", 0.05);
    declare_parameter("vy_max_integral", 0.2);
    declare_parameter("vy_deadband", 0.01);

    // PID gains - Angular Z
    declare_parameter("wz_kp", 1.2);
    declare_parameter("wz_ki", 0.15);
    declare_parameter("wz_kd", 0.08);
    declare_parameter("wz_max_integral", 0.5);
    declare_parameter("wz_deadband", 0.02);

    // Velocity limits
    declare_parameter("max_vx", 0.5);
    declare_parameter("max_vy", 0.35);
    declare_parameter("max_wz", 1.8);

    // Acceleration limits (m/s^2 or rad/s^2)
    declare_parameter("max_ax", 1.5);
    declare_parameter("max_ay", 0.5);
    declare_parameter("max_awz", 1.5);

    // EMA smoothing (0 = raw PID, 1 = full smooth)
    declare_parameter("alpha", 0.3);

    // Enable/disable PID (passthrough mode for comparison)
    declare_parameter("enabled", true);
  }

  void load_parameters()
  {
    wheeltec_pid::PidGains vx_gains;
    vx_gains.kp = get_parameter("vx_kp").as_double();
    vx_gains.ki = get_parameter("vx_ki").as_double();
    vx_gains.kd = get_parameter("vx_kd").as_double();
    vx_gains.max_integral = get_parameter("vx_max_integral").as_double();
    vx_gains.deadband = get_parameter("vx_deadband").as_double();
    vx_pid_.set_gains(vx_gains);

    wheeltec_pid::PidGains vy_gains;
    vy_gains.kp = get_parameter("vy_kp").as_double();
    vy_gains.ki = get_parameter("vy_ki").as_double();
    vy_gains.kd = get_parameter("vy_kd").as_double();
    vy_gains.max_integral = get_parameter("vy_max_integral").as_double();
    vy_gains.deadband = get_parameter("vy_deadband").as_double();
    vy_pid_.set_gains(vy_gains);

    wheeltec_pid::PidGains wz_gains;
    wz_gains.kp = get_parameter("wz_kp").as_double();
    wz_gains.ki = get_parameter("wz_ki").as_double();
    wz_gains.kd = get_parameter("wz_kd").as_double();
    wz_gains.max_integral = get_parameter("wz_max_integral").as_double();
    wz_gains.deadband = get_parameter("wz_deadband").as_double();
    wz_pid_.set_gains(wz_gains);

    max_vx_ = get_parameter("max_vx").as_double();
    max_vy_ = get_parameter("max_vy").as_double();
    max_wz_ = get_parameter("max_wz").as_double();
    max_ax_ = get_parameter("max_ax").as_double();
    max_ay_ = get_parameter("max_ay").as_double();
    max_awz_ = get_parameter("max_awz").as_double();
    alpha_ = get_parameter("alpha").as_double();
    enabled_ = get_parameter("enabled").as_bool();
  }

  // ─── Callbacks ───
  void cmd_vel_raw_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    cmd_setpoint_ = *msg;
    last_cmd_time_ = now();
    cmd_received_ = true;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    odom_vx_ = msg->twist.twist.linear.x;
    odom_vy_ = msg->twist.twist.linear.y;
    odom_wz_ = msg->twist.twist.angular.z;
    odom_received_ = true;
  }

  // ─── Main Control Loop (runs at control_frequency Hz) ───
  void control_loop()
  {
    auto current_time = now();
    double dt = (current_time - last_control_time_).seconds();
    last_control_time_ = current_time;

    if (dt <= 0.0 || dt > 1.0) {
      return;  // Skip invalid dt
    }

    // Command timeout: stop if no cmd received recently
    auto time_since_cmd = (current_time - last_cmd_time_).seconds();
    if (time_since_cmd > cmd_timeout_.count()) {
      publish_stop();
      vx_pid_.reset();
      vy_pid_.reset();
      wz_pid_.reset();
      prev_out_vx_ = 0.0;
      prev_out_vy_ = 0.0;
      prev_out_wz_ = 0.0;
      return;
    }

    geometry_msgs::msg::Twist output;

    if (!enabled_) {
      // Passthrough mode: direct forward without PID
      output = cmd_setpoint_;
    } else if (!odom_received_) {
      // No odom yet: passthrough with ramping only
      output = cmd_setpoint_;
    } else {
      // ─── PID Compute ───
      double raw_vx = vx_pid_.compute(cmd_setpoint_.linear.x, odom_vx_, dt);
      double raw_vy = vy_pid_.compute(cmd_setpoint_.linear.y, odom_vy_, dt);
      double raw_wz = wz_pid_.compute(cmd_setpoint_.angular.z, odom_wz_, dt);

      // Velocity clamping
      raw_vx = std::clamp(raw_vx, -max_vx_, max_vx_);
      raw_vy = std::clamp(raw_vy, -max_vy_, max_vy_);
      raw_wz = std::clamp(raw_wz, -max_wz_, max_wz_);

      // Acceleration limiting (rate limiter)
      raw_vx = rate_limit(raw_vx, prev_out_vx_, max_ax_, dt);
      raw_vy = rate_limit(raw_vy, prev_out_vy_, max_ay_, dt);
      raw_wz = rate_limit(raw_wz, prev_out_wz_, max_awz_, dt);

      // EMA smoothing
      raw_vx = alpha_ * raw_vx + (1.0 - alpha_) * prev_out_vx_;
      raw_vy = alpha_ * raw_vy + (1.0 - alpha_) * prev_out_vy_;
      raw_wz = alpha_ * raw_wz + (1.0 - alpha_) * prev_out_wz_;

      output.linear.x = raw_vx;
      output.linear.y = raw_vy;
      output.angular.z = raw_wz;
    }

    prev_out_vx_ = output.linear.x;
    prev_out_vy_ = output.linear.y;
    prev_out_wz_ = output.angular.z;

    cmd_vel_pub_->publish(output);
    publish_debug(output);
  }

  // ─── Helpers ───
  double rate_limit(double target, double current, double max_accel, double dt)
  {
    double max_change = max_accel * dt;
    double diff = target - current;
    if (std::fabs(diff) > max_change) {
      return current + std::copysign(max_change, diff);
    }
    return target;
  }

  void publish_stop()
  {
    geometry_msgs::msg::Twist stop;
    cmd_vel_pub_->publish(stop);
  }

  void publish_debug(const geometry_msgs::msg::Twist & output)
  {
    // Debug: [setpoint_vx, setpoint_vy, setpoint_wz, odom_vx, odom_vy, odom_wz, out_vx, out_vy, out_wz]
    std_msgs::msg::Float64MultiArray debug_msg;
    debug_msg.data = {
      cmd_setpoint_.linear.x, cmd_setpoint_.linear.y, cmd_setpoint_.angular.z,
      odom_vx_, odom_vy_, odom_wz_,
      output.linear.x, output.linear.y, output.angular.z
    };
    debug_pub_->publish(debug_msg);
  }

  // ─── Dynamic Parameter Callback ───
  rcl_interfaces::msg::SetParametersResult on_parameter_change(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & param : params) {
      RCLCPP_INFO(get_logger(), "Parameter changed: %s = %s",
        param.get_name().c_str(), param.value_to_string().c_str());
    }

    // Reload all parameters
    load_parameters();

    return result;
  }

  // ─── Members ───
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_raw_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // PID controllers per axis
  wheeltec_pid::PidAxis vx_pid_;
  wheeltec_pid::PidAxis vy_pid_;
  wheeltec_pid::PidAxis wz_pid_;

  // Setpoint from Nav2
  geometry_msgs::msg::Twist cmd_setpoint_;

  // Feedback from odometry
  double odom_vx_ = 0.0;
  double odom_vy_ = 0.0;
  double odom_wz_ = 0.0;

  // Previous output for smoothing and rate limiting
  double prev_out_vx_ = 0.0;
  double prev_out_vy_ = 0.0;
  double prev_out_wz_ = 0.0;

  // Limits
  double max_vx_, max_vy_, max_wz_;
  double max_ax_, max_ay_, max_awz_;
  double alpha_;
  bool enabled_ = true;

  // Timing
  rclcpp::Time last_cmd_time_;
  rclcpp::Time last_control_time_;
  std::chrono::duration<double> cmd_timeout_;

  bool cmd_received_ = false;
  bool odom_received_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PidControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
