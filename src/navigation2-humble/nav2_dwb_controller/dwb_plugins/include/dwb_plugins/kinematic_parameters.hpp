


#ifndef DWB_PLUGINS__KINEMATIC_PARAMETERS_HPP_
#define DWB_PLUGINS__KINEMATIC_PARAMETERS_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace dwb_plugins
{



struct KinematicParameters
{
  friend class KinematicsHandler;

  inline double getMinX() {return min_vel_x_;}
  inline double getMaxX() {return max_vel_x_;}
  inline double getAccX() {return acc_lim_x_;}
  inline double getDecelX() {return decel_lim_x_;}

  inline double getMinY() {return min_vel_y_;}
  inline double getMaxY() {return max_vel_y_;}
  inline double getAccY() {return acc_lim_y_;}
  inline double getDecelY() {return decel_lim_y_;}

  inline double getMinSpeedXY() {return min_speed_xy_;}
  inline double getMaxSpeedXY() {return max_speed_xy_;}

  inline double getMinTheta() {return -max_vel_theta_;}
  inline double getMaxTheta() {return max_vel_theta_;}
  inline double getAccTheta() {return acc_lim_theta_;}
  inline double getDecelTheta() {return decel_lim_theta_;}
  inline double getMinSpeedTheta() {return min_speed_theta_;}

  inline double getMinSpeedXY_SQ() {return min_speed_xy_sq_;}
  inline double getMaxSpeedXY_SQ() {return max_speed_xy_sq_;}

protected:

  double min_vel_x_{0};
  double min_vel_y_{0};
  double max_vel_x_{0};
  double max_vel_y_{0};
  double base_max_vel_x_{0};
  double base_max_vel_y_{0};
  double max_vel_theta_{0};
  double base_max_vel_theta_{0};
  double min_speed_xy_{0};
  double max_speed_xy_{0};
  double base_max_speed_xy_{0};
  double min_speed_theta_{0};
  double acc_lim_x_{0};
  double acc_lim_y_{0};
  double acc_lim_theta_{0};
  double decel_lim_x_{0};
  double decel_lim_y_{0};
  double decel_lim_theta_{0};


  double min_speed_xy_sq_{0};
  double max_speed_xy_sq_{0};
};



class KinematicsHandler
{
public:
  KinematicsHandler();
  ~KinematicsHandler();
  void initialize(const nav2_util::LifecycleNode::SharedPtr & nh, const std::string & plugin_name);

  inline KinematicParameters getKinematics() {return *kinematics_.load();}

  void setSpeedLimit(const double & speed_limit, const bool & percentage);

  using Ptr = std::shared_ptr<KinematicsHandler>;

protected:
  std::atomic<KinematicParameters *> kinematics_;


  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  

  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
  void update_kinematics(KinematicParameters kinematics);
  std::string plugin_name_;
};

}

#endif
