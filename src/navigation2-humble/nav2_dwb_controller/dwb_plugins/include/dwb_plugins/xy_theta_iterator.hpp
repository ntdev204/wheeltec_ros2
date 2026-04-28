


#ifndef DWB_PLUGINS__XY_THETA_ITERATOR_HPP_
#define DWB_PLUGINS__XY_THETA_ITERATOR_HPP_

#include <memory>
#include <string>

#include "dwb_plugins/velocity_iterator.hpp"
#include "dwb_plugins/one_d_velocity_iterator.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace dwb_plugins
{
class XYThetaIterator : public VelocityIterator
{
public:
  XYThetaIterator()
  : kinematics_handler_(nullptr), x_it_(nullptr), y_it_(nullptr), th_it_(nullptr) {}
  void initialize(
    const nav2_util::LifecycleNode::SharedPtr & nh,
    KinematicsHandler::Ptr kinematics,
    const std::string & plugin_name) override;
  void startNewIteration(const nav_2d_msgs::msg::Twist2D & current_velocity, double dt) override;
  bool hasMoreTwists() override;
  nav_2d_msgs::msg::Twist2D nextTwist() override;

protected:
  

  bool isValidSpeed(double x, double y, double theta);
  virtual bool isValidVelocity();
  void iterateToValidVelocity();
  int vx_samples_, vy_samples_, vtheta_samples_;
  KinematicsHandler::Ptr kinematics_handler_;

  std::shared_ptr<OneDVelocityIterator> x_it_, y_it_, th_it_;
};
}

#endif
