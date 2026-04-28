













#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__REMOVE_PASSED_GOALS_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__REMOVE_PASSED_GOALS_ACTION_HPP_

#include <vector>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace nav2_behavior_tree
{

class RemovePassedGoals : public BT::ActionNodeBase
{
public:
  typedef std::vector<geometry_msgs::msg::PoseStamped> Goals;

  RemovePassedGoals(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);


  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<Goals>("input_goals", "Original goals to remove viapoints from"),
      BT::OutputPort<Goals>("output_goals", "Goals with passed viapoints removed"),
      BT::InputPort<double>("radius", 0.5, "radius to goal for it to be considered for removal"),
      BT::InputPort<std::string>("global_frame", std::string("map"), "Global frame"),
      BT::InputPort<std::string>("robot_base_frame", std::string("base_link"), "Robot base frame"),
    };
  }

private:
  void halt() override {}
  BT::NodeStatus tick() override;

  double viapoint_achieved_radius_;
  std::string robot_base_frame_, global_frame_;
  double transform_tolerance_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
};

}

#endif
