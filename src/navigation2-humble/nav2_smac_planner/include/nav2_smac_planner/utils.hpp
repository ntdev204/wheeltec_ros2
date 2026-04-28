













#ifndef NAV2_SMAC_PLANNER__UTILS_HPP_
#define NAV2_SMAC_PLANNER__UTILS_HPP_

#include <vector>
#include <memory>

#include "Eigen/Core"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2/utils.h"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/inflation_layer.hpp"

namespace nav2_smac_planner
{



inline geometry_msgs::msg::Pose getWorldCoords(
  const float & mx, const float & my, const nav2_costmap_2d::Costmap2D * costmap)
{
  geometry_msgs::msg::Pose msg;
  msg.position.x =
    static_cast<float>(costmap->getOriginX()) + (mx + 0.5) * costmap->getResolution();
  msg.position.y =
    static_cast<float>(costmap->getOriginY()) + (my + 0.5) * costmap->getResolution();
  return msg;
}



inline geometry_msgs::msg::Quaternion getWorldOrientation(
  const float & theta)
{

  tf2::Quaternion q;
  q.setEuler(0.0, 0.0, theta);
  return tf2::toMsg(q);
}



inline double findCircumscribedCost(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap)
{
  double result = -1.0;
  bool inflation_layer_found = false;
  std::vector<std::shared_ptr<nav2_costmap_2d::Layer>>::iterator layer;


  for (layer = costmap->getLayeredCostmap()->getPlugins()->begin();
    layer != costmap->getLayeredCostmap()->getPlugins()->end();
    ++layer)
  {
    std::shared_ptr<nav2_costmap_2d::InflationLayer> inflation_layer =
      std::dynamic_pointer_cast<nav2_costmap_2d::InflationLayer>(*layer);
    if (!inflation_layer) {
      continue;
    }

    inflation_layer_found = true;
    double circum_radius = costmap->getLayeredCostmap()->getCircumscribedRadius();
    double resolution = costmap->getCostmap()->getResolution();
    result = static_cast<double>(inflation_layer->computeCost(circum_radius / resolution));
  }

  if (!inflation_layer_found) {
    RCLCPP_WARN(
      rclcpp::get_logger("computeCircumscribedCost"),
      "No inflation layer found in costmap configuration. "
      "If this is an SE2-collision checking plugin, it cannot use costmap potential "
      "field to speed up collision checking by only checking the full footprint "
      "when robot is within possibly-inscribed radius of an obstacle. This may "
      "significantly slow down planning times!");
  }

  return result;
}



inline void fromJsonToMetaData(const nlohmann::json & json, LatticeMetadata & lattice_metadata)
{
  json.at("turning_radius").get_to(lattice_metadata.min_turning_radius);
  json.at("grid_resolution").get_to(lattice_metadata.grid_resolution);
  json.at("num_of_headings").get_to(lattice_metadata.number_of_headings);
  json.at("heading_angles").get_to(lattice_metadata.heading_angles);
  json.at("number_of_trajectories").get_to(lattice_metadata.number_of_trajectories);
  json.at("motion_model").get_to(lattice_metadata.motion_model);
}



inline void fromJsonToPose(const nlohmann::json & json, MotionPose & pose)
{
  pose._x = json[0];
  pose._y = json[1];
  pose._theta = json[2];
}



inline void fromJsonToMotionPrimitive(
  const nlohmann::json & json, MotionPrimitive & motion_primitive)
{
  json.at("trajectory_id").get_to(motion_primitive.trajectory_id);
  json.at("start_angle_index").get_to(motion_primitive.start_angle);
  json.at("end_angle_index").get_to(motion_primitive.end_angle);
  json.at("trajectory_radius").get_to(motion_primitive.turning_radius);
  json.at("trajectory_length").get_to(motion_primitive.trajectory_length);
  json.at("arc_length").get_to(motion_primitive.arc_length);
  json.at("straight_length").get_to(motion_primitive.straight_length);
  json.at("left_turn").get_to(motion_primitive.left_turn);

  for (unsigned int i = 0; i < json["poses"].size(); i++) {
    MotionPose pose;
    fromJsonToPose(json["poses"][i], pose);
    motion_primitive.poses.push_back(pose);
  }
}

}

#endif
