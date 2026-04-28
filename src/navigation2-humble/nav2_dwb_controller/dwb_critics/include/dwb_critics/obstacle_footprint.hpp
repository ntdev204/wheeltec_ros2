


#ifndef DWB_CRITICS__OBSTACLE_FOOTPRINT_HPP_
#define DWB_CRITICS__OBSTACLE_FOOTPRINT_HPP_

#include <vector>
#include "dwb_critics/base_obstacle.hpp"

namespace dwb_critics
{
typedef std::vector<geometry_msgs::msg::Point> Footprint;



Footprint getOrientedFootprint(
  const geometry_msgs::msg::Pose2D & pose,
  const Footprint & footprint_spec);



class ObstacleFootprintCritic : public BaseObstacleCritic
{
public:
  bool prepare(
    const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D & vel,
    const geometry_msgs::msg::Pose2D & goal, const nav_2d_msgs::msg::Path2D & global_plan) override;
  double scorePose(const geometry_msgs::msg::Pose2D & pose) override;
  virtual double scorePose(
    const geometry_msgs::msg::Pose2D & pose,
    const Footprint & oriented_footprint);
  double getScale() const override {return costmap_->getResolution() * scale_;}

protected:
  

  double lineCost(int x0, int x1, int y0, int y1);

  

  double pointCost(int x, int y);

  Footprint footprint_spec_;
};
}

#endif
