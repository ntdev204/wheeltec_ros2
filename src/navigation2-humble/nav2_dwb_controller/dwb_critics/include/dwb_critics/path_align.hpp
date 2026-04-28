

#ifndef DWB_CRITICS__PATH_ALIGN_HPP_
#define DWB_CRITICS__PATH_ALIGN_HPP_

#include <vector>
#include <string>
#include "dwb_critics/path_dist.hpp"

namespace dwb_critics
{


class PathAlignCritic : public PathDistCritic
{
public:
  PathAlignCritic()
  : zero_scale_(false), forward_point_distance_(0.0) {}
  void onInit() override;
  bool prepare(
    const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D & vel,
    const geometry_msgs::msg::Pose2D & goal, const nav_2d_msgs::msg::Path2D & global_plan) override;
  double getScale() const override;
  double scorePose(const geometry_msgs::msg::Pose2D & pose) override;

protected:
  bool zero_scale_;
  double forward_point_distance_;
};

}
#endif
