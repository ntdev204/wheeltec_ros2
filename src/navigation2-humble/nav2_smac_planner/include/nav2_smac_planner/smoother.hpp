













#ifndef NAV2_SMAC_PLANNER__SMOOTHER_HPP_
#define NAV2_SMAC_PLANNER__SMOOTHER_HPP_

#include <cmath>
#include <vector>
#include <iostream>
#include <memory>
#include <queue>
#include <utility>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_smac_planner/types.hpp"
#include "nav2_smac_planner/constants.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "angles/angles.h"
#include "tf2/utils.h"
#include "ompl/base/StateSpace.h"
#include "ompl/base/spaces/DubinsStateSpace.h"

namespace nav2_smac_planner
{



struct PathSegment
{
  unsigned int start;
  unsigned int end;
};



struct BoundaryPoints
{
  

  BoundaryPoints(double & x_in, double & y_in, double & theta_in)
  : x(x_in), y(y_in), theta(theta_in)
  {}

  double x;
  double y;
  double theta;
};



struct BoundaryExpansion
{
  double path_end_idx{0.0};
  double expansion_path_length{0.0};
  double original_path_length{0.0};
  std::vector<BoundaryPoints> pts;
  bool in_collision{false};
};

typedef std::vector<BoundaryExpansion> BoundaryExpansions;
typedef std::vector<geometry_msgs::msg::PoseStamped>::iterator PathIterator;
typedef std::vector<geometry_msgs::msg::PoseStamped>::reverse_iterator ReversePathIterator;



class Smoother
{
public:
  

  explicit Smoother(const SmootherParams & params);

  

  ~Smoother() {}

  

  void initialize(
    const double & min_turning_radius);

  

  bool smooth(
    nav_msgs::msg::Path & path,
    const nav2_costmap_2d::Costmap2D * costmap,
    const double & max_time);

protected:
  

  bool smoothImpl(
    nav_msgs::msg::Path & path,
    bool & reversing_segment,
    const nav2_costmap_2d::Costmap2D * costmap,
    const double & max_time);

  

  inline double getFieldByDim(
    const geometry_msgs::msg::PoseStamped & msg,
    const unsigned int & dim);

  

  inline void setFieldByDim(
    geometry_msgs::msg::PoseStamped & msg, const unsigned int dim,
    const double & value);

  

  std::vector<PathSegment> findDirectionalPathSegments(const nav_msgs::msg::Path & path);

  

  void enforceStartBoundaryConditions(
    const geometry_msgs::msg::Pose & start_pose,
    nav_msgs::msg::Path & path,
    const nav2_costmap_2d::Costmap2D * costmap,
    const bool & reversing_segment);

  

  void enforceEndBoundaryConditions(
    const geometry_msgs::msg::Pose & end_pose,
    nav_msgs::msg::Path & path,
    const nav2_costmap_2d::Costmap2D * costmap,
    const bool & reversing_segment);

  

  unsigned int findShortestBoundaryExpansionIdx(const BoundaryExpansions & boundary_expansions);

  

  void findBoundaryExpansion(
    const geometry_msgs::msg::Pose & start,
    const geometry_msgs::msg::Pose & end,
    BoundaryExpansion & expansion,
    const nav2_costmap_2d::Costmap2D * costmap);

  

  template<typename IteratorT>
  BoundaryExpansions generateBoundaryExpansionPoints(IteratorT start, IteratorT end);

  

  inline void updateApproximatePathOrientations(
    nav_msgs::msg::Path & path,
    bool & reversing_segment);

  double min_turning_rad_, tolerance_, data_w_, smooth_w_;
  int max_its_, refinement_ctr_;
  bool is_holonomic_, do_refinement_;
  MotionModel motion_model_;
  ompl::base::StateSpacePtr state_space_;
};

}

#endif
