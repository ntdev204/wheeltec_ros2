













#ifndef NAV2_COLLISION_MONITOR__CIRCLE_HPP_
#define NAV2_COLLISION_MONITOR__CIRCLE_HPP_

#include <memory>
#include <vector>
#include <string>

#include "nav2_collision_monitor/polygon.hpp"

namespace nav2_collision_monitor
{



class Circle : public Polygon
{
public:
  

  Circle(
    const nav2_util::LifecycleNode::WeakPtr & node,
    const std::string & polygon_name,
    const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::string & base_frame_id,
    const tf2::Duration & transform_tolerance);
  

  ~Circle();

  

  void getPolygon(std::vector<Point> & poly) const override;

  

  int getPointsInside(const std::vector<Point> & points) const override;

protected:
  

  bool getParameters(std::string & polygon_pub_topic, std::string & footprint_topic) override;




  double radius_;

  double radius_squared_;
};

}

#endif
