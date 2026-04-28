

















#ifndef NAV2_RVIZ_PLUGINS__PARTICLE_CLOUD_DISPLAY__PARTICLE_CLOUD_DISPLAY_HPP_
#define NAV2_RVIZ_PLUGINS__PARTICLE_CLOUD_DISPLAY__PARTICLE_CLOUD_DISPLAY_HPP_

#include <memory>
#include <vector>

#include "nav2_msgs/msg/particle_cloud.hpp"

#include "rviz_rendering/objects/shape.hpp"
#include "rviz_common/message_filter_display.hpp"

namespace Ogre
{
class ManualObject;
}

namespace rviz_common
{
namespace properties
{
class EnumProperty;
class ColorProperty;
class FloatProperty;
}
}

namespace rviz_rendering
{
class Arrow;
class Axes;
}

namespace nav2_rviz_plugins
{
class FlatWeightedArrowsArray;
struct OgrePoseWithWeight
{
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  float weight;
};


class ParticleCloudDisplay : public rviz_common::MessageFilterDisplay<nav2_msgs::msg::ParticleCloud>
{
  Q_OBJECT

public:


  ParticleCloudDisplay(
    rviz_common::DisplayContext * display_context,
    Ogre::SceneNode * scene_node);
  ParticleCloudDisplay();
  ~ParticleCloudDisplay() override;

  void processMessage(nav2_msgs::msg::ParticleCloud::ConstSharedPtr msg) override;
  void setShape(QString shape);

protected:
  void onInitialize() override;
  void reset() override;

private Q_SLOTS:

  void updateShapeChoice();


  void updateArrowColor();


  void updateGeometry();

private:
  void initializeProperties();
  bool validateFloats(const nav2_msgs::msg::ParticleCloud & msg);
  bool setTransform(std_msgs::msg::Header const & header);
  void updateDisplay();
  void updateArrows2d();
  void updateArrows3d();
  void updateAxes();
  void updateArrow3dGeometry();
  void updateAxesGeometry();

  std::unique_ptr<rviz_rendering::Axes> makeAxes();
  std::unique_ptr<rviz_rendering::Arrow> makeArrow3d();

  std::vector<OgrePoseWithWeight> poses_;
  std::unique_ptr<FlatWeightedArrowsArray> arrows2d_;
  std::vector<std::unique_ptr<rviz_rendering::Arrow>> arrows3d_;
  std::vector<std::unique_ptr<rviz_rendering::Axes>> axes_;

  Ogre::SceneNode * arrow_node_;
  Ogre::SceneNode * axes_node_;

  rviz_common::properties::EnumProperty * shape_property_;
  rviz_common::properties::ColorProperty * arrow_color_property_;
  rviz_common::properties::FloatProperty * arrow_alpha_property_;

  rviz_common::properties::FloatProperty * arrow_min_length_property_;
  rviz_common::properties::FloatProperty * arrow_max_length_property_;

  float min_length_;
  float max_length_;
  float length_scale_;
  float head_radius_scale_;
  float head_length_scale_;
  float shaft_radius_scale_;
};

}

#endif
