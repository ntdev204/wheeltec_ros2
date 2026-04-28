

















#ifndef NAV2_RVIZ_PLUGINS__PARTICLE_CLOUD_DISPLAY__FLAT_WEIGHTED_ARROWS_ARRAY_HPP_
#define NAV2_RVIZ_PLUGINS__PARTICLE_CLOUD_DISPLAY__FLAT_WEIGHTED_ARROWS_ARRAY_HPP_

#include <vector>

#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreSceneNode.h>
#include <OgreVector3.h>
#include <OgreQuaternion.h>

#include "nav2_rviz_plugins/particle_cloud_display/particle_cloud_display.hpp"

namespace nav2_rviz_plugins
{

struct OgrePoseWithWeight;

class FlatWeightedArrowsArray
{
public:
  explicit FlatWeightedArrowsArray(Ogre::SceneManager * scene_manager_);
  ~FlatWeightedArrowsArray();

  void createAndAttachManualObject(Ogre::SceneNode * scene_node);
  void updateManualObject(
    Ogre::ColourValue color,
    float alpha,
    float min_length,
    float max_length,
    const std::vector<nav2_rviz_plugins::OgrePoseWithWeight> & poses);
  void clear();

private:
  void setManualObjectMaterial();
  void setManualObjectVertices(
    const Ogre::ColourValue & color,
    float min_length,
    float max_length,
    const std::vector<nav2_rviz_plugins::OgrePoseWithWeight> & poses);

  Ogre::SceneManager * scene_manager_;
  Ogre::ManualObject * manual_object_;
  Ogre::MaterialPtr material_;
};

}

#endif
