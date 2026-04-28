

















#include "nav2_rviz_plugins/particle_cloud_display/flat_weighted_arrows_array.hpp"

#include <vector>
#include <string>
#include <algorithm>

#include <OgreSceneManager.h>
#include <OgreTechnique.h>

#include "rviz_rendering/material_manager.hpp"

namespace nav2_rviz_plugins
{

FlatWeightedArrowsArray::FlatWeightedArrowsArray(Ogre::SceneManager * scene_manager)
: scene_manager_(scene_manager), manual_object_(nullptr) {}

FlatWeightedArrowsArray::~FlatWeightedArrowsArray()
{
  if (manual_object_) {
    scene_manager_->destroyManualObject(manual_object_);
  }
}

void FlatWeightedArrowsArray::createAndAttachManualObject(Ogre::SceneNode * scene_node)
{
  manual_object_ = scene_manager_->createManualObject();
  manual_object_->setDynamic(true);
  scene_node->attachObject(manual_object_);
}

void FlatWeightedArrowsArray::updateManualObject(
  Ogre::ColourValue color,
  float alpha,
  float min_length,
  float max_length,
  const std::vector<nav2_rviz_plugins::OgrePoseWithWeight> & poses)
{
  clear();

  color.a = alpha;
  setManualObjectMaterial();
  rviz_rendering::MaterialManager::enableAlphaBlending(material_, alpha);

  manual_object_->begin(
    material_->getName(), Ogre::RenderOperation::OT_LINE_LIST, "rviz_rendering");
  setManualObjectVertices(color, min_length, max_length, poses);
  manual_object_->end();
}

void FlatWeightedArrowsArray::clear()
{
  if (manual_object_) {
    manual_object_->clear();
  }
}

void FlatWeightedArrowsArray::setManualObjectMaterial()
{
  static int material_count = 0;
  std::string material_name = "FlatWeightedArrowsMaterial" + std::to_string(material_count++);
  material_ = rviz_rendering::MaterialManager::createMaterialWithNoLighting(material_name);
}

void FlatWeightedArrowsArray::setManualObjectVertices(
  const Ogre::ColourValue & color,
  float min_length,
  float max_length,
  const std::vector<nav2_rviz_plugins::OgrePoseWithWeight> & poses)
{
  manual_object_->estimateVertexCount(poses.size() * 6);

  float scale = max_length - min_length;
  float length;
  for (const auto & pose : poses) {
    length = std::min(std::max(pose.weight * scale + min_length, min_length), max_length);
    Ogre::Vector3 vertices[6];
    vertices[0] = pose.position;
    vertices[1] =
      pose.position + pose.orientation * Ogre::Vector3(length, 0, 0);
    vertices[2] = vertices[1];
    vertices[3] = pose.position + pose.orientation * Ogre::Vector3(
      0.75f * length, 0.2f * length, 0);
    vertices[4] = vertices[1];
    vertices[5] = pose.position + pose.orientation * Ogre::Vector3(
      0.75f * length, -0.2f * length,
      0);

    for (const auto & vertex : vertices) {
      manual_object_->position(vertex);
      manual_object_->colour(color);
    }
  }
}

}
