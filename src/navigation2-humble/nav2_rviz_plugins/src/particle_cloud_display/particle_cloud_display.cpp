

















#include "nav2_rviz_plugins/particle_cloud_display/particle_cloud_display.hpp"

#include <memory>
#include <string>

#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>

#include "rviz_common/logging.hpp"
#include "rviz_common/msg_conversions.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/validate_floats.hpp"

#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_rendering/objects/axes.hpp"

#include "nav2_rviz_plugins/particle_cloud_display/flat_weighted_arrows_array.hpp"

namespace nav2_rviz_plugins
{
namespace
{
struct ShapeType
{
  enum
  {
    Arrow2d,
    Arrow3d,
    Axes,
  };
};

}

ParticleCloudDisplay::ParticleCloudDisplay(
  rviz_common::DisplayContext * display_context,
  Ogre::SceneNode * scene_node)
: ParticleCloudDisplay()
{
  context_ = display_context;
  scene_node_ = scene_node;
  scene_manager_ = context_->getSceneManager();

  arrows2d_ = std::make_unique<FlatWeightedArrowsArray>(scene_manager_);
  arrows2d_->createAndAttachManualObject(scene_node);
  arrow_node_ = scene_node_->createChildSceneNode();
  axes_node_ = scene_node_->createChildSceneNode();
  updateShapeChoice();
}

ParticleCloudDisplay::ParticleCloudDisplay()
: min_length_(0.02f), max_length_(0.3f)
{
  initializeProperties();

  shape_property_->addOption("Arrow (Flat)", ShapeType::Arrow2d);
  shape_property_->addOption("Arrow (3D)", ShapeType::Arrow3d);
  shape_property_->addOption("Axes", ShapeType::Axes);
  arrow_alpha_property_->setMin(0);
  arrow_alpha_property_->setMax(1);
  arrow_min_length_property_->setMax(max_length_);
  arrow_max_length_property_->setMin(min_length_);
}

void ParticleCloudDisplay::initializeProperties()
{
  shape_property_ = new rviz_common::properties::EnumProperty(
    "Shape", "Arrow (Flat)", "Shape to display the pose as.", this, SLOT(updateShapeChoice()));

  arrow_color_property_ = new rviz_common::properties::ColorProperty(
    "Color", QColor(255, 25, 0), "Color to draw the arrows.", this, SLOT(updateArrowColor()));

  arrow_alpha_property_ = new rviz_common::properties::FloatProperty(
    "Alpha",
    1.0f,
    "Amount of transparency to apply to the displayed poses.",
    this,
    SLOT(updateArrowColor()));

  arrow_min_length_property_ = new rviz_common::properties::FloatProperty(
    "Min Arrow Length", min_length_, "Minimum length of the arrows.", this, SLOT(updateGeometry()));

  arrow_max_length_property_ = new rviz_common::properties::FloatProperty(
    "Max Arrow Length", max_length_, "Maximum length of the arrows.", this, SLOT(updateGeometry()));


  length_scale_ = max_length_ - min_length_;
  shaft_radius_scale_ = 0.0435;
  head_length_scale_ = 0.3043;
  head_radius_scale_ = 0.1304;
}

ParticleCloudDisplay::~ParticleCloudDisplay()
{


}

void ParticleCloudDisplay::onInitialize()
{
  MFDClass::onInitialize();
  arrows2d_ = std::make_unique<FlatWeightedArrowsArray>(scene_manager_);
  arrows2d_->createAndAttachManualObject(scene_node_);
  arrow_node_ = scene_node_->createChildSceneNode();
  axes_node_ = scene_node_->createChildSceneNode();
  updateShapeChoice();
}

void ParticleCloudDisplay::processMessage(const nav2_msgs::msg::ParticleCloud::ConstSharedPtr msg)
{
  if (!validateFloats(*msg)) {
    setStatus(
      rviz_common::properties::StatusProperty::Error,
      "Topic",
      "Message contained invalid floating point values (nans or infs)");
    return;
  }

  if (!setTransform(msg->header)) {
    return;
  }

  poses_.resize(msg->particles.size());

  for (std::size_t i = 0; i < msg->particles.size(); ++i) {
    poses_[i].position = rviz_common::pointMsgToOgre(msg->particles[i].pose.position);
    poses_[i].orientation = rviz_common::quaternionMsgToOgre(msg->particles[i].pose.orientation);
    poses_[i].weight = static_cast<float>(msg->particles[i].weight);
  }

  updateDisplay();

  context_->queueRender();
}

bool ParticleCloudDisplay::validateFloats(const nav2_msgs::msg::ParticleCloud & msg)
{
  for (auto & particle : msg.particles) {
    if (!rviz_common::validateFloats(particle.pose) ||
      !rviz_common::validateFloats(particle.weight))
    {
      return false;
    }
  }
  return true;
}

bool ParticleCloudDisplay::setTransform(std_msgs::msg::Header const & header)
{
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->getTransform(header, position, orientation)) {
    setMissingTransformToFixedFrame(header.frame_id);
    return false;
  }
  setTransformOk();

  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);
  return true;
}

void ParticleCloudDisplay::updateDisplay()
{
  int shape = shape_property_->getOptionInt();
  switch (shape) {
    case ShapeType::Arrow2d:
      updateArrows2d();
      arrows3d_.clear();
      axes_.clear();
      break;
    case ShapeType::Arrow3d:
      updateArrows3d();
      arrows2d_->clear();
      axes_.clear();
      break;
    case ShapeType::Axes:
      updateAxes();
      arrows2d_->clear();
      arrows3d_.clear();
      break;
  }
}

void ParticleCloudDisplay::updateArrows2d()
{
  arrows2d_->updateManualObject(
    arrow_color_property_->getOgreColor(),
    arrow_alpha_property_->getFloat(),
    min_length_,
    max_length_,
    poses_);
}

void ParticleCloudDisplay::updateArrows3d()
{
  while (arrows3d_.size() < poses_.size()) {
    arrows3d_.push_back(makeArrow3d());
  }
  while (arrows3d_.size() > poses_.size()) {
    arrows3d_.pop_back();
  }

  Ogre::Quaternion adjust_orientation(Ogre::Degree(-90), Ogre::Vector3::UNIT_Y);
  float shaft_length;
  for (std::size_t i = 0; i < poses_.size(); ++i) {
    shaft_length = std::min(
      std::max(
        poses_[i].weight * length_scale_ + min_length_,
        min_length_), max_length_);
    arrows3d_[i]->set(
      shaft_length,
      shaft_length * shaft_radius_scale_,
      shaft_length * head_length_scale_,
      shaft_length * head_radius_scale_
    );
    arrows3d_[i]->setPosition(poses_[i].position);
    arrows3d_[i]->setOrientation(poses_[i].orientation * adjust_orientation);
  }
}

void ParticleCloudDisplay::updateAxes()
{
  while (axes_.size() < poses_.size()) {
    axes_.push_back(makeAxes());
  }
  while (axes_.size() > poses_.size()) {
    axes_.pop_back();
  }
  float shaft_length;
  for (std::size_t i = 0; i < poses_.size(); ++i) {
    shaft_length = std::min(
      std::max(
        poses_[i].weight * length_scale_ + min_length_,
        min_length_), max_length_);
    axes_[i]->set(shaft_length, shaft_length * shaft_radius_scale_);
    axes_[i]->setPosition(poses_[i].position);
    axes_[i]->setOrientation(poses_[i].orientation);
  }
}

std::unique_ptr<rviz_rendering::Arrow> ParticleCloudDisplay::makeArrow3d()
{
  Ogre::ColourValue color = arrow_color_property_->getOgreColor();
  color.a = arrow_alpha_property_->getFloat();

  auto arrow = std::make_unique<rviz_rendering::Arrow>(
    scene_manager_,
    arrow_node_,
    min_length_,
    min_length_ * shaft_radius_scale_,
    min_length_ * head_length_scale_,
    min_length_ * head_radius_scale_
  );

  arrow->setColor(color);
  return arrow;
}

std::unique_ptr<rviz_rendering::Axes> ParticleCloudDisplay::makeAxes()
{
  return std::make_unique<rviz_rendering::Axes>(
    scene_manager_,
    axes_node_,
    min_length_,
    min_length_ * shaft_radius_scale_
  );
}

void ParticleCloudDisplay::reset()
{
  MFDClass::reset();
  arrows2d_->clear();
  arrows3d_.clear();
  axes_.clear();
}

void ParticleCloudDisplay::updateShapeChoice()
{
  int shape = shape_property_->getOptionInt();
  bool use_axes = shape == ShapeType::Axes;

  arrow_color_property_->setHidden(use_axes);
  arrow_alpha_property_->setHidden(use_axes);

  if (initialized()) {
    updateDisplay();
  }
}

void ParticleCloudDisplay::updateArrowColor()
{
  int shape = shape_property_->getOptionInt();
  Ogre::ColourValue color = arrow_color_property_->getOgreColor();
  color.a = arrow_alpha_property_->getFloat();

  if (shape == ShapeType::Arrow2d) {
    updateArrows2d();
  } else if (shape == ShapeType::Arrow3d) {
    for (const auto & arrow : arrows3d_) {
      arrow->setColor(color);
    }
  }
  context_->queueRender();
}

void ParticleCloudDisplay::updateGeometry()
{
  min_length_ = arrow_min_length_property_->getFloat();
  max_length_ = arrow_max_length_property_->getFloat();
  length_scale_ = max_length_ - min_length_;

  arrow_min_length_property_->setMax(max_length_);
  arrow_max_length_property_->setMin(min_length_);

  int shape = shape_property_->getOptionInt();
  switch (shape) {
    case ShapeType::Arrow2d:
      updateArrows2d();
      arrows3d_.clear();
      axes_.clear();
      break;
    case ShapeType::Arrow3d:
      updateArrow3dGeometry();
      arrows2d_->clear();
      axes_.clear();
      break;
    case ShapeType::Axes:
      updateAxesGeometry();
      arrows2d_->clear();
      arrows3d_.clear();
      break;
  }

  context_->queueRender();
}

void ParticleCloudDisplay::updateArrow3dGeometry()
{
  float shaft_length;
  for (std::size_t i = 0; i < poses_.size() && i < arrows3d_.size(); ++i) {
    shaft_length = std::min(
      std::max(
        poses_[i].weight * length_scale_ + min_length_,
        min_length_), max_length_);
    arrows3d_[i]->set(
      shaft_length,
      shaft_length * shaft_radius_scale_,
      shaft_length * head_length_scale_,
      shaft_length * head_radius_scale_
    );
  }
}

void ParticleCloudDisplay::updateAxesGeometry()
{
  float shaft_length;
  for (std::size_t i = 0; i < poses_.size() && i < axes_.size(); ++i) {
    shaft_length = std::min(
      std::max(
        poses_[i].weight * length_scale_ + min_length_,
        min_length_), max_length_);
    axes_[i]->set(shaft_length, shaft_length * shaft_radius_scale_);
  }
}

void ParticleCloudDisplay::setShape(QString shape)
{
  shape_property_->setValue(shape);
}

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(nav2_rviz_plugins::ParticleCloudDisplay, rviz_common::Display)
