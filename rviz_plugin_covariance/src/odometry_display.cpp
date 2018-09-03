
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/frame_manager.h>

#include "covariance_visual.h"

#include "odometry_display.h"

namespace rviz_plugin_covariance
{

OdometryDisplay::OdometryDisplay()
{
  color_property_ = new rviz::ColorProperty( "Color", QColor( 204, 51, 204),
                                             "Color to draw the ellipsoid.",
                                             this, SLOT( updateColorAndAlpha() ));

  alpha_property_ = new rviz::FloatProperty( "Alpha", 1.0,
                                             "0 is fully transparent, 1.0 is fully opaque.",
                                             this, SLOT( updateColorAndAlpha() ));

  scale_property_ = new rviz::FloatProperty( "Scale", 1.0,
                                             "Ellipsoid scale factor.",
                                             this, SLOT( updateScale() ));

  show_axis_property_ = new rviz::BoolProperty( "Axis", false,
                                                "Show odometry axis.",
                                                this, SLOT( updateShowAxis() ));

  show_position_property_ = new rviz::BoolProperty( "Position", true,
                                                    "Show position.",
                                                    this, SLOT( updateShowPosition() ));

  show_orientation_property_ = new rviz::BoolProperty( "Orientation", true,
                                                       "Show orientation (only for 6DOF).",
                                                       this, SLOT( updateShowOrientation() ));

  use_6dof_property_ = new rviz::BoolProperty( "6DOF", false,
                                               "Use 6DOF (x, y, z, roll, pitch, yaw) or 3DOF (x, y, yaw).",
                                               this, SLOT( updateUse6DOF() ));

  updateColorAndAlpha();
  updateScale();
  updateShowAxis();
  updateShowPosition();
  updateShowOrientation();
  updateUse6DOF();
}

void OdometryDisplay::onInitialize()
{
  MFDClass::onInitialize();
  ROS_DEBUG_STREAM("onInitialize called in Rviz plugin for odom covariance!");
  updateColorAndAlpha();
  updateScale();
  updateShowAxis();
  updateShowPosition();
  updateShowOrientation();
  updateUse6DOF();
}

// should probably be called onUpdateTopic(), called when topic changes
void OdometryDisplay::updateTopic()
{
    ROS_DEBUG_STREAM("updateTopic called in Rviz plugin for odom covariance!");
    updateColorAndAlpha();
    updateScale();
    updateShowAxis();
    updateShowPosition();
    updateShowOrientation();
    updateUse6DOF();
    MFDClass::updateTopic();
}


OdometryDisplay::~OdometryDisplay()
{
    ROS_DEBUG_STREAM("updateTopic called in Rviz plugin for odom covariance!");
    updateColorAndAlpha();
    updateScale();
    updateShowAxis();
    updateShowPosition();
    updateShowOrientation();
    updateUse6DOF();
    MFDClass::updateTopic();
}

void OdometryDisplay::onEnable()
{
  MFDClass::subscribe();
  ROS_DEBUG_STREAM("onEnable called in Rviz plugin for odom covariance!");
  updateColorAndAlpha();
  updateScale();
  updateShowAxis();
  updateShowPosition();
  updateShowOrientation();
  updateUse6DOF();
}

void OdometryDisplay::reset()
{
  MFDClass::reset();
}

void OdometryDisplay::updateColorAndAlpha()
{
  const float alpha = alpha_property_->getFloat();
  const Ogre::ColourValue color = color_property_->getOgreColor();

  if (visual_)
  {
    visual_->setColor(color.r, color.g, color.b, alpha);
  }
}

void OdometryDisplay::updateScale()
{
  const float scale = scale_property_->getFloat();

  if (visual_)
  {
    visual_->setScale(scale);
  }
}

void OdometryDisplay::updateShowAxis()
{
  const bool show_axis = show_axis_property_->getBool();

  if (visual_)
  {
    visual_->setShowAxis(show_axis);
  }
}

void OdometryDisplay::updateShowPosition()
{
  const bool show_position = show_position_property_->getBool();

  if (visual_)
  {
    visual_->setShowPosition(show_position);
  }
}

void OdometryDisplay::updateShowOrientation()
{
  const bool show_orientation = show_orientation_property_->getBool();

  if (visual_)
  {
    visual_->setShowOrientation(show_orientation);
  }
}

void OdometryDisplay::updateUse6DOF()
{
  const bool use_6dof = use_6dof_property_->getBool();

  if (visual_)
  {
    visual_->setUse6DOF(use_6dof);
  }
}

void OdometryDisplay::processMessage(const nav_msgs::OdometryConstPtr& msg)
{
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!context_->getFrameManager()->getTransform( msg->header.frame_id,
                                                  msg->header.stamp,
                                                  position, orientation ))
  {
    ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'",
               msg->header.frame_id.c_str(), qPrintable( fixed_frame_ ) );
    return;
  }

  if (!visual_)
  {
    visual_.reset(new CovarianceVisual(context_->getSceneManager(), scene_node_));
  }

  visual_->setMessage(msg);
  visual_->setFramePosition(position);
  visual_->setFrameOrientation(orientation);

  updateColorAndAlpha();
  updateScale();
  // Everything must be updated, it seems
  updateShowAxis();
  updateShowPosition();
  updateShowOrientation();
  updateUse6DOF();
}

} // end namespace rviz_plugin_covariance

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugin_covariance::OdometryDisplay, rviz::Display)
