
#ifndef ODOMETRY_DISPLAY_H
#define ODOMETRY_DISPLAY_H

#ifndef Q_MOC_RUN
#include <nav_msgs/Odometry.h>
#include <rviz/message_filter_display.h>
#endif

namespace rviz
{
class ColorProperty;
class FloatProperty;
class BoolProperty;
}

namespace Ogre
{
class SceneNode;
}

namespace rviz_plugin_covariance
{

class CovarianceVisual;

class OdometryDisplay : public rviz::MessageFilterDisplay<nav_msgs::Odometry>
{
Q_OBJECT
public:
  OdometryDisplay();
  virtual ~OdometryDisplay();

protected:
  virtual void onInitialize();
  virtual void reset();
  virtual void onEnable();
  virtual void updateTopic();

private Q_SLOTS:
  void updateColorAndAlpha();
  void updateScale();
  void updateShowAxis();
  void updateShowPosition();
  void updateShowOrientation();
  void updateUse6DOF();

private:
  void processMessage(const nav_msgs::OdometryConstPtr& msg);

  boost::shared_ptr<CovarianceVisual> visual_;

  rviz::ColorProperty* color_property_;
  rviz::FloatProperty* alpha_property_;
  rviz::FloatProperty* scale_property_;

  rviz::BoolProperty* show_axis_property_;
  rviz::BoolProperty* show_position_property_;
  rviz::BoolProperty* show_orientation_property_;
  rviz::BoolProperty* use_6dof_property_;
};

} // end namespace rviz_plugin_covariance

#endif // ODOMETRY_DISPLAY_H
