
#ifndef COVARIANCE_DISPLAY_H
#define COVARIANCE_DISPLAY_H

#ifndef Q_MOC_RUN
#include <geometry_msgs/PoseWithCovarianceStamped.h>
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

class CovarianceDisplay : public rviz::MessageFilterDisplay<geometry_msgs::PoseWithCovarianceStamped>
{
Q_OBJECT
public:
  CovarianceDisplay();
  virtual ~CovarianceDisplay();

protected:
  virtual void onInitialize();
  virtual void reset();
  //virtual void onEnableChanged();
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
  void processMessage(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

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

#endif // COVARIANCE_DISPLAY_H
