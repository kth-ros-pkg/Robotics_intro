
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <ros/console.h>
#include <rviz/ogre_helpers/axes.h>
#include <rviz/ogre_helpers/shape.h>

#include <Eigen/Dense>

#include "covariance_visual.h"

template<typename Derived>
inline bool isfinite(const Eigen::MatrixBase<Derived>& x)
{
  return ( (x - x).array() == (x - x).array()).all();
}


template<typename Derived>
inline bool isnan(const Eigen::MatrixBase<Derived>& x)
{
  return !((x.array() == x.array())).all();
}

std::pair<Eigen::Matrix3d, Eigen::Vector3d> computeEigenValuesAndVectors(const geometry_msgs::PoseWithCovariance& msg,  unsigned offset)
{
  Eigen::Matrix3d covariance   = Eigen::Matrix3d::Zero();
  Eigen::Vector3d eigenValues  = Eigen::Vector3d::Identity();
  Eigen::Matrix3d eigenVectors = Eigen::Matrix3d::Zero();

  for (size_t i = 0; i < 3; ++i)
  {
    for (size_t j = 0; j < 3; ++j)
    {
      covariance (i, j) = msg.covariance[(i + offset) * 6 + j + offset];
    }
  }

  // Compute eigen values and eigen vectors.
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(covariance);

  if (eigensolver.info() == Eigen::Success)
  {
    eigenValues  = eigensolver.eigenvalues();
    eigenVectors = eigensolver.eigenvectors();
  }
  else
  {
    ROS_WARN_THROTTLE(1, "Failed to compute eigen vectors/values. Is the covariance matrix correct?");
  }

  return std::make_pair(eigenVectors, eigenValues);
}

Ogre::Quaternion computeRotation(const geometry_msgs::PoseWithCovariance& msg, std::pair<Eigen::Matrix3d, Eigen::Vector3d>& pair)
{
  Ogre::Matrix3 rotation;

  for (size_t i = 0; i < 3; ++i)
  {
    for (size_t j = 0; j < 3; ++j)
    {
      rotation[i][j] = pair.first(i, j);
    }
  }

  return Ogre::Quaternion(rotation);
}

namespace rviz_plugin_covariance
{

CovarianceVisual::CovarianceVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node)
  : frame_node_(parent_node->createChildSceneNode())
  , scene_manager_(scene_manager)
  , scale_(1.0)
{
  axes_.reset(new rviz::Axes(scene_manager_, frame_node_));

  position_node_    = axes_->getSceneNode()->createChildSceneNode();
  orientation_node_ = axes_->getSceneNode()->createChildSceneNode();

  position_shape_.reset(new rviz::Shape(rviz::Shape::Sphere, scene_manager_, position_node_));
  orientation_shape_.reset(new rviz::Shape(rviz::Shape::Cone, scene_manager_, orientation_node_));

  axes_->getSceneNode()->setVisible(show_axis_);
  position_node_->setVisible(show_position_);
  orientation_node_->setVisible(use_6dof_ && show_orientation_);
}

CovarianceVisual::~CovarianceVisual()
{
  scene_manager_->destroySceneNode(orientation_node_);
  scene_manager_->destroySceneNode(position_node_);
  scene_manager_->destroySceneNode(frame_node_);
}

void CovarianceVisual::setMessage(const geometry_msgs::PoseWithCovariance& msg)
{
  // Construct pose position and orientation.
  const geometry_msgs::Point& p = msg.pose.position;
  Ogre::Vector3 position(p.x, p.y, p.z);
  Ogre::Quaternion orientation(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z);

  // Set position and orientation for axes scene node.
  if (!position.isNaN())
  {
    axes_->setPosition(position);
  }
  else
  {
    ROS_WARN_STREAM_THROTTLE(1, "Position contains NaN: " << position);
  }

  if (!orientation.isNaN())
  {
    axes_->setOrientation(orientation);
  }
  else
  {
    ROS_WARN_STREAM_THROTTLE(1, "Orientation contains NaN: " << orientation);
  }

  if (use_6dof_)
  {
    // Check for NaN in covariance
    for (size_t i = 0; i < 3; ++i)
    {
      if (isnan(msg.covariance[i]))
      {
        ROS_WARN_THROTTLE(1, "Covariance contains NaN");
        return;
      }
    }

    // Compute eigen values and vectors for both shapes.
    std::pair<Eigen::Matrix3d, Eigen::Vector3d> positionEigenVectorsAndValues(computeEigenValuesAndVectors(msg, 0));
    std::pair<Eigen::Matrix3d, Eigen::Vector3d> orientationEigenVectorsAndValues(computeEigenValuesAndVectors(msg, 3));

    Ogre::Quaternion positionQuaternion(computeRotation(msg, positionEigenVectorsAndValues));
    Ogre::Quaternion orientationQuaternion(computeRotation(msg, orientationEigenVectorsAndValues));

    position_node_->setOrientation(positionQuaternion);
    orientation_node_->setOrientation(orientationQuaternion);

    // Compute scaling.
    Ogre::Vector3 positionScaling(
        std::sqrt(positionEigenVectorsAndValues.second[0]),
        std::sqrt(positionEigenVectorsAndValues.second[1]),
        std::sqrt(positionEigenVectorsAndValues.second[2]));
    positionScaling *= scale_;

    Ogre::Vector3 orientationScaling(
        std::sqrt(orientationEigenVectorsAndValues.second[0]),
        std::sqrt(orientationEigenVectorsAndValues.second[1]),
        std::sqrt(orientationEigenVectorsAndValues.second[2]));
    orientationScaling *= scale_;

    // Set the scaling.
    if (!positionScaling.isNaN())
    {
      position_node_->setScale(positionScaling);
    }
    else
    {
      ROS_WARN_STREAM_THROTTLE(1, "PositionScaling contains NaN: " << positionScaling);
    }

    if (!orientationScaling.isNaN())
    {
      orientation_node_->setScale(orientationScaling);
    }
    else
    {
      ROS_WARN_STREAM_THROTTLE(1, "OrientationScaling contains NaN: " << orientationScaling);
    }

    // Debugging.
    ROS_DEBUG_STREAM_THROTTLE(1,
       "Position:\n"
       << position << "\n"
       << "Positional part 3x3 eigen values:\n"
       << positionEigenVectorsAndValues.second << "\n"
       << "Positional part 3x3 eigen vectors:\n"
       << positionEigenVectorsAndValues.first << "\n"
       << "Sphere orientation:\n"
       << positionQuaternion << "\n"
       << positionQuaternion.getYaw() << "\n"
       << "Sphere scaling:\n"
       << positionScaling << "\n"
       << "Rotational part 3x3 eigen values:\n"
       << orientationEigenVectorsAndValues.second << "\n"
       << "Rotational part 3x3 eigen vectors:\n"
       << orientationEigenVectorsAndValues.first << "\n"
       << "Cone orientation:\n"
       << orientationQuaternion << "\n"
       << orientationQuaternion.getRoll() << " "
       << orientationQuaternion.getPitch() << " "
       << orientationQuaternion.getYaw() << "\n"
       << "Cone scaling:\n"
       << orientationScaling);
  }
  else // 3DOF
  {
    // Take (x, y, th) part from the covariance matrix:
    //   x y z R P Y
    // x * *       x
    // y * *       x
    // z
    // R
    // P
    // Y x x       *
    //
    // Actually, we only take the elements marked with '*', although we could
    // also take a 3x3 matrix with the '*' and 'x' elements.
    Eigen::Matrix2d cov_xy; cov_xy << msg.covariance[0], msg.covariance[1],
                                      msg.covariance[6], msg.covariance[7];
    const double cov_yaw = msg.covariance[35];

    // Check for NaN in covariance
    if (isnan(cov_xy) || isnan(cov_yaw))
    {
      ROS_WARN_STREAM_THROTTLE(1, "Covariance contains NaN: " <<
          "C_xy = " << cov_xy << ", C_yaw = " << cov_yaw);
      return;
    }

    // Compute eigen values and vectors for xy
    Eigen::Vector2d eigen_values  = Eigen::Vector2d::Identity();
    Eigen::Matrix2d eigen_vectors = Eigen::Matrix2d::Zero();

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigensolver(cov_xy);
    if (eigensolver.info() == Eigen::Success)
    {
      eigen_values  = eigensolver.eigenvalues();
      eigen_vectors = eigensolver.eigenvectors();
    }
    else
    {
      ROS_WARN_THROTTLE(1, "Failed to compute eigen values/vectors. Is the covariance matrix correct?");
    }

    // Compute ellipsoid angle, and axes
    const double yaw = atan2(eigen_vectors(1, 0), eigen_vectors(0, 0));
    const double axis_major = sqrt(eigen_values[0]);
    const double axis_minor = sqrt(eigen_values[1]);
    const double axis_yaw   = sqrt(cov_yaw);

    // Compute the ellipsoid orientation
    Ogre::Quaternion positionQuaternion;
    Ogre::Matrix3 R;
    R.FromEulerAnglesXYZ(Ogre::Radian(0.0), Ogre::Radian(0.0), Ogre::Radian(yaw));
    positionQuaternion.FromRotationMatrix(R);

    // Set the orientation
    position_node_->setOrientation(positionQuaternion);

    // Compute the ellipsoid scale
    Ogre::Vector3 positionScaling(
        std::isnormal(axis_major) ? axis_major : 0.001,
        std::isnormal(axis_minor) ? axis_minor : 0.001,
        std::isnormal(axis_yaw  ) ? axis_yaw   : 0.001);
    positionScaling *= scale_;

    // Set the scaling
    if (!positionScaling.isNaN())
    {
      position_node_->setScale(positionScaling);
    }
    else
    {
      ROS_WARN_STREAM_THROTTLE(1, "PositionScaling contains NaN: " << positionScaling);
    }

    // Debugging
    ROS_DEBUG_STREAM_THROTTLE(1,
       "Position:\n"
       << position << "\n"
       << "C_xy:\n"
       << cov_xy << "\n"
       << "C_yaw:\n"
       << cov_yaw << "\n"
       << "axis (major, minor, yaw): (" << axis_major << ", " << axis_minor << ", " << axis_yaw << ")\n"
       << "yaw: " << yaw << "\n"
       << "Positional part 2x2 eigen values:\n"
       << eigen_values << "\n"
       << "Positional part 2x2 eigen vectors:\n"
       << eigen_vectors << "\n"
       << "Sphere orientation:\n"
       << positionQuaternion << "\n"
       << positionQuaternion.getRoll() << " "
       << positionQuaternion.getPitch() << " "
       << positionQuaternion.getYaw() << "\n"
       << "Sphere scaling:\n"
       << positionScaling);
  }
}

void CovarianceVisual::setMessage(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  setMessage(msg->pose);
}

void CovarianceVisual::setMessage(const nav_msgs::OdometryConstPtr& msg)
{
  setMessage(msg->pose);
}

void CovarianceVisual::setFramePosition(const Ogre::Vector3& position)
{
  frame_node_->setPosition(position);
}

void CovarianceVisual::setFrameOrientation(const Ogre::Quaternion& orientation)
{
  frame_node_->setOrientation(orientation);
}

void CovarianceVisual::setColor(float r, float g, float b, float a)
{
  position_shape_->setColor(r, g, b, a);
  orientation_shape_->setColor(r, g, b, a);
}

void CovarianceVisual::setScale(float scale)
{
  scale_ = scale;
}

void CovarianceVisual::setShowAxis(bool show_axis)
{
  show_axis_ = show_axis;

  axes_->getSceneNode()->setVisible(show_axis);
  position_node_->setVisible(show_position_);
  orientation_node_->setVisible(use_6dof_ && show_orientation_);
}

void CovarianceVisual::setShowPosition(bool show_position)
{
  show_position_ = show_position;

  position_node_->setVisible(show_position_);
}

void CovarianceVisual::setShowOrientation(bool show_orientation)
{
  show_orientation_ = show_orientation;

  orientation_node_->setVisible(use_6dof_ && show_orientation_);
}

void CovarianceVisual::setUse6DOF(bool use_6dof)
{
  use_6dof_ = use_6dof;

  orientation_node_->setVisible(use_6dof && show_orientation_);
}

} // end namespace rviz_plugin_covariance
