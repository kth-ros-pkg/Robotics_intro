///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2016, PAL Robotics S.L.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of PAL Robotics S.L. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

#include <cassert>
#include <boost/foreach.hpp>

#include <gazebo/sensors/SensorManager.hh>

#include <urdf_parser/urdf_parser.h>
#include <pluginlib/class_list_macros.h>
#include <angles/angles.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <joint_limits_interface/joint_limits_urdf.h>
#include <transmission_interface/transmission_interface_loader.h>

#include <pal_hardware_gazebo/pal_hardware_gazebo.h>

#include <dynamic_introspection/dynamic_introspection.h>
#include <gazebo/gazebo_config.h>

typedef Eigen::Vector3d eVector3;
typedef Eigen::Isometry3d eMatrixHom;
typedef Eigen::Matrix3d eMatrixRot;
typedef Eigen::Quaternion<double> eQuaternion;

using std::vector;
using std::string;

namespace xh
{
class XmlrpcHelperException : public ros::Exception
{
public:
  XmlrpcHelperException(const std::string& what) : ros::Exception(what)
  {
  }
};

typedef XmlRpc::XmlRpcValue Struct;
typedef XmlRpc::XmlRpcValue Array;

template <class T>
void fetchParam(ros::NodeHandle nh, const std::string& param_name, T& output)
{
  XmlRpc::XmlRpcValue val;
  bool ok = false;
  try
  {
    ok = nh.getParam(param_name, val);
  }
  catch (const ros::InvalidNameException& e)
  {
  }

  if (!ok)
  {
    std::ostringstream err_msg;
    err_msg << "could not load parameter '" << param_name
            << "'. (namespace: " << nh.getNamespace() << ")";
    throw XmlrpcHelperException(err_msg.str());
  }

  output = static_cast<T>(val);
}
}

inline std::vector<std::string> getIds(const ros::NodeHandle& nh, const std::string& key)
{
  using std::vector;
  using std::string;

  xh::Struct xh_st;
  try
  {
    xh::fetchParam(nh, key, xh_st);
  }
  catch (const xh::XmlrpcHelperException&)
  {
    ROS_DEBUG_STREAM("Requested data found in the parameter server (namespace "
                     << nh.getNamespace() + "/" + key << ").");
    return vector<string>();
  }

  vector<string> out;
  for (xh::Struct::iterator it = xh_st.begin(); it != xh_st.end(); ++it)
  {
    out.push_back(it->first);
  }
  return out;
}

void convert(const urdf::Vector3& in, eVector3& out)
{
  out = eVector3(in.x, in.y, in.z);
}

void convert(const urdf::Rotation& in, eMatrixRot& out)
{
  out = eQuaternion(in.w, in.x, in.y, in.z);
}

inline eMatrixHom createMatrix(eMatrixRot const& rot, eVector3 const& trans)
{
  eMatrixHom temp;
  temp.setIdentity();
  temp = (rot);
  temp.translation() = trans;
  return temp;
}

void convert(const urdf::Pose& in, eMatrixHom& out)
{
  eVector3 r;
  convert(in.position, r);
  eMatrixRot E;
  convert(in.rotation, E);
  out = createMatrix(E, r);
}

template <typename T>
Eigen::Matrix<T, 3, 3> skew(const Eigen::Matrix<T, 3, 1>& vec)
{
  return (Eigen::Matrix<T, 3, 3>() << T(0), -vec(2), vec(1), vec(2), T(0), -vec(0),
          -vec(1), vec(0), T(0))
      .finished();
}

namespace gazebo_ros_control
{
using namespace hardware_interface;

bool PalHardwareGazebo::parseForceTorqueSensors(ros::NodeHandle& nh,
                                                gazebo::physics::ModelPtr model,
                                                const urdf::Model* const urdf_model)
{
  using std::vector;
  using std::string;

  const string ft_ns = "force_torque";
  vector<string> ft_ids = getIds(nh, ft_ns);
  ros::NodeHandle ft_nh(nh, ft_ns);
  typedef vector<string>::const_iterator Iterator;
  for (Iterator it = ft_ids.begin(); it != ft_ids.end(); ++it)
  {
    std::string sensor_name = *it;
    std::string sensor_joint_name;
    std::string sensor_frame_id;
    ros::NodeHandle ft_sensor_nh(ft_nh, sensor_name);
    xh::fetchParam(ft_sensor_nh, "frame", sensor_frame_id);
    xh::fetchParam(ft_sensor_nh, "sensor_joint", sensor_joint_name);

    ForceTorqueSensorDefinitionPtr ft(
        new ForceTorqueSensorDefinition(sensor_name, sensor_joint_name, sensor_frame_id));

    ft->gazebo_joint = model->GetJoint(ft->sensorJointName);

    // Get sensor parent transform
    boost::shared_ptr<const urdf::Link> urdf_sensor_link;
    boost::shared_ptr<const urdf::Joint> urdf_sensor_joint;
    urdf_sensor_link = urdf_model->getLink(ft->sensorFrame);
    urdf_sensor_joint = urdf_model->getJoint(sensor_joint_name);

    if (!urdf_sensor_link)
    {
      ROS_ERROR_STREAM("Problem finding link: " << ft->sensorFrame
                                                << " to attach FT sensor in robot model");
      return false;
    }

    if (!urdf_sensor_joint)
    {
      ROS_ERROR_STREAM("Problem finding joint: " << ft->sensorJointName
                                                 << " to attach FT sensor in robot model");
      return false;
    }

    // Recursively follow the transform until the parent
    bool parentFound = false;
    eMatrixHom sensorTransform;
    sensorTransform.setIdentity();

    // Check that is not the actual first link
    if (urdf_sensor_link->name == urdf_sensor_joint->child_link_name)
    {
      parentFound = true;
    }

    while (!parentFound)
    {
      std::cerr << "      " << urdf_sensor_link->name << " - "
                << urdf_sensor_joint->child_link_name << std::endl;

      urdf::Pose tf_urdf = urdf_sensor_link->parent_joint->parent_to_joint_origin_transform;
      eMatrixHom tf_eigen;
      convert(tf_urdf, tf_eigen);
      sensorTransform = tf_eigen * sensorTransform;

      urdf_sensor_link = urdf_sensor_link->getParent();

      if (urdf_sensor_joint->child_link_name == urdf_sensor_link->name)
      {
        parentFound = true;
      }
    }

    if (!parentFound)
    {
      ROS_ERROR_STREAM("No frame found for force torque sensor");
    }

    // std::cerr<<"Sensor name: "<<sensor_name<<"transform:
    // "<<std::endl<<sensorTransform.matrix()<<std::endl;
    // std::cerr<<"Sensor name: "<<sensor_name<<"transform transpose:
    // "<<std::endl<<sensorTransform.matrix().transpose()<<std::endl;

    ft->sensorTransform = sensorTransform;

    if (!ft->gazebo_joint)
    {
      ROS_ERROR_STREAM("Could not find joint '"
                       << ft->sensorJointName << "' to which a force-torque sensor is attached.");
      return false;
    }

    forceTorqueSensorDefinitions_.push_back(ft);
    ROS_INFO_STREAM("Parsed fake FT sensor: " << sensor_name << " in frame: " << sensor_frame_id);
  }
  return true;
}

bool PalHardwareGazebo::parseIMUSensors(ros::NodeHandle& nh, gazebo::physics::ModelPtr model,
                                        const urdf::Model* const urdf_model)
{
  using std::vector;
  using std::string;

  const string imu_ns = "imu";
  vector<string> imu_ids = getIds(nh, imu_ns);
  ros::NodeHandle imu_nh(nh, imu_ns);
  typedef vector<string>::const_iterator Iterator;
  for (Iterator it = imu_ids.begin(); it != imu_ids.end(); ++it)
  {
    std::string sensor_name = *it;
    std::string sensor_frame_id;
    ros::NodeHandle imu_sensor_nh(imu_nh, sensor_name);
    xh::fetchParam(imu_sensor_nh, "frame", sensor_frame_id);

    std::string gazeboSensorName;
    xh::fetchParam(imu_sensor_nh, "gazebo_sensor_name", gazeboSensorName);

    //      std::shared_ptr<gazebo::sensors::ImuSensor> imu_sensor;
    gazebo::sensors::ImuSensorPtr imu_sensor =
        std::dynamic_pointer_cast<gazebo::sensors::ImuSensor>(
            gazebo::sensors::SensorManager::Instance()->GetSensor(gazeboSensorName));
    if (!imu_sensor)
    {
      ROS_ERROR_STREAM("Could not find base IMU sensor.");
      return false;
    }

    ImuSensorDefinitionPtr imu(new ImuSensorDefinition(sensor_name, sensor_frame_id));
    imu->gazebo_imu_sensor = imu_sensor;
    imuSensorDefinitions_.push_back(imu);

#if GAZEBO_MAJOR_VERSION >= 8 || (GAZEBO_MAJOR_VERSION == 7 && GAZEBO_MINOR_VERSION >= 11)
    imu_sensor->SetWorldToReferenceOrientation(ignition::math::Quaterniond::Identity);
#endif
    ROS_INFO_STREAM("Parsed imu sensor: " << sensor_name << " in frame: " << sensor_frame_id);
  }
  return true;
}

PalHardwareGazebo::PalHardwareGazebo() : DefaultRobotHWSim()
{
}

bool PalHardwareGazebo::initSim(const std::string& robot_ns, ros::NodeHandle nh,
                                gazebo::physics::ModelPtr model,
                                const urdf::Model* const urdf_model,
                                std::vector<transmission_interface::TransmissionInfo> transmissions)
{
  ROS_INFO_STREAM("Loading PAL HARWARE GAZEBO");

  if (!DefaultRobotHWSim::initSim(robot_ns, nh, model, urdf_model, transmissions))
  {
    return false;
  }

  parseForceTorqueSensors(nh, model, urdf_model);

  for (size_t i = 0; i < forceTorqueSensorDefinitions_.size(); ++i)
  {
    ForceTorqueSensorDefinitionPtr& ft = forceTorqueSensorDefinitions_[i];
    ft_sensor_interface_.registerHandle(ForceTorqueSensorHandle(
        ft->sensorName, ft->sensorFrame, &ft->force[0], &ft->torque[0]));
  }

  registerInterface(&ft_sensor_interface_);
  ROS_DEBUG_STREAM("Registered force-torque sensors.");

  // Hardware interfaces: Base IMU sensors
  parseIMUSensors(nh, model, urdf_model);

  for (size_t i = 0; i < imuSensorDefinitions_.size(); ++i)
  {
    ImuSensorDefinitionPtr& imu = imuSensorDefinitions_[i];

    ImuSensorHandle::Data data;
    data.name = imu->sensorName;
    data.frame_id = imu->sensorFrame;
    data.orientation = &imu->orientation[0];
    data.linear_acceleration = &imu->linear_acceleration[0];
    data.angular_velocity = &imu->base_ang_vel[0];
    imu_sensor_interface_.registerHandle(ImuSensorHandle(data));
  }

  registerInterface(&imu_sensor_interface_);
  ROS_DEBUG_STREAM("Registered IMU sensor.");

  return true;
}

void PalHardwareGazebo::readSim(ros::Time time, ros::Duration period)
{
  // read all resources
  BOOST_FOREACH (RwResPtr res, rw_resources_)
  {
    res->read(time, period, e_stop_active_);
  }

  // Read force-torque sensors
  for (size_t i = 0; i < forceTorqueSensorDefinitions_.size(); ++i)
  {
    ForceTorqueSensorDefinitionPtr& ft = forceTorqueSensorDefinitions_[i];
    gazebo::physics::JointWrench ft_wrench = ft->gazebo_joint->GetForceTorque(0u);

    ft->force[0] = ft_wrench.body2Force.x;
    ft->force[1] = ft_wrench.body2Force.y;
    ft->force[2] = ft_wrench.body2Force.z;
    ft->torque[0] = ft_wrench.body2Torque.x;
    ft->torque[1] = ft_wrench.body2Torque.y;
    ft->torque[2] = ft_wrench.body2Torque.z;

    // Transform to sensor frame
    Eigen::MatrixXd transform(6, 6);
    transform.setZero();
    transform.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
    transform.block(3, 3, 3, 3) = ft->sensorTransform.rotation().transpose();
    eVector3 r = ft->sensorTransform.translation();
    transform.block(3, 0, 3, 3) = skew(r) * ft->sensorTransform.rotation().transpose();

    Eigen::VectorXd wrench(6);
    wrench << ft->torque[0], ft->torque[1], ft->torque[2], ft->force[0], ft->force[1],
        ft->force[2];
    Eigen::VectorXd transformedWrench(6);

    transformedWrench = transform * wrench;

    ft->torque[0] = transformedWrench(0);
    ft->torque[1] = transformedWrench(1);
    ft->torque[2] = transformedWrench(2);
    ft->force[0] = transformedWrench(3);
    ft->force[1] = transformedWrench(4);
    ft->force[2] = transformedWrench(5);
  }

  // Read IMU sensor
  for (size_t i = 0; i < imuSensorDefinitions_.size(); ++i)
  {
    ImuSensorDefinitionPtr& imu = imuSensorDefinitions_[i];

    gazebo::math::Quaternion imu_quat = imu->gazebo_imu_sensor->Orientation();
    imu->orientation[0] = imu_quat.x;
    imu->orientation[1] = imu_quat.y;
    imu->orientation[2] = imu_quat.z;
    imu->orientation[3] = imu_quat.w;

    gazebo::math::Vector3 imu_ang_vel = imu->gazebo_imu_sensor->AngularVelocity();
    imu->base_ang_vel[0] = imu_ang_vel.x;
    imu->base_ang_vel[1] = imu_ang_vel.y;
    imu->base_ang_vel[2] = imu_ang_vel.z;

    gazebo::math::Vector3 imu_lin_acc = imu->gazebo_imu_sensor->LinearAcceleration();
    imu->linear_acceleration[0] = imu_lin_acc.x;
    imu->linear_acceleration[1] = imu_lin_acc.y;
    imu->linear_acceleration[2] = imu_lin_acc.z;
  }
}

void PalHardwareGazebo::writeSim(ros::Time time, ros::Duration period)
{
  boost::unique_lock<boost::mutex> lock(mutex_);
  BOOST_FOREACH (RwResPtr res, active_w_resources_rt_)
  {
    res->write(time, period, e_stop_active_);
  }
  PUBLISH_DEBUG_DATA_TOPIC;
}
}

PLUGINLIB_EXPORT_CLASS(gazebo_ros_control::PalHardwareGazebo, gazebo_ros_control::RobotHWSim)
