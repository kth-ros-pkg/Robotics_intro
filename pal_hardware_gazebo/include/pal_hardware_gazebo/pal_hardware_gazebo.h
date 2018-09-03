#ifndef PAL_HARDWARE_GAZEBO_H
#define PAL_HARDWARE_GAZEBO_H

#include <vector>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <control_toolbox/pid.h>

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <gazebo_ros_control/robot_hw_sim.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/ImuSensor.hh>

#include <gazebo_ros_control/default_robot_hw_sim.h>

typedef Eigen::Isometry3d eMatrixHom;

namespace gazebo_ros_control
{

  class ForceTorqueSensorDefinition{
  public:
      gazebo::physics::JointPtr gazebo_joint;
      std::string sensorName;
      std::string sensorJointName;
      std::string sensorFrame;
      double force[3];
      double torque[3];
      eMatrixHom sensorTransform;

      ForceTorqueSensorDefinition(const std::string &name,
                                  const std::string &sensor_joint_name,
                                  const std::string &frame){
          sensorName = name;
          sensorJointName = sensor_joint_name;
          sensorFrame = frame;
          for(size_t i=0; i<3; ++i){
            force[i] = 0.;
            torque[i] = 0.;
          }
      }
  };
  typedef boost::shared_ptr<ForceTorqueSensorDefinition> ForceTorqueSensorDefinitionPtr;

  class ImuSensorDefinition{
  public:
      std::shared_ptr<gazebo::sensors::ImuSensor> gazebo_imu_sensor;
      std::string sensorName;
      std::string sensorFrame;

      double orientation[4];
      double linear_acceleration[3];
      double base_ang_vel[3];

      ImuSensorDefinition(const std::string &name, const std::string &frame){
          sensorName = name;
          sensorFrame = frame;
          for(size_t i=0; i<4; ++i){
            orientation[i] = 0.;
          }

          for(size_t i=0; i<3; ++i){
            linear_acceleration[i] = 0.;
            base_ang_vel[i] = 0.;
          }
      }
  };

  typedef boost::shared_ptr<ImuSensorDefinition> ImuSensorDefinitionPtr;

class PalHardwareGazebo : public DefaultRobotHWSim
{
public:

  PalHardwareGazebo();

  // Simulation-specific
  bool initSim(const std::string& robot_ns,
               ros::NodeHandle nh,
               gazebo::physics::ModelPtr model,
               const urdf::Model* const urdf_model,
               std::vector<transmission_interface::TransmissionInfo> transmissions);
  void readSim(ros::Time time, ros::Duration period);
  void writeSim(ros::Time time, ros::Duration period);

private:

  bool parseForceTorqueSensors(ros::NodeHandle &nh,
                               gazebo::physics::ModelPtr model,
                               const urdf::Model* const urdf_model);

  bool parseIMUSensors(ros::NodeHandle &nh,
                       gazebo::physics::ModelPtr model,
                       const urdf::Model* const urdf_model);

  // Simulation-specific
  //std::vector<gazebo::physics::JointPtr> sim_joints_;
  //gazebo::physics::JointPtr right_ankle_;
  //boost::shared_ptr<gazebo::sensors::ImuSensor> imu_sensor_;

  // Hardware interface: sensors
  hardware_interface::ForceTorqueSensorInterface ft_sensor_interface_;
  hardware_interface::ImuSensorInterface         imu_sensor_interface_;

  std::vector<ForceTorqueSensorDefinitionPtr> forceTorqueSensorDefinitions_;
  std::vector<ImuSensorDefinitionPtr> imuSensorDefinitions_;

};

}

#endif // PAL_HARDWARE_GAZEBO_H
