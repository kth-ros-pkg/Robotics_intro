#ifndef TEMPERATURE_SENSOR_CONTROLLER_H
#define TEMPERATURE_SENSOR_CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_state_interface.h>
#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_publisher.h>
#include <boost/shared_ptr.hpp>
#include <pal_hardware_interfaces/actuator_temperature_interface.h>
#include <temperature_sensor_controller/ActuatorTemperatureState.h>

namespace temperature_sensor_controller
{

class TemperatureSensorController: public controller_interface::Controller<hardware_interface::ActuatorTemperatureSensorInterface>
{

public:

  TemperatureSensorController() : publish_rate_(0.0) {}

  virtual bool init(hardware_interface::ActuatorTemperatureSensorInterface* hw,
                    ros::NodeHandle&                         root_nh,
                    ros::NodeHandle&                         controller_nh);
  virtual void starting(const ros::Time& time);
  virtual void update(const ros::Time& time, const ros::Duration& /*period*/);
  virtual void stopping(const ros::Time& /*time*/);

private:

  std::vector<hardware_interface::ActuatorTemperatureSensorHandle> actuator_state_;
  boost::shared_ptr<realtime_tools::RealtimePublisher<temperature_sensor_controller::ActuatorTemperatureState> > realtime_pub_;
  ros::Time last_publish_time_;
  double publish_rate_;
  unsigned int num_hw_joints_; ///< Number of joints present in the JointStateInterface, excluding extra joints
};

}

#endif

