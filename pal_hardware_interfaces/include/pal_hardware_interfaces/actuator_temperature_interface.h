#ifndef _ACTUATOR_TEMPERATURE_SENSOR_ROS_CONTROL_HARDWARE_INTERFACE_
#define _ACTUATOR_TEMPERATURE_SENSOR_ROS_CONTROL_HARDWARE_INTERFACE_

#include <cassert>
#include <string>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/actuator_command_interface.h>

namespace hardware_interface
{

  class ActuatorTemperatureSensorHandle
  {
  public:
    ActuatorTemperatureSensorHandle(){}


    ActuatorTemperatureSensorHandle(const std::string& name, double* value)
      : name_(name), value_(value)
    {
      if (!value)
      {
        throw HardwareInterfaceException("Cannot create handle '" + name + "'. Temperature sensor value data pointer is null.");
      }

    }

    std::string getName() const {return name_;}

    double getValue() const {assert(value_); return *value_;}

    double* getValuePtr() const {assert(value_); return value_;}

  private:
    std::string name_;
    double* value_;
  };

  class ActuatorTemperatureSensorInterface : public HardwareResourceManager<ActuatorTemperatureSensorHandle, DontClaimResources> {};

}


#endif
