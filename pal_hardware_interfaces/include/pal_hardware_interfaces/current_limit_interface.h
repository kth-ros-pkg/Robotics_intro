///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2014, PAL Robotics S.L.
//
//////////////////////////////////////////////////////////////////////////////

#ifndef PAL_ROS_CONTROL_CURRENT_LIMIT_INTERFACE_H
#define PAL_ROS_CONTROL_CURRENT_LIMIT_INTERFACE_H

#include <string>
#include <hardware_interface/internal/hardware_resource_manager.h>

namespace pal_ros_control
{

/** \brief A handle used to set the current limit for an actuator. */
class CurrentLimitHandle
{
public:
  CurrentLimitHandle() : name_(), curr_lim_(0) {}

  /**
   * \param name The name of the actuator
   * \param curr_lim A pointer to the storage for this actuator's current limit value
   */
  CurrentLimitHandle(const std::string& name, double* curr_lim)
    : name_(name), curr_lim_(curr_lim)
  {
    if (!curr_lim_)
    {
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name +
                                                           "'. Current limit data pointer is null.");
    }
  }

  std::string getName() const                  {return name_;}
  double      getCurrentLimit() const          {assert(curr_lim_); return *curr_lim_;}
  void        setCurrentLimit(double curr_lim) {assert(curr_lim_); *curr_lim_ = curr_lim;}

private:
  std::string name_;
  double* curr_lim_;
};

/** \brief Hardware interface to support setting the current limit for an array of actuators. */
class CurrentLimitInterface : public
  hardware_interface::HardwareResourceManager<CurrentLimitHandle, hardware_interface::ClaimResources> {};
}

#endif // Header guard
