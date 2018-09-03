/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Open Source Robotics Foundation
 *  Copyright (c) 2013, The Johns Hopkins University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Open Source Robotics Foundation
 *     nor the names of its contributors may be
 *     used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman, Jonathan Bohren
   Desc:   Hardware Interface for any simulated robot in Gazebo
*/

#include <algorithm>
#include <cassert>
#include <set>
#include <stdexcept>

#include <boost/foreach.hpp>

#include <transmission_interface/transmission_interface_loader.h>
#include <gazebo_ros_control/default_robot_hw_sim.h>

namespace
{
/**
 * \return Map of all (resource name, hw interface) pairs claimed by the input
 * controllers.
 */
std::map<std::string, std::vector<std::string> > getResources(
    const std::list<hardware_interface::ControllerInfo>& ctrls)
{
  std::map<std::string, std::vector<std::string> > out;
  BOOST_FOREACH (const hardware_interface::ControllerInfo& ctrl, ctrls)
  {
    BOOST_FOREACH (const hardware_interface::InterfaceResources& interface_resource,
                   ctrl.claimed_resources)
    {
      BOOST_FOREACH (const std::string& name, interface_resource.resources)
      {
        out[name] = std::vector<std::string>(1, interface_resource.hardware_interface);
      }
    }
  }
  return out;
}

typedef boost::shared_ptr<gazebo_ros_control::internal::ReadWriteResource> RwResPtr;
std::list<RwResPtr>::iterator findResource(std::list<RwResPtr>& resource_list,
                                           const std::string& resource_name,
                                           const std::string& hardware_interface)
{
  for (std::list<RwResPtr>::iterator it = resource_list.begin(); it != resource_list.end(); ++it)
  {
    RwResPtr resource = *it;
    const std::vector<std::string>& ifaces = resource->getHardwareInterfaceTypes();
    if (resource->getName() == resource_name &&
        std::find(ifaces.begin(), ifaces.end(), hardware_interface) != ifaces.end())
    {
      return it;
    }
  }
  return resource_list.end();
}

template <class T>
std::string listElements(const T& container)
{
  std::string out;
  BOOST_FOREACH (typename T::const_reference val, container)
  {
    out += std::string(val) + ", ";
  }
  if (!container.empty())
  {
    out = out.substr(0, out.size() - 2);
  }
  return out;
}

}  // namespace

namespace gazebo_ros_control
{
hardware_interface::JointCommandModes convert(const std::string& type)
{
  hardware_interface::JointCommandModes res = hardware_interface::JointCommandModes::NOMODE;
  // RwResPtr& resource = it;
  if (type == "hardware_interface/PositionJointInterface")
  {
    res = hardware_interface::JointCommandModes::MODE_POSITION;
  }
  else if (type == "hardware_interface/VelocityJointInterface")
  {
    res = hardware_interface::JointCommandModes::MODE_VELOCITY;
  }
  else if (type == "hardware_interface/EffortJointInterface")
  {
    res = hardware_interface::JointCommandModes::MODE_EFFORT;
  }
  else if (type == "hardware_interface/JointStateInterface")
  {
    res = hardware_interface::JointCommandModes::NOMODE;
  }

  return res;
}

bool DefaultRobotHWSim::initSim(const std::string& robot_namespace, ros::NodeHandle model_nh,
                                gazebo::physics::ModelPtr parent_model,
                                const urdf::Model* const urdf_model,
                                std::vector<transmission_interface::TransmissionInfo> transmissions)
{
  // register hardware interfaces
  // TODO: Automate, so generic interfaces can be added
  registerInterface(&js_interface_);
  registerInterface(&ej_interface_);
  registerInterface(&pj_interface_);
  registerInterface(&vj_interface_);
  registerInterface(&jm_interface_);

  // cache transmisions information
  transmission_infos_ = transmissions;

  // populate hardware interfaces, bind them to raw Gazebo data
  namespace ti = transmission_interface;
  BOOST_FOREACH (const ti::TransmissionInfo& tr_info, transmission_infos_)
  {
    BOOST_FOREACH (const ti::JointInfo& joint_info, tr_info.joints_)
    {
      BOOST_FOREACH (const std::string& iface_type, joint_info.hardware_interfaces_)
      {
        // TODO: Wrap in method for brevity?
        RwResPtr res;
        // TODO: A plugin-based approach would do better than this 'if-elseif' chain
        // To do this, move contructor logic to init method, and unify signature
        if (iface_type == "hardware_interface/JointStateInterface")
        {
          res.reset(new internal::JointState());
        }
        else if (iface_type == "hardware_interface/PositionJointInterface")
        {
          res.reset(new internal::PositionJoint());
        }
        else if (iface_type == "hardware_interface/VelocityJointInterface")
        {
          res.reset(new internal::VelocityJoint());
        }
        else if (iface_type == "hardware_interface/EffortJointInterface")
        {
          res.reset(new internal::EffortJoint());
        }

        // initialize and add to list of managed resources
        if (res)
        {
          try
          {
            res->init(joint_info.name_, model_nh, parent_model, urdf_model, this);
            rw_resources_.push_back(res);
            ROS_DEBUG_STREAM("Registered joint '" << joint_info.name_
                                                  << "' in hardware interface '"
                                                  << iface_type << "'.");
          }
          catch (const internal::ExistingResourceException&)
          {
          }  // resource already added, no problem
          catch (const std::runtime_error& ex)
          {
            ROS_ERROR_STREAM("Failed to initialize gazebo_ros_control plugin.\n"
                             << ex.what());
            return false;
          }
          catch (...)
          {
            ROS_ERROR_STREAM("Failed to initialize gazebo_ros_control plugin.\n"
                             << "Could not add resource '" << joint_info.name_
                             << "' to hardware interface '" << iface_type << "'.");
            return false;
          }
        }
      }
    }
  }

  // initialize the emergency stop code
  e_stop_active_ = false;

  // joint mode switching
  mode_switch_enabled_ = true;
  model_nh.getParam("gazebo_ros_control/enable_joint_mode_switching",
                    mode_switch_enabled_);  // TODO: Check namespace
  const std::string enabled_str = mode_switch_enabled_ ? "enabled" : "disabled";
  ROS_INFO_STREAM("Joint mode switching is " << enabled_str);

  // initialize active writers
  initActiveWriteResources();

  // Register mode state interface

  hardware_interface::JointModeInterface* mode_iface =
      this->get<hardware_interface::JointModeInterface>();

  joint_mode_state_.resize(active_w_resources_rt_.size());
  unsigned int joint_mode_index = 0;
  for (auto it = active_w_resources_rt_.begin(); it != active_w_resources_rt_.end(); ++it)
  {
    joint_mode_state_[joint_mode_index] = convert((*it)->getHardwareInterfaceTypes()[0]);
    mode_iface->registerHandle(hardware_interface::JointModeHandle(
        (*it)->getName(), &joint_mode_state_[joint_mode_index]));

    joint_mode_map_[(*it)->getName()] = joint_mode_index;
    ++joint_mode_index;
  }

  return true;
}

void DefaultRobotHWSim::readSim(ros::Time time, ros::Duration period)
{
  // read all resources
  BOOST_FOREACH (RwResPtr res, rw_resources_)
  {
    res->read(time, period, e_stop_active_);
  }

  // Updata joint mode state
  BOOST_FOREACH (RwResPtr res, active_w_resources_rt_)
  {
    joint_mode_state_[joint_mode_map_[res->getName()]] = convert(res->getHardwareInterfaceTypes()[0]);
  }
}

void DefaultRobotHWSim::writeSim(ros::Time time, ros::Duration period)
{
  boost::unique_lock<boost::mutex> lock(mutex_);
  BOOST_FOREACH (RwResPtr res, active_w_resources_rt_)
  {
    res->write(time, period, e_stop_active_);
  }
}

void DefaultRobotHWSim::eStopActive(const bool active)
{
  e_stop_active_ = active;
}

bool DefaultRobotHWSim::prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                                      const std::list<hardware_interface::ControllerInfo>& stop_list)
{
  using std::list;
  using std::map;
  using std::string;
  using std::vector;
  using hardware_interface::ControllerInfo;
  using transmission_interface::TransmissionInfo;
  using transmission_interface::JointInfo;

  // no-op if mode switching is disabled
  if (!mode_switch_enabled_)
  {
    return true;
  }

  typedef map<string, vector<string> > ResourceToInterfaces;
  typedef ResourceToInterfaces::value_type ValueType;
  const ResourceToInterfaces start_resource_to_ifaces = getResources(start_list);
  const ResourceToInterfaces stop_resource_to_ifaces = getResources(stop_list);

  // preconditions
  BOOST_FOREACH (const ValueType& res_ifaces_, start_resource_to_ifaces)
  {
    const vector<string>& ifaces = res_ifaces_.second;
    if (ifaces.size() > 1)
    {
      ROS_ERROR_STREAM("gazebo_ros_control plugin does not support resources "
                       << "that write to multiple hardware interfaces.");
      return false;
    }
    assert(!ifaces.empty());
  }

  BOOST_FOREACH (const ValueType& res_ifaces_, stop_resource_to_ifaces)
  {
    const vector<string>& ifaces = res_ifaces_.second;
    if (ifaces.size() > 1)
    {
      ROS_ERROR_STREAM("gazebo_ros_control plugin does not support resources "
                       << "that write to multiple hardware interfaces.");
      return false;
    }
    assert(!ifaces.empty());
  }

  // seed active resources with what is currently being used in the control loop
  {
    boost::unique_lock<boost::mutex> lock(mutex_);
    active_w_resources_nrt_ = active_w_resources_rt_;
  }

  // resource removal
  BOOST_FOREACH (const ValueType& stop_res_ifaces, stop_resource_to_ifaces)
  {
    const string& stop_res_name = stop_res_ifaces.first;
    const string& stop_res_iface = stop_res_ifaces.second.front();  // size == 1

    // determine if resource to stop is also in the start list
    bool name_in_start_list = false;
    bool iface_in_start_list = false;
    BOOST_FOREACH (const ValueType& start_res_ifaces_, start_resource_to_ifaces)
    {
      const string& start_res_name = start_res_ifaces_.first;
      const string& start_res_iface = start_res_ifaces_.second.front();  // size == 1

      if (stop_res_name == start_res_name)
      {
        name_in_start_list = true;  // so far only name matches
        if (stop_res_iface == start_res_iface)
        {
          iface_in_start_list = true;  // both name and hardware interface match
          break;
        }
      }
    }

    // same resource is being stopped, then started. No need to remove or add
    if (name_in_start_list && iface_in_start_list)
    {
      continue;
    }

    // actual resource removal
    list<RwResPtr>::iterator rem_it =
        findResource(active_w_resources_nrt_, stop_res_name, stop_res_iface);
    if (rem_it != active_w_resources_nrt_.end())
    {
      ROS_DEBUG_STREAM("Removing resource '"
                       << (*rem_it)->getName() << "' with hardware interfaces '"
                       << listElements((*rem_it)->getHardwareInterfaceTypes()) << "'.");
      rem_it = active_w_resources_nrt_.erase(rem_it);
    }
    else
    {
      ROS_ERROR_STREAM("Unexpected error. Could not find resource '"
                       << stop_res_name << "' with hardware interface '" << stop_res_iface
                       << "' in list of available resources");
      return false;
    }
    // add a default resource (if any is registered for this resource name) if
    // the removed resource is not claimed by the controllers to start. This prevents
    // undesired things like the robot falling down due to absence of a control action.
    if (!name_in_start_list)
    {
      map<string, RwResPtr>::const_iterator default_res_it =
          default_active_resources_.find(stop_res_name);
      if (default_res_it != default_active_resources_.end())
      {
        RwResPtr default_res = default_res_it->second;
        active_w_resources_nrt_.insert(rem_it,
                                       default_res);  // in same place of removed resource
        ROS_DEBUG_STREAM("Adding default resource '"
                         << default_res->getName() << "' with hardware interfaces '"
                         << listElements(default_res->getHardwareInterfaceTypes())
                         << "'.");
      }
    }
  }

  // resource addition
  BOOST_FOREACH (const ValueType& start_res_ifaces_, start_resource_to_ifaces)
  {
    const string& start_res_name = start_res_ifaces_.first;
    const string& start_res_iface = start_res_ifaces_.second.front();  // size == 1

    // determine if resource to stop is in the list of active write resources
    bool name_in_active_w = false;
    bool iface_in_active_w = false;
    BOOST_FOREACH (RwResPtr active_res, active_w_resources_nrt_)
    {
      string active_res_name = active_res->getName();
      vector<string> active_res_ifaces = active_res->getHardwareInterfaceTypes();

      if (start_res_name == active_res_name)
      {
        name_in_active_w = true;  // so far only name matches
        if (std::find(active_res_ifaces.begin(), active_res_ifaces.end(), start_res_iface) !=
            active_res_ifaces.end())
        {
          iface_in_active_w = true;  // both name and hardware interface match
          break;
        }
      }
    }

    // do nothing if resource (name and interface) is already active
    if (name_in_active_w && iface_in_active_w)
    {
      continue;
    }

    // there is a resource active using a different interface, remove it, as
    // there can currently be only one. This might be due to, for instance, a
    // default resource added to enforce a control action when no controllers
    // are claiming it
    if (name_in_active_w)
    {
      bool remove_ok = false;
      for (list<RwResPtr>::iterator it = active_w_resources_nrt_.begin();
           it != active_w_resources_nrt_.end(); ++it)
      {
        const string active_res_name = (*it)->getName();
        if (active_res_name == start_res_name)
        {
          ROS_DEBUG_STREAM("Removing (possibly default) resource '"
                           << active_res_name << "' with hardware interfaces '"
                           << listElements((*it)->getHardwareInterfaceTypes()) << "'.");
          active_w_resources_nrt_.erase(it);
          remove_ok = true;
          break;
        }
      }
      if (!remove_ok)
      {
        ROS_ERROR_STREAM("Unexpected error. Could not find resource '"
                         << start_res_name << "' in list of active resources.");
        return false;
      }
    }

    // add resource
    bool add_ok = false;
    BOOST_FOREACH (RwResPtr res, rw_resources_)
    {
      // find resource in list of available resources
      const vector<string> res_ifaces = res->getHardwareInterfaceTypes();
      if (start_res_name == res->getName() &&
          std::find(res_ifaces.begin(), res_ifaces.end(), start_res_iface) != res_ifaces.end())
      {
        ROS_DEBUG_STREAM("Adding resource '"
                         << res->getName() << "' with hardware interfaces '"
                         << listElements(res->getHardwareInterfaceTypes()) << "'.");
        active_w_resources_nrt_.push_back(res);
        add_ok = true;
        break;
      }
    }
    if (!add_ok)
    {
      ROS_ERROR_STREAM("Unexpected error. Could not find resource '"
                       << start_res_name << "' with hardware interface '"
                       << start_res_iface << "' in list of available resources");
      return false;
    }
  }

  return true;
}

void DefaultRobotHWSim::doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                                 const std::list<hardware_interface::ControllerInfo>& stop_list)
{
  if (!mode_switch_enabled_)
  {
    return;
  }

  boost::unique_lock<boost::mutex> lock(mutex_);
  active_w_resources_rt_.swap(active_w_resources_nrt_);
}

void DefaultRobotHWSim::initActiveWriteResources()
{
  // TODO: Make plugin-based?
  using std::find;
  using std::map;
  using std::string;
  using std::vector;
  namespace hi = hardware_interface;
  namespace hii = hi::internal;

  // if mode switching is disabled, all available resources are considered active
  // this list is static and will not change over time
  if (!mode_switch_enabled_)
  {
    active_w_resources_rt_ = rw_resources_;
    return;
  }

  // list all resource data associated to each unique resource name
  typedef map<string, vector<RwResPtr> > IfaceToResources;  // hardware interface -> list
                                                            // of resources exposing it

  map<string, IfaceToResources> res_map;  // resource name -> IfaceToResources
  BOOST_FOREACH (RwResPtr res, rw_resources_)
  {
    const string name = res->getName();
    vector<string> ifaces = res->getHardwareInterfaceTypes();

    IfaceToResources& iface_to_res_map = res_map[name];
    BOOST_FOREACH (const string& iface, ifaces)
    {
      vector<RwResPtr>& resources = iface_to_res_map[iface];
      resources.push_back(res);
    }
  }

  // select which resources to activate by default
  {
    typedef map<string, IfaceToResources>::value_type ValueType;
    BOOST_FOREACH (const ValueType& iface_to_res_map, res_map)
    {
      IfaceToResources::const_iterator it;
      const IfaceToResources& iface_to_res = iface_to_res_map.second;

      // try to find a position control interface
      // TODO: Move to function, duplicated below
      it = iface_to_res.find(hii::demangledTypeName<hi::PositionJointInterface>());
      if (it != iface_to_res.end())
      {
        const vector<RwResPtr>& resources = it->second;
        if (!resources.empty())
        {
          RwResPtr resource = resources.front();
          default_active_resources_[resource->getName()] = resource;
          ROS_DEBUG_STREAM("Adding '" << resource->getName() << "' using the '"
                                      << hii::demangledTypeName<hi::PositionJointInterface>()
                                      << "' hardware interface to the set of resources "
                                         "active by default.");
          continue;
        }
      }

      // try to find a velocity control interface
      it = iface_to_res.find(hii::demangledTypeName<hi::VelocityJointInterface>());
      if (it != iface_to_res.end())
      {
        const vector<RwResPtr>& resources = it->second;
        if (!resources.empty())
        {
          RwResPtr resource = resources.front();
          default_active_resources_[resource->getName()] = resource;
          ROS_DEBUG_STREAM("Adding '" << resource->getName() << "' using the '"
                                      << hii::demangledTypeName<hi::VelocityJointInterface>()
                                      << "' hardware interface to the set of resources "
                                         "active by default.");
          continue;
        }
      }
    }
  }

  // initialize list of active resources
  {
    typedef map<string, RwResPtr>::value_type ValueType;
    BOOST_FOREACH (ValueType val, default_active_resources_)
    {
      active_w_resources_rt_.push_back(val.second);
    }
  }
}

}  // namespace

PLUGINLIB_EXPORT_CLASS(gazebo_ros_control::DefaultRobotHWSim, gazebo_ros_control::RobotHWSim)
