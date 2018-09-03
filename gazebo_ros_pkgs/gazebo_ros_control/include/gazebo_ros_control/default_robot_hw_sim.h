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

/* Author: Dave Coleman, Jonathan Bohren, Adolfo Rodriguez Tsouroukdissian
   Desc:   Hardware Interface for any simulated robot in Gazebo
*/

#ifndef _GAZEBO_ROS_CONTROL___DEFAULT_ROBOT_HW_SIM_H_
#define _GAZEBO_ROS_CONTROL___DEFAULT_ROBOT_HW_SIM_H_

#include <list>
#include <map>
#include <vector>

#include <boost/thread/mutex.hpp>

#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>

#include <ros/ros.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

#include <gazebo_ros_control/robot_hw_sim.h>

// TODO: Don't include explicitly, but let plugins figure it out?
#include <gazebo_ros_control/internal/joint_state.h>
#include <gazebo_ros_control/internal/position_joint.h>
#include <gazebo_ros_control/internal/velocity_joint.h>
#include <gazebo_ros_control/internal/effort_joint.h>

#include <hardware_interface/joint_mode_interface.h>

// URDF
#include <urdf/model.h>

namespace gazebo_ros_control
{
// TODO: Doc that we currently don't support writing to multiple interfaces at the same
// time
class DefaultRobotHWSim : public gazebo_ros_control::RobotHWSim
{
public:
  virtual bool initSim(const std::string& robot_namespace, ros::NodeHandle model_nh,
                       gazebo::physics::ModelPtr parent_model, const urdf::Model* const urdf_model,
                       std::vector<transmission_interface::TransmissionInfo> transmissions);

  virtual void readSim(ros::Time time, ros::Duration period);
  virtual void writeSim(ros::Time time, ros::Duration period);

  virtual void eStopActive(const bool active);

  virtual bool prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                             const std::list<hardware_interface::ControllerInfo>& stop_list) override;
  virtual void doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                        const std::list<hardware_interface::ControllerInfo>& stop_list) override;

protected:
  hardware_interface::JointStateInterface js_interface_;
  hardware_interface::EffortJointInterface ej_interface_;
  hardware_interface::PositionJointInterface pj_interface_;
  hardware_interface::VelocityJointInterface vj_interface_;
  hardware_interface::JointModeInterface jm_interface_;

  std::vector<int> joint_mode_state_;
  std::map<std::string, int> joint_mode_map_;

  typedef boost::shared_ptr<internal::ReadWriteResource> RwResPtr;
  std::list<RwResPtr> rw_resources_;  ///< available read and/or write resources resources
  std::list<RwResPtr> active_w_resources_rt_;  ///< subset of available resources
                                               /// currently writing control commands
  mutable std::list<RwResPtr> active_w_resources_nrt_;  // TODO: Remove mutable in jade

  std::map<std::string, RwResPtr> default_active_resources_;  ///< Resources to activate
                                                              /// when no controllers are
  /// running (if any)
  // TODO: Add parameter to disable default active resources

  bool e_stop_active_;        ///< Evaluates to true when in e-stop
  bool mode_switch_enabled_;  ///< Evaluates to true when joint mode switching is enabled
                              ///// TODO: Remove!

  typedef std::vector<transmission_interface::TransmissionInfo> TransmissionInfoList;
  TransmissionInfoList transmission_infos_;  ///< Used to reason about mode switching

  mutable boost::mutex mutex_;  // TODO: Remove mutable in jade

  /**
   * \brief initialize the list of resources writing control commands.
   *
   * The subet of resources that write control commands is updated every time
   * new controllers are started or stopped, but on robot initialization no
   * controllers are running. This method determines which resources should be
   * active by default on robot innitialization, so defined behavior like
   * position holding is achieved, instead of the robot falling down.
   *
   * The exact behavior for each resource is determined by the respective
   * \ref ReadWriteResource specialization. When multiple control modes are
   * available for a given resource, this method tries to make an educated
   * guess, which by default amounts to activate a position-controlled interface
   * if available, else a velocity-controlled interface.
   *
   * If a different behavior is desired, the method can be specialized.
   *
   */
  virtual void initActiveWriteResources();
};
}

#endif  // #ifndef __GAZEBO_ROS_CONTROL_PLUGIN_DEFAULT_ROBOT_HW_SIM_H_
