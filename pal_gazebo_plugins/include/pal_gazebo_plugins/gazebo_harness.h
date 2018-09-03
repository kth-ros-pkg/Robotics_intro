/*
 *  gazebo_harness.h
 *  Copyright (c) 2014 PAL Robotics sl. All Rights Reserved
 *  Created on: 10/9/2014
 *      Author: luca
 */

#ifndef GAZEBO_HARNESS_H
#define GAZEBO_HARNESS_H

///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2014, PAL Robotics S.L.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
// * Neither the name of PAL Robotics S.L. nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
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

/*
 *  gazebo_wifi_ap.cpp
 *  Copyright (c) 2014 PAL Robotics sl. All Rights Reserved
 *  Created on: 5/19/2014
 *      Author: luca
 * \brief A plugin for gazebo for simulating wifi signal emitted by wifi access points
 * \author  Luca Marchionni (luca.marchionni@pal-robotics.com)
 */

#ifndef GAZEBO_WIFI_AP_CPP
#define GAZEBO_WIFI_AP_CPP

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/String.h>

namespace gazebo {

  class Entity;

  class GazeboHarness : public ModelPlugin {

  public:
    GazeboHarness();
    ~GazeboHarness();
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

  protected:
    virtual void UpdateChild();

    /// \brief ROS callback queue thread
    private: void RosQueueThread();

    /// \brief: thread out Load function with
    /// with anything that might be blocking.
    private: void DeferredLoad();

    /// \brief sets robot mode via ros topic
    /// \sa SetRobotMode(const std::string &_str)
    void SetRobotModeTopic(const std_msgs::String::ConstPtr &_str);

    /// \brief sets robot mode
    /// \param[in] _str sets robot mode by a string. Supported modes are:
    /// - "no_gravity" Gravity disabled for the robot.
    /// - "nominal" Nominal "normal" physics.
    /// - "pinned" Robot is pinned to inertial world by the pelvis.
    /// - "feet" same as no_gravity except for r_foot and l_foot links.
    void SetRobotMode(const std::string &_str);

    /// \brief add a constraint between 2 links
    /// \param[in] _world a pointer to the current World
    /// \param[in] _model a pointer to the Model the new Joint will be under
    /// \param[in] _link1 parent link in the new Joint
    /// \param[in] _link2 child link in the new Joint
    /// \param[in] _type string specifying joint type
    /// \param[in] _anchor a Vector3 anchor offset of the new joint
    /// \param[in] _axis Vector3 containing xyz axis of the new joint
    /// \param[in] _upper upper linit of the new joint
    /// \param[in] _lower lower linit of the new joint
    /// \return Joint created between _link1 and _link2 under _model.
    private: physics::JointPtr AddJoint(physics::WorldPtr _world,
                                      physics::ModelPtr _model,
                                      physics::LinkPtr _link1,
                                      physics::LinkPtr _link2,
                                      std::string _type,
                                      math::Vector3 _anchor,
                                      math::Vector3 _axis,
                                      double _upper, double _lower);
    /// \brief Remove a joint.
    /// \param[in] _joint Joint to remove.
    private: void RemoveJoint(physics::JointPtr &_joint);

  private:

    physics::WorldPtr world_;
    physics::ModelPtr robot_ptr_;
    physics::LinkPtr pinLink_;
    physics::JointPtr pinJoint_;

    event::ConnectionPtr update_connection_;
    boost::thread deferredLoadThread_;

    // ROS stuff
    ros::NodeHandle* rosNode_;
    ros::CallbackQueue rosQueue_;
    boost::thread callbackQueueThread_;
    ros::Subscriber modeSub_;

  };

}

#endif // GAZEBO_WIFI_AP_CPP

#endif // GAZEBO_HARNESS_H
