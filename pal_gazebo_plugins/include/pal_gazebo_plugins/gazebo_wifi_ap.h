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

namespace gazebo {

  class Entity;

  class GazeboWifiAP : public ModelPlugin {

  public:
    GazeboWifiAP();
    ~GazeboWifiAP();
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

  protected:
    virtual void UpdateChild();

  /// \brief ROS callback queue thread
  private: void RosQueueThread();

  /// \brief: thread out Load function with
  /// with anything that might be blocking.
  private: void DeferredLoad();

  private:

    physics::WorldPtr world_;
    physics::ModelPtr parent_;
    event::ConnectionPtr update_connection_;

    std::string robot_model_name_;
    std::string topic_;
    std::string essid_;
    double max_value_;
    double min_value_;
    double max_range_;
    double sigma_;

    gazebo::physics::ModelPtr robot_ptr_;

    double      update_rate_;
    boost::thread deferredLoadThread_;

    // ROS stuff
    ros::NodeHandle* rosNode_;
    ros::CallbackQueue rosQueue_;
    boost::thread callbackQueeuThread_;
    ros::Publisher pubWifiMsg_;

    // Controls stuff
    ros::Time lastUpdateTime_;

    // Mutex
    boost::mutex mutex_;
  };

}

#endif // GAZEBO_WIFI_AP_CPP
