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

#include <assert.h>
#include <pal_gazebo_plugins/gazebo_wifi_ap.h>
#include <gazebo/math/gzmath.hh>
#include <sdf/sdf.hh>
#include <ros/ros.h>
#include <pal_multirobot_msgs/WifiServiceDetection.h>

namespace gazebo {

  GazeboWifiAP::GazeboWifiAP() {}

  // Destructor
  GazeboWifiAP::~GazeboWifiAP()
  {
    event::Events::DisconnectWorldUpdateBegin(this->update_connection_);
    this->rosNode_->shutdown();
    this->rosQueue_.clear();
    this->rosQueue_.disable();
    this->callbackQueeuThread_.join();
    delete this->rosNode_;
  }

  // Load the controller
  void GazeboWifiAP::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {

    this->parent_ = _parent;
    this->world_ = _parent->GetWorld();

    this->update_rate_ = 1.0;
    if (!_sdf->HasElement("update_rate"))
    {
      ROS_INFO("ft sensor plugin missing <update_rate>, defaults to %f", this->update_rate_);
    }
    else
      this->update_rate_ = _sdf->GetElement("update_rate")->Get<double>();

    this->robot_model_name_ = "reemc";
    if (!_sdf->HasElement("robot_model")) {
      ROS_INFO("GazeboWifiAP Plugin missing <robot_model>, defaults to \"%s\"",
          this->robot_model_name_.c_str());
    } else {
      this->robot_model_name_ =
        _sdf->GetElement("robot_model")->Get<std::string>();
    }

    this->topic_ = "wifi_msg";
    if (!_sdf->HasElement("topic")) {
      ROS_INFO("GazeboWifiAP Plugin missing <topic>, defaults to \"%s\"",
          this->topic_.c_str());
    } else {
      this->topic_ =
        _sdf->GetElement("topic")->Get<std::string>();
    }

    static int counter = 0;
    this->essid_ = "wifi_" + counter;
    counter++;
    if (!_sdf->HasElement("essid")) {
      ROS_INFO("GazeboWifiAP Plugin missing <essid>, defaults to \"%s\"",
          this->essid_.c_str());
    } else {
      this->essid_ =
        _sdf->GetElement("essid")->Get<std::string>();
    }

    this->max_value_ = -20.0;
    if (!_sdf->HasElement("max_value")) {
      ROS_WARN("GazeboWifiAP Plugin missing <max_value>, defaults to \"%f\"", this->max_value_);
    } else {
      this->max_value_ = _sdf->GetElement("max_value")->Get<double>();
    }

    this->min_value_ = -100.0;
    if (!_sdf->HasElement("min_value")) {
      ROS_WARN("GazeboWifiAP Plugin missing <min_value>, defaults to \"%f\"", this->min_value_);
    } else {
      this->min_value_ = _sdf->GetElement("min_value")->Get<double>();
    }

    this->max_range_ = 100.0;
    if (!_sdf->HasElement("max_range")) {
      ROS_WARN("GazeboWifiAP Plugin missing <max_range>, defaults to \"%f\"", this->max_range_);
    } else {
      this->max_range_ = _sdf->GetElement("max_range")->Get<double>();
    }


    this->sigma_ = 1.0;
    if (!_sdf->HasElement("sigma")) {
      ROS_WARN("GazeboWifiAP Plugin missing <sigma>, defaults to \"%f\"", this->sigma_);
    } else {
      this->sigma_ = _sdf->GetElement("sigma")->Get<double>();
    }

    // ros callback queue for processing subscription
    this->deferredLoadThread_ = boost::thread(
      boost::bind(&GazeboWifiAP::DeferredLoad, this));

  }

  ////////////////////////////////////////////////////////////////////////////////
  void GazeboWifiAP::DeferredLoad()
  {
    // initialize ros
    if (!ros::isInitialized())
    {
      gzerr << "Not loading plugin since ROS hasn't been "
            << "properly initialized.  Try starting gazebo with ros plugin:\n"
            << "  gazebo -s libgazebo_ros_api_plugin.so\n";
      return;
    }

    // ros stuff
    this->rosNode_ = new ros::NodeHandle("");

    this->pubWifiMsg_ =
      this->rosNode_->advertise<pal_multirobot_msgs::WifiServiceDetection>(topic_, 10);

    // ros callback queue for processing subscription
    this->callbackQueeuThread_ = boost::thread(
      boost::bind(&GazeboWifiAP::RosQueueThread, this));

    // listen to the update event (broadcast every simulation iteration)
    this->update_connection_ =
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboWifiAP::UpdateChild, this));

    this->lastUpdateTime_ = ros::Time::now();
  }


  // Update the controller
  void GazeboWifiAP::UpdateChild() {

    if (this->pubWifiMsg_.getNumSubscribers() <= 0)
    {
      return;
    }

    boost::mutex::scoped_lock sclock(this->mutex_);

    static const ros::Duration update_period(1.0/this->update_rate_);

    ros::Time now = ros::Time::now();
    if ( ( now - this->lastUpdateTime_) >= update_period)
    {

        this->lastUpdateTime_ = now;

      if(robot_ptr_.get())
      {

        /// Get robot position
        ///
        ///
        /// Publish wifi msg

        math::Pose diff_pose  = (robot_ptr_->GetWorldPose() - parent_->GetWorldPose());
        double ray_dist = sqrt(diff_pose.pos.GetSquaredLength());
        ROS_ERROR_STREAM("ray dist  " << ray_dist );


        double signal_strength =  (max_value_ - min_value_)*(1./(sigma_*sqrt(2.0*M_PI)))*exp(-0.5*pow(-ray_dist/sigma_,2)) + min_value_;

        pal_multirobot_msgs::WifiServiceDetection msg;
        msg.header.stamp = ros::Time::now();
        msg.id = essid_;
        msg.signal = signal_strength;
        pubWifiMsg_.publish(msg);
      }
      else
      {
        robot_ptr_ = this->world_->GetModel(this->robot_model_name_);
        ROS_ERROR("Robot model not yet available, waiting ...");

      }
    }
  }

  void GazeboWifiAP::RosQueueThread()
  {
  //  static const double timeout = 0.01;
    ros::Rate rate(this->update_rate_);

    while (this->rosNode_->ok())
    {
      this->rosQueue_.callAvailable(/*ros::WallDuration(timeout)*/);
      rate.sleep();
    }
  }

  GZ_REGISTER_MODEL_PLUGIN(GazeboWifiAP)
}



