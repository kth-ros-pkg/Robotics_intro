/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
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
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
 *     contributors may be used to endorse or promote products derived
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
 */

/** \author Jose Capriles. */

#include "pal_gazebo_plugins/gazebo_ros_forcetorque.h"

#include <string>

using std::string;

namespace gazebo
{
GZ_REGISTER_SENSOR_PLUGIN(FTPlugin)

////////////////////////////////////////////////////////////////////////////////
FTPlugin::FTPlugin() : SensorPlugin()
{
  this->footForce = 0;
  this->footTorque = 0;
}

////////////////////////////////////////////////////////////////////////////////
FTPlugin::~FTPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
  this->rosNode->shutdown();
  this->rosQueue.clear();
  this->rosQueue.disable();
  this->callbackQueeuThread.join();
  delete this->rosNode;
}

////////////////////////////////////////////////////////////////////////////////
void FTPlugin::Load(sensors::SensorPtr _sensor,
                                 sdf::ElementPtr _sdf)
{

  this->robot_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
    this->robot_namespace_ =
      _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  // "publishing contact/collisions to this topic name: "
  //   << this->bumper_topic_name_ << std::endl;
  this->topic_name_ = "ft_data";
  if (_sdf->GetElement("topicName"))
    this->topic_name_ =
      _sdf->GetElement("topicName")->Get<std::string>();

  // "transform contact/collisions pose, forces to this body (link) name: "
  //   << this->frame_name_ << std::endl;
  if (!_sdf->HasElement("frameName"))
  {
    ROS_INFO("ft sensor plugin missing <frameName>, defaults to world");
    this->frame_name_ = "world";
  }
  else
    this->frame_name_ = _sdf->GetElement("frameName")->Get<std::string>();

  this->update_rate_ = 100.0;
  if (!_sdf->HasElement("updateRate"))
  {
    ROS_INFO("ft sensor plugin missing <updateRate>, defaults to %f", this->update_rate_);
  }
  else
    this->update_rate_ = _sdf->GetElement("updateRate")->Get<double>();

//  this->model = _sensor;

//  // Get the world name.
//  this->world = this->model->GetWorld();
//  this->sdf = _sdf;
//  this->lastControllerUpdateTime = this->world->GetSimTime();

//  this->rAnkleJoint = this->model->GetJoint("r_leg_lax");
//  if (!this->rAnkleJoint)
//    gzerr << "right ankle joint (r_leg_lax) not found\n";

//  this->lAnkleJoint = this->model->GetJoint("l_leg_lax");
//  if (!this->lAnkleJoint)
//    gzerr << "left ankle joint (l_leg_lax) not found\n";

  // Get sensor
  this->footContactSensor = std::dynamic_pointer_cast<sensors::ContactSensor> (_sensor);
  if (!this->footContactSensor)
    gzerr << "contact_sensor not found\n" << "\n";

  // ros callback queue for processing subscription
  this->deferredLoadThread = boost::thread(
    boost::bind(&FTPlugin::DeferredLoad, this));


}


////////////////////////////////////////////////////////////////////////////////
void FTPlugin::DeferredLoad()
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
  this->rosNode = new ros::NodeHandle("");

  this->pubFootContact =
    this->rosNode->advertise<geometry_msgs::Wrench>("/" + this->robot_namespace_+ "/" + this->topic_name_, 10);

  // ros callback queue for processing subscription
  this->callbackQueeuThread = boost::thread(
    boost::bind(&FTPlugin::RosQueueThread, this));

  // Connect to the sensor update event.
  this->updateConnection = this->footContactSensor->ConnectUpdated(
              boost::bind(&FTPlugin::UpdateChild, this));

  // Make sure the parent sensor is active.
  this->footContactSensor->SetActive(true);

  this->lastUpdateTime = ros::Time::now();
}

void FTPlugin::UpdateChild()
{
    if (this->pubFootContact.getNumSubscribers() <= 0)
      return;

    boost::mutex::scoped_lock sclock(this->mutex_);

    static const ros::Duration update_period(1.0/this->update_rate_);

    ros::Time now = ros::Time::now();
    if ( ( now - this->lastUpdateTime) >= update_period)
    {

        double rate = footContactSensor->GetUpdateRate();
        //ROS_DEBUG("sensor update rate %f, dT elapsed %f [sec]", rate, ( now - this->lastUpdateTime).toSec() ); // TODO: Remove?

        this->lastUpdateTime = now;

        // Get all the contacts.
        msgs::Contacts contacts;
        contacts = this->footContactSensor->GetContacts();

        for (int i = 0; i < contacts.contact_size(); ++i)
        {
            math::Vector3 fTotal;
            math::Vector3 tTotal;
            for (int j = 0; j < contacts.contact(i).position_size(); ++j)
            {
                fTotal += math::Vector3(
                            contacts.contact(i).wrench(j).body_1_wrench().force().x(),
                            contacts.contact(i).wrench(j).body_1_wrench().force().y(),
                            contacts.contact(i).wrench(j).body_1_wrench().force().z());
                tTotal += math::Vector3(
                            contacts.contact(i).wrench(j).body_1_wrench().torque().x(),
                            contacts.contact(i).wrench(j).body_1_wrench().torque().y(),
                            contacts.contact(i).wrench(j).body_1_wrench().torque().z());
            }
            // low pass filter over time
            double e = 0.99;
            this->footForce = this->footForce * e + fTotal * (1.0 - e);
            this->footTorque = this->footTorque * e + tTotal * (1.0 - e);

            geometry_msgs::Wrench msg;
            msg.force.x = this->footForce.x;
            msg.force.y = this->footForce.y;
            msg.force.z = this->footForce.z;
            msg.torque.x = this->footTorque.x;
            msg.torque.y = this->footTorque.y;
            msg.torque.z = this->footTorque.z;
            this->pubFootContact.publish(msg);
        }
    }
    else
    {
        // ROS_DEBUG("Filtering update period"); // TODO: Remove?
    }
}


void FTPlugin::RosQueueThread()
{
//  static const double timeout = 0.01;
  ros::Rate rate(this->update_rate_);

  while (this->rosNode->ok())
  {
    this->rosQueue.callAvailable(/*ros::WallDuration(timeout)*/);
    rate.sleep();
  }
}
}

