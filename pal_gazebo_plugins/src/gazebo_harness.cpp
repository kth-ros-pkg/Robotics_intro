/*
 *  gazebo_harness.cpp
 *  Copyright (c) 2014 PAL Robotics sl. All Rights Reserved
 *  Created on: 10/9/2014
 *      Author: luca
 */


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
#include <pal_gazebo_plugins/gazebo_harness.h>
#include <gazebo/math/gzmath.hh>
#include <sdf/sdf.hh>
#include <ros/ros.h>

namespace gazebo {

  GazeboHarness::GazeboHarness(){}

  // Destructor
  GazeboHarness::~GazeboHarness()
  {
//    event::Events::DisconnectWorldUpdateBegin(this->update_connection_);
    this->rosNode_->shutdown();
    this->rosQueue_.clear();
    this->rosQueue_.disable();
    this->callbackQueueThread_.join();
    delete this->rosNode_;
  }

  // Load the controller
  void GazeboHarness::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {

    this->robot_ptr_ = _parent;
    this->world_ = _parent->GetWorld();

    if (_sdf->HasElement("pin_link"))
    {
      this->pinLink_ = this->robot_ptr_->GetLink(_sdf->Get<std::string>("pin_link"));
      ROS_ERROR_STREAM("Got pin_link value: " << _sdf->Get<std::string>("pin_link"));
    }
    else
    {
      ROS_INFO("Can't find <robots><pin_link> blocks, using default.");
      this->pinLink_ = this->robot_ptr_->GetLink("base_link");
    }
    if (!this->pinLink_)
    {
      ROS_ERROR("robot pin link couldn't be initialized.");
      return;
    }

    // ros callback queue for processing subscription
    this->deferredLoadThread_ = boost::thread(
      boost::bind(&GazeboHarness::DeferredLoad, this));

  }

  ////////////////////////////////////////////////////////////////////////////////
  void GazeboHarness::DeferredLoad()
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

    this->modeSub_ = this->rosNode_->subscribe("robot/mode",1,&GazeboHarness::SetRobotModeTopic,this);

    // ros callback queue for processing subscription
    this->callbackQueueThread_ = boost::thread(
      boost::bind(&GazeboHarness::RosQueueThread, this));

//    // listen to the update event (broadcast every simulation iteration)
    this->update_connection_ =
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboHarness::UpdateChild, this));

  }


  // Update the controller
  void GazeboHarness::UpdateChild() {

//    boost::mutex::scoped_lock sclock(this->mutex_);



//      if(!robot_ptr_.get())
//      {
//        robot_ptr_ = this->world_->GetModel(this->robot_model_name_);
//        ROS_ERROR("Robot model not yet available, waiting ...");

//      }
//    }
  }

  void GazeboHarness::RosQueueThread()
  {
    ros::Rate rate(1.0);
    while (this->rosNode_->ok())
    {
      this->rosQueue_.callAvailable(/*ros::WallDuration(timeout)*/);
      rate.sleep();
    }
  }


  void GazeboHarness::SetRobotModeTopic(const std_msgs::String::ConstPtr &_str)
  {
    ROS_ERROR_STREAM("Got topic request for changing robot mode");
    this->SetRobotMode(_str->data);
  }

  void GazeboHarness::SetRobotMode(const std::string &_str)
  {
    if (_str == "harnessed")
    {
      this->world_->SetPaused(true);
      ROS_ERROR_STREAM("Creating joint for harness");

      // pinning robot, and turning on effect of gravity
      if (!this->pinJoint_)
      {
        math::Pose robot_pose = this->pinLink_->GetWorldPose();
        robot_pose.pos.z += 0.2;
        this->robot_ptr_->SetLinkWorldPose(robot_pose, this->pinLink_);
        this->pinJoint_ = this->AddJoint(this->world_, this->robot_ptr_,
                                         physics::LinkPtr(),    this->pinLink_,
                                         "revolute", math::Vector3(0.0, 0.0, 0.0), math::Vector3(0, 0, 1), 0.0, 0.0);
      }
      ROS_ERROR_STREAM("Setting gravity on");
      physics::Link_V links = this->robot_ptr_->GetLinks();
      for (unsigned int i = 0; i < links.size(); ++i)
      {
        links[i]->SetGravityMode(true);
      }


      this->world_->SetPaused(false);
      ROS_ERROR_STREAM("Robot set on harness");

    }
    else if (_str == "nominal")
    {
      // reinitialize pinning
      physics::Link_V links = this->robot_ptr_->GetLinks();
      for (unsigned int i = 0; i < links.size(); ++i)
      {
        links[i]->SetGravityMode(true);
      }
      if (this->pinJoint_)
        this->RemoveJoint(this->pinJoint_);

      ROS_ERROR_STREAM("Robot removed harness");
    }
    else
    {
      ROS_INFO("available modes:no_gravity, feet, pinned, nominal");
    }
  }


  physics::JointPtr GazeboHarness::AddJoint(physics::WorldPtr _world,
                                            physics::ModelPtr _model,
                                            physics::LinkPtr _link1,
                                            physics::LinkPtr _link2,
                                            std::string _type,
                                            math::Vector3 _anchor,
                                            math::Vector3 _axis,
                                            double _upper, double _lower)
  {
    physics::JointPtr joint = _world->GetPhysicsEngine()->CreateJoint(
          _type, _model);
    joint->Attach(_link1, _link2);
    // load adds the joint to a vector of shared pointers kept
    // in parent and child links, preventing joint from being destroyed.
    joint->Load(_link1, _link2, math::Pose(_anchor, math::Quaternion()));
    // joint->SetAnchor(0, _anchor);
    joint->SetAxis(0, _axis);
    joint->SetHighStop(0, _upper);
    joint->SetLowStop(0, _lower);
    if (_link1)
      joint->SetName(_link1->GetName() + std::string("_") +
                     _link2->GetName() + std::string("_joint"));
    else
      joint->SetName(std::string("world_") +
                     _link2->GetName() + std::string("_joint"));
    joint->Init();
    /*
   // disable collision between the link pair
   if (_link1)
   _link1->SetCollideMode("fixed");
   if (_link2)
   _link2->SetCollideMode("fixed");
   */
    return joint;
  }

  void GazeboHarness::RemoveJoint(physics::JointPtr &_joint)
  {
    bool paused = this->world_->IsPaused();
    this->world_->SetPaused(true);
    if (_joint)
    {
      // reenable collision between the link pair
      physics::LinkPtr parent = _joint->GetParent();
      physics::LinkPtr child = _joint->GetChild();
      if (parent)
        parent->SetCollideMode("all");
      if (child)
        child->SetCollideMode("all");
      _joint->Detach();
      _joint.reset();
    }
    this->world_->SetPaused(paused);
  }

  GZ_REGISTER_MODEL_PLUGIN(GazeboHarness)
}



