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

/** \author Paul Mathieu. */

#ifndef TF_LOOKUP_H
#define TF_LOOKUP_H

#include <ros/ros.h>
#include <ros/time.h>
#include <actionlib/server/action_server.h>

#include "tf_lookup/TfLookupAction.h"
#include "tf_lookup/tf_stream_server.h"

namespace tf_lookup
{
  ROS_DECLARE_MESSAGE(lookupTransformRequest);
  ROS_DECLARE_MESSAGE(lookupTransformResponse);
}

namespace geometry_msgs
{ ROS_DECLARE_MESSAGE(TransformStamped); }

namespace tf
{
  class TransformListener;
  class StampedTransform;
}

namespace tf_lookup
{
  class TfLookup
  {
    private:
      typedef actionlib::ActionServer<tf_lookup::TfLookupAction> AlServer;

    public:
      TfLookup();
      virtual ~TfLookup();
      /**
       * @brief Advertises the services the plugin provides so they can be
       *        called from any ros node.
       */
      virtual void start(ros::NodeHandle &n);

    private:
      void periodicCheck(const ros::TimerEvent& te);
      bool srvLookupTransform(tf_lookup::lookupTransformRequest &req,
          tf_lookup::lookupTransformResponse &res);
      bool _lookupTransform(const std::string& target,
          const std::string& source, const ros::Time& time,
          tf::StampedTransform& trans);
      bool lookupTransform(const std::string& target,
          const std::string& source, const ros::Time& time,
          geometry_msgs::TransformStamped& trans);
      void alGoalCb(AlServer::GoalHandle gh);

      ros::ServiceServer                     _srvLookupTransform;
      boost::shared_ptr<tf::TransformListener> _tfListener;
      ros::Time                              _lastTime;
      ros::Timer                             _check_timer;
      TfStreamServer                         _tf_streamer;
      boost::shared_ptr<AlServer>              _al_server;
  };
}

#endif // TFLOOKUP_H
