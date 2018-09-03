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

#ifndef TFSTREAMCLIENT_H
#define TFSTREAMCLIENT_H

#include <map>
#include <string>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <ros/time.h>

namespace geometry_msgs
{ ROS_DECLARE_MESSAGE(TransformStamped); }
namespace tf
{ ROS_DECLARE_MESSAGE(tfMessage); }
namespace actionlib
{
  template <class T>
  class SimpleActionClient;
  class SimpleClientGoalState;
}

namespace tf_lookup
{
  ROS_DECLARE_MESSAGE(TfStreamAction);
  ROS_DECLARE_MESSAGE(TfStreamGoal);
  ROS_DECLARE_MESSAGE(TfStreamResult);

  class TfSCTransform;

  class TfStreamClient
  {
    friend class TfSCTransform;

    public:
      typedef boost::shared_ptr<TfSCTransform> Handle;

    private:
      typedef boost::function
        <void(const geometry_msgs::TransformStampedConstPtr&)> Callback;
      typedef std::list<TfSCTransform*>::iterator              ListIter;
      typedef actionlib::SimpleActionClient
        <tf_lookup::TfStreamAction>                        AlClient;
      typedef tf_lookup::TfStreamGoal                      AlGoal;
      typedef tf_lookup::TfStreamResultConstPtr            AlResultConstPtr;
      typedef tf::tfMessageConstPtr                            FeedConstPtr;

    public:
      TfStreamClient(ros::NodeHandle& nh);

      virtual ~TfStreamClient();

      /**
       * Adds a subscription to a transform. The handle returned is used to
       * manage the subscription. When the last handle dies, the callback is
       * deleted and the subscription is canceled
       */
      Handle addTransform(const std::string& target,
          const std::string& source, const Callback& cb);

    private:
      void mainCallback(const FeedConstPtr& feed);
      void alCallback(const actionlib::SimpleClientGoalState& state,
          const AlResultConstPtr& result);
      void updateTransforms();

      std::map<std::string, TfSCTransform*>  _transforms;
      ros::NodeHandle                        _nh;
      boost::shared_ptr<ros::Subscriber>     _sub;
      boost::shared_ptr<AlClient>            _al_client;
      std::string                            _sub_id;
      ros::Timer                             _retry_timer;
  };
}

#endif
