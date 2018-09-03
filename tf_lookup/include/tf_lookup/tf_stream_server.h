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

#ifndef TFSTREAMSERVER_H
#define TFSTREAMSERVER_H

#include <string>
#include <map>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <actionlib/server/action_server.h>

#include "tf_lookup/TfStreamAction.h"

namespace geometry_msgs
{ ROS_DECLARE_MESSAGE(TransformStamped); }

namespace tf_lookup
{
  class TfStream;

  class TfStreamServer
  {
    friend class TfStream;

    private:
      typedef actionlib::ActionServer<tf_lookup::TfStreamAction> AlServer;
      typedef boost::shared_ptr<TfStream> StreamPtr;
      typedef boost::function<bool(const std::string&, const std::string&,
          const ros::Time&, geometry_msgs::TransformStamped&)> LookupFun;
      typedef boost::function<std::string(const std::string&)> ResolveFun;
      typedef std::vector<tf_lookup::Subscription> TrVect;

    public:
      TfStreamServer();
      virtual ~TfStreamServer();

      void start(ros::NodeHandle& nh, const LookupFun& lookup_fun, const ResolveFun& resolve_fun);

    private:
      void alGoalCb(AlServer::GoalHandle gh);
      void alCancelCb(AlServer::GoalHandle gh);
      void alStreamer();
      std::string generateId();
      void updateStream(AlServer::GoalHandle gh);
      void addStream(AlServer::GoalHandle gh);

      boost::shared_ptr<AlServer>      _al_server;
      ros::NodeHandle                  _nh;
      ros::Timer                       _al_stream_timer;
      std::map<std::string, StreamPtr> _streams;
      LookupFun                        _lookup_fun;
      ResolveFun                       _resolve_fun;
  };
}

#endif
