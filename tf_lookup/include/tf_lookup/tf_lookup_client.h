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

#ifndef TFLOOKUPCLIENT_H
#define TFLOOKUPCLIENT_H

#include <memory>
#include <string>
#include <list>
#include <ros/ros.h>
#include <boost/function.hpp>
#include <actionlib/client/action_client.h>

#include "tf_lookup/TfLookupAction.h"

namespace geometry_msgs
{ ROS_DECLARE_MESSAGE(TransformStamped); }

namespace tf_lookup
{
  class TfLookupClient
  {
    private:
      typedef geometry_msgs::TransformStamped                        Transform;
      typedef geometry_msgs::TransformStampedConstPtr         TransformConstPtr;
      typedef boost::function<void(bool, const TransformConstPtr&)>  Callback;
      typedef actionlib::ActionClient<tf_lookup::TfLookupAction> AlClient;
      typedef tf_lookup::TfLookupGoal                            Goal;
      typedef std::pair<AlClient::GoalHandle, Callback>              GhCbPair;

      struct gh_compare : public std::unary_function<GhCbPair, bool>
      {
        explicit gh_compare(const AlClient::GoalHandle& gh) : _gh(gh) {}
        bool operator() (const GhCbPair& p) { return p.first == _gh; }
        AlClient::GoalHandle _gh;
      };

    public:
      TfLookupClient(ros::NodeHandle &nh);
      virtual ~TfLookupClient();

      bool queryTransform(const std::string& target, const std::string& source,
          const Callback& cb);

    private:
      void tfAlTransitionCb(AlClient::GoalHandle gh);

      ros::NodeHandle           _nh;
      std::shared_ptr<AlClient> _al_client;
      std::list<GhCbPair>       _al_goals;
  };
}

#endif
