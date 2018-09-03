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

#ifndef TFLOOKUPCLIENT_IMPL_H
#define TFLOOKUPCLIENT_IMPL_H

#include "tf_lookup/tf_lookup_client.h"

#include <geometry_msgs/TransformStamped.h>
#include <ros/time.h>

namespace tf_lookup
{
  TfLookupClient::TfLookupClient(ros::NodeHandle &nh)
    : _nh(nh)
  {
    _al_client.reset(new AlClient(_nh, "tf_lookup"));
  }

  TfLookupClient::~TfLookupClient()
  {}

  bool TfLookupClient::queryTransform(const std::string& target,
      const std::string& source, const Callback& cb)
  {
    if (!_al_client->isServerConnected())
      return false;

    Goal goal;
    goal.target_frame = target;
    goal.source_frame = source;
    goal.transform_time = ros::Time(0);

    AlClient::GoalHandle gh = _al_client->sendGoal(goal,
          boost::bind(&TfLookupClient::tfAlTransitionCb, this, _1));
    _al_goals.emplace_back(gh, cb);

    return true;
  }

  void TfLookupClient::tfAlTransitionCb(AlClient::GoalHandle gh)
  {
    std::list<GhCbPair>::iterator it =
      std::find_if(_al_goals.begin(), _al_goals.end(), gh_compare(gh));
    if (it == _al_goals.end())
      return;

    switch (gh.getCommState().state_)
    {
      case actionlib::CommState::DONE:
        switch(gh.getTerminalState().state_)
        {
          case actionlib::TerminalState::SUCCEEDED:
            it->second(true,
                TransformConstPtr(new Transform(gh.getResult()->transform)));
            break;
          default:
            it->second(false, TransformConstPtr());
            break;
        }
        _al_goals.erase(it);
        break;
      default:
        break;
    }
  }
}

#endif
