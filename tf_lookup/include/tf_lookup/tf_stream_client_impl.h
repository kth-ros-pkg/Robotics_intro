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

#ifndef TFSTREAMCLIENT_IMPL_H
#define TFSTREAMCLIENT_IMPL_H

#include "tf_lookup/tf_stream_client.h"

#include <boost/foreach.hpp>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tfMessage.h>

#include "tf_lookup/TfStreamAction.h"
#include "tf_lookup/tf_sc_transform.h"

void strip(std::string& str, char c)
{
  str.erase(
    std::remove_if(
      str.begin(), str.end(),
      [c](char c_i) -> bool {
        return c_i == c;
      }),
    str.end());
}

std::string key_from_transformation(const std::string& target, const std::string& source)
{
  std::string t = target;
  std::string s = source;

  strip(t, '/');
  strip(s, '/');

  return t + "@" + s;
}

tf_lookup::Subscription transformation_from_key(const std::string& key)
{
  tf_lookup::Subscription s;

  size_t mid = key.find("@");
  s.target = key.substr(0, mid);
  s.source = key.substr(mid+1);

  return s;
}


namespace tf_lookup
{
  typedef std::map<std::string, TfSCTransform*> Transforms_T;

  TfStreamClient::TfStreamClient(ros::NodeHandle& nh) : _nh(nh)
  {
    _al_client.reset(new AlClient(_nh, "tf_stream"));
  }

  TfStreamClient::~TfStreamClient()
  {}

  TfStreamClient::Handle TfStreamClient::addTransform(const std::string& target,
      const std::string& source, const Callback& cb)
  {
    const std::string key = key_from_transformation(target, source);

    return Handle(new TfSCTransform(key, this, cb));
  }

  void TfStreamClient::updateTransforms()
  {
    /* Retry every second if the server is not reachable */
    if (!_al_client->isServerConnected() || (!_sub && _sub_id == "pending"))
    {
      _retry_timer = _nh.createTimer(ros::Duration(1.0),
          boost::bind(&TfStreamClient::updateTransforms, this), true);
      return;
    }

    AlGoal g;
    if (_sub)
    {
      g.update = true;
      g.subscription_id = _sub_id;

      _retry_timer.stop();
    }
    else
      _sub_id = "pending";

    g.transforms.reserve(_transforms.size());
    BOOST_FOREACH (const Transforms_T::value_type& t, _transforms)
    {
      g.transforms.push_back(transformation_from_key(t.first));
    }

    _al_client->sendGoal(g,
        boost::bind(&TfStreamClient::alCallback, this, _1, _2));
  }

  void TfStreamClient::mainCallback(const FeedConstPtr& feed)
  {
    BOOST_FOREACH (const geometry_msgs::TransformStamped& t, feed->transforms)
    {
      const std::string& parent = t.header.frame_id;
      const std::string& child = t.child_frame_id;
      const std::string tr = key_from_transformation(parent, child);

      Transforms_T::iterator it = _transforms.find(tr);
      if (it == _transforms.end())
      {
        ROS_WARN("we have received an unsollicited transform: [%s]->[%s]",
            parent.c_str(), child.c_str());
        continue;
      }
      it->second->_cb(boost::shared_ptr<geometry_msgs::TransformStamped>
          (new geometry_msgs::TransformStamped(t)));
    }
  }

  void TfStreamClient::alCallback(const actionlib::SimpleClientGoalState& gs,
      const AlResultConstPtr& result)
  {
    if (gs != actionlib::SimpleClientGoalState::SUCCEEDED)
      return;

    if(!_sub)
    {
      if (result->topic == "") /* these aren't the droids you're looking for */
      {
        updateTransforms();
        return;
      }
      _sub.reset(new ros::Subscriber(_nh.subscribe(result->topic, 1,
            &TfStreamClient::mainCallback, this)));

      _sub_id = result->subscription_id;
    }
  }


  TfSCTransform::TfSCTransform(const std::string& key,
      TfStreamClient* psc, const TfStreamClient::Callback& cb)
    : _psc(psc), _cb(cb), _key(key)
  {
    ROS_INFO_STREAM("created tf stream for " << _key);
    _psc->_transforms[key] = this;
    _psc->updateTransforms();
  }

  TfSCTransform::~TfSCTransform()
  {
    _psc->_transforms.erase(_key);
    _psc->updateTransforms();
  }
}

#endif
