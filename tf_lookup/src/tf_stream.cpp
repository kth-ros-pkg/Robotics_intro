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

#include "tf_lookup/tf_stream.h"

#include "tf_lookup/Subscription.h"
#include <tf/tfMessage.h>
#include <boost/foreach.hpp>

namespace tf_lookup
{
  TfStream::TfStream(ros::NodeHandle& nh, const std::string& id,
      const LookupFun& lookup_fun) :
    _id(id),
    _pub(nh.advertise<tf::tfMessage>(_id, 10)),
    _lookup_fun(lookup_fun),
    _last_subscriber_time(ros::Time::now())
  {}

  TfStream::~TfStream()
  {}

  void TfStream::updateTransforms(const TrVect& transforms)
  {
    ROS_DEBUG_STREAM("updating stream " << _id << "with "
        << transforms.size() << " transforms");
    _transforms = transforms;
  }

  bool TfStream::shouldCleanup()
  {
    if (_pub.getNumSubscribers() > 0)
      _last_subscriber_time = ros::Time::now();

    if (ros::Time::now().toSec() - _last_subscriber_time.toSec() < 5.0) //XXX: magic value
      return false;

    return true;
  }

  void TfStream::publish()
  {
    tf::tfMessage m;
    ROS_DEBUG_STREAM("publishing for stream " << _id);
    BOOST_FOREACH(Subscription& t, _transforms)
    {
      ROS_DEBUG_STREAM("  - transform from " << t.target << " to " << t.source);
      geometry_msgs::TransformStamped trans;
      if (_lookup_fun(t.target, t.source, ros::Time(0), trans))
        m.transforms.push_back(trans);
    }

    _pub.publish(m);
  }
}
