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

#ifndef TFSTREAM_H
#define TFSTREAM_H

#include <string>
#include <vector>
#include <ros/ros.h>
#include <ros/time.h>

namespace geometry_msgs
{ ROS_DECLARE_MESSAGE(TransformStamped); }

namespace tf_lookup
{
  ROS_DECLARE_MESSAGE(Subscription);

  /** Publish a given set of tf transforms on a topic.
   */
  class TfStream
  {
    private:
      typedef std::vector<tf_lookup::Subscription> TrVect;
      typedef boost::function<bool(const std::string&, const std::string&,
          const ros::Time&, geometry_msgs::TransformStamped&)> LookupFun;

    public:

      /** Initializes the stream.
       * \param nh         NodeHandle for scoping the advertised topic.
       * \param id         Unique id for this stream.
       * \param lookup_fun Function to call to look up a tf transform.
       */
      TfStream(ros::NodeHandle& nh, const std::string& id,
          const LookupFun& lookup_fun);

      virtual ~TfStream();

      void updateTransforms(const TrVect& transforms);
      void publish();
      bool shouldCleanup();

    private:
      TfStream(const TfStream& rhs);
      TfStream& operator=(const TfStream& rhs);

      std::string               _id;
      ros::Publisher            _pub;
      TrVect                    _transforms;
      LookupFun                 _lookup_fun;
      ros::Time                 _last_subscriber_time;
  };
}

#endif
