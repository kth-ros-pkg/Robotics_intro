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

#include "tf_lookup/tf_stream_server.h"

#include "tf_lookup/tf_stream.h"

namespace tf_lookup
{
  TfStreamServer::TfStreamServer()
  {}

  TfStreamServer::~TfStreamServer()
  {}

  void TfStreamServer::start(ros::NodeHandle& nh, const LookupFun& lookup_fun, const ResolveFun& resolve_fun)
  {
    _nh = nh;
    _lookup_fun = lookup_fun;
    _resolve_fun = resolve_fun;
    _al_server.reset(new AlServer(_nh, "tf_stream",
          boost::bind(&TfStreamServer::alGoalCb, this, _1), false));
    _al_server->start();
    _al_stream_timer = _nh.createTimer(ros::Duration(0.1),
        boost::bind(&TfStreamServer::alStreamer, this));
  }

  void TfStreamServer::alStreamer()
  {
    std::map<std::string, StreamPtr>::iterator it;
    for (it = _streams.begin(); it != _streams.end(); )
    {
      if (it->second->shouldCleanup())
        _streams.erase(it++);
      else
        ++it;
    }

    for (it = _streams.begin(); it != _streams.end(); ++it)
      it->second->publish();
  }

  std::string TfStreamServer::generateId()
  {
    std::ostringstream s;
    s << "tfs_" << ros::Time::now().toNSec();
    return s.str();
  }

  void TfStreamServer::updateStream(AlServer::GoalHandle gh)
  {
    const AlServer::GoalConstPtr& goal = gh.getGoal();
    const std::string& id = goal->subscription_id;
    if (_streams.find(id) == _streams.end())
    {
      gh.setCanceled();
      return;
    }

    gh.setAccepted();

    TrVect transforms;
    int ts = goal->transforms.size();
    transforms.resize(ts);
    for (int i=0; i < ts; ++i)
    {
      transforms[i].target = _resolve_fun(goal->transforms[i].target);
      transforms[i].source = _resolve_fun(goal->transforms[i].source);
    }

    _streams[id]->updateTransforms(transforms);
    gh.setSucceeded();
  }

  void TfStreamServer::addStream(AlServer::GoalHandle gh)
  {
    gh.setAccepted();
    std::string id = generateId();
    _streams[id].reset(new TfStream(_nh, id, _lookup_fun));
    _streams[id]->updateTransforms(gh.getGoal()->transforms);
    AlServer::Result r;
    r.subscription_id = id;
    r.topic = _nh.resolveName(id);
    gh.setSucceeded(r);
  }

  void TfStreamServer::alGoalCb(AlServer::GoalHandle gh)
  {
    if (!gh.getGoal())
    {
      gh.setCanceled(AlServer::Result(), "something went wrong, goal canceled");
      return;
    }

    if (gh.getGoal()->update)
      updateStream(gh);
    else
      addStream(gh);
  }
}
