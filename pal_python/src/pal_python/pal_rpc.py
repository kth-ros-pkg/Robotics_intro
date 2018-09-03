#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# pal_python: pal_rpc.py
#
# Copyright (c) 2014 PAL Robotics SL. All Rights Reserved
#
# Permission to use, copy, modify, and/or distribute this software for
# any purpose with or without fee is hereby granted, provided that the
# above copyright notice and this permission notice appear in all
# copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND ISC DISCLAIMS ALL WARRANTIES WITH
# REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL ISC BE LIABLE FOR ANY
# SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT
# OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#
# Authors:
#   * Paul Mathieu

import threading
import rospy
import actionlib


class EasyActionServer:
    """
    Supposedly easier to use than the SimpleActionServer.
    This action server will replace ROS services. It provides a
    simplistic asynchronous RPC call interface with single goal.
    The cancel callback can be optionaly specified to allow for interruptible
    actions.

    Intended use:

        def ac_cb(goal):
            print("received a goal: {}".format(goal))

        EasyActionServer("/foo_action", foo.msg.FooAction, ac_cb)

    """
    def __init__(self, ac_name, ac_type, cb, immediate_success=True):
        """ If immediate_success is False, the user will have to call reply()
        to set_succeeded() the goal. Otherwise, the goal will be automatically
        set to succeeded after the callback return. """
        self._cb = cb
        self._opt_cancel_cb = None
        self._ghdl = None
        self._immed = immediate_success
        self._ac = actionlib.ActionServer(ac_name, ac_type, auto_start=False,
                                          goal_cb=self._goal_cb,
                                          cancel_cb=self._cancel_cb)
        self._ac.start()

    def reply(self, result=None):
        """ Only useful if `immediate_success=False` was given to the
        constructor. Will mark the action as succeeded. """
        if self._immed:
            rospy.logwarn("EasyActionServer.reply() has no "
                          "effect if initialized with immediate_success=True")
            return
        if self._ghdl is not None:  # goal has not been canceled or pre-empted
            self._succeed(result)

    def set_cancel_cb(self, cancel_cb):
        """ Only useful if `immediate_success=False` was given to the
        constructor (otherwise the action will immediately succeed). Will
        register an optional cancel callback. """
        if self._immed:
            rospy.logwarn("EasyActionServer.set_cancel_cb() has no "
                          "effect if initialized with immediate_success=True")
            return
        self._opt_cancel_cb = cancel_cb

    def _succeed(self, result=None):
        if self._ghdl is None:
            rospy.logerr("trying to succeed on an invalid goal handle")
            return
        self._ghdl.set_succeeded(result)
        self._ghdl = None

    def _goal_cb(self, ghdl):
        if self._ghdl is not None:
            self._ghdl.set_aborted()
        self._ghdl = ghdl
        self._ghdl.set_accepted()
        self._cb(ghdl.get_goal())
        if self._immed:
            self._succeed()

    def _cancel_cb(self, ghdl):
        if ghdl != self._ghdl:
            rospy.logerr("trying to cancel an invalid goal handle")
            return
        self._cancel()
        if self._opt_cancel_cb is not None:
            self._opt_cancel_cb()

    def _cancel(self):
        if self._ghdl is None:
            rospy.logerr("trying to cancel an invalid goal handle")
            return
        self._ghdl.set_canceled()
        self._ghdl = None


class AsyncServiceClient:
    """
    Simple non-blocking service client.

    Intended use:

        srv_cl = AsyncServiceClient("/foo", foo.srv.Foo)
        req = foo.srv.FooRequest()
        req.bar = "baz"
        srv_cl.call(req)

    If you want to use the result:

        def callback(result):
            rospy.loginfo(result)
        srv_cl.call(req, cb=callback)

    Note that the callback (if not None) will be called from within a worker
    thread. If the service is not available, or there is some error, the callback
    won't be called.
    """
    def __init__(self, srv_name, srv_type, persistent=False):
        self._online = False
        self._srv_name = srv_name
        self._srv_type = srv_type
        self._request = None
        self._wakeup = threading.Condition()
        threading.Thread(target=self._register_proxy,
                         args=[persistent]).start()
        threading.Thread(target=self._worker).start()

    def call(self, *args, **kwargs):
        """ Asynchronously send a request to the service provider.

        Usage:
          call(*args, cb=None)

        Returns False if the service is not available. Otherwise, if `cb'
        is not None and there is no error, it'll be called with the result.
        """
        if kwargs and kwargs.keys() != ['cb']:
            raise ValueError('The only valid keyword argument is "cb".')
        cb = kwargs.get('cb', None)

        if self._online:
            with self._wakeup:
                self._request = (args, cb)
                self._wakeup.notify_all()
            return True
        return False

    def _register_proxy(self, persistent):
        try:
            rospy.wait_for_service(self._srv_name)
            self._srvc = rospy.ServiceProxy(self._srv_name, self._srv_type,
                                            persistent)
            self._online = True
        except rospy.ServiceException as e:
            rospy.logerr("registering service {} failed: {}"
                         .format(self._srv_name, e))
        except rospy.ROSInterruptException:
            pass

    def _worker(self):
        while not rospy.is_shutdown():
            if self._request is None:
                with self._wakeup:
                    self._wakeup.wait(1.0)
                continue
            req, cb = self._request
            self._request = None
            try:
                result = self._call_service(*req)  # ones are in the queue
                if cb:
                    cb(result)
            except rospy.ROSInterruptException:
                break

    def _call_service(self, *args):
        try:
            return self._srvc(*args)
        except rospy.ServiceException as e:
            rospy.logerr("service call to {} failed: {}"
                         .format(self._srv_name, e))
