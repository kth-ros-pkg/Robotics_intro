#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# pal_python: pal_environ.py
#
# Copyright (c) 2013 PAL Robotics SL. All Rights Reserved
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
#   * Siegfried-A. Gevatter Pujals

import os
import sys
import time

import rospy

from pal_python import pal_common

def is_desktop():
    """
    Check whether running on the desktop or a robot.

    This is achieved by looking at the $TARGET environment variable.
    """
    return os.environ.get('TARGET') == 'desktop'

def get_robot():
    """
    Return the name of the robot model (eg. "reemh3"), for use eg. in
    configuration directories.
    """
    # Expected environment variables:
    #  - On a computer, PAL_ROBOT should be unset and PAL_HOST should
    #    be a generic robot name (eg. "reemh3").
    #  - On a robot, PAL_ROBOT should be the generic robot name and
    #    PAL_HOST should also specify the computer (eg. "reemh3c"/"reemh3m").
    robot = os.environ.get('PAL_ROBOT')
    host = os.environ.get('PAL_HOST')
    if robot:
        if not robot == host[:-1]:
            sys.stderr.write('Warning: PAL_ROBOT and PAL_HOST don\'t match.\n')
        return robot
    else:
        # Running on a desktop
        if host[-1] in ('c', 'm') and host != 'reemc':
            sys.stderr.write('Warning: invalid PAL_HOST for desktop.\n')
        return host

def get_maps_pkg(robot):
    return robot + '_maps'

def get_nav_pkg(robot):
    return robot + '_2dnav'

def get_performance_mode():
    """
    Return the robot's performance setting.

    This is an integer value and depends on the model of the robot.
    """
    return rospy.get_param('/pal/performanceMode', 10)

def wait_for_file(filename, timeout=None):
    """
    Wait until `filename' exists for up to `timeout' seconds. If the timeout
    is exceeded, IOError is raised.
    """
    # We use system time and not ROS time here, since we are waiting
    # for a physical event (file being generated) vs. some simulation
    # event. Additionaly, this way we don't require rospy.init_node().
    start_time = time.time()
    while not os.path.isfile(filename):
        elapsed_time = time.time() - start_time
        if timeout is not None and elapsed_time > timeout:
            raise IOError(
                'File "%s" not found (after %d seconds).' % (filename, timeout))
        rospy.sleep(0.1)

def wait_for_nodes(nodes, timeout=None):
    """
    Wait until all the `nodes' are running up to `timeout' seconds.
    If the timeout expires, False is returned.
    """
    # We use system time and not ROS time here, since we are waiting
    # for a physical event (file being generated) vs. some simulation
    # event. Additionaly, this way we don't require rospy.init_node().
    start_time = time.time()
    while not all([pal_common.is_node_running(n) for n in nodes]):
        elapsed_time = time.time() - start_time
        if timeout is not None and elapsed_time > timeout:
            return False
        rospy.sleep(0.1)
    return True
