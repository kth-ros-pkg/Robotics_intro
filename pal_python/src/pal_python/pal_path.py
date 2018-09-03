#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# pal_python: pal_path.py
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
#   * Siegfried-A. Gevatter
#   * Enrique Fernandez

import os

import rospkg

from pal_python import pal_environ

def assert_relative_path(name):
    """
    Raise an assertion if the given path is absolute (starts with a slash).
    """
    assert not name.startswith('/'), \
           'Filename "%s" is absolute path, relative path expected.' % (name,)

def get_base_path():
    """
    Return the path to the root directory. This is /opt/pal/$PAL_DISTRO on
    the robot and  ~/svn (or the aprorpiate branch) on a desktop.
    """
    if pal_environ.is_desktop():
        return os.path.join(os.path.expanduser('~/'),
                            os.environ.get('PAL_BRANCH'))
    else:
        return os.path.join('/opt/pal',
                            os.environ.get('PAL_DISTRO'))

def get_launch_path(name=''):
    """
    Return the path to the launch directory or, if `name' is not empty,
    to the given *Start.sh/*Stop.sh file.

    Takes into account that when running on a development machine
    the package can be found installed from debians or in the
    current workspace, this last one having preference.
    """
    assert_relative_path(name)

    if pal_environ.is_desktop():
        base = os.path.join(rospkg.RosPack().get_path('pal_startup_base'),
                            'scripts')
        # We check if there is not a scripts directory, in that case
        # pal_startup_base is installed from our debians and we need to
        # use a slightly different path
        if not os.path.exists(base):
            base = rospkg.RosPack().get_path('pal_startup_base')
            # The path we get from RosPack is:
            # /opt/pal/cobalt/share/pal_startup_base/launch/*.sh
            # But the actual executable files are here:
            # /opt/pal/cobalt/bin/pal_startup_base/launch/*.sh
            # So we replace that part of the path
            base = base.replace('/share/', '/bin/')
    else:
        base = os.path.join(get_base_path(),
                            'bin/pal_startup_base')
    return os.path.join(base, 'launch', name)

def get_bin_path(name=''):
    """
    Return the path to the bin/ directory or, if `name' is not empty,
    to the given executable file.

    This can be used for launching binaries that are still in the
    robot/ directory.
    """
    assert_relative_path(name)

    base = get_base_path()
    if pal_environ.is_desktop():
        base = os.path.join(base, 'robot/local/output')
    return os.path.join(base, 'bin', name)

def get_ros_bin_path(name=''):
    """
    Return the path to the ros/$ROS_DISTRO/bin/ directory or,
    if `name' is not empty, to the given executable file.

    This can be used for obtaining the path to `roslaunch', etc.
    """
    assert_relative_path(name)

    ros_distro = os.environ.get('ROS_DISTRO')
    return os.path.join('/opt/ros', ros_distro, 'bin', name)

def get_config_path(name=''):
    """
    Return the path to the configuration directory ("etc") or, if `name'
    is not empty, the given configuration subdirectory or file.

    If running on a desktop, the path depends on the $PAL_HOST environment
    variable.
    """
    assert_relative_path(name)

    path = get_base_path()
    if pal_environ.is_desktop():
        path = os.path.join(path, 'robot/sources/etc', pal_environ.get_robot())
    else:
        path = os.path.join(path, 'etc')
    return os.path.join(path, name)

global __maps_path
__maps_path = None
def get_maps_path(name='', robot=None):
    """
    """
    global __maps_path
    assert_relative_path(name)

    if not __maps_path:
        if robot is None:
            robot = pal_environ.get_robot()
        pkg = pal_environ.get_maps_pkg(robot)
        __maps_path = os.path.join(os.getenv('HOME'), '.pal', pkg)

    return os.path.join(__maps_path, name)
