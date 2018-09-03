#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# pal_python: pal_launch.py
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
import sys
import subprocess

from pal_python import pal_path

# TODO: move this to pal_command/pal_execute?
def execute_command(*command):
    """
    Run the given command and wait for completion. Return the exit
    code, or None if the command couldn't be run at all.
    """
    sys.stderr.write('Running command: "%s".\n' % ' '.join(command))
    try:
        retcode = subprocess.call(command, shell=False)
        if retcode < 0:
            sys.stderr.write(' -> Terminated by signal %d.\n' % -retcode)
        else:
            sys.stderr.write(' -> Returned %d.\n' % retcode)
        return retcode
    except OSError as e:
        sys.stderr.write(' -> Execution failed: %s\n' % e)
        return None

def spawn_command(*command):
    """
    Run the given command, without waiting for completion. Return the Popen
    object (which provides .send_signal, .kill(), .wait(), etc), or None
    if the  command couldn't be run at all.

    The command is created with its own process group; this mean that
    signals (eg. Ctrl+C) won't be forwarded to it.
    """
    sys.stderr.write('Spawning command: "%s".\n' % ' '.join(command))
    try:
        return subprocess.Popen(command, shell=False, preexec_fn=os.setpgrp)
    except OSError as e:
        sys.stderr.write(' -> Execution failed: %s\n' % e)
        return None

# NavigationFunctions.py used to log Command+Result to /tmp/navigation_sm.log.
# It also treated 36608, 15 and 256 as zero, without saying why.

class LaunchError(OSError):
    """
    Exception executing a file from the launch directory (usually a
    *Start.sh or *Stop.sh script).
    """

    def __init__(self, component, launchfile, code):
        super(LaunchError, self).__init__()
        self.component = component
        self.errno = code
        self.filename = launchfile
        self.strerror = '"%s" failed with code %d' % (launchfile, code)
        self.message = self.strerror

# TODO: Move ErrorCollection to pal_common.py and here have
#       LaunchErrorCollection(OSError, ErrorCollection), plus
#       add methods for merging ErrorCollections?
class LaunchErrorCollection(OSError):
    """
    Exception aggregating one or more exceptions that occurred trying to
    perform several related actions (eg. launching various components).
    """

    def __init__(self, errors=None):
        super(LaunchErrorCollection, self).__init__()
        self.errors = errors if errors is not None else []

    def add_error(self, error):
        self.errors.append(error)

    @property
    def message(self):
        return ' | '.join(e.message for e in self.errors)

    def empty(self):
        """
        True if this error collection is empty (ie. no errors).
        """
        return len(self.errors) == 0

def roslaunch(stack, fname, *params):
    roslaunch_bin = pal_path.get_ros_bin_path('roslaunch')
    launchfile = fname if fname.endswith('.launch') else '%s.launch' % fname
    process = spawn_command(roslaunch_bin, stack, launchfile, *params)
    if process is None:
        raise LaunchError(stack, launchfile, '[execution failed]')

def rosrun(stack, fname, *params):
    rosrun_bin = pal_path.get_ros_bin_path('rosrun')
    process = spawn_command(rosrun_bin, stack, fname, *params)
    if process is None:
        raise LaunchError(stack, fname, '[execution failed]')

def start(component, *params):
    launchfile = '%sStart.sh' % component
    process = spawn_command(pal_path.get_launch_path(launchfile), *params)
    if process is None:
        raise LaunchError(component, launchfile, '[execution failed]')

def stop(component):
    launchfile = '%sStop.sh' % component
    process = spawn_command(pal_path.get_launch_path(launchfile))
    if process is None:
        raise LaunchError(component, launchfile, '[execution failed]')

def run(launchfile, *args):
    result = execute_command(pal_path.get_launch_path(launchfile), *args)
    if result != 0:
        raise LaunchError(launchfile, launchfile, result)

def start_all(*components):
    errors = LaunchErrorCollection()
    # TODO: write execute_command_async (using subprocess.Popen) and
    #       use it here to run all in parallel? Maybe only if parallel=True?
    for component in components:
        try:
            start(component)
        except LaunchError, e:
            errors.add_error(e)
    if not errors.empty():
        raise errors

def stop_all(*components):
    errors = LaunchErrorCollection()
    for component in components:
        try:
            stop(component)
        except LaunchError, e:
            errors.add_error(e)
    if not errors.empty():
        raise errors
