#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# pal_python: pal_hci.py
#
# Copyright (c) 2013, 2014 PAL Robotics SL. All Rights Reserved
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
#   * Paul Mathieu

import rospy
import actionlib
import copy
import time
from actionlib_msgs.msg import GoalStatus

import pal_device_msgs.srv as PDMS
import pal_device_msgs.msg as PDM
from std_msgs.msg import ColorRGBA

from .pal_common import configurable
from .pal_rpc import AsyncServiceClient


@configurable
class LedColor(object):

    defaults = {
        'rgb': [0.0, 0.0, 0.0],
        'blinking': False,
    }

    def __init__(self, cfg={}):
        self._read_config(cfg)

    def __eq__(self, other):
        if not isinstance(other, LedColor):
            return False
        return self.rgb == other.rgb and self.blinking == other.blinking


class ReemLedClient(object):
    """
    A simple client to access REEM's ear LEDs. Right now you will need one
    client per color you want to use. Don't worry though, there will only
    be one service client for everybody (per call type).

    Intended use:

        blue = LedColor({'rgb': [0.0, 0.0, 1.0]})
        led_cl = ReemLedClient(blue)
        led_cl.fire(1.0)  # will set the color of the ears to blue for 1 sec

    """
    _led_manager_client = None

    def __init__(self, color, priority=96):
        self._color = color
        self._priority = priority
        self._last_color = None
        self._last_effect = None
        if ReemLedClient._led_manager_client is None:
            ReemLedClient._led_manager_client = actionlib.SimpleActionClient("/pal_led_manager/do_effect",
                                                                            PDM.DoTimedLedEffectAction)
        self.goal = PDM.DoTimedLedEffectGoal()
        self.goal.priority = self._priority
        self.goal.devices = []
        color = ColorRGBA(r=color.rgb[0], g=color.rgb[1], b=color.rgb[2], a=1.0)
        if self._color.blinking:
            self.goal.params.effectType = PDM.LedEffectParams.BLINK
            self.goal.params.blink.first_color = color
            self.goal.params.blink.second_color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.0)
            self.goal.params.blink.first_color_duration = rospy.Duration(0.2)
            self.goal.params.blink.second_color_duration = rospy.Duration(0.2)
        else:
            self.goal.params.effectType = PDM.LedEffectParams.FIXED_COLOR
            self.goal.params.fixed_color.color = color

    def fire(self, duration=None):
        if duration is None:
            duration = 0
        self.goal.effectDuration = rospy.Duration(duration)
        ReemLedClient._led_manager_client.send_goal(self.goal)

    def cancel(self):
        self.client.cancel_goal()

TTS_ACTION_NAME = '/sound'

global __tts_client
__tts_client = None


# TODO: Add a non-blocking version?
#       Possible design:
#         * sync:   TtsSpeak(text).wait()
#         * async:  s = TtsSpeak(text, opt_cb).start()
#                   s.status == True
def tts_speak(text, wait_before_speaking=0):
    """
    Lets the robot say the given text out aloud. This is a blocking call.
    """

    from pal_interaction_msgs.msg import SoundAction, SoundGoal

    global __tts_client
    if __tts_client is None:
        __tts_client = actionlib.SimpleActionClient(TTS_ACTION_NAME, SoundAction)
        rospy.logdebug('Waiting for "%s"...' % TTS_ACTION_NAME)
        if not __tts_client.wait_for_server(rospy.Duration(1.0)):
            rospy.logwarn('Couldn\'t connect to "%s" server.' % TTS_ACTION_NAME)
            return False

    tts_goal = SoundGoal()
    tts_goal.wait_before_speaking = rospy.Duration(wait_before_speaking)
    tts_goal.text = text

    rospy.logdebug('Sent speak command with "%s" (wait: %.3f seconds).' % (
        text, wait_before_speaking))
    __tts_client.send_goal(tts_goal)
    __tts_client.wait_for_result()

    result = __tts_client.get_state()
    rospy.logdebug('Result for last speech command: %s' % result)
    return result == GoalStatus.SUCCEEDED
