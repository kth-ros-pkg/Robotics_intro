#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# pal_python: pal_diagnostics.py
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

import rospy
import threading
import diagnostic_msgs.msg as DM


class DiagnosticPublisher:
    """
    Simple diagnostic updater.
    Example:
        from pal_python import DiagnosticPublisher as Diag

        class Blarg:
            def __init__(self):
                self.diag = Diag("blarg: Blarg test")

            def update_diag(self, data):
                with self.diag.lock:
                    fail_rate = '{0:.4f}%'.format(data.fail_rate)
                    self.diag.fields['Test failure rate'] = fail_rate
                    if data.lost:
                        self.diag.fields['Current state'] = 'LOST'
                        self.diag.message("Houston, we might be lost, a bit.")
                        self.diag.level(Diag.WARN)

            def loop(self):
                data = do_stuff()
                self.update_diag(data)
    """

    OK = DM.DiagnosticStatus.OK
    WARN = DM.DiagnosticStatus.WARN
    ERROR = DM.DiagnosticStatus.ERROR

    def __init__(self, name, period=rospy.Duration(1.0)):
        """ By default, publishes diagnostic every second """
        self.pub = rospy.Publisher("/diagnostics", DM.DiagnosticArray)
        self.entry = DM.DiagnosticStatus()
        self.entry.name = name
        self.lock = threading.Lock()
        self._tmr = rospy.Timer(period, self._publish)
        self.fields = {}

    def message(self, msg):
        self.entry.message = msg

    def level(self, lvl):
        self.entry.level = lvl

    def _publish(self, evt):
        msg = DM.DiagnosticArray()
        msg.header.stamp = rospy.get_rostime()
        with self.lock:
            self.entry.values = []
            for k, v in self.fields.iteritems():
                self.entry.values.append(DM.KeyValue(str(k), str(v)))
            msg.status.append(self.entry)
            self.pub.publish(msg)
