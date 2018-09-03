#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# pal_python: pal_thread.py
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

from contextlib import contextmanager
import threading

class SharedData(object):
    """
    A class for sharing data between threads.

    Example usage:
        # Setup:
        data = pal_thread.SharedData(foo=1)

        # Thread 1:
        with data.lock():
            data.foo = 2

        # Thread 2:
        print data['foo']
    """

    def __init__(self, **kwargs):
        self._lock = threading.Lock()
        self._lock_owner = None
        # We store values in a dictionary so the attributes can be
        # enumerated.
        self._data = {}
        for key, value in kwargs.iteritems():
            self._data[key] = value

    def _is_lock_owner(self):
        """
        Whether the current thread owns an active lock.
        """
        return self._lock_owner == threading.current_thread().ident

    @contextmanager
    def lock(self):
        """
        Acquire a lock.
        """
        self._lock_owner = threading.current_thread().ident
        with self._lock:
            yield self
        self._lock_owner = None

    def __getattr__(self, key):
        """
        Access a value. Requires that the lock is already held.
        """
        if key.startswith('_'):
            return super(SharedData, self).__getattr__(key)
        assert self._is_lock_owner()
        return self._data[key]

    def __getitem__(self, key):
        """
        Access a value, acquiring a lock first. Blocking call.

        Be aware that calling this method successively to get
        different attributes is not thread safe.
        """
        with self.lock():
            return self._data[key]

    def __setattr__(self, key, value):
        """
        Set a value. Requires that the lock is already held.
        """
        if key.startswith('_'):
            return super(SharedData, self).__setattr__(key, value)
        assert self._is_lock_owner()
        self._data[key] = value

class SharedMessage(SharedData):
    """
    Same as SharedData, but with a get_message() method that returns
    all the data in a ROS message of the given type.
    """

    def __init__(self, message_type, **kwargs):
        super(SharedMessage, self).__init__(**kwargs)
        self._message_type = message_type

    def get_message(self):
        with self.lock():
            message = self._message_type()
            for key, value in self._data.iteritems():
                setattr(message, key, value)
        return message
