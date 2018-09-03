#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# map_setup.py
#
# Copyright (c) 2012-2013 PAL Robotics SL. All Rights Reserved
#
# Authors:
#   * Enrique Fernández

import rospy
import rosparam

from std_msgs.msg import String

import os

class MapSetupException(Exception):
    pass

class MapSetup:

    def __init__(self):
        try:
            self._map = rospy.get_param("~map")
        except KeyError as e:
            rospy.logfatal("map param not set!: %s " % e)
            raise MapSetupException("map param not set")

        # Clean/delete params:
        try:
            rospy.delete_param("mmap")
        except:
            pass

        # Publish map_in_use (latched):
        self._map_in_use_pub = rospy.Publisher("map_in_use", String, queue_size=1, latch=True)
        self._map_in_use_pub.publish("submap_0")

        # Set params:
        rospy.set_param("map_package_in_use"       , "deprecated")
        rospy.set_param("nice_map_in_use"          , os.path.join(self._map, "map.bmp"))
        rospy.set_param("map_transformation_in_use", os.path.join(self._map, "transformation.xml"))

        # Load
        paramlist = rosparam.load_file(os.path.join(self._map, "mmap.yaml"), "mmap")
        for param, ns in paramlist:
            try:
                rosparam.upload_params(ns, param)
            except:
                pass # ignore empty params

def main():
    rospy.init_node("map_setup", log_level=rospy.ERROR)

    MapSetup()

    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == "__main__":
    main()
