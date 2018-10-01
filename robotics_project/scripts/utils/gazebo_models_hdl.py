#!/usr/bin/env python

import numpy as np
import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel, DeleteModelRequest 
from geometry_msgs.msg import Pose

import signal
import sys

class ModelsHandler(object):
    def __init__(self):

		self.node_name = "gazebo_models_handler"
   		rospy.loginfo("Gazebo_models_handler launched")

		self.aruco_cube = rospy.get_param(rospy.get_name() + '/aruco_cube_sdf', '')
		self.dlt_model_srv = "/gazebo/delete_model"
		self.spawn_model_srv = "/gazebo/spawn_sdf_model"

		# Wait for service providers
		rospy.wait_for_service(self.dlt_model_srv, timeout=30)
		rospy.wait_for_service(self.spawn_model_srv, timeout=30)
		self.delete_model_srv = rospy.ServiceProxy(self.dlt_model_srv, DeleteModel)

		rospy.loginfo("%s: Services received!", self.node_name)

		# Initial pose of the cube
		initial_pose = Pose()
		initial_pose.position.x = -1.130530
		initial_pose.position.y = -6.653650
		initial_pose.position.z = 0.862500

		f = open(self.aruco_cube,'r')
		sdffile = f.read()

		spawn_model_prox = rospy.ServiceProxy(self.spawn_model_srv, SpawnModel)
		spawn_model_prox("aruco_cube", sdffile, "/", initial_pose, "world")

		rospy.on_shutdown(self.shutdown_cb)
		rospy.spin()

    def shutdown_cb(self):
		rospy.loginfo("%s: Calling gazebo delete_models", self.node_name)

		try:
			delete_cube = DeleteModelRequest('aruco_cube')
			delete_model_res = self.delete_model_srv(delete_cube)

		except rospy.ServiceException, e:
			print "Service call to gazebo delete_models failed: %s"%e

  		try:
			delete_robot = DeleteModelRequest('tiago_steel')
			delete_model_res = self.delete_model_srv(delete_robot)
		
		except rospy.ServiceException, e:
			print "Service call to gazebo delete_models failed: %s"%e




if __name__ == "__main__":
    
    rospy.init_node('gazebo_models_handler')
    try:
        ModelsHandler()

    except rospy.ROSInterruptException:
        pass