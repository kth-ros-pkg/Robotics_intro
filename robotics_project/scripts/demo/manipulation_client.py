#!/usr/bin/env python

# Copyright (c) 2016 PAL Robotics SL. All Rights Reserved
#
# Permission to use, copy, modify, and/or distribute this software for
# any purpose with or without fee is hereby granted, provided that the
# above copyright notice and this permission notice appear in all
# copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#
# Author:
#   * Sam Pfeiffer
#   * Job van Dieten
#   * Jordi Pages

import rospy
import time
from robotics_project.msg import PickUpPoseAction, PickUpPoseGoal
from geometry_msgs.msg import PoseStamped, Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from actionlib import SimpleActionClient
from robotics_project.srv import MoveHead, MoveHeadRequest, MoveHeadResponse

import tf2_ros
from tf2_geometry_msgs import do_transform_pose

import numpy as np
from std_srvs.srv import Empty, SetBool, SetBoolResponse

import cv2
from cv_bridge import CvBridge

from moveit_msgs.msg import MoveItErrorCodes
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
	if not name[:1] == '_':
		code = MoveItErrorCodes.__dict__[name]
		moveit_error_dict[code] = name

class SphericalService(object):
	def __init__(self):
		rospy.loginfo("Starting Spherical Grab Service")

		self.pick_srv_nm = rospy.get_param(rospy.get_name() + '/pick_srv')
		self.place_srv_nm = rospy.get_param(rospy.get_name() + '/place_srv')
		self.mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')

		self.place_gui = rospy.Service(self.place_srv_nm, SetBool, self.start_aruco_place)
		self.pick_gui = rospy.Service(self.pick_srv_nm, SetBool, self.start_aruco_pick)
		self.move_head_srv = rospy.Service(self.mv_head_srv_nm, MoveHead, self.move_head)
		
		self.head_cmd = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=1)

		rospy.loginfo("Launching SphericalService constructor")
		self.pick_type = ManipulateAruco()


	def start_aruco_pick(self, req):
		success = self.pick_type.pick_and_place_aruco("pick")
		reply = SetBoolResponse()
		reply.success = success
		reply.message = ""
		return reply

	def start_aruco_place(self, req):
		success = self.pick_type.pick_and_place_aruco("place")
		reply = SetBoolResponse()
		reply.success = success
		reply.message = ""
		return reply

	def move_head(self, req):
		jt = JointTrajectory()
		jt.joint_names = ['head_1_joint', 'head_2_joint']
		jtp = JointTrajectoryPoint()

		response = MoveHeadResponse()

		if req.motion == "down":
			jtp.positions = [0.0, -0.75]
			response.success = True
		elif req.motion == "up":
			jtp.positions = [0.0, 0.0]
			response.success = True
		else:
			response.success = False
		
		jtp.time_from_start = rospy.Duration(2.0)
		jt.points.append(jtp)
		rospy.loginfo("Moving head " + req.motion)
		self.head_cmd.publish(jt)
		rospy.loginfo("Done.")
		return response

class ManipulateAruco(object):
	def __init__(self):
		rospy.loginfo("Initalizing ManipulateAruco...")
		self.aruco_pose_top = rospy.get_param(rospy.get_name() + '/marker_pose_topic')
		self.pickup_pose_top = rospy.get_param(rospy.get_name() + '/pickup_marker_pose')
		self.place_pose_top = rospy.get_param(rospy.get_name() + '/place_marker_pose')

		self.bridge = CvBridge()
		self.tfBuffer = tf2_ros.Buffer()
		self.tf_l = tf2_ros.TransformListener(self.tfBuffer)
		        
		rospy.loginfo("Waiting for /pickup_pose AS...")
		self.pick_as = SimpleActionClient(self.pickup_pose_top, PickUpPoseAction)
		self.pick_as.wait_for_server()

		rospy.loginfo("Waiting for /place_pose AS...")
		self.place_as = SimpleActionClient(self.place_pose_top, PickUpPoseAction)
		self.place_as.wait_for_server()

		rospy.loginfo("Waiting for '/play_motion' AS...")
		self.play_m_as = SimpleActionClient('/play_motion', PlayMotionAction)
		if not self.play_m_as.wait_for_server(rospy.Duration(300)):
			rospy.logerr("Could not connect to /play_motion AS")
			exit()
		rospy.loginfo("Connected!")
		rospy.sleep(1.0)

		rospy.loginfo("Setting publishers to torso and head controller...")
		self.torso_cmd = rospy.Publisher(
			'/torso_controller/command', JointTrajectory, queue_size=1)

		self.detected_pose_pub = rospy.Publisher('/detected_aruco_pose',
							 PoseStamped,
							 queue_size=1,
							 latch=True)

		self.aruco_pose_rcv = False
		self.aruco_pose_subs = rospy.Subscriber(self.aruco_pose_top, PoseStamped, self.aruco_pose_cb)

		self.pick_g = PickUpPoseGoal()

		rospy.loginfo("Done initializing ManipulateAruco.")

   	def strip_leading_slash(self, s):
		return s[1:] if s.startswith("/") else s
		
	def pick_and_place_aruco(self, string_operation):

		success = False

		if string_operation == "pick":
			self.prepare_robot_pandp()
			rospy.sleep(2.0)

			while not rospy.is_shutdown() and self.aruco_pose_rcv == False:
				rospy.loginfo("spherical_grasp_gui: Waiting for an aruco detection...")
				rospy.sleep(1.0)

			aruco_pose = self.aruco_pose
			aruco_pose.header.frame_id = self.strip_leading_slash(aruco_pose.header.frame_id)
			rospy.loginfo("Got: " + str(aruco_pose))

			rospy.loginfo("spherical_grasp_gui: Transforming from frame: " +
			aruco_pose.header.frame_id + " to 'base_footprint'")
			ps = PoseStamped()
			ps.pose.position = aruco_pose.pose.position
			ps.header.stamp = self.tfBuffer.get_latest_common_time("base_footprint", aruco_pose.header.frame_id)
			ps.header.frame_id = aruco_pose.header.frame_id
			transform_ok = False
			while not transform_ok and not rospy.is_shutdown():
				try:
					transform = self.tfBuffer.lookup_transform("base_footprint", 
										   ps.header.frame_id,
										   rospy.Time(0))
					aruco_ps = do_transform_pose(ps, transform)
					transform_ok = True
				except tf2_ros.ExtrapolationException as e:
					rospy.logwarn(
						"Exception on transforming point... trying again \n(" +
						str(e) + ")")
					rospy.sleep(0.01)
					ps.header.stamp = self.tfBuffer.get_latest_common_time("base_footprint", aruco_pose.header.frame_id)

                        rospy.loginfo("Setting cube pose based on Aruco detection")
			self.pick_g.object_pose.pose.position = aruco_ps.pose.position
                        self.pick_g.object_pose.pose.position.z -= 0.1*(1.0/2.0)

                        rospy.loginfo("aruco pose in base_footprint:" + str(self.pick_g))

			self.pick_g.object_pose.header.frame_id = 'base_footprint'
			self.pick_g.object_pose.pose.orientation.w = 1.0
			self.detected_pose_pub.publish(self.pick_g.object_pose)
			rospy.loginfo("Gonna pick:" + str(self.pick_g))
			self.pick_as.send_goal_and_wait(self.pick_g)
			rospy.loginfo("Done!")

			result = self.pick_as.get_result()
			if str(moveit_error_dict[result.error_code]) != "SUCCESS":
				rospy.logerr("Failed to pick, not trying further")
				success = False
			else: 
				success = True
					
			self.prepare_robot_nav()
			return success

		if string_operation == "place":

            # Place the object on table in front
			rospy.loginfo("Placing aruco marker")
			self.place_as.send_goal_and_wait(self.pick_g)
			rospy.loginfo("Done!")

			result = self.place_as.get_result()
			if str(moveit_error_dict[result.error_code]) != "SUCCESS":
				rospy.logerr("Failed to place, not trying further")
				success = False
			else: 	
				success = True
			
			return success

        def move_torso(self, string_operation):
		jt = JointTrajectory()
		jt.joint_names = ['torso_lift_joint']
		jtp = JointTrajectoryPoint()
		if string_operation == "lift":
			jtp.positions = [0.34]
		elif string_operation == "lower":
			jtp.positions = [0.15]
		else:
			return

		jtp.time_from_start = rospy.Duration(2.5)
		jt.points.append(jtp)
		rospy.loginfo("Moving torso " + string_operation)
		self.torso_cmd.publish(jt)

	def prepare_robot_pandp(self):
		rospy.loginfo("Unfold arm safely")
		pmg = PlayMotionGoal()
		pmg.motion_name = 'pregrasp'
		pmg.skip_planning = False
		self.play_m_as.send_goal_and_wait(pmg)
		rospy.loginfo("Done.")
		rospy.loginfo("Robot prepared.")

	def prepare_robot_nav(self):
		# Move torso to its maximum height
		self.move_torso("lift")

		# Raise arm
		rospy.loginfo("Moving arm to a safe pose")
		pmg = PlayMotionGoal()
		pmg.motion_name = 'pick_final_pose'
		pmg.skip_planning = False
		self.play_m_as.send_goal_and_wait(pmg)
		rospy.loginfo("Raise object done.")

	def aruco_pose_cb(self, aruco_pose_msg):
		self.aruco_pose = aruco_pose_msg
		self.aruco_pose_rcv = True



if __name__ == '__main__':
	rospy.init_node('manipulation_client')
	sphere = SphericalService()
	rospy.spin()

