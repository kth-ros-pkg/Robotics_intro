#!/usr/bin/env python

"""
Created on 25/07/16
@author: Sammy Pfeiffer

Grasp controller to close with a determined error on position only
so to skip overheating.

"""

import rospy
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import Empty, EmptyResponse
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure


class GripperGrasper(object):
    def __init__(self):
        rospy.loginfo("Initializing GripperGrasper...")
        # This node Dynamic params
        self.ddr = DDynamicReconfigure("grasper")
        self.max_position_error = self.ddr.add_variable("max_position_error",
                                                        "Max absolute value of controller state of any joint to stop closing",
                                                        0.002, 0.00001, 0.045)
        self.timeout = self.ddr.add_variable("timeout",
                                             "timeout for the closing action",
                                             5.0, 0.0, 30.0)
        self.closing_time = self.ddr.add_variable("closing_time",
                                                  "Time for the closing goal",
                                                  2.0, 0.01, 30.0)
        self.rate = self.ddr.add_variable("rate",
                                          "Rate Hz at which the node closing will do stuff",
                                          5, 1, 50)

        self.ddr.start(self.ddr_cb)
        rospy.loginfo("Initialized dynamic reconfigure on: " + str(rospy.get_name()))

        # Subscriber to the gripper state
        self.last_state = None
        self.state_sub = rospy.Subscriber('/gripper_controller/state',
                                          JointTrajectoryControllerState,
                                          self.state_cb,
                                          queue_size=1)
        rospy.loginfo("Subscribed to topic: " + str(self.state_sub.resolved_name))

        # Publisher on the gripper command topic
        self.cmd_pub = rospy.Publisher('/gripper_controller/command',
                                       JointTrajectory,
                                       queue_size=1)
        rospy.loginfo("Publishing to topic: " + str(self.cmd_pub.resolved_name))

        # Grasping service to offer
        self.grasp_srv = rospy.Service('/gripper_controller/grasp',
                                       Empty,
                                       self.grasp_cb)
        rospy.loginfo("Offering grasp service on: " + str(self.grasp_srv.resolved_name))
        rospy.loginfo("Done initializing GripperGrasper!")

    def ddr_cb(self, config, level):
        self.max_position_error = config['max_position_error']
        self.timeout = config['timeout']
        self.closing_time = config['closing_time']
        self.rate = config['rate']
        return config


    def state_cb(self, data):
        self.last_state = data

    def grasp_cb(self, req):
        rospy.logdebug("Received grasp request!")
        # From wherever we are close gripper

        # Keep closing until the error of the state reaches
        # max_position_error on any of the gripper joints
        # or we reach timeout
        initial_time = rospy.Time.now()
        closing_amount = [0.0, 0.0]
        # Initial command, wait for it to do something
        self.send_close(closing_amount)
        rospy.sleep(self.closing_time)
        r = rospy.Rate(self.rate)
        on_optimal_close = False
        while not rospy.is_shutdown() and (rospy.Time.now() - initial_time) < rospy.Duration(self.timeout) and not on_optimal_close:
            if -self.last_state.error.positions[0] > self.max_position_error:
                rospy.logdebug("Over error joint 0...")
                closing_amount = self.get_optimal_close()
                on_optimal_close = True

            elif -self.last_state.error.positions[1] > self.max_position_error:
                rospy.logdebug("Over error joint 1...")
                closing_amount = self.get_optimal_close()
                on_optimal_close = True
            self.send_close(closing_amount)
            r.sleep()

        return EmptyResponse()

    def get_optimal_close(self):
        optimal_0 = self.last_state.actual.positions[0] - self.max_position_error
        optimal_1 = self.last_state.actual.positions[1] - self.max_position_error
        return [optimal_0, optimal_1]

    def send_close(self, closing_amount):
        rospy.loginfo("Closing: " + str(closing_amount))
        jt = JointTrajectory()
        jt.joint_names = ['gripper_right_finger_joint',
                          'gripper_left_finger_joint']
        p = JointTrajectoryPoint()
        p.positions = closing_amount
        p.time_from_start = rospy.Duration(self.closing_time)
        jt.points.append(p)

        self.cmd_pub.publish(jt)


if __name__ == '__main__':
    rospy.init_node('gripper_grasping')
    gg = GripperGrasper()
    rospy.spin()
