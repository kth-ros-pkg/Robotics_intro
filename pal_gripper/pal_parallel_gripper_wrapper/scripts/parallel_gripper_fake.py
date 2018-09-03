#!/usr/bin/env python

"""Action server wrapping
the real follow joint trajectory action server
that contains two joints, to just show one joint
to control a gripper"""

import rospy
import actionlib
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, \
    FollowJointTrajectoryGoal, \
    FollowJointTrajectoryResult, \
    FollowJointTrajectoryFeedback, \
    JointTrajectoryControllerState


class WrappedFJTA(object):

    def __init__(self):
        self.controller_name = rospy.get_param("~controller_name", None)
        self.pub_rate = rospy.get_param("~pub_rate", 50)
        if not self.controller_name:
            rospy.logerr("No controller name found in param: ~controller_name")
            exit(0)
        self.real_controller_name = rospy.get_param(
            "~real_controller_name", None)
        if not self.real_controller_name:
            rospy.logerr(
                "No real controller name given in param: ~real_controller_name")
            exit(0)
        self.real_joint_names = rospy.get_param("~real_joint_names", None)
        if not self.real_joint_names:
            rospy.logerr(
                "No real joint names given in param: ~real_joint_names")
            exit(0)
        self.gripper_joint_name = rospy.get_param("~gripper_joint_name", None)
        if not self.gripper_joint_name:
            rospy.logerr(
                "No gripper joint name given in param: ~gripper_joint_name")
            exit(0)

        self._cmd_sub = rospy.Subscriber('/' + self.controller_name + '_controller/command',
                                         JointTrajectory,
                                         self.cmd_cb,
                                         queue_size=5)
        rospy.loginfo("Subscribed to " + self._cmd_sub.resolved_name)
        self._last_state = None
        self._state_sub = rospy.Subscriber('/' + self.real_controller_name + '_controller/state',
                                           JointTrajectoryControllerState,
                                           self.state_cb,
                                           queue_size=1)
        rospy.loginfo("Subscribed to " + self._state_sub.resolved_name)

        self._state_pub = rospy.Publisher('/' + self.controller_name + '_controller/state',
                                          JointTrajectoryControllerState,
                                          queue_size=5)
        rospy.loginfo("Publishing to " + self._state_pub.resolved_name)
        self._state_timer = rospy.Timer(rospy.Duration(1.0 / self.pub_rate),
                                        self.pub_state)
        rospy.loginfo("At a rate of " + str(self.pub_rate))

        self._cmd_pub = rospy.Publisher('/' + self.real_controller_name + '_controller/command',
                                        JointTrajectory,
                                        queue_size=5)
        rospy.loginfo("Publishing to " + self._cmd_pub.resolved_name)

        # Create ActionClient to the real server
        self._ac = actionlib.SimpleActionClient('/' + self.real_controller_name + '_controller/follow_joint_trajectory',
                                                FollowJointTrajectoryAction)
        rospy.loginfo("Created SimpleActionClient to " + '/' +
                      self.real_controller_name + '_controller/follow_joint_trajectory')
        self._ac.wait_for_server()

        self._as = actionlib.SimpleActionServer('/' + self.controller_name + '_controller/follow_joint_trajectory',
                                                FollowJointTrajectoryAction,
                                                execute_cb=self.execute_cb,
                                                auto_start=False)
        rospy.loginfo("FollowJointTrajectory AS set at " + '/' +
                      self.controller_name + '_controller/follow_joint_trajectory')
        self._as.start()

    def cmd_cb(self, trajectory):
        """
        :type data: JointTrajectory
        """
        rospy.loginfo("Got a command goal!")
        # Just resend goal to the real cmd topic
        # With two joints instead of one
        jt = self.substitute_trajectory(trajectory)

        self._cmd_pub.publish(jt)

    def substitute_trajectory(self, trajectory):
        jt = JointTrajectory()
        jt.joint_names = self.real_joint_names
        jt.header = trajectory.header
        for jtp_in in trajectory.points:
            jtp = JointTrajectoryPoint()
            if jtp_in.positions:
                joint_position = jtp_in.positions[0]
                # Each finger will go to half of the size
                jtp.positions = [joint_position / 2.0, joint_position / 2.0]
                jtp.time_from_start = jtp_in.time_from_start
            else:
                rospy.logwarn("Trajectory has no positions...")
            if jtp_in.velocities:
                jtp.velocities.append(jtp_in.velocities[0])
            if jtp_in.accelerations:
                jtp.accelerations.append(jtp_in.accelerations[0])
            if jtp_in.effort:
                jtp.effort.append(jtp_in.effort[0])
            jt.points.append(jtp)
        return jt

    def state_cb(self, data):
        """
        :type data: JointTrajectoryControllerState
        """
        self._last_state = data

    def execute_cb(self, goal):
        """
        :type goal: FollowJointTrajectoryGoal
        """
        rospy.loginfo("Got a goal!")
        # Just resend the goal to the real as
        # while checking for cancel goals

        # Create goal
        g = FollowJointTrajectoryGoal()
        # We ignore path_tolerance and goal_tolerance... does not make sense in
        # a gripper
        g.goal_time_tolerance = goal.goal_time_tolerance
        jt = self.substitute_trajectory(goal.trajectory)
        g.trajectory = jt

        if not self._ac.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Action server for gripper not found!")
            self._as.set_aborted(FollowJointTrajectoryResult())

        # Send goal subscribing to feedback to republish it
        self._ac.send_goal(g, feedback_cb=self._feedback_cb)

        # wait for goal to finish while checking if cancel is requested
        while not self._ac.wait_for_result(rospy.Duration(0.1)):
            rospy.loginfo("Waiting for goal to finish...")
            # Deal with goal cancelled
            if self._as.is_preempt_requested():
                self._ac.cancel_all_goals()
                self._as.set_preempted()
                return

        result = self._ac.get_result()
        res = FollowJointTrajectoryResult()
        if result:
            res.error_code = FollowJointTrajectoryResult.SUCCESSFUL
            self._as.set_succeeded(res)
        else:
            res.error_code = result
            self._as.set_aborted(res)

    def _feedback_cb(self, fb_msg):
        fb = self.state_msg(fb_msg, 'feedback')
        self._as.publish_feedback(fb)

    def pub_state(self, _):
        # Just repub the last state but with one joint stuff
        if self._last_state:
            st = self.state_msg(self._last_state, 'state')

            self._state_pub.publish(st)

    def state_msg(self, ref_state, type_msg):
        if type_msg == 'state':
            st = JointTrajectoryControllerState()
        elif type_msg == 'feedback':
            st = FollowJointTrajectoryFeedback()
        st.joint_names = [self.gripper_joint_name]
        st.header = ref_state.header

        st.desired.positions.extend(
            [p * 2.0 for p in ref_state.desired.positions[:1]])
        st.desired.velocities.extend(ref_state.desired.velocities[:1])
        st.desired.accelerations.extend(ref_state.desired.accelerations[:1])
        st.desired.effort.extend(ref_state.desired.effort[:1])
        st.desired.time_from_start = ref_state.desired.time_from_start

        st.actual.positions.extend(
            [p * 2.0for p in ref_state.actual.positions[:1]])
        st.actual.velocities.extend(ref_state.actual.velocities[:1])
        st.actual.accelerations.extend(ref_state.actual.accelerations[:1])
        st.actual.effort.extend(ref_state.actual.effort[:1])
        st.actual.time_from_start = ref_state.actual.time_from_start

        st.error.positions.extend(
            [p * 2.0 for p in ref_state.error.positions[:1]])
        st.error.velocities.extend(ref_state.error.velocities[: 1])
        st.error.accelerations.extend(ref_state.error.accelerations[: 1])
        st.error.effort.extend(ref_state.error.effort[: 1])

        return st


if __name__ == '__main__':
    rospy.init_node('wrapped_fjta')
    wfjta = WrappedFJTA()
    rospy.spin()
