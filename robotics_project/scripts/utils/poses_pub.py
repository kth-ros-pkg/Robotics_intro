#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseArray, PoseStamped
from IPython import embed

class NavPosesPublisher(object):
    def __init__(self):
        self.pick_pose = rospy.get_param(rospy.get_name() + '/pick_pose')
        self.place_pose = rospy.get_param(rospy.get_name() + '/place_pose')
        self.pick_pose_top = rospy.get_param(rospy.get_name() + '/pick_pose_topic')
        self.place_pose_top = rospy.get_param(rospy.get_name() + '/place_pose_topic')
        self.frame = rospy.get_param(rospy.get_name() + '/map_frame')

        # Extract pick and place poses from ROS params
        self.pick_pose = self.pick_pose.split(',')
        self.place_pose = self.place_pose.split(',')
        self.poses_publisher()

    def poses_publisher(self):
               
        # Pick pose
        pick_pose_msg = PoseStamped()
        pick_pose_msg.header.frame_id = self.frame
        pick_pose_msg.pose.position.x = float(self.pick_pose[0])
        pick_pose_msg.pose.position.y = float(self.pick_pose[1])
        pick_pose_msg.pose.position.z = float(self.pick_pose[2])
        pick_pose_msg.pose.orientation.x = float(self.pick_pose[3])
        pick_pose_msg.pose.orientation.y = float(self.pick_pose[4])
        pick_pose_msg.pose.orientation.z = float(self.pick_pose[5])
        pick_pose_msg.pose.orientation.w = float(self.pick_pose[6])
        
        # Place pose
        place_pose_msg = PoseStamped()
        place_pose_msg.header.frame_id = self.frame
        place_pose_msg.pose.position.x = float(self.place_pose[0])
        place_pose_msg.pose.position.y = float(self.place_pose[1])
        place_pose_msg.pose.position.z = float(self.place_pose[2])
        place_pose_msg.pose.orientation.x = float(self.place_pose[3])
        place_pose_msg.pose.orientation.y = float(self.place_pose[4])
        place_pose_msg.pose.orientation.z = float(self.place_pose[5])
        place_pose_msg.pose.orientation.w = float(self.place_pose[6])

        pick_pub = rospy.Publisher(self.pick_pose_top, PoseStamped, queue_size=10)
        place_pub = rospy.Publisher(self.place_pose_top, PoseStamped, queue_size=10)
        rate = rospy.Rate(1) # 1hz
        
        # Main loop
        while not rospy.is_shutdown():

            pick_pose_msg.header.stamp = rospy.Time.now()
            place_pose_msg.header.stamp = rospy.Time.now()
            
            pick_pub.publish(pick_pose_msg)
            place_pub.publish(place_pose_msg)
            
            rate.sleep()

if __name__ == '__main__':

    rospy.init_node('poses_publisher')
    
    try:
        NavPosesPublisher()
    except rospy.ROSInterruptException:
        pass