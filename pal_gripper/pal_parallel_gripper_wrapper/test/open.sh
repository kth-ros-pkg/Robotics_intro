#!/bin/bash

rostopic pub /parallel_gripper_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names: ['parallel_gripper_joint']
points:
- positions: [0.096]
  velocities: []
  accelerations: []
  effort: []
  time_from_start: {secs: 2, nsecs: 0}" --once
