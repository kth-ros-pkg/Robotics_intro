#!/bin/bash

rosbag record --split --size=1024 /tf /mobile_base_controller/cmd_vel /mobile_base_controller/odom /joy_vel /nav_vel /twist_mux/cmd_vel /joint_states

