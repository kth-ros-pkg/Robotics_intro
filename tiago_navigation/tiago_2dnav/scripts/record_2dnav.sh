#!/bin/bash

rosbag record -o 2dnav --split --size=1024 /tf /scan /base_rgbd_camera/depth/image_raw /base_rgbd_camera/depth/camera_info /base_rgbd_camera/depth/points /floor_filter_xtion/filtered_cloud /mobile_base_controller/odom

