#!/bin/bash

rosbag record -o mapping --split --size=1024 /tf /scan /mobile_base_controller/odom

