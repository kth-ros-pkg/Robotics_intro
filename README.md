# Robotics_intro

Packages for the final project of the course DD2410 "Introduction to Robotics".
Project based on the PAL Robotics platform TIAGo.

## Install and run
Clone the repository inside your catkin workspace, install the dependencies, build it and run it:
For this instructions, we'll assumed you have created a ws called catkin_ws. Run the following commands in a terminal:

$ cd ~/catkin_ws/src
$ git clone "this_repo_link" "folder_name"
cd ..
$ rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO
$ catkin_make -DCATKIN_ENABLE_TESTING=0
$ source devel/setup.bash

In order to run the system:
$ roslaunch robotics_project launch_project.launch

You should be able to visualize the system in both Rviz and Gazebo and then you're ready to start to work.
See the instructions for the project in Canvas.
