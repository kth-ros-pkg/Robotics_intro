# Robotics_intro

Packages for the final project of the course DD2410 "Introduction to Robotics".
Project based on the PAL Robotics platform TIAGo.

## Install
Clone this repository inside your catkin workspace, install the dependencies, build it and run it.
For these instructions, we'll assumed you have created a ws called catkin_ws. 
Run the following commands in a terminal:
```
$ cd ~/catkin_ws/src
$ git clone "this_repo_link" "folder_name"
$ cd 
$ rosdep install --from-paths catkin_ws --ignore-src --rosdistro=$ROS_DISTRO
$ cd ~/catkin_ws
$ catkin_make -DCATKIN_ENABLE_TESTING=0
$ source devel/setup.bash
```
## Run
In order to run the system:
```
$ roslaunch robotics_project gazebo_project.launch
$ roslaunch robotics_project launch_project.launch
```
You should be able to visualize the system in both Rviz and Gazebo and then you're ready to start to work.
See the instructions for the project in Canvas.

## License

This project is licensed under the Modified BSD License - see [LICENSE](https://opensource.org/licenses/BSD-3-Clause) for details

## Acknowledgments

* Based on packages from [PAL Robotics](http://www.pal-robotics.com/en/home/)
