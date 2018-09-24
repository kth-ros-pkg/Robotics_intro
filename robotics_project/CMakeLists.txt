cmake_minimum_required(VERSION 2.8.3)
project(robotics_project)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  rospy
  message_generation
  std_msgs
  std_srvs
  dynamic_reconfigure
  actionlib
  actionlib_msgs
  aruco_ros
  geometry_msgs
  moveit_msgs
  moveit_commander
)

catkin_python_setup()

add_service_files(
  FILES
  MoveHead.srv
)

add_action_files(
  FILES
  PickUpPose.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs geometry_msgs std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/SphericalGrasp.cfg
  )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robotics_project
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch config
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})