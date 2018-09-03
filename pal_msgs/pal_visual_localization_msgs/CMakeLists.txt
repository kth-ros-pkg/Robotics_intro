cmake_minimum_required(VERSION 2.8.3)
project(pal_visual_localization_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  message_generation
  std_msgs
)

## Generate actions in the 'action' folder
add_action_files(
   FILES
   VisualLocAddPlace.action
    VisualLocRecognize.action
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs std_msgs actionlib_msgs
)


catkin_package(CATKIN_DEPENDS message_runtime)


include_directories(
  ${catkin_INCLUDE_DIRS}
)


