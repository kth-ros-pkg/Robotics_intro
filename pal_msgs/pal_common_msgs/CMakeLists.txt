cmake_minimum_required(VERSION 2.8.3)
project(pal_common_msgs)

find_package(catkin REQUIRED COMPONENTS actionlib_msgs std_msgs message_generation)

add_action_files(
  DIRECTORY action
  FILES
    Disable.action
    Empty.action
)

add_message_files(
  DIRECTORY msg
  FILES
    JointCurrent.msg
    )

generate_messages(DEPENDENCIES actionlib_msgs std_msgs)

catkin_package(CATKIN_DEPENDS actionlib_msgs std_msgs message_runtime)
