cmake_minimum_required(VERSION 2.8.3)
project(pal_behaviour_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs actionlib_msgs)

add_action_files(
  DIRECTORY action
  FILES
    BehaviourTask.action
    Presentation.action
)

add_message_files(
  DIRECTORY msg
  FILES
    BehaviourArgument.msg
)

add_service_files(
  DIRECTORY srv
  FILES
    BehaviourEvent.srv
)

generate_messages(DEPENDENCIES std_msgs actionlib_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs actionlib_msgs)
