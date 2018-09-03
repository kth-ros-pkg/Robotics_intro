cmake_minimum_required(VERSION 2.8.3)
project(pal_web_msgs)

find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    std_msgs
)


add_message_files(
  DIRECTORY msg
  FILES
  WebGoTo.msg
)

#add_action_files(
  #DIRECTORY action
  #FILES
    #.action
#)

#add_service_files(
  #DIRECTORY srv
  #FILES
    #.srv
#)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)
