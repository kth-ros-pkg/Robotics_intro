cmake_minimum_required(VERSION 2.8.3)
project(pal_motion_model_msgs)

find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    nav_msgs
    std_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
    MotionModelList.msg
    MotionModelMap.msg
    MotionModel.msg
)

add_service_files(
  DIRECTORY srv
  FILES
    GetMotionMap.srv
)

generate_messages(DEPENDENCIES nav_msgs std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime nav_msgs std_msgs)
