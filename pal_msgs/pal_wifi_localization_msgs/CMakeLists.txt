cmake_minimum_required(VERSION 2.8.3)
project(pal_wifi_localization_msgs)

find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    nav_msgs
    geometry_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
    WifiSignalList.msg
    WifiSignalMap.msg
    WifiSignal.msg
)

add_service_files(
  DIRECTORY srv
  FILES
    GetWifiMap.srv
)

generate_messages(DEPENDENCIES nav_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS message_runtime nav_msgs geometry_msgs)
