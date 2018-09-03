cmake_minimum_required(VERSION 2.8.3)
project(pal_device_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_message_files(
  DIRECTORY msg
  FILES
    BatteryState.msg
    Bumper.msg
    LedGroup.msg
)

add_service_files(
  DIRECTORY srv
  FILES
    CancelEffect.srv
    TimedBlinkEffect.srv
    TimedColourEffect.srv
    TimedFadeEffect.srv
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
