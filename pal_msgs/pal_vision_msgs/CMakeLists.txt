cmake_minimum_required(VERSION 2.8.3)
project(pal_vision_msgs)

find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    sensor_msgs
    geometry_msgs
    actionlib_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
    DetectedObject.msg
    DetectedPerson.msg
    FaceDetection.msg
    FaceDetections.msg
    FollowMeResponse.msg
    Gesture.msg
    HeadPanTilt.msg
    HogDetection.msg
    HogDetections.msg
    LegDetections.msg
    Rectangle.msg
)

add_service_files(
  DIRECTORY srv
  FILES
    FaceTrackingStart.srv
    FaceTrackingStop.srv
    FollowMeStart.srv
    FollowMeStop.srv
    LookToPixel.srv
)

add_action_files(DIRECTORY action FILES FaceRecognition.action)

generate_messages(DEPENDENCIES sensor_msgs geometry_msgs actionlib_msgs)

catkin_package(CATKIN_DEPENDS message_runtime sensor_msgs geometry_msgs actionlib_msgs)
