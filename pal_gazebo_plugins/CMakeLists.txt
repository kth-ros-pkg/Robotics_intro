cmake_minimum_required(VERSION 2.8.3)
project(pal_gazebo_plugins)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

find_package(catkin REQUIRED
  COMPONENTS
    gazebo_ros
    std_msgs
    std_srvs
    tf
    pal_multirobot_msgs
    roscpp
    control_toolbox
    nav_msgs)

# Depend on system install of Gazebo and SDFormat
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

include_directories(include ${catkin_INCLUDE_DIRS}
                            ${GAZEBO_INCLUDE_DIRS}
                            ${Boost_INCLUDE_DIRS}
                            ${SDFormat_INCLUDE_DIRS})


# Remove wrongly placed link flag in GAZEBO_LIBRARIES
string(REPLACE "-lpthread;" "" GAZEBO_LIBRARIES "${GAZEBO_LIBRARIES}")
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gazebo_ros_forcetorque
            gazebo_pal_hand
            gazebo_wifi_ap
            gazebo_underactuated_finger
  CATKIN_DEPENDS
    std_msgs
    std_srvs
    tf
    pal_multirobot_msgs
    roscpp
    control_toolbox
  DEPENDS
    GAZEBO
    Boost)

link_directories(${GAZEBO_LIBRARY_DIRS})

## Plugins
add_library(gazebo_ros_forcetorque src/gazebo_ros_forcetorque.cpp)
target_link_libraries(gazebo_ros_forcetorque ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

add_library(gazebo_pal_hand src/gazebo_pal_hand.cpp)
target_link_libraries(gazebo_pal_hand ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

add_library(gazebo_wifi_ap src/gazebo_wifi_ap.cpp)
target_link_libraries(gazebo_wifi_ap ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(gazebo_wifi_ap pal_multirobot_msgs_generate_messages_cpp)

add_library(gazebo_harness src/gazebo_harness.cpp)
target_link_libraries(gazebo_harness ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

add_library(gazebo_underactuated_finger src/gazebo_underactuated_finger.cpp)
target_link_libraries(gazebo_underactuated_finger ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

add_library(gazebo_world_odometry src/gazebo_world_odometry.cpp)
target_link_libraries(gazebo_world_odometry ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

add_library(gazebo_attachment src/gazebo_attachment.cpp)
target_link_libraries(gazebo_attachment ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS
  gazebo_ros_forcetorque
  gazebo_pal_hand
  gazebo_wifi_ap
  gazebo_harness
  gazebo_underactuated_finger
  gazebo_world_odometry
  gazebo_attachment
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

install(DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)
