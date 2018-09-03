cmake_minimum_required(VERSION 2.8.3)
project(joint_torque_sensor_state_controller)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS realtime_tools hardware_interface pluginlib controller_interface sensor_msgs)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Declare catkin package
catkin_package(
  CATKIN_DEPENDS realtime_tools hardware_interface pluginlib controller_interface sensor_msgs
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  )

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} src/joint_torque_sensor_state_controller.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(joint_state_relay_without_nans src/joint_state_relay_without_nans.cpp)
target_link_libraries(joint_state_relay_without_nans ${catkin_LIBRARIES})

# if(CATKIN_ENABLE_TESTING)
#   find_package(rostest REQUIRED)
#
#   add_rostest_gtest(joint_torque_sensor_state_controller_test
#                     test/joint_state_controller.test
#                     test/joint_state_controller_test.cpp)
#   target_link_libraries(joint_state_controller_test ${PROJECT_NAME})
# endif()

# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES joint_torque_sensor_state_plugin.xml joint_torque_sensor_state_controller.yaml joint_torque_sensor_state_controller.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

