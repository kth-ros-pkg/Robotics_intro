cmake_minimum_required(VERSION 2.8.3)
project(dynamic_introspection)

find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)

set(CATKIN_COMPONENTS 
  roscpp
  message_generation
  rosbag
  pluginlib
  control_toolbox
  visualization_msgs
  backward_ros
  std_msgs)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_COMPONENTS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

add_message_files(DIRECTORY msg FILES BoolParameter.msg
  IntrospectionMsg.msg
  DoubleParameter.msg
  IntParameter.msg
  MarkerParameter.msg)


generate_messages(DEPENDENCIES visualization_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${CATKIN_COMPONENTS}
  DEPENDS EIGEN3
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  )

add_library(${PROJECT_NAME} src/dynamic_introspection.cpp
                            src/dynamic_introspection_utils.cpp
                            src/introspection_online_utils.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(dynamic_introspection_example test/dynamic_introspection_example.cpp)
target_link_libraries(dynamic_introspection_example ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(introspection_bag_analyzer src/introspection_bag_analyzer.cpp)
target_link_libraries(introspection_bag_analyzer ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(dynamic_introspection_bag_example test/dynamic_introspection_bag_example.cpp)
target_link_libraries(dynamic_introspection_bag_example ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(dynamic_introspection_rosbag_recording_server src/dynamic_introspection_rosbag_recording_server.cpp)
target_link_libraries(dynamic_introspection_rosbag_recording_server ${PROJECT_NAME} ${catkin_LIBRARIES})

add_subdirectory(test)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

install(DIRECTORY matlab/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

