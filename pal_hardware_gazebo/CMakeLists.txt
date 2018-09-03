cmake_minimum_required(VERSION 2.8.3)
project(pal_hardware_gazebo)

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  angles
  control_toolbox
  hardware_interface
  joint_limits_interface
  gazebo_ros_control
  pal_hardware_interfaces
  dynamic_introspection
)

find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

catkin_package(
    INCLUDE_DIRS include ${GAZEBO_INCLUDE_DIRS}
    CATKIN_DEPENDS control_toolbox hardware_interface joint_limits_interface gazebo_ros_control
    DEPENDS gazebo Eigen
    LIBRARIES ${PROJECT_NAME}
)

include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} SYSTEM ${EIGEN_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(${PROJECT_NAME} src/pal_hardware_gazebo.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${EIGEN_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)
install (FILES pal_hardware_gazebo_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
