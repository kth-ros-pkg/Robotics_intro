cmake_minimum_required(VERSION 2.8.3)
project(pal_hardware_interfaces)

find_package(catkin REQUIRED COMPONENTS hardware_interface)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES pal_hardware_interfaces
   CATKIN_DEPENDS hardware_interface
#  DEPENDS system_lib
)

install(DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h*"
)
