cmake_minimum_required(VERSION 2.8.3)
project(tf_lookup)

find_package(catkin REQUIRED COMPONENTS actionlib actionlib_msgs geometry_msgs message_generation roscpp tf)

catkin_python_setup()

add_action_files(DIRECTORY action FILES TfLookup.action TfStream.action)
add_service_files(FILES lookupTransform.srv)
add_message_files(FILES Subscription.msg)

generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS actionlib_msgs geometry_msgs message_runtime tf
    )

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(tf_lookup
    src/tf_lookup_main.cpp
    src/tf_lookup.cpp
    src/tf_stream.cpp
    src/tf_stream_server.cpp)

target_link_libraries(tf_lookup ${catkin_LIBRARIES})
add_dependencies(tf_lookup ${PROJECT_NAME}_gencpp)

install(TARGETS tf_lookup
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    )
