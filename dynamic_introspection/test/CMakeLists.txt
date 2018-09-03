add_executable(singleton_test singleton_test.cpp)
target_link_libraries(singleton_test ${catkin_LIBRARIES})
add_dependencies(singleton_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(test_class_a test_class_a.cpp)
target_link_libraries(test_class_a ${PROJECT_NAME} ${catkin_LIBRARIES})

add_library(test_class_b test_class_b.cpp)
target_link_libraries(test_class_b ${PROJECT_NAME} ${catkin_LIBRARIES})

add_library(test_class_plugins test_class_plugins.cpp)
target_link_libraries(test_class_plugins test_class_a test_class_b ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(test_class_plugin_main test_class_plugin_main.cpp)
target_link_libraries(test_class_plugin_main ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(test_class_main test_class_main.cpp)
target_link_libraries(test_class_main test_class_a test_class_b ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(thread_safe_test thread_safe/thread_safe_test.cpp)
target_link_libraries(thread_safe_test ${PROJECT_NAME} ${catkin_LIBRARIES})
