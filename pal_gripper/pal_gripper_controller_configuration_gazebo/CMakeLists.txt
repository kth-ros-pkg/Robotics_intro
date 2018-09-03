cmake_minimum_required(VERSION 2.8.3)
project(pal_gripper_controller_configuration_gazebo)

find_package(catkin REQUIRED)
catkin_package()

foreach(dir launch)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

install(PROGRAMS scripts/home_gripper.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

