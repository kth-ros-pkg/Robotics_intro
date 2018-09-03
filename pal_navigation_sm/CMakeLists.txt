cmake_minimum_required(VERSION 2.8.3)
project(pal_navigation_sm)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

install(
  PROGRAMS
    scripts/map_setup.py
    scripts/pal_navigation_main_sm.py
    scripts/navigation.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

foreach(dir config launch)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()
