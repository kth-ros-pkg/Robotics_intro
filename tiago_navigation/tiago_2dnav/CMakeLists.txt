cmake_minimum_required(VERSION 2.8.3)
project(tiago_2dnav)

find_package(catkin REQUIRED)

catkin_package()

install(
    PROGRAMS
        scripts/record_mapping.sh
        scripts/record_move_base.sh
        scripts/record_2dnav.sh
        scripts/record_odom.sh
        scripts/record_patrol.sh
        scripts/record_rgbd.sh
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

foreach(dir launch config)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
