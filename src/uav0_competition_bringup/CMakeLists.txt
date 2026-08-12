cmake_minimum_required(VERSION 3.0.2)
project(uav0_competition_bringup)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS scripts/start_uav0_detection_landing_stack.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
