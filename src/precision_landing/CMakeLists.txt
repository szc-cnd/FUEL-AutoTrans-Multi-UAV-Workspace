cmake_minimum_required(VERSION 3.0.2)
project(precision_landing)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  roscpp rospy std_msgs std_srvs sensor_msgs geometry_msgs mavros_msgs
  cv_bridge image_transport camera_info_manager rostest
  tf2 tf2_geometry_msgs
)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d aruco)
find_package(Threads REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs std_srvs sensor_msgs geometry_msgs mavros_msgs
                 cv_bridge image_transport camera_info_manager
                 tf2 tf2_geometry_msgs
  DEPENDS EIGEN3 OpenCV
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_library(landing_controller src/landing_controller.cpp)
target_link_libraries(landing_controller ${catkin_LIBRARIES})

add_library(landing_state_machine src/landing_state_machine.cpp)
target_link_libraries(landing_state_machine ${catkin_LIBRARIES})

add_library(aruco_tracker src/aruco_tracker.cpp)
target_link_libraries(aruco_tracker ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(precision_landing_node src/precision_landing_node.cpp)
target_link_libraries(precision_landing_node
  landing_controller landing_state_machine aruco_tracker
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

add_executable(landing_test_mission_node src/landing_test_mission_node.cpp)
target_link_libraries(landing_test_mission_node ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS test/test_precision_landing_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS precision_landing_node landing_test_mission_node
        landing_controller landing_state_machine aruco_tracker
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(PROGRAMS scripts/start_landing_test.sh scripts/start_onboard_landing.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  add_rostest(test/precision_landing_node.test)
  catkin_add_nosetests(test/test_config.py)

  catkin_add_gtest(test_landing_controller
    test/test_landing_controller.cpp test/gtest_main.cpp)
  if(TARGET test_landing_controller)
    target_link_libraries(test_landing_controller landing_controller ${catkin_LIBRARIES})
  endif()

  catkin_add_gtest(test_landing_state_machine
    test/test_landing_state_machine.cpp test/gtest_main.cpp)
  if(TARGET test_landing_state_machine)
    target_link_libraries(test_landing_state_machine landing_state_machine ${catkin_LIBRARIES})
  endif()

  catkin_add_gtest(test_mode_request_authorization
    test/test_mode_request_authorization.cpp test/gtest_main.cpp)
  if(TARGET test_mode_request_authorization)
    target_link_libraries(test_mode_request_authorization ${catkin_LIBRARIES})
  endif()

  catkin_add_gtest(test_aruco_tracker
    test/test_aruco_tracker.cpp test/gtest_main.cpp)
  if(TARGET test_aruco_tracker)
    target_link_libraries(test_aruco_tracker aruco_tracker ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
  endif()
endif()
