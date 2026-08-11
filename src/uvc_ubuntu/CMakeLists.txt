cmake_minimum_required(VERSION 3.0.2)
project(uvc_ubuntu)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  rospy
  sensor_msgs
  std_msgs
  tf2_geometry_msgs
  tf2_ros
)
find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

set(BUILD_EXAMPLE OFF CACHE BOOL "Disable bundled libuvc examples" FORCE)
set(BUILD_TEST OFF CACHE BOOL "Disable bundled libuvc tests" FORCE)
add_subdirectory(libuvc EXCLUDE_FROM_ALL)

catkin_package()

add_executable(uvc_demo
  main.c
  hik_sensor.c
  hik_capture.c
  uvc_camera.c
)
target_include_directories(uvc_demo PRIVATE
  ${catkin_INCLUDE_DIRS}
  ${LIBUSB_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/libuvc/include
  ${CMAKE_CURRENT_BINARY_DIR}/libuvc/include
)
target_compile_options(uvc_demo PRIVATE
  -Wall
  -O2
  -DDEBUG
  ${LIBUSB_CFLAGS_OTHER}
)
target_link_libraries(uvc_demo
  ${LIBUSB_LIBRARIES}
  LibUVC::UVCStatic
  Threads::Threads
)

catkin_install_python(PROGRAMS
  thermal_detect.py
  thermal_raw_view.py
  thermal_ros_node.py
  thermal_d435_fusion_node.py
  thermal_d435_calib_node.py
  check_uvc_fps.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS uvc_demo
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
