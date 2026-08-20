cmake_minimum_required(VERSION 2.8.3)
project(autotrans_reference_bridge)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  traj_utils
  quadrotor_msgs
  bspline
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs traj_utils quadrotor_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(reference_bridge_node
  src/reference_bridge_node.cpp
)
add_dependencies(reference_bridge_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(reference_bridge_node ${catkin_LIBRARIES})

add_executable(fuel_autotrans_bridge_node
  src/fuel_autotrans_bridge_node.cpp
)
add_dependencies(fuel_autotrans_bridge_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(fuel_autotrans_bridge_node ${catkin_LIBRARIES})

add_executable(autotrans_trajectory_arbiter_node
  src/autotrans_trajectory_arbiter_node.cpp
)
add_dependencies(autotrans_trajectory_arbiter_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(autotrans_trajectory_arbiter_node ${catkin_LIBRARIES})

install(TARGETS reference_bridge_node fuel_autotrans_bridge_node
  autotrans_trajectory_arbiter_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
