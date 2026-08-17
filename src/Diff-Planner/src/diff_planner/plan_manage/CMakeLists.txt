cmake_minimum_required(VERSION 2.8.3)
project(diff_planner)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  ldop
  quadrotor_msgs
  plan_env
  path_searching
  traj_opt
  traj_utils
  message_generation
  cv_bridge
  pcl_conversions
)

# catkin_package(CATKIN_DEPENDS message_runtime)
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES diff_planner
 CATKIN_DEPENDS ldop plan_env path_searching traj_opt traj_utils
#  DEPENDS system_lib
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)


add_executable(diff_planner_node
  src/diff_planner_node.cpp 
  src/diff_replan_fsm.cpp
  src/swing_obstacle_guard.cpp
  src/planner_manager.cpp
  )
target_link_libraries(diff_planner_node 
  ${catkin_LIBRARIES}
  ## 2026-07-07: 显式链接 Diff 侧唯一的可视化库，避免联合工作空间里误连到其他 traj_utils 实现。
  diff_traj_utils
  )
add_dependencies(diff_planner_node ${ldop_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(diff_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

## 2026-07-07: 在联合工作空间里避开 FUEL/plan_manage 的同名 CMake target 冲突，但仍输出 diff_planner/traj_server 可执行文件供原 launch 使用。
add_executable(diff_traj_server_target src/traj_server.cpp)
set_target_properties(diff_traj_server_target PROPERTIES OUTPUT_NAME traj_server)
target_link_libraries(diff_traj_server_target ${catkin_LIBRARIES})
# add_dependencies(diff_traj_server_target ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(goal_height_bridge src/goal_height_bridge.cpp)
target_link_libraries(goal_height_bridge ${catkin_LIBRARIES})

## 2026-07-08: 新增结构感知的走廊搜索管理器，用局部雷达点云给 Diff-Planner 提供“沿通道推进”的子目标，避免未知开阔区追地图边界。
add_executable(corridor_search_manager src/corridor_search_manager.cpp)
target_link_libraries(corridor_search_manager ${catkin_LIBRARIES} ${PCL_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_swing_obstacle_guard
    test/test_swing_obstacle_guard.cpp
    src/swing_obstacle_guard.cpp
  )
  if(TARGET test_swing_obstacle_guard)
    target_link_libraries(test_swing_obstacle_guard ${catkin_LIBRARIES})
  endif()
endif()
# add_executable(monitor_node src/monitor_node.cpp)
# target_link_libraries(monitor_node ${catkin_LIBRARIES})
