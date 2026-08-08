cmake_minimum_required(VERSION 2.8.3)
## 2026-07-07: 为了和 Diff-Planner 的同名 traj_utils 共存，FUEL 侧改成 fuel_traj_utils 包名，库目标仍保留 traj_utils。
project(fuel_traj_utils)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
  bspline
  bspline_opt
  fuel_path_searching
  poly_traj
  roscpp
  std_msgs
  cv_bridge
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES fuel_traj_utils
 CATKIN_DEPENDS fuel_path_searching bspline bspline_opt poly_traj
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_library( fuel_traj_utils
    src/planning_visualization.cpp 
    )
target_link_libraries( fuel_traj_utils
    ${catkin_LIBRARIES} 
    )  

add_executable(process_msg 
    src/process_msg.cpp
)
target_link_libraries( process_msg
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    )  
