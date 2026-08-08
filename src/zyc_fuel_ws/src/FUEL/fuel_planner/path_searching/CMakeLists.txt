cmake_minimum_required(VERSION 2.8.3)
## 2026-07-07: 为了和 Diff-Planner 的同名 path_searching 共存，FUEL 侧改成 fuel_path_searching 包名，库目标仍保留 path_searching。
project(fuel_path_searching)

set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  fuel_plan_env
  cv_bridge
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES fuel_path_searching
 CATKIN_DEPENDS fuel_plan_env
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

add_library( fuel_path_searching
    src/kinodynamic_astar.cpp 
    # src/astar.cpp
    src/astar2.cpp
    src/topo_prm.cpp
    )
target_link_libraries( fuel_path_searching
    ${catkin_LIBRARIES} 
    )  
