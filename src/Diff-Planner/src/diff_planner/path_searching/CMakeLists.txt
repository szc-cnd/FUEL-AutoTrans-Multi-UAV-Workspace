cmake_minimum_required(VERSION 2.8.3)
project(path_searching)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  plan_env
  cv_bridge
)

# 7.28
# catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES path_searching
#  CATKIN_DEPENDS plan_env
# #  DEPENDS system_lib
# )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES diff_path_searching_target
  CATKIN_DEPENDS plan_env
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

## 2026-07-07: 在联合工作空间里避开 FUEL/fuel_path_searching 导出的同名 CMake target 冲突，但仍输出 path_searching 库名供原依赖使用。
add_library( diff_path_searching_target 
    src/dyn_a_star.cpp
    )
set_target_properties(diff_path_searching_target PROPERTIES OUTPUT_NAME path_searching)
target_link_libraries( diff_path_searching_target
    ${catkin_LIBRARIES} 
    )  
