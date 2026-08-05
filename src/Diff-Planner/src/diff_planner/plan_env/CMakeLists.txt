cmake_minimum_required(VERSION 2.8.3)
project(plan_env)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(OpenCV REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  cv_bridge
  message_filters
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

# 7.28
# catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES plan_env
#  CATKIN_DEPENDS roscpp std_msgs
# #  DEPENDS system_lib
# )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES diff_plan_env_target
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

## 2026-07-07: 在联合工作空间里避开 FUEL/fuel_plan_env 导出的同名 CMake target 冲突，但仍输出 plan_env 库名供原依赖使用。
add_library( diff_plan_env_target 
    src/grid_map.cpp 
    src/raycast.cpp
    )
set_target_properties(diff_plan_env_target PROPERTIES OUTPUT_NAME plan_env)

target_link_libraries( diff_plan_env_target
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    )  
