cmake_minimum_required(VERSION 2.8.3)
project(traj_utils)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
# set(ENABLE_PRECOMPILED_HEADERS "OFF")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
)

find_package(Eigen3 REQUIRED)
# find_package(PCL 1.7 REQUIRED) #to help find Eigen

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  DataDisp.msg
  PolyTraj.msg
  MINCOTraj.msg
  )
  
# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
 INCLUDE_DIRS include
 ## 2026-07-07: 在 match 联合工作空间里给 Diff 侧导出唯一库名，避免 devel/lib/libtraj_utils.so 被其他包覆盖后导致错误链接。
 LIBRARIES diff_traj_utils
 CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
)

# link_directories(${PCL_LIBRARY_DIRS})

## 2026-07-07: 在联合工作空间里给 Diff 侧生成唯一的共享库文件名，避免与其他 traj_utils 实现共用 libtraj_utils.so。
add_library( diff_traj_utils
    src/planning_visualization.cpp 
    )
target_link_libraries( diff_traj_utils
    ${catkin_LIBRARIES} 
    )  
