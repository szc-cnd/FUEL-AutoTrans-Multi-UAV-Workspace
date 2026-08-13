cmake_minimum_required(VERSION 2.8.3)
## 2026-07-07: 为了和 Diff-Planner 的同名 plan_env 共存，FUEL 侧改成 fuel_plan_env 包名，库目标仍保留 plan_env 以减少源码改动。
project(fuel_plan_env)

set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

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

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES fuel_plan_env
 CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
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


add_library( fuel_plan_env
    src/sdf_map.cpp 
    src/obj_predictor.cpp 
    src/edt_environment.cpp 
    src/raycast.cpp
    src/map_ros.cpp
    )
target_link_libraries( fuel_plan_env
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    )  

add_executable(obj_generator
    src/obj_generator.cpp 
    )
target_link_libraries(obj_generator 
    ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  add_executable(metric_voxel_policy_test
      test/metric_voxel_policy_test.cpp)
  add_test(NAME metric_voxel_policy_test COMMAND metric_voxel_policy_test)
endif()
