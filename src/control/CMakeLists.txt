cmake_minimum_required(VERSION 3.0.2)
project(exploration_control)

## Use C++11
add_compile_options(-std=c++11)

## Find catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  # 2026-07-14: 后机跟随器读取双机里程计、局部点云并发布安全历史路径。
  nav_msgs
  sensor_msgs
  visualization_msgs
  quadrotor_msgs
  tf
  # 使用 MAVROS 状态和位置目标消息；AUTO.LAND 由精降节点请求。
  mavros_msgs
)

# 这里只消费LDOP生成的ROS消息，不链接要求C++20的ldop_core；否则会把
# UFOMap的编译标准传递给本包内仍使用C++11的旧飞控节点。
find_package(ldop REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  # 2026-07-14: 导出后机跟随器新增的里程计、点云和可视化依赖。
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs nav_msgs sensor_msgs visualization_msgs quadrotor_msgs tf mavros_msgs ldop
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${ldop_INCLUDE_DIRS}
)

## Executable node
add_executable(cxr_egoctrl_v1 src/cxr_egoctrl_v1.cpp)

# 2026-07-14: 独立后机管理器只跟随前机已经飞过的安全历史，不复制第二套 FUEL。
add_executable(leader_safe_path_follower src/leader_safe_path_follower.cpp)
add_dependencies(leader_safe_path_follower ${catkin_EXPORTED_TARGETS} ${ldop_EXPORTED_TARGETS})  # 确保LDOP消息先生成再编译跟随器。

## Link against catkin libraries (includes tf)
target_link_libraries(cxr_egoctrl_v1
  ${catkin_LIBRARIES}
)

# 2026-07-14: 链接 ROS 消息、点云迭代器和 TF 相关依赖。
target_link_libraries(leader_safe_path_follower
  ${catkin_LIBRARIES}
)

install(TARGETS cxr_egoctrl_v1 leader_safe_path_follower
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_landing_search_speed_limit.py)
endif()
