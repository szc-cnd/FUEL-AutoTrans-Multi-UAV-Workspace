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
  # 2026-07-13: 终点二维码确认后由控制器调用 PX4 AUTO.LAND 服务。
  mavros_msgs
  # 2026-07-28: 后机跟随器读取LDOT结构化动态框与预测轨迹进行摆球让行。
  ldot_detector
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  # 2026-07-14: 导出后机跟随器新增的里程计、点云和可视化依赖。
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs nav_msgs sensor_msgs visualization_msgs quadrotor_msgs tf mavros_msgs ldot_detector
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Executable node
add_executable(cxr_egoctrl_v1 src/cxr_egoctrl_v1.cpp)

# 2026-07-14: 独立后机管理器只跟随前机已经飞过的安全历史，不复制第二套 FUEL。
add_executable(leader_safe_path_follower src/leader_safe_path_follower.cpp)
add_dependencies(leader_safe_path_follower ${catkin_EXPORTED_TARGETS})  # 2026-07-28: 确保LDOT消息先生成再编译跟随器。

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
