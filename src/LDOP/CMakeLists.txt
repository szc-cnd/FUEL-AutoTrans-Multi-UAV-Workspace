cmake_minimum_required(VERSION 3.0.2)
project(ldop)

get_filename_component(LDOP_WORKSPACE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
list(PREPEND CMAKE_PREFIX_PATH "/opt/ros/noetic")

# LDOP may live in a mixed catkin workspace while its third-party packages
# are kept in the dedicated ldop workspace or in the user's local prefix.
# Keep the local workspace locations first, then fall back to those existing
# installations so the package remains buildable after being moved.
set(LDOP_UFOMAP_PREFIXES
  "${LDOP_WORKSPACE_ROOT}/external/install/ufomap"
  "/home/oem/ldop_ws/external/install/ufomap"
)
set(LDOP_LZF_PREFIXES
  "${LDOP_WORKSPACE_ROOT}/external/install/lzf/usr"
  "/home/oem/.local/usr"
)
foreach(prefix IN LISTS LDOP_UFOMAP_PREFIXES LDOP_LZF_PREFIXES)
  if(EXISTS "${prefix}")
    list(PREPEND CMAKE_PREFIX_PATH "${prefix}")
  endif()
endforeach()

# 生成 compile_commands.json，clangd 会用它来知道
# “每个源文件到底是怎么编译的”。
# 没有这个文件时，clangd 的跳转、补全、报错经常不准。
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 给编译器添加常用警告选项。
# -Wall/-Wextra: 打开更多常见警告
# -Wpedantic: 对不够规范的写法更严格
# 学习阶段建议保留，能尽早发现代码问题。
add_compile_options(-Wall -Wextra -Wpedantic)

# find_package(catkin REQUIRED COMPONENTS ...) 的作用是：
# 告诉 catkin“这个包编译时依赖哪些 ROS 包”。
# 这里列出的包，通常也应该出现在 package.xml 里。
# 这些依赖会提供头文件、库、消息类型等。
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  visualization_msgs
)
find_package(Threads REQUIRED)
find_package(OpenMP REQUIRED)
find_package(TBB REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(liblzf CONFIG REQUIRED)
find_package(ufomap CONFIG REQUIRED
  HINTS "${LDOP_WORKSPACE_ROOT}/external/install/ufomap"
  PATH_SUFFIXES lib/cmake/ufomap
)

# UFOMap 的公开头文件会包含 <liblzf/lzf.h>，但部分 liblzf CMake
# 安装只导出了 .../include/liblzf。补充其父目录并显式链接动态库，
# 避免依赖恰好安装在系统默认搜索路径中。
get_filename_component(LIBLZF_INCLUDE_ROOT "${liblzf_INCLUDE_DIRS}" DIRECTORY)

add_message_files(
  FILES
  DynamicObject.msg
  DynamicObjectArray.msg
  DynamicObjectPredictionPoint.msg
  DynamicObjectPredictionBranch.msg
  DynamicObjectPrediction.msg
  DynamicObjectPredictionArray.msg
  FusedDynamicObject.msg
  FusedDynamicObjectArray.msg
  FusedDynamicObjectVirtualImu.msg
  FusedDynamicObjectVirtualImuArray.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

# catkin_package(...) 不是“编译当前包”，而是“向外声明当前包能提供什么”。
# 如果未来别的 ROS 包要依赖 ldop，它们会从这里获取导出信息。
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ldop_core
  CATKIN_DEPENDS geometry_msgs message_runtime nav_msgs roscpp sensor_msgs std_msgs visualization_msgs
  DEPENDS EIGEN3
)

# include_directories(...) 指定编译时去哪些目录找头文件。
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
  ${LIBLZF_INCLUDE_ROOT}
)

# add_library(...) 定义一个“库”目标。
# 这里把真正的核心逻辑放进 ldop_core 里，而不是全塞进 main 函数。
# 这样做的好处是：
# 1. 结构更清晰
# 2. 后续更容易拆分模块
# 3. 别的可执行程序将来也能复用这部分代码
add_library(ldop_core
  src/ldop.cpp
  src/dynamic_object_clusterer.cpp
  src/dynamic_object_pose_fusion.cpp
  src/dynamic_object_tracker.cpp
  src/dynamic_object_predictor.cpp
  src/prediction_interaction_context.cpp
  src/motion_model.cpp
  src/multi_model_kalman_filter.cpp
  src/ufomap_mapper.cpp
)

# 只让LDOP目标使用C++20，避免在catkin单体构建中把该标准泄漏给后续
# 仍使用ROS Noetic/C++11的飞控与规划包。
target_compile_features(ldop_core PUBLIC cxx_std_20)
set_target_properties(ldop_core PROPERTIES CXX_EXTENSIONS OFF)

# add_dependencies(...) 用来声明“编译顺序依赖”。
add_dependencies(ldop_core ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# target_link_libraries(...) 表示链接阶段要把哪些库连进来。
# 这里 ldop_core 需要使用 roscpp、UFOMap 和线程库。
target_link_libraries(ldop_core PUBLIC
  ${catkin_LIBRARIES}
  UFO::Map
  liblzf::liblzf
  ${CMAKE_THREAD_LIBS_INIT}
)

# add_executable(...) 定义一个可执行文件。
# 这里生成的是节点程序 ldop_node，
# 对应的源码入口就是 src/ldop_node.cpp 里的 main 函数。
add_executable(ldop_node
  src/ldop_node.cpp
)

# 和上面的库一样，节点也声明一下对 catkin 生成目标的依赖。
add_dependencies(ldop_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 节点最终需要链接两部分：
# 1. ldop_core：你自己写的核心逻辑库
# 2. ${catkin_LIBRARIES}：ROS 相关库
# 3. UFOMap：体素地图建图库
# 这样 main 函数就可以调用你自己的类，同时也能正常使用 ROS API。
target_link_libraries(ldop_node
  ldop_core
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_dynamic_object_predictor
    test/test_dynamic_object_predictor.cpp
  )
  if(TARGET test_dynamic_object_predictor)
    target_link_libraries(test_dynamic_object_predictor
      ldop_core
      ${catkin_LIBRARIES}
      gtest
    )
  endif()

  catkin_add_gtest(test_prediction_interaction_context
    test/test_prediction_interaction_context.cpp
  )
  if(TARGET test_prediction_interaction_context)
    target_link_libraries(test_prediction_interaction_context
      ldop_core
      ${catkin_LIBRARIES}
      gtest
    )
  endif()

  catkin_add_gtest(test_ufomap_mapper
    test/test_ufomap_mapper.cpp
  )
  if(TARGET test_ufomap_mapper)
    target_link_libraries(test_ufomap_mapper
      ldop_core
      ${catkin_LIBRARIES}
      gtest
    )
  endif()

  catkin_add_gtest(test_ldop_timing_log
    test/test_ldop_timing_log.cpp
  )
  if(TARGET test_ldop_timing_log)
    target_link_libraries(test_ldop_timing_log
      ${catkin_LIBRARIES}
      gtest
    )
  endif()

  catkin_add_gtest(test_dynamic_object_pose_fusion
    test/test_dynamic_object_pose_fusion.cpp
  )
  if(TARGET test_dynamic_object_pose_fusion)
    target_link_libraries(test_dynamic_object_pose_fusion
      ldop_core
      ${catkin_LIBRARIES}
      gtest
    )
  endif()
endif()
