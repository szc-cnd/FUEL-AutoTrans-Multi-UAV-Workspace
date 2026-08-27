cmake_minimum_required(VERSION 2.8.3)
project(payload_mpc_controller)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

# ARM NEON flags
if("${CMAKE_HOST_SYSTEM_PROCESSOR}" STREQUAL "armv7l")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=armv7-a -mfpu=neon -mfloat-abi=hard -funsafe-math-optimizations")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv7-a -mfpu=neon -mfloat-abi=hard -funsafe-math-optimizations")
  message("enabling ARM neon optimizations")
endif()

# flags for speed (should already be enabled by default)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -fopenmp -O3 -march=native")

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  mavros_msgs
  rosgraph_msgs
  quadrotor_msgs
  uav_utils

)

catkin_package()

#include dir
# include_directories(
#   include
#   include/payload_mpc_controller
# )

# add_library(mpc_solver
#   externals/qpoases/SRC/Bounds.cpp
#   externals/qpoases/SRC/Constraints.cpp
#   externals/qpoases/SRC/CyclingManager.cpp
#   externals/qpoases/SRC/Indexlist.cpp
#   externals/qpoases/SRC/MessageHandling.cpp
#   externals/qpoases/SRC/QProblem.cpp
#   externals/qpoases/SRC/QProblemB.cpp
#   externals/qpoases/SRC/SubjectTo.cpp
#   externals/qpoases/SRC/Utils.cpp
#   externals/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
#   model/quadrotor_payload_mpc/acado_qpoases_interface.cpp
#   model/quadrotor_payload_mpc/acado_integrator.c
#   model/quadrotor_payload_mpc/acado_solver.c
#   model/quadrotor_payload_mpc/acado_auxiliary_functions.c)

# target_include_directories(mpc_solver PUBLIC
#   model/quadrotor_payload_mpc/
#   externals/qpoases
#   externals/qpoases/INCLUDE
#   externals/qpoases/SRC
#   )

add_executable(mpc_controller_node 
  src/mpc_controller_node.cpp
  src/mpc_controller.cpp
  src/mpc_input.cpp
  src/mpc_fsm.cpp
  src/mpc_wrapper.cpp
  # MPC solver
  externals/qpoases/SRC/Bounds.cpp
  externals/qpoases/SRC/Constraints.cpp
  externals/qpoases/SRC/CyclingManager.cpp
  externals/qpoases/SRC/Indexlist.cpp
  externals/qpoases/SRC/MessageHandling.cpp
  externals/qpoases/SRC/QProblem.cpp
  externals/qpoases/SRC/QProblemB.cpp
  externals/qpoases/SRC/SubjectTo.cpp
  externals/qpoases/SRC/Utils.cpp
  externals/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
  model/quadrotor_payload_mpc/acado_qpoases_interface.cpp
  model/quadrotor_payload_mpc/acado_integrator.c
  model/quadrotor_payload_mpc/acado_solver.c
  model/quadrotor_payload_mpc/acado_auxiliary_functions.c
  )
target_include_directories(mpc_controller_node PUBLIC
  include/payload_mpc_controller
  include 
  ${EIGEN3_INCLUDE_DIR}
  SYSTEM
  ${catkin_INCLUDE_DIRS} 
  # MPC solver
  model/quadrotor_payload_mpc/
  externals/qpoases
  externals/qpoases/INCLUDE
  externals/qpoases/SRC
  )
target_link_libraries(mpc_controller_node
  # mpc_solver
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_disturbance_slew_limiter
    test/test_disturbance_slew_limiter.cpp
  )
  if(TARGET test_disturbance_slew_limiter)
    target_include_directories(test_disturbance_slew_limiter PRIVATE
      include/payload_mpc_controller
      ${EIGEN3_INCLUDE_DIR}
    )
  endif()
  catkin_add_gtest(test_force_attitude_aligner
    test/test_force_attitude_aligner.cpp
  )
  if(TARGET test_force_attitude_aligner)
    target_include_directories(test_force_attitude_aligner PRIVATE
      include/payload_mpc_controller
      ${EIGEN3_INCLUDE_DIR}
    )
  endif()
  catkin_add_gtest(test_force_observer_input_synchronizer
    test/test_force_observer_input_synchronizer.cpp
  )
  if(TARGET test_force_observer_input_synchronizer)
    target_include_directories(test_force_observer_input_synchronizer PRIVATE
      include/payload_mpc_controller
      ${EIGEN3_INCLUDE_DIR}
    )
  endif()
  catkin_add_gtest(test_odom_spike_guard
    test/test_odom_spike_guard.cpp
  )
  if(TARGET test_odom_spike_guard)
    target_include_directories(test_odom_spike_guard PRIVATE
      include/payload_mpc_controller
      ${EIGEN3_INCLUDE_DIR}
    )
  endif()
  catkin_add_gtest(test_recovery_control
    test/test_recovery_control.cpp
  )
  if(TARGET test_recovery_control)
    target_include_directories(test_recovery_control PRIVATE
      include/payload_mpc_controller
      ${EIGEN3_INCLUDE_DIR}
    )
  endif()
  catkin_add_nosetests(scripts/tests/test_autotrans_mpc_logger_battery.py)
  catkin_add_nosetests(test/test_position_command_latch_static.py)
catkin_add_nosetests(test/test_velocity_hard_constraint_static.py)
  catkin_add_nosetests(test/test_chinese_log_locale_static.py)
  catkin_add_nosetests(test/test_log_rate_policy_static.py)
  catkin_add_nosetests(test/test_takeoff_timeout_hover_static.py)
  catkin_add_nosetests(test/test_fsm_simplification_static.py)
  catkin_add_nosetests(test/test_cav0_sync_contract_static.py)
  catkin_add_nosetests(test/test_landing_search_yaw_hold_static.py)
endif()

catkin_install_python(PROGRAMS
  scripts/autotrans_mpc_logger.py
  scripts/generate_evo_report.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(FILES
  scripts/analyze_autotrans_wind_mpc.m
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)
