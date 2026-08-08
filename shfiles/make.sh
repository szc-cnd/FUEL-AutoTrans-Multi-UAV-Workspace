
set -e

ROS1_BUILD_ARGS="-DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS1 -DNLopt_DIR=/home/asus/.local/rosdeps/nlopt/prefix/usr/lib/x86_64-linux-gnu/cmake/nlopt_cxx -DLIVOX_LIDAR_SDK_LIBRARY=/home/asus/.local/livox_sdk2/prefix/lib/liblivox_lidar_sdk_static.a -DCMAKE_CXX_FLAGS=-I/home/asus/.local/livox_sdk2/prefix/include"

catkin_make -j4 -l4 --pkg quadrotor_msgs ${ROS1_BUILD_ARGS}
catkin_make -j4 -l4 ${ROS1_BUILD_ARGS}
