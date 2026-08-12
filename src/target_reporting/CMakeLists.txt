cmake_minimum_required(VERSION 3.0.2)
project(target_reporting)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge geometry_msgs nav_msgs rosbag rospy sensor_msgs std_msgs tf2_geometry_msgs tf2_msgs tf2_ros visualization_msgs
)

catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS
  scripts/checkerboard_handeye_calibrate.py
  scripts/odometry_to_tf.py
  scripts/publish_camera_extrinsic_tf.py
  scripts/target_reporter_node.py
  scripts/target_rviz_marker_node.py
  scripts/target_report_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
