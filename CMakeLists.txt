cmake_minimum_required(VERSION 3.0.2)
project(pm01_marathon)

find_package(catkin REQUIRED COMPONENTS rospy)
catkin_package()

catkin_install_python(PROGRAMS
  scripts/course.py
  scripts/odom_bridge.py
  scripts/lane_paths.py
  scripts/scan_to_cloud.py
  scripts/rslidar_points.py
  scripts/gnss_ksxt.py
  scripts/glio_twist.py
  scripts/sidewalk_traffic.py
  scripts/cloud_to_scan.py
  scripts/gait_player.py
  scripts/policy_gait.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch worlds urdf config meshes texture
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
