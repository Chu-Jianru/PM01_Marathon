cmake_minimum_required(VERSION 3.0.2)
project(gazebo_line_demo)

find_package(catkin REQUIRED COMPONENTS rospy)
catkin_package()

catkin_install_python(PROGRAMS
  scripts/course.py
  scripts/odom_bridge.py
  scripts/lane_paths.py
  scripts/scan_to_cloud.py
  scripts/rslidar_points.py
  scripts/gnss_ksxt.py
  scripts/cloud_to_scan.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch worlds urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
