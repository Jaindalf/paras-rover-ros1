cmake_minimum_required(VERSION 3.0.2)
project(wheel_odometry)

find_package(catkin REQUIRED COMPONENTS
  rospy
  nav_msgs
  geometry_msgs
  tf
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/odom_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)