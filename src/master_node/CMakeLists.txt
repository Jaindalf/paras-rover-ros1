cmake_minimum_required(VERSION 3.0.2)
project(master_node)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/master_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)