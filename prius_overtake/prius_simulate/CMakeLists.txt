project(prius_simulate)

cmake_minimum_required(VERSION 2.8.3)

find_package(catkin REQUIRED COMPONENTS visualstates
  rospy
  prius_msgs
  sensor_msgs
)

catkin_package()
include_directories(${catkin_INCLUDE_DIRS})
install(PROGRAMS prius_simulate.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
