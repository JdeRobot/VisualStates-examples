cmake_minimum_required(VERSION 2.8.3)
project(prius_gazebo_plugins)

set (CMAKE_CXX_STANDARD 11)

find_package(gazebo 9 REQUIRED)
find_package(ignition-msgs0 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  prius_msgs
  roscpp
  sensor_msgs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS})

add_library(PriusHybridPlugin SHARED plugins/PriusHybridPlugin.cc include/PriusHybridPlugin.hh)
target_link_libraries(PriusHybridPlugin ${GAZEBO_LIBRARIES} ${IGNITION-MSGS_LIBRARIES})

add_library(gazebo_ros_block_laser SHARED plugins/gazebo_ros_block_laser.cpp include/gazebo_ros_block_laser.h)
target_link_libraries(gazebo_ros_block_laser ${GAZEBO_LIBRARIES} ${IGNITION-MSGS_LIBRARIES})

install(TARGETS PriusHybridPlugin gazebo_ros_block_laser
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
