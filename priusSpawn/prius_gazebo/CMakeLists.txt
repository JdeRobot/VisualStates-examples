cmake_minimum_required(VERSION 2.8.3)
project(prius_gazebo)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY models DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY worlds DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(PROGRAMS nodes/joystick_translator
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
