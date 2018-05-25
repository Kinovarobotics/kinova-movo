cmake_minimum_required(VERSION 2.8.3)

project(smach_viewer)

find_package(catkin REQUIRED COMPONENTS rostest)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS smach_ros smach_msgs
  )

# Install targets
install(PROGRAMS scripts/smach_viewer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
