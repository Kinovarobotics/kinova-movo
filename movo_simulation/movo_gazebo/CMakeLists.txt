cmake_minimum_required(VERSION 2.8.3)
project(movo_gazebo)

find_package(catkin REQUIRED COMPONENTS roslaunch rospy)

catkin_package()

install(
  PROGRAMS
    scripts/init_sim
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY controller launch worlds models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
