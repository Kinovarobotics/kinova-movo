cmake_minimum_required(VERSION 2.8.3)
project(movo_assisted_teleop)

find_package(cmake_modules REQUIRED)

set(THIS_PACKAGE_ROS_DEPS
  tf
  costmap_2d
  roscpp
  roslib
  geometry_msgs
  move_base_msgs
  actionlib
  message_filters
  base_local_planner
  angles
  pluginlib
  sensor_msgs
  filters
)

find_package(catkin REQUIRED COMPONENTS ${THIS_PACKAGE_ROS_DEPS})
find_package(Eigen REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS} ${Eigen_LIBRARY_DIRS})
message("Assisted telop catkin_LIBRARY_DIRS: " ${catkin_LIBRARY_DIRS})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${THIS_PACKAGE_ROS_DEPS}
  DEPENDS Eigen
)

add_executable(movo_assisted_teleop src/movo_assisted_teleop.cpp)
target_link_libraries(movo_assisted_teleop trajectory_planner_ros ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

install(TARGETS movo_assisted_teleop
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


