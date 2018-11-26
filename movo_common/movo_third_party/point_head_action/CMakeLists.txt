# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(head_action)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
   geometry_msgs 
   trajectory_msgs 
   sensor_msgs 
   roscpp 
   tf 
   tf_conversions 
   actionlib 
   kdl_parser
   sensor_msgs
   control_msgs)

find_package(orocos_kdl REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS}
   ${orocos_kdl_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS trajectory_msgs sensor_msgs control_msgs geometry_msgs
)

add_executable(head_action src/point_frame.cpp)
target_link_libraries(head_action ${Boost_LIBRARIES} 
   ${orocos_kdl_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(head_action ${catkin_EXPORTED_TARGETS})

install(TARGETS head_action
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
