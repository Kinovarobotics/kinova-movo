cmake_minimum_required(VERSION 2.8.3)
project(range_sensor_filters)

find_package(catkin REQUIRED COMPONENTS roscpp tf filters laser_geometry pcl_ros geometry_msgs sensor_msgs message_filters)

catkin_package(
    DEPENDS 
    CATKIN_DEPENDS roscpp tf filters laser_geometry pcl_ros geometry_msgs sensor_msgs message_filters
    INCLUDE_DIRS include
    LIBRARIES range_sensor_filters
)

include_directories(${PROJECT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS})
add_library(range_sensor_filters src/range_sensor_filters.cpp)
target_link_libraries(range_sensor_filters ${catkin_LIBRARIES})
add_dependencies(range_sensor_filters ${catkin_EXPORTED_TARGETS})

install(TARGETS range_sensor_filters
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
   
install(FILES
  range_sensor_filters_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
