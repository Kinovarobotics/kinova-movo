cmake_minimum_required(VERSION 2.8.3)
project(grasp_msgs)

find_package(catkin REQUIRED
  COMPONENTS
    actionlib
    geometry_msgs
    message_generation
    moveit_msgs
    sensor_msgs
    shape_msgs
)

link_directories(${catkin_LIBRARY_DIRS}})

add_action_files(
  FILES
    FindGraspableObjects.action
    GraspPlanning.action
)

add_message_files(
  FILES
    GraspableObject.msg
    Object.msg
    ObjectProperty.msg
)

generate_messages(DEPENDENCIES geometry_msgs moveit_msgs sensor_msgs shape_msgs)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    moveit_msgs
    sensor_msgs
    shape_msgs
)
