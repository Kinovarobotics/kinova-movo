cmake_minimum_required(VERSION 2.8.3)
project(movo_class)

## Uncomment if the package has a setup.py
catkin_python_setup()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS rospy roslaunch movo_demos std_msgs message_runtime sensor_msgs)

find_package(OpenCV REQUIRED
  COMPONENTS
    opencv_core
    opencv_imgproc
    opencv_imgcodecs
  CONFIG
)

catkin_python_setup()
file(GLOB python_scripts_DIR 
	"scripts/*.py"
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

## Declare a catkin package
catkin_package(

)

roslaunch_add_file_check(launch)

## Install ## 

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


