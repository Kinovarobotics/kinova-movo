cmake_minimum_required(VERSION 2.8.3)
project(movo_navigation_apps)

find_package(catkin REQUIRED COMPONENTS message_generation roslaunch)

## Declare a catkin package
catkin_package()

roslaunch_add_file_check(launch)

install(DIRECTORY config maps goals launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
