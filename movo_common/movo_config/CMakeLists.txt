cmake_minimum_required(VERSION 2.8.3)
project(movo_config)

find_package(catkin REQUIRED)

catkin_package()

install(FILES movo_config.bash
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_add_env_hooks(50.movo_config SHELLS bash DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

