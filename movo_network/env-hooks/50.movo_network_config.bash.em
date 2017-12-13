@[if DEVELSPACE]@
. "@(CMAKE_CURRENT_SOURCE_DIR)/movo_network_config.bash"
@[else]@
if [ -z "$CATKIN_ENV_HOOK_WORKSPACE" ]; then
  CATKIN_ENV_HOOK_WORKSPACE="@(CMAKE_INSTALL_PREFIX)"
fi
. "$CATKIN_ENV_HOOK_WORKSPACE/share/movo_network_config/movo_network_config.bash"
@[end if]@
