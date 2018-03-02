# This configures the environment variables for a MoVo simulation
# This is necessary to run before starting the simulation 
#

# If there is an onboard PC powered by the system this will run the watchdog
# to make sure it gets gracefully shutdown before the system power cuts out.
#export MOVO_POWERS_PC_ONBOARD=true

#Default Movo network 
export MOVO_IP_ADDRESS=10.66.171.5

# The reference frame for all the accessories and sensors
export MOVO_PARENT_LINK=base_chassis_link

#
# This enables the laser scan matcher which generates a /movo/lsm/pose_stamped in the odom frame
#
export MOVO_ENABLE_LSM=true

#
# Set this if you want the platform odometry to be corrected by LSM. This is stable indoors, but should be tested with
# teleoperation before using it
#
export MOVO_USE_LSM_TO_CORRECT_ODOMETRY=true

# This will run the full system tele-op node (ie can control all the joints in the system) it is not
# collision aware and is really only meant for demonstration purposes. If set false, teleop just controls
# the mobile platform
export RUN_FULL_SYSTEM_TELEOP=true

# Joystick configurations for joystick set MOVO_JOY_IS_ATTACHED if the joystick
# is physically attached to this PC
export MOVO_JOY_IS_ATTACHED=false
export MOVO_JOY_DEV=/dev/input/js0

#define if movo is wearing skins
export MOVO_HAS_BODY=true

#Front laser hardware config
export MOVO_LASER1_IP=10.66.171.8
export MOVO_LASER1_PORT=2112
export LASER1_MAX_RANGE=10.0
export LASER1_MIN_RANGE=0.01
export LASER1_MAX_ANGLE=2.0
export LASER1_MIN_ANGLE=-2.0
export LASER1_PREFIX="front"

#Rear laser hardware config
export MOVO_LASER2_IP=10.66.171.9
export MOVO_LASER2_PORT=2112
export LASER2_MAX_RANGE=10.0
export LASER2_MIN_RANGE=0.01
export LASER2_MAX_ANGLE=2.0
export LASER2_MIN_ANGLE=-2.0
export LASER2_PREFIX="rear"

#Kinova arm configurations (the right arm should be the default if there is only one)
export MOVO_HAS_KINOVA_ARM=true
export MOVO_HAS_TWO_KINOVA_ARMS=true
export KINOVA_RIGHT_ARM_IP_ADDRESS=10.66.171.15
export KINOVA_LEFT_ARM_IP_ADDRESS=10.66.171.16
export KINOVA_ARM_IFACE=eth0
export MOVO_HAS_KINOVA_ARM_7DOF=true
export MOVO_HAS_KINOVA_ARM_6DOF=false

#gripper configurations

# Must set all three environment variables for on gripper
# type to true. The moveit configurations only support the same
# type of gripper for left and right

#Kinova KG2
export USE_KG2_FOR_MOVEIT_CONFIG=false
export MOVO_HAS_RIGHT_KG2_GRIPPER=false
export MOVO_HAS_LEFT_KG2_GRIPPER=false

#Kinova KG3
export USE_KG3_FOR_MOVEIT_CONFIG=true
export MOVO_HAS_RIGHT_KG3_GRIPPER=true
export MOVO_HAS_LEFT_KG3_GRIPPER=true

#Robotiq 85 two finger gripper
export USE_R85_FOR_MOVEIT_CONFIG=false
export MOVO_HAS_RIGHT_ROBOTIQ_GRIPPER=false
export MOVO_HAS_LEFT_ROBOTIQ_GRIPPER=false

#export for kinect2 bridge
export OCL_IGNORE_SELF_TEST=1

#allows for ssh launch of Kinect bridge
export DISPLAY=:0


