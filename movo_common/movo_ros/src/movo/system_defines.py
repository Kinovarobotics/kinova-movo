"""--------------------------------------------------------------------
Copyright (c) 2017, Kinova Robotics inc.

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the names of its contributors
      may be used to endorse or promote products derived from this software
      without specific prior written permission.
      
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR 
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 
 \file   system_defines.py

 \brief  This module defines the interface for the SI MOVO

 \Platform: Linux/ROS Indigo
--------------------------------------------------------------------"""

"""------------------------------------------------------------------------
SI MOVO Command structures
There are three types of messages the SI MOVO will accept:

2. Holonomic motion command message: This message contains three 32-bit IEEE754 floating 
point variables in the standard ROS twist message format. Only vx,vy, and vtheta are 
accepted. Note that the command range is the maximum, each will be limited on the 
platform by the user configurable limits stored in NVM. 

4. User Configuration message: This message will contain 16 32-bit variables. Each defines a
user configurable parameter that is stored on the machine in NVM and loaded at startup.
The machine uses F-RAM NVM so these may be written as often as needed.

3. General purpose command message: This message will contain two 32-bit variables. The
first is a 32-bit integer general purpose command which is the ID of the configuration 
command (see below). The second is the general purpose parameter which may be integer
or 32-bit IEEE754 floating point value depending on the command
------------------------------------------------------------------------"""

"""
Defines for the structure of commands sent to Vector each message
consists of an ID and a number 32-bit words depending on the command.
The data bytes are big-endian (highest byte in lowest index).
Command ID: 16-bit ID.
Command Variables: data_bytes[0....n]
Bytes 0-3: 32-bit variable 1 (MS byte at index 0 LS byte at index 3)
Bytes 4-7: 32-bit variable 2 (MS byte at index 4 LS byte at index 7)
repeats in 4 byte blocks for all the variables

The format to movo_comm.py is 
[command_id (16-bit), [variable_1, variable_2,...variable_n]]
the checksum and breakout into a byte array is handled
by the communication driver
"""
"""
Platform motion command for the base
"""
MOTION_CMD_ID                = 0x1800
MOTION_CMD_VEL_X_INDEX       = 0
MOTION_CMD_VEL_Y_INDEX       = 1
MOTION_CMD_YAW_RATE_INDEX    = 2

"""
This section defines the layout of the motion test
"""
MOTION_TEST_CMD_ID          = 0x1804
MOTION_TEST_TYPE_INDEX      = 0
MOTION_TEST_DURATION_INDEX  = 1
MOTION_TEST_MAGNITUDE_INDEX = 2

MOTION_TEST_TYPE_MASK  = 0x80000007
MOTION_TEST_RESET_ODOM = 0x80000000
MOTION_TEST_X          = 0x00000001
MOTION_TEST_Y          = 0x00000002
MOTION_TEST_YAW        = 0x00000004

"""
Linear actuator commands
"""
LINEAR_ACTUATOR_POSITION_CMD_ID       = 0x1900
LINEAR_ACTUATOR_VELOCITY_LIMIT_CMD_ID = 0x1901

"""
Kinova actuator commands
"""
EIB_GENERAL_CMD_ID             = 0x4000
KINOVA_ACTUATOR_CMD_ID         = 0x4001

EIB_GENERAL_CMD_SEND_CONTINUOUS_DATA = 0
EIB_POLLING                          = 0
EIB_STREAMING                        = 1

KINOVA_ACTUATOR_RSP_SIZE_BYTES = 86 

"""
Load machine configuration command is sent to update the NVM configuration
parameters that get loaded on the machine during initialization
"""
LOAD_MACH_CONFIG_CMD_ID  = 0x1801

"""------------------------------------------------------------------------
Start Variables Stored in NV F-RAM memory limits are defined in dynamic
reconfigure
------------------------------------------------------------------------"""

"""------------------------------------------------------------------------
32-bit floating point variable representing the maximum velocity 
limit in m/s.
------------------------------------------------------------------------"""
X_VEL_LIMIT_INDEX                     = (0)
DEFAULT_MAXIMUM_VELOCITY_MPS        = (0.5)

"""------------------------------------------------------------------------
32-bit floating point variable representing the maximum velocity 
limit in m/s.
------------------------------------------------------------------------"""
Y_VEL_LIMIT_INDEX                     = (1)
DEFAULT_MAXIMUM_VELOCITY_MPS        = (0.5)

"""------------------------------------------------------------------------
32-bit floating point variable representing the maximum acceleration 
limit in m/s^2.
------------------------------------------------------------------------"""
ACCEL_LIMIT_INDEX                   = (2)
DEFAULT_MAXIMUM_ACCELERATION_MPS2   = (1.0)

"""------------------------------------------------------------------------
32-bit floating point variable representing the
maximum deceleration limit in m/s^2.
------------------------------------------------------------------------"""
DECEL_LIMIT_INDEX                   = (3)
DEFAULT_MAXIMUM_DECELERATION_MPS2   = (1.0)

"""------------------------------------------------------------------------
32-bit floating point variable representing the maximum DTZ response deceleration 
limit in m/s^2.
------------------------------------------------------------------------"""
DTZ_DECEL_LIMIT_INDEX               = (4)
DEFAULT_MAXIMUM_DTZ_DECEL_RATE_MPS2 = (1.0)

"""------------------------------------------------------------------------
32-bit floating point variable representing the yaw rate limit in rad/s.
Yaw rate is defined as the differential wheel velocity
------------------------------------------------------------------------"""
YAW_RATE_LIMIT_INDEX                = (5)
DEFAULT_MAXIMUM_YAW_RATE_RPS        = (1.0)

"""------------------------------------------------------------------------
32-bit floating point variable representing yaw acceleration in rad/s^2.
------------------------------------------------------------------------"""
YAW_ACCEL_LIMIT_INDEX               = (6)
DEFAULT_MAX_YAW_ACCEL_RPS2          = (1.0)

"""------------------------------------------------------------------------
32-bit floating point variable representing tire diameter in meters. 
This updates the rolling tire diameter used in software to calculate
velocity, position, differential wheel speed (yaw rate) and accelerations. 
------------------------------------------------------------------------"""
WHEEL_DIAMETER_INDEX                = (7)
DEFAULT_WHEEL_DIAMETER_M            = (0.1524)

"""------------------------------------------------------------------------
32-bit floating point variable representing wheel base length in meters.
It is the distance between the center of the front and rear tire contact patches.
This updates the wheel base length used in software to calculate
velocity, position, differential wheel speed (yaw rate) and accelerations.
------------------------------------------------------------------------"""
WHEEL_BASE_LENGTH_INDEX             = (8)
DEFAULT_WHEEL_BASE_LENGTH_M         = (0.45593)

"""------------------------------------------------------------------------
32-bit floating point variable representing track width in meters.
It is the distance between the center of the left and right tire contact patches.
This updates the track width used in software to calculate differential wheel speeds (yaw rate). 
------------------------------------------------------------------------"""
WHEEL_TRACK_WIDTH_INDEX             = (9)
DEFAULT_WHEEL_TRACK_WIDTH_M         = (0.36335)

"""------------------------------------------------------------------------
32-bit IEEE754 floating point variable representing gear ratio (unitless).
This updates the gearbox ratio used in software to convert from motor speed 
to gearbox output speed 
------------------------------------------------------------------------"""
GEAR_RATIO_INDEX                    = (10)
DEFAULT_TRANSMISSION_RATIO          = (5.0)

"""------------------------------------------------------------------------
32-bit integer variable with configuration bits representing each variable defined below.
This bitmap setscertain behaviors that need to be defined at startup
------------------------------------------------------------------------"""
CONFIG_BITMAP_INDEX         = (11)


"""
Allows the machine to run with charger connected
"""
DISABLE_AC_PRESENT_CSI        = 1
ENABLE_AC_PRESENT_CSI         = 0

"""
Selects cutoff requency for input commands in the controllers. Used to smooth
out the user commands if the host controller issues stepwise commands
"""
MOTION_MAPPING_10HZ_FILTER         = (0x0)
MOTION_MAPPING_4HZ_FILTER          = (0x1)
MOTION_MAPPING_1HZ_FILTER          = (0x2)
MOTION_MAPPING_05HZ_FILTER         = (0x4)
MOTION_MAPPING_02HZ_FILTER         = (0x8)
VALID_MOTION_MAPPING_FILTER_MASK   = (0xF)

"""
Shifts for the variables in the bitmaps
"""
DISABLE_AC_PRESENT_CSI_SHIFT = 0
MOTION_MAPPING_FILTER_SHIFT  = 4

"""
CONFIG_BITMAP_EXAMPLE = ((ENABLE_AC_PRESENT_CSI << DISABLE_AC_PRESENT_CSI_SHIFT) |
                         (MOTION_MAPPING_10HZ_FILTER << MOTION_MAPPING_FILTER_SHIFT))

"""


"""
Load ethernet configuration command is sent to update the NVM ethernet configuration
parameters that get loaded on the machine during initialization
"""
LOAD_ETH_CONFIG_CMD_ID  = 0x1802

"""------------------------------------------------------------------------
32-bit integer variable representing the ethernet IP address representing a
dotted quad IP address.
The conversion is:
integer = (first octet * 16777216) + (second octet * 65536) + (third octet * 256) + (fourth octet)
Bounds for this item are valid IP addresses
------------------------------------------------------------------------"""
ETH_IP_ADDRESS_INDEX                  = (0)
DEFAULT_ETH_IP_ADDRESS                = (95109642) #10.66.171.5

"""------------------------------------------------------------------------
32-bit integer variable representing the ethernet port used to communicate
with the Vector platform
Bounds for this item are valid ethernet ports
------------------------------------------------------------------------"""
ETH_PORT_NUMBER_INDEX                  = (1)
DEFAULT_UI_ETH_PORT                    = (8080)

"""------------------------------------------------------------------------
32-bit integer variable representing the ethernet subnet mask representing a
dotted quad IP address.
The conversion is:
integer = (first octet * 16777216) + (second octet * 65536) + (third octet * 256) + (fourth octet)
Bounds for this item are valid IP subnet masks
------------------------------------------------------------------------"""
ETH_SUBNET_MASK_INDEX                  = (2)
DEFAULT_ETH_GATEWAY                    = (28000778) #10.66.171.1

"""------------------------------------------------------------------------
32-bit integer variable representing the ethernet gateway representing a
dotted quad IP address.
The conversion is:
integer = (first octet * 16777216) + (second octet * 65536) + (third octet * 256) + (fourth octet)
Bounds for this item are valid IP subnet masks
------------------------------------------------------------------------"""
ETH_SUBNET_MASK_INDEX                  = (3)
DEFAULT_ETH_SUBNET_MASK                = (16777215) #255.255.255.0

"""------------------------------------------------------------------------
End Variables Stored in NV F-RAM memory
------------------------------------------------------------------------"""

"""
General purpose command is used to set runtime variables
such as mode change requests, audio requests, etc.
"""
GENERAL_PURPOSE_CMD_ID       = 0x1803
GENERAL_PURPOSE_CMD_INDEX    = 0
GENERAL_PURPOSE_PARAM_INDEX  = 1

"""------------------------------------------------------------------------
This command results in no action but a response from the Vector. The general
purpose parameter is ignored.
------------------------------------------------------------------------"""
GENERAL_PURPOSE_CMD_NONE                   = (0)

"""------------------------------------------------------------------------
This command updates the operational mode request on the machine. The general 
purpose parameter is a 32 bit integer representing the mode request ID.
------------------------------------------------------------------------"""
GENERAL_PURPOSE_CMD_SET_OPERATIONAL_MODE    = (1)

DISABLE_REQUEST   = 1
POWERDOWN_REQUEST = 2
DTZ_REQUEST       = 3
STANDBY_REQUEST   = 4
TRACTOR_REQUEST   = 5

MOVO_MODES_DICT = dict({TRACTOR_REQUEST:5,STANDBY_REQUEST:4,POWERDOWN_REQUEST:6})

"""------------------------------------------------------------------------
This command requests the faultlog from the machine. The general purpose parameter 
is 0. This request will send the entire faultlog for 1248 bytes includin the 
checksum. 311 words without the checksum
------------------------------------------------------------------------"""
GENERAL_PURPOSE_CMD_SEND_FAULTLOG         = (2)
"""
Define the number of faultlog packets and the size (in 32-bit words) of each packet.
"""
NUMBER_OF_FAULTLOG_WORDS                  = 311

"""
This command resets the position data on the machine. The general purpose command
is a bitmap of which integrators are to be reset.
""" 
GENERAL_PURPOSE_CMD_RESET_ODOMETRY        = (3)

RESET_LINEAR_POSITION               = (0x00000001)
RESET_RIGHT_FRONT_POSITION          = (0x00000002)
RESET_LEFT_FRONT_POSITION           = (0x00000004)
RESET_RIGHT_REAR_POSITION           = (0x00000008)
RESET_LEFT_REAR_POSITION            = (0x00000010)
RESET_ODOM_ESTIMATE                 = (0x00000020)
VALID_RESET_POSITION_MASK           = (0x0000003F)

"""
This command resets all configurable parameters to their default values. 
The general purpose parameter is ignored for this request.
""" 
GENERAL_PURPOSE_CMD_RESET_PARAMS_TO_DEFAULT = (4)

"""
Sets the flag to send continuous data at system frequency (100.0Hz)
"""
GENERAL_PURPOSE_CMD_SEND_CONTINUOUS_DATA    = (5)

"""------------------------------------------------------------------------
Vector Fault definitions
This section is used to define the decoding of fault status words sent 
by the Vector. The meaning of specific faults can be found in the interface
guide.
------------------------------------------------------------------------"""
NO_FAULT                                    = 0x00000000
ALL_FAULTS                                  = 0xFFFFFFFF

"""
Transient faults: These faults are not latching and can be asserted and then
cleared during runtime. There are currently no transient faults for the Vector
"""
transient_fault_decode = dict({
    0x00000000: ""})

"""
Critical faults: These faults are latching.
"""
critical_fault_decode = dict({
    0x00000000: "",                          
    0x00000001:"CRITICAL_FAULT_INIT",
    0x00000002:"CRITICAL_FAULT_INIT_PROPULSION",
    0x00000004:"CRITICAL_FAULT_INIT_TIMEOUT",
    0x00000008:"CRITICAL_FAULT_FORW_SPEED_LIMITER_HAZARD",
    0x00000010:"CRITICAL_FAULT_AFT_SPEED_LIMITER_HAZARD",
    0x00000020:"CRITICAL_FAULT_CHECK_STARTUP",
    0x00000040:"CRITICAL_FAULT_APP_VELOCITY_CTL_FAILED",
    0x00000080:"CRITICAL_FAULT_APP_POSITION_CTL_FAILED",
    0x00000100:"CRITICAL_FAULT_AP_MODE_TRANS_TIMEOUT",
    0x00000200:"CRITICAL_FAULT_PITCH_ANGLE_EXCEEDED",
    0x00000400:"CRITICAL_FAULT_ROLL_ANGLE_EXCEEDED",
    0x00000800:"CRITICAL_FAULT_BATTERY_FAULT",
    0x00001000:"CRITICAL_FAULT_BIB_INTERNAL_FAULT",
    0x00002000:"CRITICAL_FAULT_EIB_PWR_CH_FAULT",
    0x00004000:"CRITICAL_FAULT_EIB_TEMPERATURE_FAULT",
    0x00008000:"CRITICAL_FAULT_EIB_INTERNAL_FAULT",
    0x00010000:"CRITICAL_FAULT_LOW_SOC",
    0x00020000:"CRITICAL_FAULT_VOLTAGE_RANGE",
    0x00040000:"CRITICAL_FALUT_INIT_KINOVA"})

"""
Communication faults: These faults are latching.
"""
comm_fault_decode = dict({
    0x00000000: "",
    0x00000001:"COMM_FAULT_ETH_RX_OVRRUN",
    0x00000002:"COMM_FAULT_ETH_TX_OVRRUN",
    0x00000004:"COMM_FAULT_UI_CMD_UNKNOWN",
    0x00000008:"COMM_FAULT_UI_BAD_PACKET_CHECKSUM",
    0x00000010:"COMM_FAULT_SPI_TRANSFER",
    0x00000020:"COMM_FAULT_SPI_RECEIVE_OVERRUN",
    0x00000040:"COMM_FAULT_SPI_TX_UNDERRUN",
    0x00000080:"COMM_FAULT_EIB",
    0x00000100:"COMM_FAULT_BIB",
    0x00000200:"COMM_FAULT_BATTERY"})


"""
MD faults: These faults are latching.
"""
md_fault_decode = dict({
    0x00000000: "",                     
    0x00000001:"MD_FAULT_DRIVE_FAULT",
    0x00000002:"MD_FAULT_BAD_STATE",
    0x00000004:"MD_FAULT_COMM",
    0x00000008:"MD_FAULT_BAD_MODE",
    0x00000010:"MD_FAULT_VELOCITY_FAULT",
    0x00000020:"MD_FAULT_DRIVE_ALARM"})

"""
Sensor faults: These faults are latching.
"""
sensor_fault_decode = dict({
    0x00000000: "",                        
    0x00000001:"SENSOR_FAULT_2P5V_VREF_RANGE_FAULT",
    0x00000002:"SENSOR_FAULT_TEMPERATURE_RANGE_FAULT",
    0x00000004:"SENSOR_FAULT_DIGITAL_INPUT",
    0x00000008:"SENSOR_FAULT_RANGE",
    0x00000010:"SENSOR_FAULT_DEFAULT",
    0x00000020:"SENSOR_FAULT_LINEAR_STRING_POT_FAULT"})
 
"""
IMU faults: These faults are latching.
"""
imu_fault_decode = dict({
    0x00000000: "",                     
    0x00000001:"IMU_FAULT_MISSING_IMU_DATA",
    0x00000002:"IMU_FAULT_FAILED_TO_CLEAR_INT",
    0x00000004:"IMU_FAULT_RATE_SENSOR_SATURATED",
    0x00000008:"IMU_FAULT_TILT_SENSOR_SATURATED",
    0x00000010:"IMU_FAULT_SPI_TRANSFER_FAILED"})

"""
Architecture faults: These faults are latching.
"""
arch_fault_decode = dict({
    0x00000000: "",                      
    0x00000001:"ARCHITECT_FAULT_COMMANDED_DTZ",
    0x00000002:"ARCHITECT_FAULT_COMMANDED_DISABLE",
    0x00000004:"ARCHITECT_FAULT_KILL_SWITCH_ACTIVE",
    0x00000001:"ARCHITECT_FAULT_DTZ_SWITCH_ACTIVE",
    0x00000008:"ARCHITECT_FAULT_FRAM_TRANSFER_FAILED",
    0x00000010:"ARCHITECT_FAULT_BAD_MODEL_IDENTIFIER",
    0x00000020:"ARCHITECT_FAULT_BAD_HW_REV",
    0x00000040:"ARCHITECT_FAULT_FRAM_CONFIGS_FAILED",
    0x00000080:"ARCHITECT_FAULT_KILL_SWITCH_MISMATCH"})

"""
Internal faults: These faults are latching.
"""
internal_fault_decode = dict({
    0x00000000: "",                          
    0x00000001:"INTERNAL_FAULT_HIT_DEFAULT_CONDITION",
    0x00000002:"INTERNAL_FAULT_HIT_SPECIAL_CASE"})

"""
All the fault groups are packed into four 32-bit fault status words. The following
defines how they are packed into the words
"""

"""
Fault status word 0
"""
FSW_ARCH_FAULTS_INDEX       = 0
FSW_ARCH_FAULTS_SHIFT       = 0
FSW_ARCH_FAULTS_MASK        = 0x0000FFFF
FSW_CRITICAL_FAULTS_INDEX   = 0
FSW_CRITICAL_FAULTS_SHIFT   = 16
FSW_CRITICAL_FAULTS_MASK    = 0xFFFF0000
"""
Fault status word 1
"""
FSW_COMM_FAULTS_INDEX       = 1
FSW_COMM_FAULTS_SHIFT       = 0
FSW_COMM_FAULTS_MASK        = 0x0000FFFF
FSW_INTERNAL_FAULTS_INDEX   = 1
FSW_INTERNAL_FAULTS_SHIFT   = 16
FSW_INTERNAL_FAULTS_MASK    = 0x000F0000
"""
Fault status word 2
"""
FSW_SENSORS_FAULTS_INDEX    = 2
FSW_SENSORS_FAULTS_SHIFT    = 0
FSW_SENSORS_FAULTS_MASK     = 0x0000FFFF
FSW_IMU_FAULTS_INDEX        = 2
FSW_IMU_FAULTS_SHIFT        = 16
FSW_IMU_FAULTS_MASK         = 0xFFFF0000
"""
Fault status word 3
"""
FSW_MD_FAULTS_INDEX         = 3
FSW_MD_FAULTS_SHIFT         = 0
FSW_MD_FAULTS_MASK          = 0xFFFFFFFF

"""
Fault group index definitions
"""
FAULTGROUP_TRANSIENT    = 0
FAULTGROUP_CRITICAL     = 1
FAULTGROUP_COMM         = 2
FAULTGROUP_SENSORS      = 3
FAULTGROUP_IMU          = 4
FAULTGROUP_MD           = 5
FAULTGROUP_ARCHITECTURE = 6  
FAULTGROUP_INTERNAL     = 7
NUM_OF_FAULTGROUPS      = 8


"""
Defines the feedback array indices
"""
START_STATUS_BLOCK                  =(0)
ROS_FSW_1_INDEX                     =(0)
ROS_FSW_2_INDEX                     =(1)
ROS_FSW_3_INDEX                     =(2)
ROS_FSW_4_INDEX                     =(3)
ROS_OPERATIONAL_TIME_INDEX          =(4)
ROS_OPERATIONAL_STATE_INDEX         =(5)
ROS_DYNAMIC_RESPONSE_INDEX          =(6)
ROS_BUILDID_INDEX                   =(7)
ROS_MACHINE_ID_INDEX                =(8)
END_STATUS_BLOCK                    =(9)

START_BATTERY_DATA_BLOCK            =(9)
ROS_BATT_STATUS                     =(9)
ROS_BATT_SOC                        =(10)
ROS_BATT_VOLTAGE                    =(11)
ROS_BATT_CURRENT                    =(12)
ROS_BATT_TEMPERATURE                =(13)
END_BATTERY_DATA_BLOCK              =(14)

START_PROPULSION_DATA_BLOCK         =(14)
ROS_LF_MOTOR_STATUS_INDEX           =(14)
ROS_RF_MOTOR_STATUS_INDEX           =(15)
ROS_LR_MOTOR_STATUS_INDEX           =(16)
ROS_RR_MOTOR_STATUS_INDEX           =(17)
ROS_LF_MOTOR_CURRENT_INDEX          =(18)
ROS_RF_MOTOR_CURRENT_INDEX          =(19)
ROS_LR_MOTOR_CURRENT_INDEX          =(20)
ROS_RR_MOTOR_CURRENT_INDEX          =(21)
ROS_LF_MOTOR_SPEED_INDEX            =(22)
ROS_RF_MOTOR_SPEED_INDEX            =(23)
ROS_LR_MOTOR_SPEED_INDEX            =(24)
ROS_RR_MOTOR_SPEED_INDEX            =(25)
ROS_LF_MOTOR_POSITION_INDEX         =(26)
ROS_RF_MOTOR_POSITION_INDEX         =(27)
ROS_LR_MOTOR_POSITION_INDEX         =(28)
ROS_RR_MOTOR_POSITION_INDEX         =(29)
ROS_LIN_MOTOR_STATUS_INDEX          =(30)
ROS_LIN_MOTOR_CURRENT_INDEX         =(31)
ROS_LIN_MOTOR_SPEED_INDEX           =(32)
ROS_LIN_MOTOR_POSITION_INDEX        =(33)

END_PROPULSION_DATA_BLOCK           =(34)

START_IMU_DATA_BLOCK                =(34)
ROS_IMU_X_ACC_INDEX                 =(34)
ROS_IMU_Y_ACC_INDEX                 =(35)
ROS_IMU_Z_ACC_INDEX                 =(36)
ROS_IMU_X_RATE_INDEX                =(37)
ROS_IMU_Y_RATE_INDEX                =(38)
ROS_IMU_Z_RATE_INDEX                =(39)
ROS_IMU_X_MAG_INDEX                 =(40)
ROS_IMU_Y_MAG_INDEX                 =(41)
ROS_IMU_Z_MAG_INDEX                 =(42)
ROS_IMU_STATUS_INDEX                =(43)
ROS_AHRS_PITCH_INDEX                =(44)
ROS_AHRS_ROLL_INDEX                 =(45)
ROS_AHRS_YAW_INDEX                  =(46)
ROS_AHRS_PITCH_RATE_INDEX           =(47)
ROS_AHRS_ROLL_RATE_INDEX            =(48)
ROS_AHRS_YAW_RATE_INDEX             =(49)
ROS_AHRS_FLAGS_INDEX                =(50)
END_IMU_DATA_BLOCK                  =(51)

START_DYNAMICS_DATA_BLOCK           =(51)
ROS_X_VEL_TARGET_INDEX              =(51)
ROS_Y_VEL_TARGET_INDEX              =(52)
ROS_YAW_TARGET_INDEX                =(53)
ROS_LIN_ACT_TARGET_INDEX            =(54)
ROS_X_VELOCITY_LIMIT_INDEX          =(55)
ROS_Y_VELOCITY_LIMIT_INDEX          =(56)
ROS_LIN_ACT_VELOCITY_LIMIT_INDEX    =(57)
ROS_YAW_RATE_LIMIT_INDEX            =(58)
ROS_RF_WHEEL_VEL_INDEX              =(59)
ROS_LF_WHEEL_VEL_INDEX              =(60)
ROS_RR_WHEEL_VEL_INDEX              =(61)
ROS_LR_WHEEL_VEL_INDEX              =(62)
ROS_RF_WHEEL_POS_INDEX              =(63)
ROS_LF_WHEEL_POS_INDEX              =(64)
ROS_RR_WHEEL_POS_INDEX              =(65)
ROS_LR_WHEEL_POS_INDEX              =(66)
ROS_LIN_ACT_VEL_INDEX               =(67)
ROS_LIN_ACT_POS_INDEX               =(68)
ROS_ODOM_X_ACCEL_INDEX              =(69)
ROS_ODOM_Y_ACCEL_INDEX              =(70)
ROS_ODOM_YAW_ACCEL_INDEX            =(71)
ROS_ODOM_X_VELOCITY_INDEX           =(72)
ROS_ODOM_Y_VELOCITY_INDEX           =(73)
ROS_ODOM_YAW_VELOCITY_INDEX         =(74)
ROS_ODOM_X_POSITION_INDEX           =(75)
ROS_ODOM_Y_POSITION_INDEX           =(76)
ROS_ODOM_YAW_POSITION_INDEX         =(77)
END_DYNAMICS_DATA_BLOCK             =(78)

START_APP_CONFIG_BLOCK              =(78)
ROS_APP_X_VEL_LIMIT_INDEX           =(78)
ROS_APP_Y_VEL_LIMIT_INDEX           =(79)
ROS_APP_ACCEL_LIMIT_INDEX           =(80)
ROS_APP_DECEL_LIMIT_INDEX           =(81)
ROS_APP_MAX_DTZ_DECEL_INDEX         =(82)
ROS_APP_YAW_RATE_LIMIT_INDEX        =(83)
ROS_APP_YAW_ACCEL_LIMIT_INDEX       =(84)
ROS_APP_WHEEL_DIAMETER_INDEX        =(85)
ROS_APP_WHEEL_BASE_LENGTH_INDEX     =(86)
ROS_APP_WHEEL_TRACK_WIDTH_INDEX     =(87)
ROS_APP_TRANSMISSION_RATIO_INDEX    =(88)
ROS_APP_CFG_BITMAP_INDEX            =(89)
ROS_APP_ETH_IP_ADDRESS_INDEX        =(90)
ROS_APP_ETH_PORT_NUMBER_INDEX       =(91)
ROS_APP_ETH_SUBNET_MASK_INDEX       =(92)
ROS_APP_ETH_GATEWAY_INDEX           =(93)
END_APP_CONFIG_BLOCK                =(94)

START_FRAM_CONFIG_BLOCK             =(94)
ROS_FRAM_X_VEL_LIMIT_INDEX          =(94)
ROS_FRAM_Y_VEL_LIMIT_INDEX          =(95)
ROS_FRAM_ACCEL_LIMIT_INDEX          =(96)
ROS_FRAM_DECEL_LIMIT_INDEX          =(97)
ROS_FRAM_MAX_DTZ_DECEL_INDEX        =(98)
ROS_FRAM_YAW_RATE_LIMIT_INDEX       =(99)
ROS_FRAM_YAW_ACCEL_LIMIT_INDEX      =(100)
ROS_FRAM_WHEEL_DIAMETER_INDEX       =(101)
ROS_FRAM_WHEEL_BASE_LENGTH_INDEX    =(102)
ROS_FRAM_WHEEL_TRACK_WIDTH_INDEX    =(103)
ROS_FRAM_TRANSMISSION_RATIO_INDEX   =(104)
ROS_FRAM_CFG_BITMAP_INDEX           =(105)
ROS_FRAM_ETH_IP_ADDRESS_INDEX       =(106)
ROS_FRAM_ETH_PORT_NUMBER_INDEX      =(107)
ROS_FRAM_ETH_SUBNET_MASK_INDEX      =(108)
ROS_FRAM_ETH_GATEWAY_INDEX          =(109)
END_FRAM_CONFIG_BLOCK               =(110)

ROS_CHECKSUM_INDEX                  =(110)


NUMBER_OF_CONFIG_PARAM_VARIABLES      =(ROS_FRAM_ETH_IP_ADDRESS_INDEX - START_FRAM_CONFIG_BLOCK)
NUMBER_OF_MOVO_RSP_WORDS            =(END_FRAM_CONFIG_BLOCK)
NUMBER_OF_FAULTLOG_WORDS              =(311)

