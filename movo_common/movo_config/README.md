start of documentation on the movo_config.bash file

Movo configuration parameters
-----------------------------

MOVO_IP_ADDRESS
IP adresse of the Waypoint vector, do not modify unless the Waypoint vector is modified accordingly.

MOVO_POWERS_PC_ONBOARD
Not used, should be removed from the file

MOVO_HAS_KINOVA_ARM
Set to true to indicates if MOVO is equiped with at least one JACO arm. Set to false otherwise

MOVO_HAS_TWO_KINOVA_ARMS
Set to false to indicates that MOVO is equiped with 1 JACO arm. In that case, the arm shall be placed on the right side.
Set to true to indicates that MOVO is equiped with 2 JACO arms. Both arms must be of the same type (6DoF or 7DoF).

KINOVA_RIGHT_ARM_IP_ADDRESS
Indicate IP address of the rigth arm to the arm controler. If the default value is to be changed, the JACO base must be configured accordingly using the Kinova Development Center software utility.

KINOVA_LEFT_ARM_IP_ADDRESS
Indicate IP address of the left arm to the arm controler. If the default value is to be changed, the JACO base must be configured accordingly using the Kinova Development Center software utility.

KINOVA_ARM_IFACE
Indicates to the arm controller the interface to be used for the communication with the JACO arms.

MOVO_HAS_KINOVA_ARM_7DOF
Set to true to indicates that the JACO arms installed are 7DoF. 

MOVO_HAS_KINOVA_ARM_6DOF
Set to true to indicates that the JACO arms installed are 6DoF.

USE_KG2_FOR_MOVEIT_CONFIG
Set to true to indicate to moveit that a gripper of type KG2 is to be used.

MOVO_HAS_RIGHT_KG2_GRIPPER
Set to true if the right is equiped with a gripper of type KG2.

MOVO_HAS_LEFT_KG2_GRIPPER
Set to true if the left is equiped with a gripper of type KG2.

USE_KG3_FOR_MOVEIT_CONFIG
Set to true to indicate to moveit that a gripper of type KG3 is to be used.

MOVO_HAS_RIGHT_KG3_GRIPPER
Set to true if the right is equiped with a gripper of type KG3.

MOVO_HAS_LEFT_KG3_GRIPPER
Set to true if the left is equiped with a gripper of type KG3.

USE_R85_FOR_MOVEIT_CONFIG
Set to true to indicate to moveit that a Robotiq gripper of type R85 is to be used.

MOVO_HAS_RIGHT_KG3_GRIPPER
Set to true if the right is equiped with a gripper of type KG3.

MOVO_HAS_LEFT_KG3_GRIPPER
Set to true if the left is equiped with a gripper of type KG3.
