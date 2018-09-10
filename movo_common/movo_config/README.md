Documentation on the movo_config.bash file

Movo configuration parameters
-----------------------------

Parameter | Values | Description
:--- | :--- | :---
MOVO_IP_ADDRESS | IP address value | IP address of the Waypoint vector. Do not modify unless the Waypoint vector is modified accordingly.
MOVO_HAS_KINOVA_ARM | true / false | Set to true to indicates if MOVO is equiped with at least one Kinova Ultra lightweight robotic arm. Set to false otherwise.
MOVO_HAS_TWO_KINOVA_ARMS | true / false | Set to false to indicate that MOVO is equiped with 1 Kinova arm. In that case, the arm needs to be installed on the right side. Set to true to indicate that MOVO is equiped with 2 Kinova arms. Both arms must be of the same type (either both 6DOF or both 7DOF but not one of each).
KINOVA_RIGHT_ARM_IP_ADDRESS | IP address value | Indicates the IP address of the right arm to the arm controller. If the default value is to be changed, the robotic arm base must be configured accordingly using the Kinova Development Center software utility.
KINOVA_LEFT_ARM_IP_ADDRESS | IP address value | Indicates the IP address of the left arm to the arm controler. If the default value is to be changed, the robotic arm base must be configured accordingly using the Kinova Development Center software utility.
KINOVA_ARM_IFACE | eth0 / wlan0 | Indicates to the arm controller the interface to be used for the communication with the JACO arms. eth0 - Ethernet, wlan0 - WiFi
MOVO_HAS_KINOVA_ARM_7DOF | true / false | Set to true to indicate that the Kinova arms installed are 7DOF. 
MOVO_HAS_KINOVA_ARM_6DOF | true / false | Set to true to indicate that the Kinova arms installed are 6DOF. **NOTE: MOVO_HAS_KINOVA_ARM_6DOF and MOVO_HAS_ARM_7DOF must not be the same. If one is true, the other must be false.**
USE_KG2_FOR_MOVEIT_CONFIG | true / false | Set to true to indicate to moveit that a gripper of type KG-Series 2-finger is to be used.
MOVO_HAS_RIGHT_KG2_GRIPPER | true / false | Set to true if the right arm is equiped with a gripper of type KG-Series 2-finger.
MOVO_HAS_LEFT_KG2_GRIPPER | true / false | Set to true if the left arm is equiped with a gripper of type KG-Series 2-finger.
USE_KG3_FOR_MOVEIT_CONFIG | true / false | Set to true to indicate to moveit that a gripper of type KG-Series 3-finger is to be used.
MOVO_HAS_RIGHT_KG3_GRIPPER | true / false | Set to true if the right arm is equiped with a gripper of type KG-Series 3-finger.
MOVO_HAS_LEFT_KG3_GRIPPER | true / false | Set to true if the left arm is equiped with a gripper of type KG-Series 3-finger.
USE_R85_FOR_MOVEIT_CONFIG | true/false | Set to true to indicate to moveit that a Robotiq gripper of type R85 (Robotiq 85 mm 2-finger) is to be used.
MOVO_HAS_RIGHT_ROBOTIQ_GRIPPER | true / false | Set to true if the right is equiped with a Robotiq R85 (Robotiq 85 mm 2-finger) gripper.
MOVO_HAS_LEFT_ROBOTIQ_GRIPPER | true/false | Set to true if the left is equiped with a Robotiq R85 (Robotiq 85 mm 2-finger) gripper.
