# MOVO hardware overview

This section provides a quick description of the basic components of MOVO

MOVO consists of a number of components. You can think of MOVO as made up of three main sections:

-   Motion base
-   Lower torso
-   Upper torso

![](../Graphics/MOVO_sections.svg)

The motion base powers MOVO and allows MOVO to move horizontally and sense the presence of obstacles at ground level. The motion base consists of:

-   Chassis, wheels, and wheel motors
-   2 Li-Ion Batteries and charging system
-   Power distribution
-   NUC Navigation PC, which serves as ROS slave
-   2 laser scanners
-   Ethernet switch
-   Lower skin panels

The lower torso of MOVO is responsible for allowing MOVO to raise or lower its height. This consists of:

-   Linear actuator
-   Linear actuator motor
-   Linear actuator position sensor and limit switches
-   Linear actuator support structure
-   Middle skin panels

![](../Graphics/adustable_height.svg)

The upper torso of MOVO allows for vision, grasping of objects, and speech-based communication. The upper section consists of:

-   0, 1, or 2 Kinova robotic arms \(JACO 6DOF or 7DOF\)
-   Pan-tilt unit \(with Kinect camera / microphone unit\)
-   NUC Vision PC, which is the ROS master, running Ubuntu 14.04 and with all development tools installed. It contains
    -   Memory
    -   Hard drive
    -   Wi-fi router
    -   DHCP server
-   Arm controller units
-   "Dummy" base units for mounting JACO arms to MOVO and carrying power and controls to JACO arms.
-   Audio amplifier and speakers
-   Ethernet switch
-   Human Machine Interface \(HMI\) panel
-   Power distribution block
-   Upper skin panels

The MOVO platform has interfaces for controlling MOVO, connecting to MOVO, and getting indications about status.

-   **[MOVO Human-Machine Interface \(HMI\) panel](../Concepts/c_hmi_panel.md)**  
This section describes the Human-Machine Interface \(HMI\) panel, located on the rear of the upper torso.
-   **[MOVO charging port](../Concepts/c_charging_port.md)**  
This section describes the charging port used to connect MOVO to its power supply and the indicators that display the charging state.
-   **[LED panel](../Concepts/c_led_panel.md)**  
This section describes the LED status indicators on the right hand side of the motion base of MOVO.
-   **[Joystick control](../Concepts/c_teleop_control.md)**  
This section describes the joystick used for remote tele-operation of MOVO and the control mapping for the joystick.

