# Upstart process

This section describes the upstart process that is launched by the boot-up process on powering up MOVO.

When the platform is started it will run an upstart service, which launches:

1.  Platform driver
2.  PC power watchdog
3.  Robot localization
4.  Robot state publisher and joint state publisher
5.  Arm driver, homing arms at initialization
6.  Gripper driver, homing the grippers at initialization
7.  Full system tele-operation node
    1.  Controls all joints on the system
    2.  Subscribes to the /joy topic, which gets published elsewhere
    3.  The joystick uses the included mapping for the Logitech Extreme Pro
    4.  The mapping and software is found in ~/movo\_ws/movo\_common/movo\_ros/src/movo/movo\_teleop\_full\_system.py
8.  Kinect bridge
9.  Pan-tilt trajectory controller
10. Pan-tilt home script - the pan-tilt does a little nod to indicate things are up and running

**Parent topic:** [Powering up MOVO](../Tasks/t_powering_up_movo.md)

