# MOVO boot-up process

This section describes the internal bootup process that takes place when MOVO is powered on.

After you press the power button for MOVO, MOVO goes through a system boot-up process. This process takes about 1 minute in total.

1.  The embedded system will apply power to the onboard PCs and to all the actuators
2.  The power button lighting will pulse
3.  The status LEDs will toggle yellow during initialization
4.  Once the drives are initialized, the drives will emit a white noise sound inidcating that they are enabled
5.  The status LED will toggle between yellow and green, indicating standby mode
6.  Once the PCs have booted, they will run an upstart job:

    1.  The arms and grippers will go to a homing position
    2.  The Kinect face LED will light up white, and then shut off
    3.  The pan-tilt will nod
    The system at this point is up. If the system doesn't perform all these steps, something went wrong.


**Parent topic:** [Powering up MOVO](../Tasks/t_powering_up_movo.md)

