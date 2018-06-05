# MOVO shutdown process

Describes the internal process when MOVO is shut down.

When the power button is pressed to shut down MOVO, the following shutdown process takes place:

1.  All power to the torque producing actuators \(except pan-tilt\) will be disabled.
2.  The Upstart service has a PC power watchdog that will be notified and the PC’s will be shut down.
3.  Any faults get written to the faultlog.
4.  The power LED will rapidly pulse and the system will completely power off in 30 seconds. The delay allows time for the orderly shutdown of onboard PC’s and peripherals.

**Parent topic:** [Powering down MOVO](../Tasks/t_powering_down_movo.md)

