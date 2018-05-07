# MOVO Human-Machine Interface \(HMI\) panel

This section describes the Human-Machine Interface \(HMI\) panel, located on the rear of the upper torso.

The Human-Machine Interface \(HMI\) panel is located at the back of MOVO at the top of the torso. The HMI panel has a number of components.

![](../Graphics/HMI_panel.svg)

This panel is always exposed, even when the skin panels are on. The components are as follows:

**Microphone jack** - 3.5 mm connector to plug in an external microphone

**USB-A connector** - Can be used to connect a peripheral such as a keyboard or mouse, whether via a wired connection or by plugging in a compatible wireless receiver to support a wireless peripheral.

**Ethernet connector** - to plug a computer into the MOVO ethernet network with a wired connection.

**Note:** It is not advisable to connect this Ethernet port on MOVO to a wall internet port in your organization. MOVO's ROS master PC contains a DHCP server to assign IP addresses for the MOVO network, and could interfere with the normal operation of your local network.

**HDMI out connector** - To plug MOVO into an external monitor or TV to view the Ubuntu desktop of the PC.

**Power on / off button** - to turn MOVO on and off. When MOVO is off, this will trigger the power-up processes in MOVO. When MOVO is powered on, this will initiate an orderly shutdown process for MOVO, including the two MOVO PCs.

**E-stop button** - for emergency stop of MOVO.

**Status LED** - to indicate status information. The status LED will flash different colors to indicate state. The colors are as follows:

|Color|Meaning|
|-----|-------|
|Flashing yellow|Powering up|
|Alternating green and yellow|Powered up|
|Flashing red|Powering down|

Theoretically, with the connection ports, you could connect peripherals to the MOVO PC and use it directly as a development machine. In some cases, this might make sense. In general though, it will make more sense to develop on a separate development machine and connect to the MOVO PC over secure shell \(ssh\).

**Parent topic:** [MOVO hardware overview](../Concepts/c_movo_hardware_overview.md)

