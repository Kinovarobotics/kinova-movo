# Table of Contents
- [Purpose](#purpose)
- [Caution](#caution)
- [Installation and Use](#installation-and-use)
    - [Without MOVO robot](#without-movo-robot)
    - [With MOVO robot](#with-movo-robot)
        - [Installation](#installation)
        - [Using installed components](#using-installed-components)
            - [Bash commands](#bash-commands)
                - [Functions for starting demos](#functions-for-starting-demos)
                - [Functions for saving maps and updating robot PCs](#functions-for-saving-maps-and-updating-robot-pcs)
                - [Generally useful functions](#generally-useful-functions)

# Purpose
Setup scripts to get a remote PC interfaced with MOVO

# Caution
**Please read this carefully! This is provided for convenience only and will only work with a Movo robot. It is intended to be used with a remote PC that has ubuntu 16.04 LTS cleanly installed and has native hardware capable of running Gazebo simulation. It will configure the system and install all necessary updates to interface with MoVo. This process may change the environment if there is one already setup. Be sure you read through setup_remote_pc in this directory if that is the case, and make sure you fully understand what it is doing. Kinova is not responsible in any way for the use of this script and potential loss of data if it is run on any other configuration.**

# Installation and Use

## Without MOVO robot

The normal automated setup process requires connection with MovoWifi of Movo Robot. If you don't have a robot, you can set up for simulation of MOVO.

1. Install ROS and setup Kinova-movo
```
sudo apt-get install git
mkdir /movo_ws && git clone https://github.com/Kinovarobotics/kinova-movo.git src/
cd ~/movo_ws/src/movo_remote_pc_setup 
chmod +x setup_remote_pc
./setup_remote_pc
```
**NOTE:** As the `setup_remote_pc` script runs, it will prompt you at different points. Pay attention to the question when it asks if you only want to run in simulation. This will take the setup procedure in a different direction.

```
exit
```

2. Run simulation demo
**Before you begin:** copy sim_demo_VM.launch to the same location of sim_demo.launch. Open a new terminal and launch the demo.

Open a new terminal and launch the demo.
```
cd ~/movo_ws/
catkin_make
roslaunch movo_demos sim_demo_VM.launch
```

**Setup for Simulation in VM linux:**

In a VM linux machine, the setup procedure is the same as above. However, you may launch "sim_demo_VM.launch", instead of "sim_demo.launch". The recommended way to install for VM linux is using VM image created by Kinova. Please contact ros@kinova.ca for a copy.

## With MOVO robot
### Installation
To install and setup all the components on a remote PC you need to checkout the latest software, the location of the ws matters

* Create a workspace and clone the repo into the src directory
```
mkdir ~/movo_ws && cd ~/movo_ws && git clone https://github.com/Kinovarobotics/kinova-movo.git src/
```
* Run the setup script and follow the prompts
```
cd ~/movo_ws/src/movo_remote_pc_setup && ./setup_remote_pc
```

### Using installed components
Now that you have run the setup script you can take advantage of convenient functions for running demos and updates on the robot. The section below defines the bash aliases and functions added by the setup script. To run one open a terminal and type the command.

**Important note: To run any of the sim_ functions please disconnect the remote PC from the robot.**

**Important note: Always open a new terminal or tab when running new functions. This ensures that any changes to the environment variables will get applied.**

#### Bash commands
This section defines the bash aliases and functions that are added to help users more easily work with Movo

##### Functions for starting demos    
```
sim_teleop
```
Starts the functions necessary to teleop the robot with the joystick. The joystick is assumed to be the one included with the robot [Logitech Extreme 3D Pro](https://www.logitechg.com/en-us/product/extreme-3d-pro-joystick). In simulation control of the pan-tilt and arms is not enabled, only the mobile base can be controlled via joystick in simulation. Full teleop control of all DOF in the robot is available for real hardware see `robot_teleop` below.

Mobile Base control
Press Button 10 to enable mobile base control. This will allow the user to drive the mobile base around with the joystick but disable control of the arm, linear actuator and pan-tilt 
1. Button 2 puts platform in Standby Mode(no motion commands allowed)
2. Button 3 puts platform in Tractor Mode
3. Trigger (Button 0) is dead man switch and must be pressed to issue motion commands when in tractor
4. For-Aft axis maps to X velocity
5. Left-Right axis maps to Y velocity
6. Twist axis maps to Z angular velocity (yaw)

```
sim_mapping
```
This function starts SLAM mapping for the robot. To create a map:
1. Drive the robot around using the joystick as described above.
2. The map will periodically update in RVIZ
3. Once you are happy with the map you can run the command save_map _filename_

```
sim_sensor_nav
```
This function simulates navigation in the local frame. Meaning that only local obstacle avoidance is enabled and no map is used. A goal can be given in RVIZ using the **2D Nav Goal** button in the RVIZ window. Waypoints can also be added by clicking the **Publish Point** button in the rviz window, clicking the waypoints of interest. Then using the **Interact** button in the rviz menu,  clicking on the box that appears above the robot and using the menu to control waypoint navigation

**Important note: The robot footprint only accounts for the tucked position. It does not dynamically adjust if the arms are extended. It is recommended to always tuck the arms when using navigation**

**Important note: Goals can only be issued within the bounding box for the local frame which is approximately a 5m square centered around the robot.**

```
sim_map_nav
```
This function simulates 2D navigation in a known map. Meaning that both global localization and local obstacle avoidance are used in path planning. A goal can be given in RVIZ using the **2D Nav Goal** button in the RVIZ window. Waypoints can also be added by clicking the **Publish Point** button in the rviz window, clicking the waypoints of interest. Then using the **Interact** button in the rviz menu,  clicking on the box that appears above the robot and using the menu to control waypoint navigation.

**Important note: The robot footprint only accounts for the tucked position. It does not dynamically adjust if the arms are extended. It is recommended to always tuck the arms when using navigation**

```
sim_assisted_teleop
```
This function simulates assisted teleop for the mobile base. Assisted teleop uses 2D costmap and base_local_planner to avoid collision while using teleop. It basically converts the requested teleop command into a trajectory and then checksthe trajectory for collisions and replans as close to the commanded trajectory as possible while avoiding collisions with things the laser can see. 

**Important note: The robot footprint only accounts for the tucked position. It does not dynamically adjust if the arms are extended. It is recommended to always tuck the arms when using navigation**
```
robot_teleop
```
Starts the functions necessary to teleop the robot with the joystick. The joystick is assumed to be the one included with the robot [Logitech Extreme 3D Pro](https://www.logitechg.com/en-us/product/extreme-3d-pro-joystick)

###### Joystick mapping
Mapping is defined in ~/movo_ws/src/movo_common/movo_ros/src/movo/movo_full_system_teleop.py. The index maps to the number on the joystick
```
        """
        Set the mapping for the various commands
        """        
        self.ctrl_map  = dict({'momentary': {'dead_man'     : {'is_button':True,'index':1,'set_val':1},
                                             'man_ovvrd'    : {'is_button':True,'index':2,'set_val':1},
                                             'standby'      : {'is_button':True,'index':3,'set_val':1},
                                             'tractor'      : {'is_button':True,'index':4,'set_val':1},
                                             'wrist1'       : {'is_button':True,'index':5,'set_val':1},
                                             'wrist2'       : {'is_button':True,'index':6,'set_val':1},
                                             'estop'        : {'is_button':True,'index':7,'set_val':1},
                                             'home_arms'    : {'is_button':True,'index':8,'set_val':1},
                                             'pan_tilt_ctl' : {'is_button':True,'index':9,'set_val':1},
                                             'base_ctl'     : {'is_button':True,'index':10,'set_val':1},
                                             'arm_ctl_right': {'is_button':True,'index':11,'set_val':1},
                                             'arm_ctl_left' : {'is_button':True,'index':12,'set_val':1}},
                               'axis'     : {'left_right'   : {'index' :1, 'invert_axis':False},
                                             'for_aft'      : {'index' :2, 'invert_axis':False},
                                             'twist'        : {'index' :3, 'invert_axis':False},
                                             'flipper'   : {'index' :4, 'invert_axis':False},
                                             'dpad_lr'   : {'index' :5, 'invert_axis':False},
                                             'dpad_ud'   : {'index' :6, 'invert_axis':False}}})
```
E-Stop:
Press button 7 to activate the software E-Stop which will cut power to all torque producing components. Pressing it again will re-enable the actuators in the system and resume normal operation.

Mobile Base:
Press Button 10 to enable mobile base control. This will allow the user to drive the mobile base around with the joystick but disable control of the arm, linear actuator and pan-tilt 
1. Button 2 puts platform in Standby Mode(no motion commands allowed)
2. Button 3 puts platform in Tractor Mode
3. Trigger (Button 0) is dead man switch and must be pressed to issue motion commands when in tractor
4. For-Aft axis maps to X velocity
5. Left-Right axis maps to Y velocity
6. Twist axis maps to Z angular velocity (yaw)

Arms and linear actuator:
Press Button 8 to home both arms
Press Button 10 to enable right arm control (mobile base, pan-tilt and left arm control will be disabled)
Press Button 11 to enable left arm control  (mobile base, pan-tilt and right arm control will be disabled)
1. Trigger (Button 0) enables commands 
  a. release to hold current position
2. D-PAD up/down rotates the end-effector around y axis
3. D-PAD left/right rotates the end-effector around x axis
4. Button 2 and 3 rotate the end-effector around Z axis
5. For-Aft axis maps to x Cartesian velocity of end-effector
6. Left-Right axis maps to y Cartesian velocity of end-effector
7. Twist axis maps to z Cartesian velocity of end-effector
8. If the thumb button (Button 1) is pressed the Twist axis maps to linear actuator velocity through a mapping function that outputs position by integrating the signal
9. Flipper paddle axis opens and closes the gripper


Pan-Tilt control:
Press Button 9 to enable pan-tilt control (base and arm control will be disabled)
1. Trigger (Button 0) enables commands 
  a. release to hold current position
2. For-Aft axis maps to tilt position
3. Twist axis maps to pan position


```
robot_mapping
```
This function starts SLAM mapping for the robot. To create a map:
1. Drive the robot around using the joystick as described above.
2. The map will periodically update in RVIZ
3. Once you are happy with the map you can run the command save_map _filename_

```
robot_sensor_nav
```
This function starts navigation in the local frame. Meaning that only local obstacle avoidance is enabled and no map is used. A goal can be given in RVIZ using the **2D Nav Goal** button in the RVIZ window. Waypoints can also be added by clicking the **Publish Point** button in the rviz window, clicking the waypoints of interest. Then using the **Interact** button in the rviz menu,  clicking on the box that appears above the robot and using the menu to control waypoint navigation

**Important note: The robot footprint only accounts for the tucked position. It does not dynamically adjust if the arms are extended. It is recommended to always tuck the arms when using navigation**

**Important note: Goals can only be issued within the bounding box for the local frame which is approximately a 5m square centered around the robot.**


```
robot_map_nav filename
```
This function runs 2D navigation in a known map.  Meaning that both global localization and local obstacle avoidance are used in path planning. A goal can be given in RVIZ using the **2D Nav Goal** button in the RVIZ window. Waypoints can also be added by clicking the **Publish Point** button in the rviz window, clicking the waypoints of interest. Then using the **Interact** button in the rviz menu,  clicking on the box that appears above the robot and using the menu to control waypoint navigation.

The map is passed as a parameter _filname_, and should match the name used from the `save_map filename` command.

**Important note: The robot footprint only accounts for the tucked position. It does not dynamically adjust if the arms are extended. It is recommended to always tuck the arms when using navigation**

```
robot_assisted_teleop
```
This function starts assisted teleop for the mobile base. Assisted teleop uses 2D costmap and base_local_planner to avoid collision while using teleop. It basically converts the requested teleop command into a trajectory and then checksthe trajectory for collisions and replans as close to the commanded trajectory as possible while avoiding collisions with things the laser can see. 

**Important note: The robot footprint only accounts for the tucked position. It does not dynamically adjust if the arms are extended. It is recommended to always tuck the arms when using navigation**

**Important note: Assisted teleop is only for the mobile base, the rest of teleoperation (arms, pan-tilt, linear actuator) works as described in `robot_teleop` with no obstacle avoidance when assisted teleop is enabled**


##### Functions for saving maps and updating robot PCs
```
save_map [filename]
```
Saves the mapfile on the robot and onboard PC. **_filename_** is the user defined mapfile name

```
sync_robot [compile options]
```
Allows user to make changes locally on the remote PC in ~/movo_ws/src and push the changes to the robot
* compile options **must specify one**
    * -nc Do not compile just copy the files from the remote PC workspace to the robot and restart the service on the robot. Useful to save time when just launch files or python files are changed.
    * -c copy the files from the remote PC workspace to the robot, compile, and restart the service

##### Generally Useful Functions
```
m2
```
Alias for SSH into movo2. Equivalent to ssh -X movo@movo2

```
m1
```
Alias for SSH into movo2. Equivalent to ssh -X movo@movo1

```
movostop
```
Stops the systemd service on movo2 that runs the ROS software on the robot at startup

```
movostart
```
Starts the systemd service on movo2 that runs the ROS software on the robot

```
movochk
```
Tails the journalctl log of the movo-core service. It shows the log as if you would have launched it in the foreground from a terminal.

```
sws
```
Sources the workspace from a top level directory (same as source ./devel/setup.sh)

```
clean_backups
```
Removes all the *~ backupfiles from a directory tree. Useful for cleanup

```
clean_pyc
```
Removes all the *.pyc compiled python from a directory tree. Useful for cleanup

```
clean_rosbuild
```
Removes the devel/ and build/ directories from a workspace. Useful for cleanup

```
kill_robot_pcs
```
Shutsdown movo1 and movo2

```
fix_gvfs
```
Run this if you ever get the bash error _cannot access .gvfs: Permission denied_. It is included for convenience

```
killgazebo
```
Kills all processes associated with Gazebo if it hangs at shutdown. It is included for convenience

```
killros
```
Kills all processes associated with ROS if any hangs at shutdown. It is included for convenience

```
fix_stl
```
Fixes an STL file containing the string solid, which ASSIMP does not like

```
fix_perm
```
Fixes permissions on executable files in a ROS workspace. It is useful when copying code from a USB stick where the permissions were not preserved.
